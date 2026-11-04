#!/usr/bin/env python3
"""Generate the Lecture 10 real-robot workcell Xacro from the Lecture 8 workcell.

Written by the MAS248 group: the scripts/ folder of the handed-out
mas248_ur5e_real package was empty. The contract follows CMakeLists.txt
and launch/_real_common.py:

  --source         canonical workcell (mas248_ur5e/urdf/ur5e_workcell.urdf.xacro), read only
  --output         derived Xacro in build/, installed as urdf/ur5e_workcell_real.urdf.xacro
  --config-output  folder for vendor config files, installed as config/ur5e/

Compared with the source, only the <xacro:ur_robot> call changes:
  * every value the real UR driver needs becomes $(arg ...), with the same arg
    names that ur_robot_driver/ur_control.launch.py passes on the command line
  * generate_ros2_control_tag is forced to true (real UR hardware interface)
  * one set of top-level xacro:arg declarations is written at the top; hardware
    defaults come from config/real_robot_defaults.yaml (single source)
Pedestal, table, spindle_body and tcp are copied unchanged.
"""
import argparse
import importlib.util
import os
import re
import shutil
import sys
import xml.dom.minidom as minidom
from pathlib import Path

import yaml

PKG = 'mas248_ur5e_real'
PKG_DIR = Path(__file__).resolve().parent.parent
DEFAULTS_FILE = PKG_DIR / 'config' / 'real_robot_defaults.yaml'
UR_TYPE = 'ur5e'
VENDOR_CONFIG_FILES = ('joint_limits.yaml', 'physical_parameters.yaml', 'visual_parameters.yaml')

# ur_robot macro parameter -> top-level xacro arg (names as in ur_control.launch.py).
MACRO_TO_ARG = {
    'name': 'name',
    'ur_type': 'ur_type',
    'tf_prefix': 'tf_prefix',
    'joint_limits_parameters_file': 'joint_limit_params',
    'kinematics_parameters_file': 'kinematics_params',
    'physical_parameters_file': 'physical_params',
    'visual_parameters_file': 'visual_params',
    'use_fake_hardware': 'use_fake_hardware',
    'fake_sensor_commands': 'fake_sensor_commands',
    'robot_ip': 'robot_ip',
    'script_filename': 'script_filename',
    'input_recipe_filename': 'input_recipe_filename',
    'output_recipe_filename': 'output_recipe_filename',
    'use_currents_as_efforts': 'use_currents_as_efforts',
}
# Every key in real_robot_defaults.yaml 'hardware' maps to a macro parameter of the same name.

# Without these the real driver cannot work: stop the build if the installed
# ur_description is too old to have them.
REQUIRED_MACRO_PARAMS = {
    'generate_ros2_control_tag', 'kinematics_parameters_file', 'robot_ip', 'headless_mode',
    'script_filename', 'input_recipe_filename', 'output_recipe_filename',
    'reverse_port', 'script_sender_port', 'safety_limits',
}


def fail(message):
    print('prepare_description.py: ' + message, file=sys.stderr)
    sys.exit(1)


def share_dir(package):
    """Installed share/ folder of a ROS package, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
            candidate = Path(prefix) / 'share' / package
            if prefix and candidate.is_dir():
                return candidate
    return None


def hardware_defaults():
    data = yaml.safe_load(DEFAULTS_FILE.read_text(encoding='utf-8'))
    return {k: str(v).lower() if isinstance(v, bool) else str(v)
            for k, v in data['hardware'].items()}


def macro_params(ur_description):
    """Parameter names of <xacro:macro name="ur_robot"> in the installed ur_description."""
    text = (ur_description / 'urdf' / 'ur_macro.xacro').read_text(encoding='utf-8')
    match = re.search(r'<xacro:macro\s+name="ur_robot"\s+params="([^"]*)"', text)
    if not match:
        fail('cannot find the ur_robot macro in ' + str(ur_description))
    return {p.split(':=')[0].lstrip('*') for p in match.group(1).split()}


def arg_defaults(hardware):
    """Default value for every top-level arg in the derived Xacro."""
    config = '$(find ' + PKG + ')/config/' + UR_TYPE + '/'
    defaults = {
        'ur_type': UR_TYPE,
        'name': UR_TYPE,                 # ur_control.launch.py passes name:=ur_type
        'tf_prefix': '',
        'robot_ip': '0.0.0.0',           # the launch files validate the real IP
        'kinematics_params': '',         # MUST be the robot calibration (launch validates it)
        'joint_limit_params': config + 'joint_limits.yaml',
        'physical_params': config + 'physical_parameters.yaml',
        'visual_params': config + 'visual_parameters.yaml',
        'use_fake_hardware': 'false',
        'fake_sensor_commands': 'false',
        'mock_control': 'false',         # passed by _real_common.py, not used by the UR macro
        'script_filename': '',
        'input_recipe_filename': '',
        'output_recipe_filename': '',
        'use_currents_as_efforts': 'true',
    }
    defaults.update(hardware)
    return defaults


def build_xacro(source, hardware, available):
    doc = minidom.parse(str(source))
    root = doc.documentElement
    if root.tagName != 'robot' or root.getAttribute('name') != 'mas248_ur5e':
        fail('expected <robot name="mas248_ur5e"> in ' + str(source))

    defaults = arg_defaults(hardware)

    # Remove the source's own top-level arg declarations that we re-declare,
    # so the real-robot defaults (e.g. use_fake_hardware=false) are the only ones.
    for node in list(root.childNodes):
        if (node.nodeType == node.ELEMENT_NODE and node.tagName == 'xacro:arg'
                and node.getAttribute('name') in defaults):
            before = node.previousSibling
            if before is not None and before.nodeType == node.TEXT_NODE and not before.data.strip():
                root.removeChild(before)   # drop the indentation left behind
            root.removeChild(node)

    calls = root.getElementsByTagName('xacro:ur_robot')
    if len(calls) != 1:
        fail('expected exactly one <xacro:ur_robot> in the source, found ' + str(len(calls)))
    ur = calls[0]

    mapping = dict(MACRO_TO_ARG)
    mapping.update({name: name for name in hardware})
    skipped = []
    for param, arg in sorted(mapping.items()):
        if param in available:
            ur.setAttribute(param, '$(arg ' + arg + ')')
        else:
            skipped.append(param)
    ur.setAttribute('generate_ros2_control_tag', 'true')

    # Header comment and arg declarations at the top of <robot>.
    first = root.firstChild
    header = doc.createComment(
        ' GENERATED by mas248_ur5e_real/scripts/prepare_description.py from '
        + source.name + '. Do not edit: change the source workcell and rebuild. ')
    root.insertBefore(doc.createTextNode('\n  '), first)
    root.insertBefore(header, first)
    for name in sorted(defaults):
        element = doc.createElement('xacro:arg')
        element.setAttribute('name', name)
        element.setAttribute('default', defaults[name])
        root.insertBefore(doc.createTextNode('\n  '), first)
        root.insertBefore(element, first)
    root.insertBefore(doc.createTextNode('\n'), first)
    return doc, skipped


def copy_vendor_config(ur_description, config_output):
    config_output.mkdir(parents=True, exist_ok=True)
    for name in VENDOR_CONFIG_FILES:
        src = ur_description / 'config' / UR_TYPE / name
        if not src.is_file():
            fail('missing vendor file ' + str(src))
        shutil.copyfile(src, config_output / name)
    # The update rate must match the installed driver: copy its file if present,
    # otherwise the source copy in config/ is installed (see CMakeLists.txt).
    driver = share_dir('ur_robot_driver')
    rate = driver / 'config' / (UR_TYPE + '_update_rate.yaml') if driver else None
    if rate and rate.is_file():
        shutil.copyfile(rate, config_output.parent / rate.name)
        return 'copied ' + rate.name + ' from ur_robot_driver'
    return 'ur_robot_driver not found: using config/' + UR_TYPE + '_update_rate.yaml from source'


def render_check(output, config_output, ur_description):
    """Render the derived Xacro once and run the same check as the launch files."""
    try:
        import xacro
    except ImportError:
        return 'xacro not importable: render check skipped'
    mappings = {
        'robot_ip': '192.168.1.10',
        # Structure check only: the real launch passes the robot calibration.
        'kinematics_params': str(ur_description / 'config' / UR_TYPE / 'default_kinematics.yaml'),
        'joint_limit_params': str(config_output / 'joint_limits.yaml'),
        'physical_params': str(config_output / 'physical_parameters.yaml'),
        'visual_params': str(config_output / 'visual_parameters.yaml'),
        'script_filename': 'render_check', 'input_recipe_filename': 'render_check',
        'output_recipe_filename': 'render_check',
    }
    xml = xacro.process_file(str(output), mappings=mappings).toxml()
    spec = importlib.util.spec_from_file_location('_real_common', PKG_DIR / 'launch' / '_real_common.py')
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)
    common.validate_rendered(xml)   # raises RuntimeError with a clear message
    return 'render check passed (links, 6 joints, one real ur_robot_driver ros2_control)'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config-output', required=True, type=Path)
    args = parser.parse_args()

    if not args.source.is_file():
        fail('missing source workcell ' + str(args.source))
    ur_description = share_dir('ur_description')
    if ur_description is None:
        fail('ur_description not found: source /opt/ros/humble/setup.bash first')

    available = macro_params(ur_description)
    missing = REQUIRED_MACRO_PARAMS - available
    if missing:
        fail('installed ur_description is too old, ur_robot macro lacks ' + str(sorted(missing)))

    doc, skipped = build_xacro(args.source, hardware_defaults(), available)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(doc.toxml(encoding='utf-8'))
    rate_note = copy_vendor_config(ur_description, args.config_output)

    try:
        check_note = render_check(args.output, args.config_output, ur_description)
    except Exception as exc:  # report any render or validation error as a build failure
        fail('generated Xacro failed the render check: ' + str(exc))

    print('Generated ' + str(args.output))
    print('Vendor config from ' + str(ur_description) + '; ' + rate_note)
    if skipped:
        print('Not supported by installed ur_description (left out): ' + ', '.join(skipped))
    print(check_note)


if __name__ == '__main__':
    main()
