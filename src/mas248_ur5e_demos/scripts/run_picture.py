"""MAS248 Lecture 9: make the UR5e draw the picture from /tmp/mas248_targets.json.

For every stroke in the JSON file:
  1. move to a point APPROACH metres above the first point (straight line at safe
     height from the previous stroke if possible, otherwise a joint-space plan)
  2. Cartesian move straight down to the first point ("pen down")
  3. Cartesian path through all points of the stroke
  4. Cartesian move straight up again ("pen up")

Every Cartesian segment is checked: if MoveIt can only plan part of it
(fraction < --min-fraction), the stroke is skipped. Partial plans are never executed.
Default is PLAN ONLY. Use --execute to run on the mock controllers (never a real robot).
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from .common import JOINTS, move_group, cartesian, execute_trajectory

HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # same as 'home' in the SRDF


def pose_to_list(p):
    """Return [x, y, z, qx, qy, qz, qw] whether the JSON pose is a list or a dict."""
    if isinstance(p, (list, tuple)):
        return list(p)
    if 'position' in p:
        pos, ori = p['position'], p['orientation']
        if isinstance(pos, (list, tuple)):
            return list(pos) + list(ori)
        return [pos['x'], pos['y'], pos['z'], ori['x'], ori['y'], ori['z'], ori['w']]
    return [p['x'], p['y'], p['z'], p['qx'], p['qy'], p['qz'], p['qw']]


def to_pose(values, dz=0.0):
    """Build a geometry_msgs/Pose from [x, y, z, qx, qy, qz, qw], optionally lifted by dz."""
    x, y, z, qx, qy, qz, qw = values
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z) + dz
    pose.orientation.x, pose.orientation.y = float(qx), float(qy)
    pose.orientation.z, pose.orientation.w = float(qz), float(qw)
    return pose


def last_joints(trajectory):
    """Joint angles at the end of a RobotTrajectory, in the order of JOINTS."""
    jt = trajectory.joint_trajectory
    if not jt.points:
        return None
    end = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [end[name] for name in JOINTS]


class PictureRunner:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.log = node.get_logger()
        self.joints = None  # where the robot is (or will be) after the last segment
        self.at_safe_height = False  # True when the tool is lifted above a previous stroke

    def start_state(self):
        """Executing: let MoveIt use the robot's real current state.
        Plan only: chain from the end of the previous planned segment."""
        return None if self.args.execute else self.joints

    def joint_move(self, joints=None, pose=None):
        result = move_group(self.node, joints=joints, pose=pose, execute=self.args.execute,
                            start_joint_positions=self.start_state())
        self.joints = last_joints(result.planned_trajectory) or self.joints

    def cartesian_move(self, poses, label):
        """Plan a Cartesian segment; execute it only if it is complete. Returns the fraction."""
        result = cartesian(self.node, poses, max_step=self.args.max_step,
                           start_joint_positions=self.start_state(),
                           jump_threshold=self.args.jump_threshold)
        if result.fraction < self.args.min_fraction:
            self.log.warn(f'{label}: only {result.fraction:.1%} of the path could be planned -> skipped')
            return result.fraction
        if self.args.execute:
            execute_trajectory(self.node, result.solution)
        self.joints = last_joints(result.solution) or self.joints
        return result.fraction

    def run(self):
        with open(self.args.json) as f:
            data = json.load(f)
        strokes = [[pose_to_list(p) for p in s['poses']] for s in data['strokes']]
        self.log.info(f'Loaded {len(strokes)} strokes from {self.args.json}')

        self.log.info('Moving to home')
        self.joint_move(joints=HOME)

        summary = []
        for i, points in enumerate(strokes, 1):
            points = points[::self.args.every]
            if self.args.close and len(points) > 2:
                points.append(points[0])  # contours are closed loops
            if len(points) < 2:
                summary.append((i, 'skipped (too few points)', 0.0))
                continue
            self.log.info(f'--- Stroke {i}/{len(strokes)}: {len(points)} points ---')
            try:
                # 1. approach: move to just above the first point.
                #    From a lifted tool we first try a straight line at safe height, which keeps
                #    the same arm configuration; otherwise (or if that fails) a joint-space plan.
                above_start = to_pose(points[0], dz=self.args.approach)
                travelled = False
                if self.at_safe_height:
                    travelled = self.cartesian_move([above_start], f'Stroke {i} travel') \
                        >= self.args.min_fraction
                if not travelled:
                    self.joint_move(pose=above_start)
                # 2. pen down
                self.at_safe_height = False
                f_down = self.cartesian_move([to_pose(points[0])], f'Stroke {i} pen down')
                if f_down < self.args.min_fraction:
                    self.at_safe_height = True  # pen down was not executed: still above
                    summary.append((i, 'skipped (pen down failed)', f_down))
                    continue
                # 3. draw the stroke
                f_draw = self.cartesian_move([to_pose(p) for p in points[1:]], f'Stroke {i} draw')
                # 4. pen up (always try, so the tool leaves the surface)
                self.cartesian_move([to_pose(points[-1] if f_draw >= self.args.min_fraction
                                             else points[0], dz=self.args.approach)],
                                    f'Stroke {i} pen up')
                self.at_safe_height = True
                status = 'drawn' if f_draw >= self.args.min_fraction else 'skipped (incomplete path)'
                summary.append((i, status, f_draw))
            except Exception as exc:  # e.g. no IK solution for the approach pose
                self.log.error(f'Stroke {i} failed: {exc}')
                summary.append((i, f'failed ({exc})', 0.0))

        self.log.info('Returning to home')
        self.at_safe_height = False
        self.joint_move(joints=HOME)

        self.log.info('===== SUMMARY =====')
        for i, status, fraction in summary:
            self.log.info(f'Stroke {i}: {status}, fraction = {fraction:.3f}')
        drawn = sum(1 for _, s, _ in summary if s == 'drawn')
        self.log.info(f'{drawn}/{len(summary)} strokes complete'
                      + ('' if self.args.execute else ' (PLAN ONLY, robot did not move)'))


def main():
    p = argparse.ArgumentParser(description='Draw the picture from the target JSON with MoveIt 2')
    p.add_argument('--json', default='/tmp/mas248_targets.json')
    p.add_argument('--execute', action='store_true', help='execute on MOCK controllers only')
    p.add_argument('--approach', type=float, default=0.05, help='pen-up height in metres')
    p.add_argument('--max-step', type=float, default=0.005, help='Cartesian step in metres')
    p.add_argument('--min-fraction', type=float, default=0.99, help='skip stroke below this')
    p.add_argument('--jump-threshold', type=float, default=1.5,
                   help='MoveIt joint-jump check for Cartesian paths (0 = off, for diagnosis only)')
    p.add_argument('--every', type=int, default=1, help='use every Nth point (fewer points)')
    p.add_argument('--close', action='store_true', help='return to the first point of each contour')
    args = p.parse_args()
    args.every = max(1, args.every)

    rclpy.init()
    node = Node('mas248_run_picture')
    try:
        PictureRunner(node, args).run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
    