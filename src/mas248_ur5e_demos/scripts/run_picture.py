"""Student assignment: draw /tmp/mas248_targets.json with MoveIt2 Cartesian paths (MOCK only).

Start: IK (seeded elbow-up, tool down) for the pose above stroke 1 -> joint-space move there.
Per stroke: Cartesian travel at lift height -> down -> along stroke -> up.
Staying Cartesian keeps one arm configuration; a free OMPL pose goal may pick a flipped one.
Every Cartesian path is checked for completion fraction BEFORE execution; partial plans are never run.
"""
import argparse
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from .common import move_group, cartesian, execute_trajectory, wait_future, JOINTS, WORLD, GROUP, TIP

# Elbow-up, tool pointing down (same as SRDF 'home'); seeds IK so the arm does not flip.
SEED = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


def to_pose(p, dz=0.0):
    pose = Pose()
    pose.position.x = float(p['x']); pose.position.y = float(p['y']); pose.position.z = float(p['z']) + dz
    pose.orientation.x = float(p['qx']); pose.orientation.y = float(p['qy'])
    pose.orientation.z = float(p['qz']); pose.orientation.w = float(p['qw'])
    return pose


def solve_ik(node, pose):
    cli = node.create_client(GetPositionIK, '/compute_ik')
    if not cli.wait_for_service(timeout_sec=20.0):
        raise RuntimeError('MoveIt /compute_ik not available')
    req = GetPositionIK.Request()
    req.ik_request.group_name = GROUP
    req.ik_request.ik_link_name = TIP
    req.ik_request.avoid_collisions = True
    req.ik_request.robot_state.joint_state.name = list(JOINTS)
    req.ik_request.robot_state.joint_state.position = list(SEED)
    req.ik_request.pose_stamped.header.frame_id = WORLD
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.timeout.sec = 1
    res = wait_future(node, cli.call_async(req), 30)
    if res.error_code.val != 1:
        raise RuntimeError(f'No collision-free IK for start pose (error {res.error_code.val})')
    names = list(res.solution.joint_state.name)
    return [res.solution.joint_state.position[names.index(j)] for j in JOINTS]


def max_joint_step(trajectory):
    pts = trajectory.joint_trajectory.points
    return max((abs(b - a) for p, q in zip(pts, pts[1:]) for a, b in zip(p.positions, q.positions)), default=0.0)


def checked_cartesian(node, waypoints, label, args):
    # MoveIt's relative jump_threshold gives false alarms on short zig-zag contour segments,
    # so it is disabled and joint continuity is checked with an absolute limit instead.
    result = cartesian(node, waypoints, max_step=args.max_step, jump_threshold=0.0)
    if result.fraction < args.min_fraction:
        raise RuntimeError(f'{label}: fraction {result.fraction:.3f} < {args.min_fraction}; refusing partial plan')
    jump = max_joint_step(result.solution)
    if jump > args.max_joint_jump:
        raise RuntimeError(f'{label}: joint jump {jump:.3f} rad > {args.max_joint_jump}; refusing (IK flip?)')
    if args.execute:
        execute_trajectory(node, result.solution)
    return result.fraction


def main():
    p = argparse.ArgumentParser(description='Draw image targets with MoveIt2 Cartesian paths (MOCK only)')
    p.add_argument('--targets', default='/tmp/mas248_targets.json')
    p.add_argument('--lift', type=float, default=0.05, help='Pen-up height above drawing poses [m]')
    p.add_argument('--max-step', type=float, default=0.005, help='Cartesian interpolation step [m]')
    p.add_argument('--min-fraction', type=float, default=0.99)
    p.add_argument('--max-joint-jump', type=float, default=0.3, help='Max joint change between trajectory points [rad]')
    p.add_argument('--max-strokes', type=int, default=0, help='0 = all strokes')
    p.add_argument('--plan-only', dest='execute', action='store_false',
                   help='Only plan the move to the start pose (robot does not move)')
    args = p.parse_args()

    with open(args.targets, encoding='utf-8') as f:
        data = json.load(f)
    strokes = [s['poses'] for s in data['strokes'] if len(s['poses']) >= 2]
    if args.max_strokes > 0:
        strokes = strokes[:args.max_strokes]

    rclpy.init(); node = Node('mas248_run_picture')
    log = node.get_logger()
    log.info(f'{len(strokes)} strokes from {args.targets}, lift={args.lift} m, '
             f'max_step={args.max_step} m, min_fraction={args.min_fraction}')
    try:
        start = solve_ik(node, to_pose(strokes[0][0], args.lift))
        log.info('Start joints: ' + ', '.join(f'{v:.2f}' for v in start))
        move_group(node, joints=start, execute=args.execute)
        if not args.execute:
            log.warn('PLAN ONLY: start move planned; Cartesian strokes need the robot there. Stopping.')
            strokes = []
        for i, poses in enumerate(strokes, 1):
            log.info(f'Stroke {i}/{len(strokes)}: {len(poses)} points')
            # Travel above start (already there for stroke 1), pen down, draw, pen up. All straight lines.
            if i > 1:
                checked_cartesian(node, [to_pose(poses[0], args.lift)], f'stroke {i} travel', args)
            checked_cartesian(node, [to_pose(poses[0])], f'stroke {i} down', args)
            frac = checked_cartesian(node, [to_pose(q) for q in poses[1:]], f'stroke {i} draw', args)
            checked_cartesian(node, [to_pose(poses[-1], args.lift)], f'stroke {i} up', args)
            log.info(f'Stroke {i} done, draw fraction {frac:.3f}')
        log.info('Drawing finished')
    except Exception as exc:
        log.error(str(exc)); raise
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
