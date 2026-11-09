"""Beroringstest: pennen ned til tegneflaten midt pa arket, og opp igjen.

Stegene, alle som rette linjer (cartesian) med pennen rett ned:
  1. til --over (3 cm) over punktet
  2. til 5 mm over punktet          (venter pa Enter for)
  3. ned til --z (beroring, z = 0)  (venter pa Enter for)
  4. opp igjen til --over

Standard: bare planlegging. Med --execute kjores stegene, og skriptet venter pa Enter
for de to siste stegene nedover. Ctrl+C avbryter. Ha hand pa nodstoppen.

Kjor:  ros2 run mas248_project touch_test              (bare planlegg)
       ros2 run mas248_project touch_test --execute    (beveg roboten)
       ros2 run mas248_project touch_test --execute --z 0.002   (stopp 2 mm over)
"""
import argparse

import rclpy
from rclpy.node import Node

# Hjelpefunksjonene kommer fra kurspakken mas248_ur5e_demos, som ikke ligger i Git.
from scripts.common import JOINTS, cartesian, execute_trajectory, make_pose

# Arkets midte i world (samme som paper_x/paper_y i xacro-fila og ARK_SENTER i run_picture)
ARK_SENTER = (0.50, 0.06)
# Laveste tillatte beroringshoyde (m): stopper skrivefeil som ville kjort pennen gjennom bordet
LAVESTE_Z = -0.003


def siste_leddvinkler(bane):
    """Leddvinklene i siste punkt av en RobotTrajectory, i JOINTS-rekkefolge."""
    jt = bane.joint_trajectory
    vinkel_for = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [vinkel_for[navn] for navn in JOINTS]


def main():
    ap = argparse.ArgumentParser(description='MAS248: beroringstest, pennen ned til arket og opp igjen.')
    ap.add_argument('--x', type=float, default=ARK_SENTER[0], help='x i world (m), standard arkets midte')
    ap.add_argument('--y', type=float, default=ARK_SENTER[1], help='y i world (m), standard arkets midte')
    ap.add_argument('--z', type=float, default=0.0, help='beroringshoyde i world (m), standard 0.0')
    ap.add_argument('--over', type=float, default=0.03, help='hoyde over beroringspunktet for start/slutt (m)')
    ap.add_argument('--execute', action='store_true', help='beveg roboten (venter pa Enter for nedstegene)')
    args = ap.parse_args()

    if args.z < LAVESTE_Z:
        raise SystemExit(f'--z {args.z} er under {LAVESTE_Z} m: avbryter')

    steg = [
        ('til 3 cm over arket', args.z + args.over, False),
        ('til 5 mm over arket', args.z + 0.005, True),
        ('ned til beroring', args.z, True),
        ('opp igjen', args.z + args.over, False),
    ]

    rclpy.init()
    node = Node('mas248_touch_test')
    log = node.get_logger()
    try:
        # Planlegg hele kjeden for noe kjores: hvert steg starter der forrige slapp.
        planer = []
        start = None                                   # forste steg starter fra robotens stilling na
        for navn, hoyde, _ in steg:
            mal = make_pose(args.x, args.y, hoyde)     # pennen rett ned, tcp x langs world x
            svar = cartesian(node, [mal], start_joint_positions=start, jump_threshold=0.0)
            if svar.fraction < 0.999:
                log.error(f'{navn}: bare {svar.fraction * 100:.1f} % kunne planlegges som rett linje. Avbryter.')
                return
            planer.append(svar.solution)
            start = siste_leddvinkler(svar.solution)
            log.info(f'{navn}: z = {hoyde:.3f} m planlagt ({len(svar.solution.joint_trajectory.points)} punkter)')

        log.info(f'Alle {len(steg)} steg planlagt for punktet ({args.x:.3f}, {args.y:.3f}), beroring ved z = {args.z:.3f} m')
        if not args.execute:
            log.warn('BARE PLANLEGGING: roboten beveget seg ikke. Bruk --execute for a kjore.')
            return

        for (navn, hoyde, sporr), plan in zip(steg, planer):
            if sporr:
                input(f'\n>>> Trykk Enter for a ga {navn} (z = {hoyde:.3f} m), Ctrl+C for a avbryte: ')
            log.info(f'Kjorer: {navn} (z = {hoyde:.3f} m) ...')
            execute_trajectory(node, plan)
        log.info('Ferdig: pennen er oppe igjen. Les av z i tf2_echo world tcp mens pennen er nede neste gang.')
    except KeyboardInterrupt:
        log.warn('Avbrutt med Ctrl+C: ingen flere steg kjores.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
