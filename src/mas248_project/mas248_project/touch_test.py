"""Beroringstest: pennen ned til tegneflaten midt pa arket, og opp igjen.

Stegene, med pennen rett ned:
  1. til --over (3 cm) over punktet
  2. til 1 cm over punktet          (venter pa Enter for)
  3. til 5 mm over punktet          (venter pa Enter for)
  4. til 3 mm over punktet          (venter pa Enter for)
  5. ned til --z (beroring, z = 0)  (venter pa Enter for)
  6. opp igjen til --over

Standard er LEDDROM (joint space): for hvert steg finnes leddvinklene med IK som starter
fra forrige stilling (sa albue og handledd ikke bytter side), og MoveIt planlegger en
bevegelse i leddrommet dit. Med --cartesian brukes rette linjer i stedet.

Standard: bare planlegging. Med --execute kjores stegene, og skriptet venter pa Enter
for hvert steg nedover etter det forste. Ctrl+C avbryter. Ha hand pa nodstoppen.

Kjor:  ros2 run mas248_project touch_test              (bare planlegg)
       ros2 run mas248_project touch_test --execute    (beveg roboten)
       ros2 run mas248_project touch_test --execute --z 0.002   (stopp 2 mm over)
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

# Hjelpefunksjonene kommer fra kurspakken mas248_ur5e_demos, som ikke ligger i Git.
from scripts.common import JOINTS, cartesian, execute_trajectory, make_pose, move_group, wait_future

# Arkets midte i world (samme som paper_x/paper_y i xacro-fila og ARK_SENTER i run_picture)
ARK_SENTER = (0.50, 0.06)
# Laveste tillatte beroringshoyde (m): stopper skrivefeil som ville kjort pennen gjennom bordet
LAVESTE_Z = -0.003


def siste_leddvinkler(bane):
    """Leddvinklene i siste punkt av en RobotTrajectory, i JOINTS-rekkefolge."""
    jt = bane.joint_trajectory
    vinkel_for = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [vinkel_for[navn] for navn in JOINTS]


def les_leddvinkler(node, timeout=5.0):
    """Robotens leddvinkler akkurat na, fra /joint_states, i JOINTS-rekkefolge."""
    siste = {}
    sub = node.create_subscription(JointState, '/joint_states',
                                   lambda m: siste.update(zip(m.name, m.position)), 10)
    slutt = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not all(n in siste for n in JOINTS):
        if node.get_clock().now().nanoseconds > slutt:
            raise RuntimeError('Fikk ingen /joint_states: kjorer driveren eller simuleringen?')
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return [siste[n] for n in JOINTS]


def ik_naer(node, pose, seed_ledd):
    """Leddvinkler for pose (TCP i world), funnet med IK som starter fra seed_ledd.
    Losningen havner i samme albue-/handleddsstilling som seed."""
    klient = node.create_client(GetPositionIK, '/compute_ik')
    if not klient.wait_for_service(timeout_sec=20.0):
        raise RuntimeError('MoveIt /compute_ik er ikke tilgjengelig')
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'manipulator'
    req.ik_request.ik_link_name = 'tcp'
    req.ik_request.avoid_collisions = True
    req.ik_request.robot_state.joint_state.name = list(JOINTS)
    req.ik_request.robot_state.joint_state.position = [float(v) for v in seed_ledd]
    req.ik_request.pose_stamped.header.frame_id = 'world'
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.timeout.sec = 1
    svar = wait_future(node, klient.call_async(req), 30)
    if svar.error_code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f'IK fant ingen losning (feilkode {svar.error_code.val})')
    vinkel_for = dict(zip(svar.solution.joint_state.name, svar.solution.joint_state.position))
    return [vinkel_for[n] for n in JOINTS]


def main():
    ap = argparse.ArgumentParser(description='MAS248: beroringstest, pennen ned til arket og opp igjen.')
    ap.add_argument('--x', type=float, default=ARK_SENTER[0], help='x i world (m), standard arkets midte')
    ap.add_argument('--y', type=float, default=ARK_SENTER[1], help='y i world (m), standard arkets midte')
    ap.add_argument('--z', type=float, default=0.0, help='beroringshoyde i world (m), standard 0.0')
    ap.add_argument('--over', type=float, default=0.03, help='hoyde over beroringspunktet for start/slutt (m)')
    ap.add_argument('--cartesian', action='store_true', help='rette linjer i stedet for leddrom')
    ap.add_argument('--direkte', action='store_true',
                    help='rett ned og opp igjen i to bevegelser, uten mellomsteg og uten Enter')
    ap.add_argument('--pause', type=float, default=2.0,
                    help='sekunder pennen blir staende nede med --direkte')
    ap.add_argument('--execute', action='store_true', help='beveg roboten (venter pa Enter for nedstegene)')
    args = ap.parse_args()

    if args.z < LAVESTE_Z:
        raise SystemExit(f'--z {args.z} er under {LAVESTE_Z} m: avbryter')

    if args.direkte:   # en bevegelse ned og en opp, ingen mellomsteg og ingen Enter
        steg = [
            ('til 3 cm over arket', args.z + args.over, False),
            ('rett ned', args.z, False),
            ('opp igjen', args.z + args.over, False),
        ]
    else:
        steg = [
            ('til 3 cm over arket', args.z + args.over, False),
            ('til 1 cm over arket', args.z + 0.010, True),
            ('til 5 mm over arket', args.z + 0.005, True),
            ('til 3 mm over arket', args.z + 0.003, True),
            ('ned til beroring', args.z, True),
            ('opp igjen', args.z + args.over, False),
        ]
    modus = 'rett linje' if args.cartesian else 'leddrom'

    rclpy.init()
    node = Node('mas248_touch_test')
    log = node.get_logger()
    try:
        # Planlegg hele kjeden for noe kjores: hvert steg starter der forrige slapp.
        start = les_leddvinkler(node)                  # robotens stilling na
        les_start = list(start)
        albue_start = start[JOINTS.index('elbow_joint')]
        log.info('Leddvinkler na (grader): ' + ', '.join(f'{n.split("_joint")[0]} {math.degrees(v):.0f}'
                                                        for n, v in zip(JOINTS, start)))
        planer = []
        for navn, hoyde, _ in steg:
            mal = make_pose(args.x, args.y, hoyde)     # pennen rett ned, tcp x langs world x
            if args.cartesian:
                svar = cartesian(node, [mal], start_joint_positions=start, jump_threshold=0.0)
                if svar.fraction < 0.999:
                    log.error(f'{navn}: bare {svar.fraction * 100:.1f} % som rett linje. Prov uten --cartesian.')
                    return
                plan = svar.solution
                slutt = siste_leddvinkler(plan)
            else:
                # IK kan av og til hoppe til en annen losning: prov noen ganger fra samme start
                ie = JOINTS.index('elbow_joint')
                for forsok in range(1, 6):
                    slutt = ik_naer(node, mal, start)
                    if (slutt[ie] > 0) == (albue_start > 0):
                        break
                    log.warn(f'{navn}: IK-forsok {forsok} ga albue {math.degrees(slutt[ie]):.0f} grader '
                             f'(armen har {math.degrees(albue_start):.0f}), prover igjen')
                else:
                    log.error(f'{navn}: fant ingen IK-losning med albuen pa samme side som na '
                              f'({math.degrees(albue_start):.0f} grader). Leddvinkler na (grader): '
                              + ', '.join(f'{math.degrees(v):.0f}' for v in les_start)
                              + '. Flytt armen til home i RViz og prov igjen.')
                    return
                plan = move_group(node, joints=slutt, execute=False,
                                  start_joint_positions=start).planned_trajectory
            planer.append(plan)
            start = slutt
            log.info(f'{navn}: z = {hoyde:.3f} m planlagt i {modus} '
                     f'({len(plan.joint_trajectory.points)} punkter)')

        log.info(f'Alle {len(steg)} steg planlagt ({modus}) for punktet ({args.x:.3f}, {args.y:.3f}), '
                 f'beroring ved z = {args.z:.3f} m')
        if not args.execute:
            log.warn('BARE PLANLEGGING: roboten beveget seg ikke. Bruk --execute for a kjore.')
            return

        for (navn, hoyde, sporr), plan in zip(steg, planer):
            if sporr:
                input(f'\n>>> Trykk Enter for a ga {navn} (z = {hoyde:.3f} m), Ctrl+C for a avbryte: ')
            if args.cartesian:
                # Planlegg steget pa nytt fra der roboten faktisk star na (ikke der forrige plan
                # sluttet), ellers kan MoveIt avvise banen fordi starten avviker (feil -4).
                svar = cartesian(node, [make_pose(args.x, args.y, hoyde)], jump_threshold=0.0)
                if svar.fraction < 0.999:
                    log.error(f'{navn}: bare {svar.fraction * 100:.1f} % fra navaerende stilling. Avbryter.')
                    return
                plan = svar.solution
            # Fjern punkter som ikke ligger senere i tid enn punktet for (for eksempel et
            # startpunkt som er med to ganger): ellers nekter execute_trajectory hele banen.
            pkt, forrige = [], -1.0
            for p in plan.joint_trajectory.points:
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                if t > forrige:
                    pkt.append(p)
                    forrige = t
            plan.joint_trajectory.points = pkt
            varighet = pkt[-1].time_from_start.sec + pkt[-1].time_from_start.nanosec * 1e-9 if pkt else 0.0
            if len(pkt) < 2 or varighet == 0.0:
                # Roboten star allerede i malet: banen har ingen lengde og ingen tider, og
                # execute_trajectory ville nektet a kjore den. Hopp over bevegelsen.
                log.info(f'{navn}: allerede nadd (z = {hoyde:.3f} m), hopper over bevegelsen')
            else:
                log.info(f'Kjorer: {navn} (z = {hoyde:.3f} m), {len(pkt)} punkter, planlagt {varighet:.2f} s ...')
                execute_trajectory(node, plan)
            if args.direkte and navn == 'rett ned':
                log.info(f'Pennen er nede: venter {args.pause:.1f} s for den loftes')
                time.sleep(args.pause)
        log.info('Ferdig: pennen er oppe igjen.')
    except KeyboardInterrupt:
        log.warn('Avbrutt med Ctrl+C: ingen flere steg kjores.')
    except RuntimeError as feil:
        log.error(str(feil))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
