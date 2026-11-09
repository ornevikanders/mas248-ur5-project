import argparse
import copy
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Hjelpefunksjonene kommer fra kurspakken mas248_ur5e_demos, som ikke ligger i Git.
from scripts.common import JOINTS, move_group, cartesian, execute_trajectory

# Samme leddvinkler som group_state "home" i SRDF-en og startverdiene i mock-URDF-en
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# Storste tillatte endring i ett ledd mellom to punkter i en bane (rad).
# Erstatter MoveIts relative jump_threshold, som vi har slatt av.
MAKS_LEDDSTEG = 0.3

# Hvor mange ganger en strek planlegges pa nytt for vi gir opp.
# OMPL er tilfeldig, sa et nytt forsok gir en ny transport og en ny tegnebane.
ANTALL_FORSOK = 3

# Malt i labben (samme verdier som i ur5e_workcell.urdf.xacro):
# arkets senter i world (x, y) og tegneflaten (bordplata = toppen av sokkelen).
# Fortegnet (positiv x og y) er bekreftet i labben 2026-11-09, se paper_x/paper_y i xacro-fila.
ARK_SENTER = (0.43, 0.06)
TEGNEFLATE_Z = 0.31

# Halv utstrekning rundt arksenteret (m). Dekker et A4-ark (0.297 x 0.210) uansett retning.
ARK_HALV = 0.15

# Tillatt omrade for TCP-mal i world (meter): (min, maks) per akse.
# Avledet fra arket, slik at feil --center-x/-y/-z til image_to_targets blir stoppet.
ARBEIDSOMRADE = {
    'x': (ARK_SENTER[0] - ARK_HALV, ARK_SENTER[0] + ARK_HALV),
    'y': (ARK_SENTER[1] - ARK_HALV, ARK_SENTER[1] + ARK_HALV),
    'z': (TEGNEFLATE_Z - 0.005, TEGNEFLATE_Z + 0.20),   # 5 mm toleranse under flaten
}

# Minste hoyde for penn oppe over tegneflaten (m). Malene i JSON-fila er penn oppe-hoyden,
# sa --tcp-offset-mm 0 ville gitt transport helt nede pa flaten.
MIN_PENN_OPPE = 0.01


def les_targets(filsti):
    """Leser targets-JSON fra image_to_targets.
    Returnerer (strek_liste, flate_z): strek_liste er en liste av strek,
    hver strek er en liste av Pose i world-rammen (z = penn oppe-hoyde fra fila).
    flate_z er hoyden til tegneflaten (penn nede)."""
    tekst = Path(filsti).read_text()                # les: filinnhold som tekst
    data = json.loads(tekst)                        # parse: tekst -> dict
    flate_z = data['frame_to_world']['xyz'][2]      # tegneflatens z i world

    strek_liste = []
    for strek in data['strokes']:                   # en strek om gangen
        poser = []                                  # ny liste for denne streken
        for pose_dict in strek['poses']:            # en pose-dict om gangen
            p = Pose()                              # nytt tomt Pose-objekt
            p.position.x = pose_dict['x']           # posisjon fra fila
            p.position.y = pose_dict['y']
            p.position.z = pose_dict['z']           # penn oppe (flate_z + tcp-offset)
            p.orientation.x = pose_dict['qx']       # verktoyorientering (kvaternion)
            p.orientation.y = pose_dict['qy']
            p.orientation.z = pose_dict['qz']
            p.orientation.w = pose_dict['qw']
            poser.append(p)                         # posen inn i strekens liste
        strek_liste.append(poser)                   # streken inn i hovedlista
    return strek_liste, flate_z


def med_ny_z(pose, z):
    """Kopi av pose med ny hoyde. Originalen endres ikke."""
    ny = copy.deepcopy(pose)
    ny.position.z = float(z)
    return ny


def siste_leddvinkler(bane):
    """Leddvinklene i siste punkt av en RobotTrajectory, i JOINTS-rekkefolge."""
    jt = bane.joint_trajectory
    if not jt.points:
        raise RuntimeError('Tom bane: kan ikke finne sluttstilling')
    vinkel_for = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [vinkel_for[navn] for navn in JOINTS]


def planlegg_strek(node, poser, flate_z, start_ledd):
    """Planlegger (uten a kjore) en hel strek fra start_ledd.
    Returnerer (transport, tegning, andel, slutt_ledd)."""
    # 1) Transport: fri bevegelse (OMPL) til penn oppe over forste punkt
    transport = move_group(node, pose=poser[0], execute=False,
                           start_joint_positions=start_ledd).planned_trajectory
    etter_transport = siste_leddvinkler(transport)

    # 2) Tegning: rett ned, langs streken pa flaten, rett opp
    veipunkter = [med_ny_z(p, flate_z) for p in poser]   # alle punktene pa flaten (penn nede)
    veipunkter.append(poser[-1])                          # til slutt: penn oppe over siste punkt
    svar = cartesian(node, veipunkter, start_joint_positions=etter_transport,
                     jump_threshold=0.0)

    return transport, svar.solution, svar.fraction, siste_leddvinkler(svar.solution)


def storste_leddsteg(bane):
    """Storste endring i ETT ledd mellom to pafolgende punkter i banen (rad)."""
    punkter = bane.joint_trajectory.points
    storst = 0.0
    for forrige, neste in zip(punkter, punkter[1:]):
        for a, b in zip(forrige.positions, neste.positions):
            storst = max(storst, abs(b - a))
    return storst


def planlegg_alle(node, strek_liste, flate_z, start_ledd, log):
    """Del 5: planlegg alle strekene i en kjede og sjekk hver av dem.
    Hver strek far inntil ANTALL_FORSOK forsok.
    Returnerer [(transport, tegning), ...] hvis ALT er godkjent, ellers None."""
    planer = []
    ledd = start_ledd                                    # kjeden starter i start_ledd
    for i, poser in enumerate(strek_liste):
        resultat = None                                  # ingen godkjent plan for denne streken enna
        for forsok in range(1, ANTALL_FORSOK + 1):
            hvor = f'Strek {i}, forsok {forsok}/{ANTALL_FORSOK}'
            try:
                transport, tegning, andel, slutt = planlegg_strek(node, poser, flate_z, ledd)
            except RuntimeError as feil:
                log.warn(f'{hvor}: planlegging feilet: {feil}')
                continue                                 # prov igjen

            steg_t = storste_leddsteg(transport)
            steg_d = storste_leddsteg(tegning)
            log.info(f'{hvor}: andel {andel:.3f}, storste leddsteg '
                     f'transport {steg_t:.3f} rad / tegning {steg_d:.3f} rad')

            if andel < 0.999:
                log.warn(f'{hvor}: bare {andel * 100:.1f} % av banen ble planlagt')
                continue
            if max(steg_t, steg_d) > MAKS_LEDDSTEG:
                log.warn(f'{hvor}: hopp i leddrommet (> {MAKS_LEDDSTEG} rad mellom to punkter)')
                continue

            resultat = (transport, tegning, slutt)       # godkjent
            break                                        # ferdig med denne streken

        if resultat is None:
            log.error(f'Strek {i}: ingen godkjent plan etter {ANTALL_FORSOK} forsok')
            return None

        transport, tegning, slutt = resultat
        planer.append((transport, tegning))              # ta vare pa planene
        ledd = slutt                                     # neste strek starter der denne slapp
    return planer


def utenfor_arbeidsomradet(strek_liste, flate_z):
    """Sjekker alle malpunktene (penn oppe) og tegneflaten (penn nede) mot ARBEIDSOMRADE.
    Returnerer en liste med beskrivelser av det som er utenfor. Tom liste = alt OK."""
    feil = []

    lav, hoy = ARBEIDSOMRADE['z']
    if not lav <= flate_z <= hoy:
        feil.append(f'tegneflaten: z = {flate_z:.3f} m utenfor [{lav}, {hoy}]')

    for i, poser in enumerate(strek_liste):
        for j, p in enumerate(poser):
            # 1e-9: unngaar at avrunding i flyttall avviser noyaktig minstehoyden
            if p.position.z < flate_z + MIN_PENN_OPPE - 1e-9:
                feil.append(f'strek {i}, punkt {j}: penn oppe z = {p.position.z:.3f} m er under '
                            f'{flate_z + MIN_PENN_OPPE:.3f} m (sett --tcp-offset-mm hoyere)')
            for akse, verdi in (('x', p.position.x), ('y', p.position.y), ('z', p.position.z)):
                lav, hoy = ARBEIDSOMRADE[akse]
                if not lav <= verdi <= hoy:
                    feil.append(f'strek {i}, punkt {j}: {akse} = {verdi:.3f} m utenfor [{lav}, {hoy}]')
    return feil


def main():
    ap = argparse.ArgumentParser(
        description='MAS248: tegn bildet fra targets-JSON med UR5e (mock). Standard: bare planlegg.')
    ap.add_argument('--json', default='/tmp/mas248_targets.json',
                    help='Sti til targets-fila fra image_to_targets')
    ap.add_argument('--execute', action='store_true',
                    help='Kjor bevegelsen (KUN mock-kontroller)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('mas248_run_picture')
    log = node.get_logger()
    try:
        strek_liste, flate_z = les_targets(args.json)
        antall_poser = sum(len(s) for s in strek_liste)
        log.info(f'Leste {len(strek_liste)} strek / {antall_poser} poser fra {args.json}')
        log.info(f'Tegneflate z = {flate_z:.3f} m')

        # --- Sjekk at alle mal ligger i arbeidsomradet ---
        feil = utenfor_arbeidsomradet(strek_liste, flate_z)
        if feil:
            log.error(f'{len(feil)} punkt(er) utenfor arbeidsomradet {ARBEIDSOMRADE}. De forste:')
            for linje in feil[:5]:
                log.error('  ' + linje)
            log.error('Ingenting blir planlagt eller kjort.')
            return
        log.info('Alle mal ligger innenfor arbeidsomradet')

        # --- Del 5: planlegg og sjekk ALLE strekene for noe kjores ---
        log.info('Del 5: forhandssjekk av alle strekene (kan ta litt tid) ...')
        planer = planlegg_alle(node, strek_liste, flate_z, HOME, log)
        if planer is None:
            log.error('Forhandssjekk FEILET: ingenting blir kjort.')
            return
        log.info(f'Forhandssjekk OK: alle {len(planer)} strek er planlagt og godkjent')

        # --- Del 6: kjor planene, bare med --execute ---
        if not args.execute:
            log.warn('BARE PLANLEGGING: roboten beveget seg ikke. Bruk --execute for mock-bevegelse.')
            return

        log.info('Del 6: flytter til home (alle planene starter der) ...')
        move_group(node, joints=HOME, execute=True)

        for i, (transport, tegning) in enumerate(planer):
            log.info(f'Strek {i}/{len(planer) - 1}: transport ...')
            execute_trajectory(node, transport)
            log.info(f'Strek {i}/{len(planer) - 1}: tegner ...')
            execute_trajectory(node, tegning)

        log.info(f'Ferdig: alle {len(planer)} strek er tegnet. Tilbake til home ...')
        move_group(node, joints=HOME, execute=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
