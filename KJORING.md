# Kjøring: simulering og ekte robot

To oppskrifter for å tegne gresskaret med UR5e:

- **A. Simulering (mock)**: ingen robot, kan kjøres hjemme (WSL) eller på lab-PC-en.
- **B. Ekte robot med bevegelse**: lab-PC-en koblet til UR5e-en.

Alle kommandoer kjøres fra `~/mas248_ws` på grenen `feature/real-workcell-geometry`.

## Verdier fra labben (2026-11-09)

| Hva | Verdi | Hvor i koden |
|---|---|---|
| `world` | bordflaten under sokkelen | `ur5e_workcell.urdf.xacro` |
| Robotfoten over bordet | 0,304 m (sjekket med linjal mot ROS) | `pedestal_height` |
| Arkets midte | x 0,50 m, y 0,06 m, z 0,0 m | `paper_x`, `paper_y`, `ARK_SENTER` |
| Arket | A4 med langsiden på tvers foran roboten (langs y) | `paper_size_x/y` |
| TCP (pennespiss) fra flensen `tool0` | 0,10 m langs y (langs pennen), 0,065 m langs z | `tcp_along_pen`, `tcp_z` |
| Robot-IP / lab-PC-IP | 192.168.1.101 / 192.168.1.102 | kommandoene under |
| Kalibrering | `~/mas248_ws/calibration/ur5e_calibration.yaml` | ligger ikke i Git |

Målene på verktøyholderen, pennen, bordet og sokkelplatene er plassholdere lest av et bilde.

## 0. Oppdater og bygg (begge oppskriftene)

```bash
cd ~/mas248_ws
git fetch origin
git checkout feature/real-workcell-geometry
git reset --hard origin/feature/real-workcell-geometry   # fjerner lokale endringer i filer i Git
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

L9-pakkene (`mas248_ur5e_moveit_config`, `mas248_ur5e_demos`) og `calibration/` ligger ikke i Git og må finnes i workspacet fra før.

---

## A. Simulering (mock)

To terminaler. Kjør `cd ~/mas248_ws && source install/setup.bash` i hver av dem først.

**T1: mock-kontrollere, MoveIt + RViz og `pen_trace`**

```bash
ros2 launch mas248_project tegn_sim.launch.py
```

I WSL: legg til `LIBGL_ALWAYS_SOFTWARE=1` foran kommandoen.

**I RViz (én gang, lagre med Ctrl+S):**

- *Add → By topic → `/mas248/pen_trace` → MarkerArray* (viser det pennen har tegnet)
- *Use Cartesian Path* skal **ikke** være krysset av

**T2: lag punktene og tegn**

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg --width-m 0.18 \
  --center-x 0.50 --center-y 0.06 --center-z 0.0 --tcp-offset-mm 30 \
  --max-contours 16 --max-points 300 --rviz &
ros2 run mas248_project run_picture --execute
```

Forventet: alle 16 strekene får `transport rett linje`, og gresskaret tegnes på arket i RViz.

**Stopp:** `kill %1` i T2 (markørene), Ctrl+C i T1. Start T1 på nytt for å tømme den gamle tegningen.

---

## B. Ekte robot med bevegelse

### Sikkerhet først

- Én person har hånden på **nødstoppen** hele tiden mens roboten kan bevege seg.
- Ingen innenfor rekkevidden til armen.
- Fartsglideren på pendanten på **10 %**.
- Ikke kjør simuleringen (A) samtidig med driveren.
- Kommer noe uventet: nødstopp eller **Stop/Pause** på pendanten. Da mister ROS kontrollen.

### 1. Det ekte arket

Midten av arket 50 cm ut fra robotaksen og 6 cm til venstre (sett fra roboten mot arket), langsiden på tvers.

### 2. Nettverk og pendant

```bash
ping -c 3 192.168.1.101
```

På pendanten:

- *Installation → URCaps → External Control*: Host IP **192.168.1.102**, port **50002**
- Et program med én node: *External Control* (ikke start det ennå)

### 3. T1: driveren med bevegelse slått på

```bash
ros2 launch mas248_ur5e_real driver.launch.py robot_ip:=192.168.1.101 \
  calibration:=$HOME/mas248_ws/calibration/ur5e_calibration.yaml \
  acknowledge_hardware:=yes enable_execution:=true
```

Trykk **Play** på External Control-programmet på pendanten.

### 4. T2: sjekk

```bash
ros2 control list_controllers | grep scaled
timeout 3 ros2 run tf2_ros tf2_echo world tcp | grep -m1 Translation
```

- `scaled_joint_trajectory_controller` skal være **active**.
- `tf2_echo` skal vise pennespissen der den faktisk er (mål høyden over bordet med linjal for å sjekke z).
- Meldingen `Invalid frame ID "world"` i første linje og `BrokenPipeError` til slutt er ufarlige.

### 5. T2: MoveIt med bevegelse slått på

```bash
ros2 launch mas248_ur5e_real moveit_real.launch.py robot_ip:=192.168.1.101 \
  calibration:=$HOME/mas248_ws/calibration/ur5e_calibration.yaml enable_execution:=true
```

### 6. Til `home` fra RViz

I *Planning*-fanen:

- Start State `<current>`, Goal State `home`
- *Use Cartesian Path* ikke krysset av
- *Velocity Scaling* og *Accel. Scaling* **0.05**
- **Plan** (ikke Plan & Execute). Se på banen: rolig opp, ikke mot bordet, ikke rundt sokkelen.
- Bare hvis banen ser rolig ut: **Execute**, med hånden på nødstoppen.

### 7. Berøringstest: roboten setter pennen på arket ved z = 0

`touch_test` går i rette linjer til 3 cm over midten av arket, så 5 mm over, så ned til z = 0 og opp igjen.
Den venter på Enter før de to stegene nedover. Ctrl+C avbryter.

```bash
ros2 run mas248_project touch_test               # bare planlegging
ros2 run mas248_project touch_test --execute     # beveger roboten
```

Mens pennen er nede (før siste Enter), i en annen terminal:

```bash
timeout 3 ros2 run tf2_ros tf2_echo world tcp | grep -m1 Translation
```

Pennen skal så vidt treffe arket. Bøyer den seg eller stopper roboten (protective stop): bordet ligger høyere enn
modellen. Prøv `--z 0.002` og juster `pedestal_height` i xacroen (senk den med avviket), bygg og start på nytt.
Treffer den ikke arket: øk `pedestal_height` tilsvarende.

### 8. T3: tegn

Først bare planlegging:

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg --width-m 0.18 \
  --center-x 0.50 --center-y 0.06 --center-z 0.0 --tcp-offset-mm 30 \
  --max-contours 16 --max-points 300 --rviz &
ros2 run mas248_project run_picture
```

Forventet: `Forhandssjekk OK: alle 16 strek er planlagt og godkjent` og `transport rett linje` på alle.
Så, med hånden på nødstoppen:

```bash
ros2 run mas248_project run_picture --execute
```

`run_picture` går først til `home`, tegner de 16 strekene (penn opp 30 mm mellom strekene) og går tilbake til `home`.

### 9. Stopp

Stop på pendanten, `kill %1` i T3, Ctrl+C i T2 og T1.

---

## Feilsøking

| Feil | Årsak / løsning |
|---|---|
| `launch configuration '...' does not exist` | Gammel `ur_controllers.yaml` med `$(var ...)` i en kommentar. Hent nyeste kode (steg 0). |
| `scaled_joint_trajectory_controller` er `inactive` | Driveren er startet uten `enable_execution:=true`. |
| Ingenting skjer når du trykker Play | Feil Host IP eller port i External Control (skal være 192.168.1.102 / 50002). |
| Advarsel om at kalibreringen ikke stemmer | Feil `ur5e_calibration.yaml` for denne roboten. Ikke kjør videre. |
| Armen svinger rundt eller tar omvei i RViz | Planlagt fra RViz med OMPL. Plan på nytt og se på banen før Execute; `run_picture` bruker rette linjer. |
| Strek med `OMPL (reserve)` i loggen | Den rette linja kunne ikke planlegges. Se på banen i RViz før du kjører. |
