# MAS248 UR5 Project

ROS 2 Humble-workspace for prosjektet i MAS248 Robotteknologi (UiA, høst 2026).
Målet er en arbeidscelle med UR5 (sokkel, verktøy og TCP), kollisjonsfri bevegelsesplanlegging med MoveIt 2, testing i simulering og til slutt kjøring på den ekte UR5-en i labben.

**Gruppe:** Anders Ørnevik, Dag Isak Stein, Henrik Ljones-Sundfjord
**Emneansvarlig:** Ilya Tyapin

---

## Innhold

1. [Krav](#krav)
2. [Førstegangsoppsett](#førstegangsoppsett)
3. [Bygg og kjør](#bygg-og-kjør)
4. [Mappestruktur](#mappestruktur)
5. [Git-arbeidsflyt](#git-arbeidsflyt)
6. [Regler for repoet](#regler-for-repoet)
7. [Feilsøking](#feilsøking)
8. [Fremdrift](#fremdrift)
9. [Lecture 9: MoveIt-planlegging og tegning av bilde](#lecture-9-moveit-planlegging-og-tegning-av-bilde)

---

## Krav

| Komponent | Versjon |
|---|---|
| Ubuntu | 22.04 LTS (WSL2 for simulering, native/dual boot for ekte robot) |
| ROS 2 | Humble |
| MoveIt | MoveIt 2 (Humble) |
| UR-pakker | `ros-humble-ur` (driver, description, moveit_config) |

---

## Førstegangsoppsett

### 1. Installer ROS 2 Humble

Følg den offisielle guiden: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Legg til i `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Installer kurspakkene

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions git -y
sudo rosdep init      # feil her betyr bare at den allerede er initialisert
rosdep update

sudo apt install ros-humble-rqt ros-humble-rqt-graph \
                 ros-humble-moveit \
                 ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-ur ros-humble-xacro \
                 ros-humble-joint-state-publisher-gui -y
```

### 3. Sett opp Git og SSH-nøkkel (én gang per maskin)

```bash
git config --global user.name "Fornavn Etternavn"
git config --global user.email "din-github-epost@eksempel.no"
git config --global init.defaultBranch main

ssh-keygen -t ed25519 -C "din-github-epost@eksempel.no"   # trykk Enter på alt
cat ~/.ssh/id_ed25519.pub
```

Kopier linjen og legg den inn på GitHub under **Settings → SSH and GPG keys → New SSH key**. Test med:

```bash
ssh -T git@github.com
```

### 4. Klon repoet som workspace

```bash
cd ~
git clone git@github.com:ornevikanders/mas248-ur5-project.git mas248_ws
cd mas248_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Legg til i `~/.bashrc`, slik at workspacet alltid er sourcet:

```bash
source ~/mas248_ws/install/setup.bash
```

> Har du andre ROS-workspaces (for eksempel `~/ros2_ws`), skal bare ett av dem sources i `~/.bashrc` om gangen.

---

## Bygg og kjør

```bash
cd ~/mas248_ws
colcon build --symlink-install
source install/setup.bash
```

Med `--symlink-install` trenger du ikke bygge på nytt når du endrer eksisterende Python-filer. Du **må** bygge på nytt når du:

- lager nye filer (noder, launch-filer, URDF/Xacro, config)
- endrer `setup.py`, `package.xml` eller `CMakeLists.txt`

Test at UR5-modellen vises i RViz:

```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

---

## Mappestruktur

```
mas248_ws/                      # roten av repoet
├── README.md
├── .gitignore
├── src/
│   └── mas248_project/         # vår ROS 2-pakke (ament_python)
│       ├── mas248_project/     # Python-noder
│       ├── launch/             # launch-filer        (kommer)
│       ├── config/             # YAML-parametere     (kommer)
│       ├── urdf/               # arbeidscelle-Xacro  (kommer)
│       ├── meshes/             # STL/DAE-modeller    (kommer)
│       ├── package.xml
│       └── setup.py
├── build/                      # generert, ikke i Git
├── install/                    # generert, ikke i Git
└── log/                        # generert, ikke i Git
```

Nye pakker (for eksempel en egen MoveIt-konfigurasjon) legges også i `src/`.

---

## Git-arbeidsflyt

### Prinsipp

- **`main` skal alltid bygge og fungere.** Ingen jobber direkte på `main`.
- Alt arbeid gjøres på en egen **gren** (branch), som flettes inn i `main` via en **Pull Request** (PR).
- En annen på gruppa ser over PR-en før den merges.

### Navn på grener

```
<type>/<kort-beskrivelse>
```

| Type | Brukes til | Eksempel |
|---|---|---|
| `feature/` | ny funksjonalitet | `feature/workcell-xacro` |
| `fix/` | feilretting | `fix/tcp-offset` |
| `docs/` | README, kommentarer, rapportfigurer | `docs/readme-setup` |
| `test/` | eksperimenter og målinger | `test/planning-time` |

### Steg for steg

**1. Start alltid fra oppdatert `main`:**

```bash
cd ~/mas248_ws
git checkout main
git pull
```

**2. Lag en ny gren for oppgaven din:**

```bash
git checkout -b feature/workcell-xacro
```

**3. Jobb, bygg og test.** Sjekk at det bygger før du committer:

```bash
colcon build --symlink-install
```

**4. Se hva du har endret, og commit:**

```bash
git status                       # hvilke filer er endret?
git add src/mas248_project/urdf/workcell.urdf.xacro
git commit -m "Add pedestal and tool to workcell xacro"
```

Commit ofte, gjerne flere ganger i løpet av en økt.

**5. Push grenen til GitHub:**

```bash
git push -u origin feature/workcell-xacro   # første gang
git push                                    # senere på samme gren
```

**6. Lag en Pull Request** på GitHub: trykk **Compare & pull request**, skriv kort hva som er gjort, og legg til en av de andre som reviewer.

**7. Etter merge:** gå tilbake til `main` og slett den gamle grenen lokalt:

```bash
git checkout main
git pull
git branch -d feature/workcell-xacro
```

### Hente andres endringer inn i grenen din

Hvis `main` har endret seg mens du jobber:

```bash
git checkout main
git pull
git checkout feature/min-gren
git merge main
```

### Commit-meldinger

Kort, på engelsk, i imperativ form (som om du gir en ordre):

- ✅ `Add TCP frame to tool link`
- ✅ `Fix collision box size for table`
- ❌ `fikset ting`
- ❌ `update`

### Merge-konflikter

Får du `CONFLICT` etter `git merge` eller `git pull`:

1. Åpne filen. Konflikten er markert slik:
   ```
   <<<<<<< HEAD
   din versjon
   =======
   den andre versjonen
   >>>>>>> main
   ```
2. Behold det som skal være med, og slett markørlinjene.
3. Fullfør:
   ```bash
   git add <filen>
   git commit
   ```

Usikker? Spør i gruppa før du løser konflikten.

---

## Regler for repoet

- ❌ Aldri commit `build/`, `install/` eller `log/` (de ligger i `.gitignore`).
- ❌ Aldri `git push --force` til `main`.
- ❌ Ikke commit ROS-bags (`.db3`) eller store videofiler. Legg dem i felles disk og lenk til dem.
- ✅ Kjør `git pull` før du begynner å jobbe.
- ✅ Kjør `colcon build` før du pusher.
- ✅ Kommenter koden: forklar *hvorfor*, ikke bare *hva*.
- ✅ Oppdater README når du legger til noe andre må vite om (nye launch-filer, avhengigheter osv.).

---

## Feilsøking

| Problem | Løsning |
|---|---|
| `Package 'mas248_project' not found` | Du har glemt å source: `source ~/mas248_ws/install/setup.bash` |
| Gammel kode kjøres etter endring | Bygg på nytt med `colcon build --symlink-install` og source igjen |
| RViz viser bare svart skjerm (WSL) | `echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc` og åpne ny terminal |
| `Permission denied (publickey)` ved push | SSH-nøkkelen er ikke lagt til på GitHub, se oppsett steg 3 |
| `Repository not found` | Du er ikke lagt til som collaborator, eller URL-en er feil |
| Lim inn virker ikke i terminalen | Bruk Ctrl+Shift+V eller høyreklikk |

---

## Fremdrift

- [x] **Milepæl 1:** ROS 2-miljø, workspace og prosjektpakke
- [x] **Milepæl 2:** Arbeidscellemodell (URDF/Xacro, sokkel, verktøy, TCP) vist i RViz2
- [ ] **Milepæl 3:** MoveIt 2-planlegging med kollisjonsobjekter i simulering
- [ ] **Milepæl 4:** Kjøring på ekte UR5 via UR-driveren
- [ ] Rapport, demovideo og eksperimentresultater

---

## Lecture 9: MoveIt-planlegging og tegning av bilde

Kjeden er: bilde → OpenCV-konturer → TCP-mål (`/tmp/mas248_targets.json`) → MoveIt 2 Cartesian-baner → kjøring på mock-kontrollere i RViz2. **Alt her er kun simulering (mock hardware).** Ikke bruk dette mot den ekte UR5-en.

### Pakker

| Pakke | Innhold |
|---|---|
| `mas248_ur5e` | Arbeidscellen: UR5e, sokkel (`pedestal_link`), bord, spindel (`spindle_body`) og `tcp` |
| `mas248_ur5e_moveit_config` | MoveIt-oppsett: SRDF, kinematikk, kontrollere og launch-filer |
| `mas248_ur5e_demos` | Skript: bildekonvertering, RViz-markører, Del A-demoer og `run_picture` |

TF-treet skal være én kjede: `world → pedestal_link → base_link → … → tool0 → spindle_body → tcp`.

### Bygg

Fra roten av workspacet (`~/mas248_ws`, eller der du har klonet repoet):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Hver ny terminal må sources: `source /opt/ros/humble/setup.bash && source install/setup.bash`.

### Terminaler som må kjøre

Start dem i denne rekkefølgen og la dem stå:

| Terminal | Kommando | Hva den gjør |
|---|---|---|
| 1 | `ros2 launch mas248_ur5e_moveit_config ros2_control.launch.py` | Mock-UR5e og kontrollere (`/joint_states`) |
| 2 | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch mas248_ur5e_moveit_config moveit.launch.py` | MoveIt (`move_group`) og RViz2 |
| 3 | `ros2 run mas248_ur5e_demos image_to_targets ... --rviz` (se under) | Lager målene og viser dem i RViz |
| 4 | `ros2 run mas248_ur5e_demos ...` | Del A-demoer eller `run_picture` |

Sjekk at alt svarer (skal gi fem `[OK]`):

```bash
ros2 run mas248_ur5e_demos check_stack
```

Ikke start `demo.launch.py` samtidig med terminal 1 og 2.

### Del A: to mål og en hindring

```bash
ros2 run mas248_ur5e_demos add_cube                     # legg boks i planning scene
ros2 run mas248_ur5e_demos move_two_targets             # bare planlegging
ros2 run mas248_ur5e_demos move_two_targets --execute   # planlegg og kjør (mock)
ros2 run mas248_ur5e_demos move_two_targets --mode tcp  # posemål i stedet for leddmål
ros2 run mas248_ur5e_demos add_cube --remove            # fjern boksen igjen
```

Det samme kan gjøres i MotionPlanning-panelet i RViz (Scene-fanen for boks, Planning-fanen for mål). **Fjern boksen før Del B**, ellers står den i veien for tegningen.

### Del B: tegn bildet

**1. Lag TCP-mål fra bildet (terminal 3):**

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg \
  --width-m 0.18 --center-x 0.55 --center-y 0 --center-z 0.46 \
  --tcp-offset-mm 30 --max-contours 4 --max-points 40 --rviz --axis-every 4
```

Dette skriver `/tmp/mas248_targets.json` og publiserer markører til RViz til du trykker Ctrl+C. At terminalen står stille er normalt. Har JSON-filen allerede blitt laget, kan markørene vises alene med `ros2 run mas248_ur5e_demos show_targets_rviz --axis-every 4`.

| Parameter | Betydning |
|---|---|
| `--width-m` | Bredden på hele bildet i meter |
| `--center-x/y/z` | Bildets sentrum i `world` (meter) |
| `--tcp-offset-mm` | TCP-høyde over tegneplanet. Sett til ca. 2 for at pennen skal ned til planet |
| `--max-contours` | Antall konturer (kanter i bildet, ikke pennestrøk) |
| `--max-points` | Maks antall punkter per kontur |

**2. Se målene i RViz:**

- Fixed Frame: `world`. MarkerArray på `/mas248/drawing_targets` er med i RViz-oppsettet. Mangler den: Add → By topic → `/mas248/drawing_targets` → MarkerArray.
- Tegningen er liten og ligger rundt (0,55, 0, 0,46). Zoom inn, eller sett Views → Focal Point til `0.55, 0, 0.46` og Distance til `0.5`.
- Under MarkerArray → Namespaces kan du slå av og på `MAS248_contour_paths`, `MAS248_image_bounds`, `MAS248_TCP_orientation_Z` osv. De blå Z-pilene skal peke **ned**.
- Views → Type: TopDownOrtho gjør det lett å se om bildet er speilvendt.

**3. Se bildebehandlingsstegene (WSL2):**

```bash
explorer.exe "$(wslpath -w ~/MAS248_Image_Stages/index.html)"
```

Eller lim inn `\\wsl.localhost\Ubuntu-22.04\home\<bruker>\MAS248_Image_Stages\index.html` i nettleseren i Windows.

**4. Kjør tegningen (terminal 4):**

```bash
ros2 run mas248_ur5e_demos run_picture
```

| Flagg | Standard | Betydning |
|---|---|---|
| `--targets` | `/tmp/mas248_targets.json` | Målfilen |
| `--max-strokes` | `0` (alle) | Tegn bare de N første strøkene |
| `--plan-only` | av | Planlegg bare flyttingen til start, roboten beveger seg ikke |
| `--lift` | `0.05` | Høyde for «pen-up» (m) |
| `--max-step` | `0.005` | Interpolasjonssteg for Cartesian-banen (m) |
| `--min-fraction` | `0.99` | Laveste fraction som godtas før kjøring |
| `--max-joint-jump` | `0.3` | Største leddendring mellom to trajektoriepunkter (rad) |

Slik virker `run_picture`:

1. Løser IK for punktet over første strøk, med en albue-opp-seed og verktøyet pekende ned (samme som `home`), og flytter dit i leddrom.
2. For hvert strøk følger den rette linjer (Cartesian): flytt over startpunktet, ned, langs konturen og opp.
3. Hver bane sjekkes for fraction og leddhopp **før** den kjøres. Delvise baner kjøres aldri.

Vellykket kjøring ender med `Stroke N done, draw fraction 1.000` for hvert strøk og `Drawing finished`. Lagre denne outputen til rapporten.

### Feilsøking for Lecture 9

| Problem | Løsning |
|---|---|
| `check_stack` gir `[MISS]` | Terminal 1 eller 2 kjører ikke, eller terminalen er ikke sourcet |
| `Image 'pumpkin_outline.png' not found` | Bruk `pumpkin_outline.jpg`, eller bygg på nytt så `.png`-kopien installeres |
| `No module named scripts.show_targets_rviz` | Bygg `mas248_ur5e_demos` på nytt og source igjen |
| Ingen markører i RViz | Terminal 3 kjører ikke, feil topic, eller Fixed Frame er ikke `world` |
| `Frame [image_center] does not exist` | Terminal 3 kjører ikke (den publiserer denne framen) |
| `No collision-free IK` / lav fraction | Fjern boksen fra Del A (`add_cube --remove`), sjekk skala og senter, og prøv `--max-points` lavere |
| `joint jump ... rad` | Armen vil snu seg midt i et strøk. Flytt bildet nærmere roboten eller gjør det mindre |
| `ros2 topic ...` gir `!rclpy.ok()` | `ros2 daemon stop`, og prøv igjen |
| RViz er svart eller krasjer (WSL) | Start med `LIBGL_ALWAYS_SOFTWARE=1` |

### Kjente begrensninger

- Konturene er kanter i bildet, ikke midtlinjer for pennen. En tykk strek blir derfor tegnet to ganger.
- Mål og plassering er ikke kalibrert mot labcellen. Sokkel- og spindelmålene i URDF-en må kontrolleres mot de ekte målene.
- `show_targets_rviz.py` manglet i den utdelte pakken og er skrevet av gruppen.
