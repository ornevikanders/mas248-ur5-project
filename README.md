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
- ❌ Ikke commit utdelte forelesningsfiler (for eksempel `mas248_ur5e_demos` og `mas248_ur5e_moveit_config`).
- ❌ Ikke commit ROS-bags (`.db3`) eller store videofiler. Legg dem i felles disk og lenk til dem.
- ✅ Kjør `git pull` før du begynner å jobbe.
- ✅ Kjør `colcon build` før du pusher.
- ✅ Kommenter koden: forklar *hvorfor*, ikke bare *hva*.
- ✅ Oppdater README når du legger til noe andre må vite om (nye launch-filer, avhengigheter osv.).

---

## Tegne bilde med `run_picture`

`run_picture` er vårt eget skript i `mas248_project`. Det krever at kurspakkene `mas248_ur5e_demos` og `mas248_ur5e_moveit_config` ligger lokalt i `src/`. De skal ikke i Git, så legg dem i `.git/info/exclude`:

```bash
printf '%s\n' 'src/mas248_ur5e_demos/' 'src/mas248_ur5e_moveit_config/' >> .git/info/exclude
```

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg --tcp-offset-mm 30 \
  --center-x 0.50 --center-y 0.06 --center-z 0.0
ros2 run mas248_project run_picture             # bare planlegging, roboten står stille
ros2 run mas248_project run_picture --execute   # planlegg og kjør (KUN mock)
```

Skriptet sjekker arbeidsområdet og at høyden med pennen oppe er minst 10 mm over tegneflaten. Deretter planlegger det alle strekene før noe kjøres, og godkjenner bare baner som er minst 99,9 % planlagt med høyst 0,3 rad endring i ett ledd mellom to punkter. **Ikke bruk det på ekte robot.** `ARBEIDSOMRADE` er avledet fra arkets målte plassering (senter `(0.50, 0.06)` m i `world`, tegneflate `z = 0` m: sokkelen og arket står på samme bord, robotfoten er 0,31 m over). Fortegnet (positiv x og y) er bekreftet i labben, se kommentaren ved `paper_x` i `ur5e_workcell.urdf.xacro`.

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
- [ ] **Milepæl 2:** Arbeidscellemodell (URDF/Xacro, sokkel, verktøy, TCP) vist i RViz2
- [ ] **Milepæl 3:** MoveIt 2-planlegging med kollisjonsobjekter i simulering
- [ ] **Milepæl 4:** Kjøring på ekte UR5 via UR-driveren
- [ ] Rapport, demovideo og eksperimentresultater
