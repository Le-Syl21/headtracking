# `anchor` — cabinet auto-calibration toolkit

**🇬🇧 [English](#-english) · 🇫🇷 [Français](#-français)**

---

## 🇬🇧 English

This folder holds the home-grown pipeline that turns a handful of **hand-drawn
lines on cabinet photos** into an ONNX model that locates the pinball cabinet's
reference frame (lockbar + side rails) in any image. That frame is what lets
headtracking recover the camera's pose **without any manual calibration** — the
star feature of the project.

```
annotator.html  ──►  anchor-lines.json  ──►  lines_to_yolo.py  ──►  YOLO-pose dataset
                                                                          │
                                                              train.sh (yolo11n-pose)
                                                                          │
                                                                   models/anchor_rgb.onnx
                                                                          │
                                              Rust decoder: keypoints → lines → vanishing
                                              points + homography → focal + camera pose
```

### 1. Draw the lines — `annotator.html`

Open the file in any browser (no server, no install). Drop a folder of cabinet
photos in, and for each image draw **4 lines**:

| line | what to trace |
|------|---------------|
| `sideleft` / `sideright` | the two cabinet **side rails** |
| `lockbar_player` | the lockbar edge on the **player** side |
| `lockbar_screen` | the lockbar edge on the **playfield/screen** side |

Each line is placed with a **pivot + rotation**: click a first point on the
edge, the line pivots to follow the cursor, click again to lock the angle — the
two endpoints are snapped to the image border for a maximal, precise baseline.
Click the **2 extremities** of each edge; the further apart, the more precise the
line. Everything auto-saves to the browser; **Export JSON** gives you
`anchor-lines.json`.

Convention: everything is in **camera / image space** (top-left = image
top-left, never "player left"). The camera↔player mirror is absorbed once,
downstream.

### 2. Lines → dataset — `lines_to_yolo.py`

A neural net cannot learn the *extrapolated* line endpoints (there is no pixel
where an extended line hits the ceiling). So the converter derives **6 keypoints
that sit on real features**, all from your 4 lines:

```
0 player_left   1 player_right   2 screen_right
3 screen_left   4 bottom_left    5 bottom_right
```

The 4 corners are line intersections (real bar corners); the 2 bottom points are
where each rail meets the last image row (the visible start of the rail). It also
writes a fixed **full-width × bottom-third bounding box** (one cabinet per image).

```bash
python lines_to_yolo.py \
  --json anchor-lines.json \
  --images /path/to/photos \
  --out dataset
```

### 3. Train + export — `train.sh`

```bash
./train.sh dataset            # epochs=200 imgsz=1280 batch=16
# -> models/anchor_rgb.onnx
```

Uses `yolo11n-pose`, geometric augmentation off, and **batch ≥ 16**.

### 4. Benchmark BEFORE shipping — `eval_dir`

Never embed a retrained model without measuring it against the capture
corpus first (an Aug 2026 retrain silently collapsed on two of the three
backends — the benchmark is what caught it):

```bash
cargo run -p anchor --example eval_dir -- /path/to/captures
# per-backend detection rate + mean score; swap models/anchor.onnx and
# rebuild to A/B different weights
```

Retraining rule: always train on the **cumulative** dataset (old + new
images). Fine-tuning on new data alone makes a model this small forget
everything it knew.

The annotations themselves live in `annotations/anchor-lines.json`, which is
**tracked** — everything under `dataset_*/` is a build artefact and is not.
Hand-drawn lines are the expensive part; a model can always be retrained from
them, so they must survive a `git clean`.

Depth and IR frames are 16-bit. Train on the auto-levelled `_depthview` /
`_irview` renderings, never on `_depth` / `_ir` directly: they share the same
pixel grid, so the annotations transfer unchanged, but `cv2.imread` collapses
16-bit to 8-bit by division — a depth map in millimetres arrives with a mean
of 5.8/255, i.e. a black image, and nothing warns you. At *inference* the Rust
decoder converts properly, so `_ir` scores like `_irview`; `_depth` still
scores well below `_depthview`.

### What we learned the hard way

Three non-obvious traps, all now baked into the scripts:

1. **Border/extrapolated keypoints are unlearnable.** Points where a line hits
   the frame edge have no visual evidence → the model guesses (200–1000 px
   error). Only points on real features (corners, visible rail) are learnable.
2. **Small batch kills BatchNorm.** With `batch < 16` the pose head never
   converges (`pose_loss` stays flat, ~11) — even overfitting a single image
   fails. `batch ≥ 16` fixes it instantly. A real dataset (dozens of images)
   makes this automatic.
3. **Never trust the lockbar *thickness* (70 mm).** It is thin and
   near-fronto-parallel → focal estimation from it is wildly ill-conditioned.
   The metric reference is the **610 mm width between the two sidebars**.

### Metric / geometry notes for the decoder

- Reference width = **610 mm** between the two sidebars (`LOCKBAR_WIDTH_MM`).
- Distance cam↔bar `= focal × 610 / pixel_width` — validated to **±0–3 %**
  against tape-measured ground truth on Kinect v1/v2 and a webcam.
- Focal: Kinect uses its **factory colour focal**; the webcam focal comes from
  the sidebar/lockbar geometry (`src/calibration/autocalib.rs`,
  `calibrate_homography`). Vanishing-point-only focal (`calibrate_from_lockbar`)
  is degenerate for a centred camera — use the homography.

### Status

The embedded production model (`crates/anchor/models/anchor.onnx`) is the
**Aug 2026 extended-dataset retrain**, benchmarked over the capture corpus
before shipping. It still only knows a handful of cabinets — contributions
of cabinet captures/annotated photos are the single biggest thing that
moves this forward, see the main README.

---

## 🇫🇷 Français

Ce dossier contient le pipeline maison qui transforme quelques **lignes
tracées à la main sur des photos de cab** en un modèle ONNX capable de
localiser le repère du flipper (lockbar + rails latéraux) dans n'importe
quelle image. C'est ce repère qui permet à headtracking de retrouver la
pose de la caméra **sans aucune calibration manuelle** — la fonctionnalité
phare du projet.

```
annotator.html  ──►  anchor-lines.json  ──►  lines_to_yolo.py  ──►  dataset YOLO-pose
                                                                          │
                                                              train.sh (yolo11n-pose)
                                                                          │
                                                                   models/anchor_rgb.onnx
                                                                          │
                                             décodeur Rust : keypoints → lignes → points
                                             de fuite + homographie → focale + pose caméra
```

### 1. Tracer les lignes — `annotator.html`

Ouvrez le fichier dans n'importe quel navigateur (pas de serveur, rien à
installer). Glissez-y un dossier de photos de cab, puis tracez **4 lignes**
par image :

| ligne | quoi tracer |
|-------|-------------|
| `sideleft` / `sideright` | les deux **rails latéraux** de la caisse |
| `lockbar_player` | l'arête de la lockbar côté **joueur** |
| `lockbar_screen` | l'arête de la lockbar côté **plateau/écran** |

Chaque ligne se pose en **pivot + rotation** : un premier clic sur l'arête,
la ligne pivote en suivant le curseur, un second clic fige l'angle — les
deux extrémités sont automatiquement étendues jusqu'aux bords de l'image
pour une base de visée longue et précise. Cliquez aux **2 extrémités** de
chaque arête : plus elles sont éloignées, plus la ligne est précise. Tout
est sauvegardé automatiquement dans le navigateur ; **Export JSON** produit
`anchor-lines.json`.

Convention : tout est exprimé dans le **repère caméra / image** (haut-gauche
= haut-gauche de l'image, jamais « gauche du joueur »). Le miroir
caméra↔joueur est absorbé une seule fois, plus loin dans la chaîne.

### 2. Des lignes au dataset — `lines_to_yolo.py`

Un réseau de neurones ne peut pas apprendre les extrémités *extrapolées*
des lignes (aucun pixel ne matérialise une ligne prolongée jusqu'au
plafond). Le convertisseur dérive donc **6 points-clés posés sur de vrais
éléments visibles**, tous calculés à partir de vos 4 lignes :

```
0 player_left   1 player_right   2 screen_right
3 screen_left   4 bottom_left    5 bottom_right
```

Les 4 coins sont des intersections de lignes (les vrais coins de la
barre) ; les 2 points du bas sont l'endroit où chaque rail coupe la
dernière ligne de l'image (le début visible du rail). Le script écrit
aussi une boîte englobante fixe **pleine largeur × tiers bas** (un seul
cab par image).

```bash
python lines_to_yolo.py \
  --json anchor-lines.json \
  --images /chemin/vers/photos \
  --out dataset
```

### 3. Entraîner + exporter — `train.sh`

```bash
./train.sh dataset            # epochs=200 imgsz=1280 batch=16
# -> models/anchor_rgb.onnx
```

Utilise `yolo11n-pose`, augmentation géométrique désactivée, et
**batch ≥ 16**.

### 4. Mesurer AVANT d'embarquer — `eval_dir`

Ne jamais embarquer un modèle réentraîné sans l'avoir mesuré sur le corpus
de captures (un réentraînement d'août 2026 s'était silencieusement
effondré sur deux des trois capteurs — c'est le banc qui l'a révélé) :

```bash
cargo run -p anchor --example eval_dir -- /chemin/vers/captures
# taux de détection + score moyen par capteur ; remplacez
# models/anchor.onnx et recompilez pour comparer des poids en A/B
```

Règle de réentraînement : toujours entraîner sur le dataset **cumulatif**
(anciennes + nouvelles images). Un fine-tuning sur les seules nouvelles
données fait tout oublier à un modèle aussi petit.

### Ce qu'on a appris à la dure

Trois pièges non évidents, désormais intégrés aux scripts :

1. **Les points-clés extrapolés ou en bord d'image sont inapprenables.**
   Un point où une ligne coupe le bord du cadre n'a aucun indice visuel →
   le modèle devine (200 à 1000 px d'erreur). Seuls les points posés sur
   de vrais éléments (coins, rail visible) s'apprennent.
2. **Un petit batch tue le BatchNorm.** Avec `batch < 16`, la tête de pose
   ne converge jamais (`pose_loss` reste plate, ~11) — même le surapprentissage
   d'une seule image échoue. `batch ≥ 16` règle le problème instantanément.
   Un vrai dataset (des dizaines d'images) le rend automatique.
3. **Ne jamais se fier à l'*épaisseur* de la lockbar (70 mm).** Fine et
   presque parallèle au capteur, elle rend l'estimation de focale
   massivement mal conditionnée. La référence métrique, c'est la
   **largeur de 610 mm entre les deux rails**.

### Notes métriques / géométrie pour le décodeur

- Largeur de référence = **610 mm** entre les deux rails
  (`LOCKBAR_WIDTH_MM`).
- Distance caméra↔barre `= focale × 610 / largeur_pixels` — validée à
  **±0-3 %** contre des mesures au mètre-ruban sur Kinect v1/v2 et webcam.
- Focale : les Kinect utilisent leur **focale couleur usine** ; celle de la
  webcam vient de la géométrie rails/lockbar (`src/calibration/autocalib.rs`,
  `calibrate_homography`). La focale par points de fuite seuls
  (`calibrate_from_lockbar`) dégénère pour une caméra centrée — utiliser
  l'homographie.

### État

Le modèle de production embarqué (`crates/anchor/models/anchor.onnx`) est
le **réentraînement d'août 2026 sur dataset étendu**, mesuré sur le corpus
de captures avant embarquement. Il ne connaît encore qu'une poignée de
cabs — les contributions de relevés et de photos annotées restent LE
levier n°1 du projet, voir le README principal.
