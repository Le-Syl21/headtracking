# Détection de tête — passage du face tracking au head tracking

> Plan de design pour remplacer la détection de **visage** (qui décroche quand
> le joueur baisse la tête sur le playfield) par une détection de **tête comme
> forme**, robuste sous tous les angles — crâne / calvitie vus de dessus inclus.
> Statut : **DESIGN validé**. Entraînement du modèle différé (post-vacances,
> géré par Sylvain). Aucune ligne de code encore posée.

---

## 1. Le problème

La caméra est **fixe sur le meuble, orientée vers le joueur** ; elle voit le
**U inversé** (lockbar + sidebars) en bas de l'image et la tête au-dessus.

Le tracker actuel localise la tête via un détecteur de **visage** (Ultraface
RFB-320 / YuNet, crate `face`). Or c'est un détecteur **frontal** : dès que le
joueur se penche pour jouer, la caméra ne voit plus que le **sommet du crâne**
(pas de visage, pas de traits) et le tracking décroche — précisément en position
de jeu.

## 2. La décision

Détecter la **tête comme forme**, pas via des traits de visage. Un vrai
**détecteur de tête** trouve le « blob » tête que le visage soit visible ou non
(de face, de profil, penché, de dos, chauve).

### Alternatives écartées (et pourquoi)

- **Détecteur de visage** (l'actuel) : la largeur du visage **rétrécit** quand
  on baisse la tête (raccourci perspectif) → mauvaise localisation *et* mauvaise
  profondeur (cf. §4).
- **MoveNet / pose corps entier** (piste initiale de Claude Code) : ses ancres
  sont **faciales** (nez, yeux, oreilles). Crâne vers la caméra → ces points
  passent en confiance quasi nulle. Échoue pile sur le cas visé. En prime, son
  décodeur utilise `GatherND`/`ArgMax`, ops « à risque » dans tract
  (cf. `crates/u-onnx/examples/movenet_probe.rs`).
- **Détecteur « person » CrowdHuman off-the-shelf** (ex. `yakhyo/yolov8-crowdhuman`) :
  détecte le **corps entier**, pas la tête. La bbox = épaules/torse → largeur
  inutilisable pour la profondeur (§4). Donne une position approximative au
  mieux.

## 3. Le modèle : YOLOv11 « head » entraîné maison

On entraîne un détecteur de tête **YOLOv11** avec le pipeline existant
(`output/u-seg/.venv`, ultralytics — le même qui a produit `crates/u-onnx/models/u.onnx`).

Trois raisons qui s'empilent :
1. **Vraies têtes, tous angles** (base CrowdHuman *head* + nos propres frames)
   → robuste tête baissée.
2. **Format YOLOv11 identique à `u.onnx`** → le décodage tract d'u-onnx
   (8400 anchors, NMS) est **réutilisé tel quel**, juste sans les canaux de masque.
3. Entraîné sur **la tête du joueur, sa caméra, son angle** : la largeur
   apparente devient **consistante et calibrée**, ce qui rend le calcul de
   profondeur (§4) précis sans prior générique bancal.

Dataset : base CrowdHuman (annotations *head*) pour la généralité **+** quelques
centaines de captures **penché-sur-le-pf** (crâne vers la cam) pour verrouiller
le cas de jeu réel. Idéalement, on peut même entraîner **U + head en un seul
modèle** (la caméra voit déjà les deux).

## 4. La profondeur : largeur de tête × échelle lockbar

La webcam n'a pas de capteur de profondeur ; le **U la fournit géométriquement** :

- Les **sidebars parallèles** + la lockbar forment une structure 3D de
  dimensions connues → **PnP** → pose de la caméra sur le fronton et **axes**
  (repère meuble) recalculés dynamiquement.
- La **largeur réelle de la lockbar** (donnée par VPX) calibre l'**échelle**,
  concrètement la **focale en pixels** de la caméra.
- Modèle sténopé : `distance_tête = focale_px × largeur_réelle_tête / largeur_bbox_px`.
  → **la largeur de la bbox tête donne la profondeur.**

Seul inconnu : la largeur réelle de la tête du joueur → **calibration one-shot**
(joueur face caméra à distance connue, une fois) ou prior ~15 cm.

C'est **pourquoi il faut un détecteur de tête** et pas de visage/person : seule
la **largeur du blob tête est stable quel que soit l'angle**. Elle bouge un peu
en *yaw* (tête tournée), bien moins qu'un visage — le lissage `filter/`
(one-euro) absorbe le résidu.

## 5. La couture Rust (indépendante du modèle)

À coder une seule fois, quel que soit le détecteur :

```
HeadAnchor { cx, cy, width, height, confidence }   // coords image
        │
        ├── Kinect  : head_from_region(cx, cy, w, h, depth, intrinsics)   // profondeur capteur
        └── Webcam  : profondeur = focale_px · largeur_réelle / width      // échelle lockbar
        → Pose { position_mm: [x, y, z], … }
```

- Refactor : `tracker/face_depth.rs::head_from_face_depth(&FaceDetection, …)`
  → `head_from_region(cx, cy, w, h, …)` (le corps — médiane + déprojection IR —
  et ses tests ne bougent pas ; seule l'entrée change).
- Nouveau crate `head` calqué sur `u-onnx` (tract, `include_bytes!` du modèle,
  décodage YOLOv11) → produit des `HeadAnchor`.
- `tracker/{kinect_v1,kinect_v2,webcam}` : `face::Detector` → `head::Detector`.
- `face` (Ultraface) devient retirable une fois les trackers basculés.

## 6. Prochaines étapes (post-vacances)

1. Entraîner le YOLOv11 head (dataset + config ultralytics). ← Sylvain
2. Poser la couture Rust `HeadAnchor` + `head_from_region` (non cassant, testé).
3. Crate `head` + câblage des trackers ; profondeur webcam via lockbar.
4. Calibration one-shot de la largeur de tête + lissage.
