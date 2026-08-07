# Minicab / small cabinet notes

**🇬🇧 [English](#-english) · 🇫🇷 [Français](#-français)**

---

## 🇬🇧 English

Head tracking on a minicab works, but the short player-to-camera distance
is the constraint that decides everything. Here is what to know before
picking a camera and a mounting spot.

### Minimum tracking distance per camera

| Camera | Minimum distance | Nature of the limit |
|---|---|---|
| **Kinect v2** | **~0.5 m** | Hard: below that, the time-of-flight sensor returns no depth at all (spec range 0.5–4.5 m) |
| **Kinect v1** | **~0.8 m** | Hard: lower bound of its structured-light depth (0.8–4 m in standard mode) |
| **Webcam** | **~0.4–0.5 m** | Soft: head **and shoulders** must fit in the frame — depth is triangulated from shoulder width; a wide-angle lens lowers the limit |

Also worth knowing, whatever the camera:

- **There is no software distance limit** — the pipeline only filters
  invalid sensor readings; the floors in the table above are the sensors'
  own hardware limits. The webcam's shoulder-width triangulation works as
  close as the framing allows.
- **The pose model is a body detector, not a face detector.** When the
  camera is so close that it only sees a face without shoulders, tracking
  degrades and eventually drops. Whatever the sensor: the player's bust
  must be in the frame.

### Recommendation

- Player at **60 cm or more** from the camera → the second-hand
  **Kinect v2** stays the best pick (IR tracking in the dark + measured
  depth).
- Player **closer than ~60 cm** → a **wide-angle webcam** is actually the
  better option: no hard depth floor, and minicabs usually live in lit
  rooms where the Kinect's own IR illumination matters less.
- **Kinect v1 is risky on a minicab**: its 0.8 m floor is often more than
  the whole player-to-backbox distance.

### Mounting & configuration tips

- **Mount high, aim down**: putting the camera at the top of the backbox
  (or above it) and angling it down increases the *oblique* distance to
  the player's head — every centimetre counts against the sensor floors
  above.
- **Measure and declare your real lockbar width** (VPX → F12 → Cabinet
  Settings): a minicab's lockbar is much narrower than the 610 mm
  widebody default, and auto-calibration is anchored on that value.
- Declare your **screen inclination** the same way as on a full-size cab.
- The auto-calibration anchor (lockbar + side rails) works at any cabinet
  scale — the camera being close even makes the anchor bigger and easier
  to detect. Contribute a capture (🎁 button in the demo): minicabs are
  exactly the kind of geometry the model has never seen.

---

## 🇫🇷 Français

Le head tracking sur un minicab fonctionne, mais la faible distance
joueur-caméra est LA contrainte qui décide de tout. Voici ce qu'il faut
savoir avant de choisir la caméra et son emplacement.

### Distance minimale de tracking par caméra

| Caméra | Distance minimale | Nature de la limite |
|---|---|---|
| **Kinect v2** | **~0,5 m** | Dure : en dessous, le capteur time-of-flight ne renvoie plus aucune profondeur (plage spec 0,5–4,5 m) |
| **Kinect v1** | **~0,8 m** | Dure : limite basse de son depth par lumière structurée (0,8–4 m en mode standard) |
| **Webcam** | **~0,4–0,5 m** | Souple : la tête **et les épaules** doivent tenir dans le cadre — la profondeur est triangulée sur la largeur d'épaules ; un objectif grand-angle abaisse la limite |

À savoir aussi, quelle que soit la caméra :

- **Il n'y a aucune limite de distance logicielle** — le pipeline ne
  filtre que les lectures capteur invalides ; les planchers du tableau
  ci-dessus sont les limites matérielles des capteurs eux-mêmes. La
  triangulation par largeur d'épaules de la webcam fonctionne d'aussi
  près que le cadrage le permet.
- **Le modèle de pose détecte un corps, pas un visage.** Quand la caméra
  est si proche qu'elle ne voit qu'un visage sans épaules, le tracking se
  dégrade puis décroche. Quel que soit le capteur : le buste du joueur
  doit être dans le cadre.

### Recommandation

- Joueur à **60 cm ou plus** de la caméra → la **Kinect v2** d'occasion
  reste le meilleur choix (tracking IR dans le noir + profondeur
  mesurée).
- Joueur à **moins de ~60 cm** → une **webcam grand-angle** est en fait
  la meilleure option : pas de plancher de profondeur dur, et un minicab
  vit souvent dans une pièce éclairée où l'éclairage IR de la Kinect
  compte moins.
- **La Kinect v1 est risquée sur un minicab** : son plancher de 0,8 m
  dépasse souvent la distance joueur-backbox totale.

### Astuces de montage et de configuration

- **Monter haut, viser bas** : placer la caméra en haut du backbox (ou
  au-dessus) et l'incliner vers le bas augmente la distance *oblique*
  jusqu'à la tête du joueur — chaque centimètre compte face aux planchers
  capteur ci-dessus.
- **Mesurer et déclarer la vraie largeur de lockbar** (VPX → F12 →
  Cabinet Settings) : celle d'un minicab est bien plus étroite que les
  610 mm par défaut d'un widebody, et l'auto-calibration est ancrée sur
  cette valeur.
- Déclarer l'**inclinaison de l'écran**, comme sur un cab pleine taille.
- L'ancre d'auto-calibration (lockbar + rails) fonctionne à toutes les
  échelles de caisse — une caméra proche rend même l'ancre plus grosse et
  plus facile à détecter. Envoyez un relevé (bouton 🎁 de la démo) : les
  minicabs sont exactement le genre de géométrie que le modèle n'a jamais
  vue.
