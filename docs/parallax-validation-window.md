# Fenêtre parallaxe de validation (fish-tank VR à la Johnny Lee)

> Spec pour une scène 3D temps réel pilotée par la pose de tête, intégrée à
> `headtracking-demo`. Objectif : **valider le tracking en dehors de VPX**.
> Statut : IMPLÉMENTÉ (M1–M3 — scène 3D, projection off-axis, sources œil
> Live/Souris/Auto-orbit). Roadmap : P2 — outillage.

---

## 1. Pourquoi

Aujourd'hui la démo affiche le flux caméra + l'OBB lockbar + la math 3D +
les deltas `→ VPX  ΔX ΔY ΔZ`. C'est du **2D** : on voit des chiffres, pas si
le tracking « rend bien ». Pour juger latence, bruit, signe des axes et gain,
il faut un retour visuel immédiat où **bouger la tête bouge le rendu**.

C'est exactement la démo de Johnny Lee 2007 (*Head Tracking for Desktop VR
Displays using the Wii Remote*) : l'écran devient une **fenêtre** sur une boîte
3D ; quand la tête bouge, on « regarde autour » des bords et les objets proches
défilent plus vite que les lointains (parallaxe de mouvement). Si l'illusion
tient, le tracking est bon ; si ça saccade / part de travers / a un lag, ça se
voit en une seconde — sans avoir à lancer VPX sur le pincab.

**Ce que ça valide concrètement :**
- signe et amplitude (gain) de chaque axe X/Y/Z avant de les câbler dans le mapping VPX ;
- latence ressentie et efficacité du filtre 1€ (jitter résiduel) ;
- robustesse de la profondeur (Z dérivé de la largeur lockbar) — c'est l'axe le plus bruité.

Non-objectif : ce n'est pas un jeu ni un rendu joli. C'est un banc de test.

---

## 2. Contenu de la scène

Scène classique « shadow box » (boîte à ombres), la plus lisible pour la parallaxe :

- une **boîte filaire** (wireframe, `GL_LINES`) qui s'enfonce derrière le plan-écran : sol, plafond, murs gauche/droite, fond. C'est le cadre de la « fenêtre ».
- **3 couches de cibles** (grilles de points `GL_POINTS`, ~5×5) à différentes profondeurs derrière la vitre, p.ex. `z = -150 / -400 / -800 mm`, une couleur par couche (proche = chaud, lointain = froid). Le décalage différentiel entre couches = la preuve visuelle de la parallaxe.
- un **réticule fixe** au centre de l'écran (espace-écran, ne bouge jamais) : repère pour percevoir le glissement off-axis.

« cible ou autre » → on part sur la grille de cibles. Variantes possibles plus
tard (objet unique qui tourne, pièce texturée) mais les cibles maximisent la
lisibilité de la parallaxe pour zéro effort de modélisation.

---

## 3. Le cœur : projection off-axis (frustum asymétrique)

C'est *la* subtilité. Une perspective normale a son apex centré ; ici l'apex
suit l'œil, qui se déplace devant un écran **fixe**. D'où un frustum
asymétrique recalculé chaque frame. Formulation de Kooima 2008 (*Generalized
Perspective Projection*).

Données : l'écran virtuel est un rectangle fixe défini par 3 coins
`pa` (bas-gauche), `pb` (bas-droite), `pc` (haut-gauche) ; l'œil est en `pe`.

```text
vr = normalize(pb - pa)          // axe droite de l'écran
vu = normalize(pc - pa)          // axe haut
vn = normalize(cross(vr, vu))    // normale, vers le spectateur

va = pa - pe                     // œil → coins
vb = pb - pe
vc = pc - pe

d  = -dot(vn, va)                // distance œil→plan écran (> 0)
n  = near, f = far               // plans du frustum

l  = dot(vr, va) * n / d         // extents au near plane
r  = dot(vr, vb) * n / d
b  = dot(vu, va) * n / d
t  = dot(vu, vc) * n / d

P  = frustum(l, r, b, t, n, f)   // glFrustum standard

// aligner le monde sur les axes écran
M  = mat4_from_rows(vr, vu, vn)  // rotation
T  = translate(-pe)              // ramener l'œil à l'origine

mvp = P * M * T * model
```

Quand `pe` se décale à droite, `l`/`r` deviennent asymétriques → on « regarde
autour » du bord droit de la fenêtre. C'est tout le truc.

**Matrices** : via `nalgebra` (déjà dans le workspace, 0.34) ou hand-rollé en
`[f32; 16]` passé à `glUniformMatrix4fv`. Pas de nouvelle dép lourde (pas de
`glam` juste pour ça, sauf si on le juge plus simple — à trancher à l'implé).

---

## 4. Source de l'œil (`pe`)

Sélecteur dans la toolbar — 3 modes, parce qu'**on doit pouvoir tester sans
caméra** (machine de dev, cf. la démo qui tourne en local sans device) :

| Mode        | Source de `pe`                                                                 | Usage |
|-------------|--------------------------------------------------------------------------------|-------|
| **Live**    | pose tête réelle = deltas lockbar `(ΔX, ΔY, ΔZ)` (mêmes que `output_line`)      | validation finale, caméra branchée |
| **Souris**  | position souris sur le panneau central → X/Y de l'œil ; molette → Z             | itération GUI sur la machine de dev |
| **Auto-orbit** | oscillation sinusoïdale lente programmée                                     | démo « mains libres », capture vidéo |

Mapping Live → œil : `pe = (ΔX·gain, ±ΔY·gain, Dview ± ΔZ·gain)` avec `Dview`
distance de visionnage nominale (~600 mm) et `gain` réglable. Les signes par
axe sont **auto par défaut** (valeurs sensées hardcodées) ; on expose des
toggles *invert X/Y/Z* + un slider *gain* comme commodités de debug du banc —
**pas** une étape de calibration manuelle du produit (l'auto reste la règle côté
plugin). Le but de ces réglages : trouver visuellement les bons signes, puis
les reporter en dur dans `camera/mapping.rs`.

---

## 5. Intégration technique dans la démo

Contraintes du code actuel :
- le `glow::Context` et le `egui_glow::Painter` vivent dans `DemoShell`
  (`tools/headtracking-demo/src/main.rs:226+`), **pas** dans `App`.
- la fenêtre passe par egui-rotate (270° pincab) : tout ce qui doit tourner
  avec l'écran doit être un **primitive egui** (egui-rotate transforme l'input
  et les primitives tessellées, pas un rendu GL brut dans le framebuffer).

### Approche retenue : rendu hors-écran (FBO) → texture egui

1. `ParallaxScene` : struct détenant les ressources GL (FBO, texture couleur,
   renderbuffer depth, programme shader, VAO/VBO cibles + boîte). Créée
   **paresseusement dans `DemoShell`** (seul détenteur du `gl`) au premier
   activation.
2. Chaque frame, si le mode parallaxe est ON : `DemoShell::redraw` rend la scène
   **dans le FBO** (depth test activé, projection off-axis depuis l'œil fourni
   par `App`), **avant** `painter.paint_primitives`.
3. La texture couleur du FBO est enregistrée une fois via
   `painter.register_native_texture(tex)` → `egui::TextureId`, repassée à `App`.
4. `App::ui` affiche cette `TextureId` comme image dans le panneau central
   quand le mode est ON.

**Pourquoi cette approche** (vs un `egui_glow::CallbackFn` qui dessine en
ligne) :
- egui-rotate **tourne l'image gratuitement** (c'est un primitive egui normal) → marche sur l'écran pincab tourné sans effort ;
- isolation d'état : on rend dans notre FBO puis on rebind le framebuffer 0 ; le paint egui n'est jamais corrompu ;
- compose proprement avec le layout existant (toolbar + input/output lines restent par-dessus).
- coût : un peu de plomberie `DemoShell ⇄ App` (un flag + l'œil dans un sens, une `TextureId` dans l'autre). Acceptable.

Le FBO se redimensionne quand la taille du panneau central change (en px
physiques).

GL : cibler GLSL `#version 330` (ou `300 es`) selon ce que négocie glutin ;
shaders minimaux (vertex applique `mvp`, fragment sort une couleur ;
`gl_PointSize` pour les cibles).

---

## 6. UI

- **Toolbar (ligne boutons)** : ajout d'un toggle `🪟 Parallax`.
- Quand ON : le panneau central montre la scène 3D (au choix : remplace le flux
  caméra, ou côte-à-côte caméra | parallaxe — à trancher, côte-à-côte est plus
  utile pour corréler). Les lignes input/output restent affichées.
- **Sélecteur source œil** : Live / Souris / Auto-orbit.
- **HUD** (overlay egui par-dessus la 3D) : `pe` courant en mm, mode, gain, FPS.
- Réglages debug repliés : gain, invert X/Y/Z, `Dview`.

---

## 7. Jalons (incrémentaux, vérifiables un par un)

| Jalon | Contenu | Critère de validation |
|-------|---------|-----------------------|
| **M0** | Plomberie : `ParallaxScene` créée dans `DemoShell`, FBO + texture + depth, `register_native_texture` → `TextureId` à `App`, toggle toolbar, panneau central affiche la texture (clear color uni). | la texture s'affiche, **et tourne** avec egui-rotate (test pincab/270°). |
| **M1** | Shaders + grille de cibles + boîte filaire, œil **fixe**, projection perspective centrée. | la scène 3D se dessine, depth correct. |
| **M2** | Projection **off-axis** depuis `pe` ; œil piloté par **Souris** + **Auto-orbit** (sans caméra). | l'illusion parallaxe tient sur la machine de dev, les couches glissent à des vitesses différentes. |
| **M3** | Source **Live** = deltas lockbar ; gain + invert ; HUD. | validation caméra réelle sur le pincab ; on en tire les bons signes pour `camera/mapping.rs`. |

Chaque jalon = un point d'arrêt où on regarde et on décide avant de continuer.

---

## 8. Risques / points ouverts

- **Signe des axes caméra→monde** : c'est *le* truc qu'on cherche à révéler ; au début ce sera probablement à l'envers, d'où les toggles invert. Attendu, pas un bug.
- **Z bruité** : la profondeur vient de la largeur lockbar en px ; le 1€ aide mais le Z restera l'axe le moins stable. La fenêtre parallaxe le rendra visible — c'est voulu (c'est un diagnostic).
- **Côte-à-côte vs empilé** : tranché à l'implé — **empilé sous le flux caméra** (panneau bas redimensionnable). Le côte-à-côte mangeait trop de largeur.
- **Perf** : scène triviale (quelques dizaines de points + lignes), négligeable devant le décodage caméra. RAS.

---

## 9. Hors scope (pour rester un banc de test)

- pas de textures / éclairage / modèles importés ;
- pas de stéréo / VR ;
- pas de persistance des réglages (c'est du debug jetable) ;
- aucun lien avec le plugin VPX : 100 % côté `headtracking-demo`.
