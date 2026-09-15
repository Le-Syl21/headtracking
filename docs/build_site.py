#!/usr/bin/env python3
"""Generate the GitHub Pages site (English at the root, French under fr/).

Run from anywhere: python3 docs/build_site.py
Pages are plain HTML so search engines index them without JavaScript.

Every statement on the site must be backed by the repository (README, CLAUDE.md,
docs/*.md, src/, crates/, tools/, the release workflow and the release assets):
update this file when the project changes, and leave out what has no source.

The generated files live next to the project documents in docs/ (INSTALL.md,
MINICAB.md, ...) and never replace them. docs/images/ holds the originals used
by the README; docs/img/ holds the site's own copies, produced once with
ImageMagick (`magick docs/images/setup.jpeg -resize 1280x -quality 80
docs/img/setup.webp`, and og.jpg cropped from setup.jpeg to 1200x630).
"""
import html
import json
import re
from pathlib import Path

DOCS = Path(__file__).resolve().parent
ROOT = DOCS.parent
SITE = "https://le-syl21.github.io/headtracking/"
# Google Search Console ownership check (the token belongs to the owner's Google account).
GOOGLE_VERIFICATION = "TqbXre6qrm9jaoj6tFwRRiI2vuQilAZLm6kUJA-etmo"
# Bing Webmaster Tools ownership check (the token belongs to the owner's Microsoft account).
BING_VERIFICATION = "74E158B181D9DA00960594ABC50DBA94"
REPO = "https://github.com/Le-Syl21/headtracking"
# Release assets embed the version in their names
# (`headtracking-<version>-<suffix>.<ext>`, see .github/workflows/release.yml),
# so no stable direct link exists: always send people to the releases page.
RELEASES = REPO + "/releases"
BLOB = REPO + "/blob/main/"
TREE = REPO + "/tree/main/"
DISCORD = "https://discord.gg/cFcNrt9AY"
VPX = "https://github.com/vpinball/vpinball"
ZADIG = "https://zadig.akeo.ie/"
AMAZON_LIST = "https://www.amazon.fr/hz/wishlist/ls/ZFKFTI03HIJ3"
LEBONCOIN = ("https://www.leboncoin.fr/recherche?text=%22Kinect%22%20Xbox%20One&shippable=1&price=20-max"
             "&transaction_status=search__no_value&owner_type=private&sort=price&order=asc")
EBAY = ("https://www.ebay.fr/sch/i.html?_nkw=kinect+xbox+one&_sacat=54968&_from=R40&LH_TitleDesc=0&_sop=2"
        "&Marque=Microsoft&_dcat=54968")

PAGES = ["index", "how-it-works", "download", "hardware", "contribute", "faq"]

# Release matrix of .github/workflows/release.yml (macOS Intel and Windows ARM
# are commented out there, so they are not offered here).
ASSETS = [
    ("linux-x86_64.tar.gz", {"en": ("Linux", "PC with an Intel or AMD 64-bit processor (x86_64)"),
                             "fr": ("Linux", "PC à processeur Intel ou AMD 64 bits (x86_64)")}),
    ("linux-aarch64.tar.gz", {"en": ("Linux", "ARM 64-bit processor (aarch64)"),
                              "fr": ("Linux", "Processeur ARM 64 bits (aarch64)")}),
    ("macos-aarch64.tar.gz", {"en": ("macOS", "Apple silicon Mac (M-series chip), macOS 11 or later"),
                              "fr": ("macOS", "Mac à puce Apple silicon (série M), macOS 11 ou plus récent")}),
    ("windows-x86_64.zip", {"en": ("Windows", "64-bit Windows PC (x86_64)"),
                            "fr": ("Windows", "PC Windows 64 bits (x86_64)")}),
]

UI = {
    "en": {
        "nav": {"index": "Home", "how-it-works": "How it works", "download": "Download",
                "hardware": "Cameras", "contribute": "Contribute", "faq": "FAQ"},
        "other": ("fr", "Version française", "FR"),
        "footer_src": "Source code and issues on GitHub", "footer_chat": "Discord",
        "footer_note": "headtracking is free software under the GNU GPL v3 or later. Visual Pinball X, BAM "
                       "and the Kinect are separate products by their own authors.",
        "system": "System", "file": "File name ends with",
    },
    "fr": {
        "nav": {"index": "Accueil", "how-it-works": "Fonctionnement", "download": "Télécharger",
                "hardware": "Caméras", "contribute": "Contribuer", "faq": "FAQ"},
        "other": ("en", "English version", "GB"),
        "footer_src": "Code source et tickets sur GitHub", "footer_chat": "Discord",
        "footer_note": "headtracking est un logiciel libre sous licence GNU GPL v3 ou ultérieure. Visual "
                       "Pinball X, BAM et la Kinect sont des produits distincts, développés par leurs propres auteurs.",
        "system": "Système", "file": "Nom du fichier se terminant par",
    },
}


def version():
    """Workspace version, read from Cargo.toml (used in structured data only)."""
    m = re.search(r'^version\s*=\s*"([^"]+)"', (ROOT / "Cargo.toml").read_text(encoding="utf-8"), re.M)
    return m.group(1) if m else None


def href(page, lang, from_lang):
    """Relative link from a page in `from_lang` to `page` in `lang`."""
    if lang == from_lang:
        base = ""
    else:
        base = "../" if from_lang == "fr" else "fr/"
    return (base + ("" if page == "index" else page + ".html")) or "./"


def url(page, lang):
    return SITE + ("fr/" if lang == "fr" else "") + ("" if page == "index" else page + ".html")


def fig(img, name, w, h, alt, caption, cls=""):
    c = f' class="{cls}"' if cls else ""
    return (f'<figure{c}><img src="{img}{name}" width="{w}" height="{h}" alt="{html.escape(alt)}" loading="lazy">'
            f'<figcaption>{caption}</figcaption></figure>')


def downloads(lang):
    u = UI[lang]
    rows = "".join(
        f'<tr><td><strong>{names[lang][0]}</strong><br><span class="muted">{names[lang][1]}</span></td>'
        f'<td><code>headtracking-…-{suffix}</code></td></tr>'
        for suffix, names in ASSETS)
    return (f'<div class="table"><table class="dl"><thead><tr><th>{u["system"]}</th><th>{u["file"]}</th></tr>'
            f'</thead><tbody>{rows}</tbody></table></div>')


# In-game settings registered by the plugin (src/config.rs, `register_settings`)
# with the ini defaults documented in docs/INSTALL.md.
def settings_table(lang):
    if lang == "en":
        head = ("Setting", "What it does")
        rows = [
            ("Backend", "<em>Auto</em> (the first camera found: Kinect v2, then Kinect v1, then webcam), "
                        "<em>Kinect v2</em>, <em>Kinect v1</em> or <em>Webcam</em>. Read when the game starts."),
            ("Camera", "Which webcam the Webcam backend uses; the list shows the real device names. A Kinect "
                       "backend always uses the first Kinect found. Read when the game starts."),
            ("Gain (all axes)", "How much your head movement moves the view, on all three axes. Default 1.0; "
                                "0.5 is a good start on a cabinet."),
            ("Gain trim, left/right · up/down · near/far",
             "Per-direction trims on top of the gain: lower one when that direction moves too much. Near/far "
             "is the one most often worth calming."),
            ("Smoothing", "<em>Stable</em> (the field-tested default), <em>Normal</em>, <em>Reactive</em>, or "
                          "<em>Custom</em>, which unlocks two extra sliders: responsiveness and motion catch-up."),
            ("Median Window", "Number of frames used to erase tracking spikes (1 = off, default 3). Each extra "
                              "frame adds about 17 ms of delay at 60 fps."),
            ("Invert X / Y / Z", "Flip left/right, up/down or closer/farther, for mirrored or unusual camera "
                                 "mountings."),
            ("Webcam Focal (px)", "Webcam focal length in pixels; 0 = automatic. Only needed if the webcam depth "
                                  "feels off."),
            ("Baseline Offset X / Y / Z (mm)", "Trim added to the neutral head position captured at the start of "
                                               "the game, without capturing it again."),
        ]
    else:
        head = ("Réglage", "Rôle")
        rows = [
            ("Backend", "<em>Auto</em> (la première caméra trouvée : Kinect v2, puis Kinect v1, puis webcam), "
                        "<em>Kinect v2</em>, <em>Kinect v1</em> ou <em>Webcam</em>. Lu au lancement de la partie."),
            ("Camera", "La webcam utilisée par le backend Webcam ; la liste affiche les vrais noms des appareils. "
                       "Un backend Kinect prend toujours la première Kinect trouvée. Lu au lancement de la partie."),
            ("Gain (all axes)", "L'ampleur avec laquelle vos mouvements de tête déplacent la vue, sur les trois "
                                "axes. 1.0 par défaut ; 0.5 est un bon point de départ sur un pincab."),
            ("Gain trim, left/right · up/down · near/far",
             "Corrections par direction, en plus du gain : baissez-en une quand cette direction bouge trop. "
             "L'axe proche/loin est celui qu'il vaut le plus souvent la peine de calmer."),
            ("Smoothing", "Lissage : <em>Stable</em> (le réglage par défaut éprouvé sur le terrain), "
                          "<em>Normal</em>, <em>Reactive</em>, ou <em>Custom</em>, qui débloque deux curseurs "
                          "supplémentaires : réactivité et rattrapage des mouvements rapides."),
            ("Median Window", "Nombre d'images servant à effacer les pics de suivi (1 = désactivé, 3 par défaut). "
                              "Chaque image en plus ajoute environ 17 ms de retard à 60 images par seconde."),
            ("Invert X / Y / Z", "Inverse gauche/droite, haut/bas ou proche/loin, pour une caméra montée à "
                                 "l'envers ou de façon inhabituelle."),
            ("Webcam Focal (px)", "Focale de la webcam en pixels ; 0 = automatique. Utile seulement si la "
                                  "profondeur mesurée par la webcam semble fausse."),
            ("Baseline Offset X / Y / Z (mm)", "Correction ajoutée à la position de tête neutre capturée au début "
                                               "de la partie, sans avoir à la recapturer."),
        ]
    body = "".join(f"<tr><td><strong>{html.escape(n)}</strong></td><td>{d}</td></tr>" for n, d in rows)
    return (f'<div class="table"><table><thead><tr><th>{head[0]}</th><th>{head[1]}</th></tr></thead>'
            f"<tbody>{body}</tbody></table></div>")


# Minimum tracking distances, from docs/MINICAB.md.
def cameras_table(lang):
    if lang == "en":
        head = ("Camera", "Tracks on", "Head distance", "Minimum distance")
        rows = [
            ("Kinect v2 (Xbox One)", "Infrared: the sensor lights the scene itself",
             "Measured by the depth sensor", "About 0.5 m (hardware limit)"),
            ("Kinect v1 (Xbox 360)", "Colour to find the cabinet, then infrared for tracking",
             "Measured by the depth sensor", "About 0.8 m (hardware limit)"),
            ("Webcam", "Colour image", "Worked out from your shoulder width",
             "About 0.4 to 0.5 m: head and shoulders must fit in the picture; a wide-angle lens lowers it"),
        ]
    else:
        head = ("Caméra", "Suit la tête grâce à", "Distance de la tête", "Distance minimale")
        rows = [
            ("Kinect v2 (Xbox One)", "L'infrarouge : le capteur éclaire lui-même la scène",
             "Mesurée par le capteur de profondeur", "Environ 0,5 m (limite matérielle)"),
            ("Kinect v1 (Xbox 360)", "La couleur pour repérer le meuble, puis l'infrarouge pour le suivi",
             "Mesurée par le capteur de profondeur", "Environ 0,8 m (limite matérielle)"),
            ("Webcam", "L'image couleur", "Déduite de la largeur de vos épaules",
             "Environ 0,4 à 0,5 m : la tête et les épaules doivent tenir dans l'image ; un objectif grand-angle "
             "abaisse cette limite"),
        ]
    th = "".join(f"<th>{h}</th>" for h in head)
    body = "".join("<tr>" + "".join(f"<td>{c}</td>" if i else f"<td><strong>{c}</strong></td>"
                                    for i, c in enumerate(r)) + "</tr>" for r in rows)
    return f'<div class="table"><table><thead><tr>{th}</tr></thead><tbody>{body}</tbody></table></div>'


UDEV = """sudo tee /etc/udev/rules.d/90-kinect2.rules > /dev/null <<'EOF'
# Microsoft Kinect v2 (Xbox One)
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"
EOF
sudo tee /etc/udev/rules.d/51-kinect.rules > /dev/null <<'EOF'
# Microsoft Kinect v1 (Xbox 360) and Kinect for Windows v1
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger"""

PLUGIN_TREE = """&lt;VPX_install&gt;/plugins/headtracking/
├── plugin.cfg
├── headtracking.dll          (Windows)
├── libheadtracking.so        (Linux)
└── libheadtracking.dylib     (macOS)"""


# ---------------------------------------------------------------- FAQ data
# (question, answer HTML) pairs; also emitted as FAQPage structured data.

def faq_items(lang):
    p = lambda name: href(name, lang, lang)  # noqa: E731
    if lang == "en":
        return [
            ("Is headtracking ready to use?",
             f"<p>It is a preview. The project is in early development: the full chain, from the camera to the "
             f"live point of view inside a running Visual Pinball X, works on a real Linux pincab with a Kinect v2, "
             f"a Kinect v1 and a webcam. Windows and macOS builds are published, and reports from people running "
             f"them are exactly what the project needs. Expect rough edges, and tell us about them on "
             f'<a href="{DISCORD}">Discord</a>.</p>'),
            ("Is it an alternative to BAM?",
             f"<p>Yes: headtracking is an open-source, cross-platform alternative to BAM for Visual Pinball X. It "
             f"works from a plain webcam or a Kinect v1 or v2, needs no Microsoft Kinect SDK, and finds the camera's "
             f'position by itself from the lockbar and side rails. See <a href="{p("how-it-works")}">how it '
             f"works</a>.</p>"),
            ("Can I keep BAM on the same Windows cabinet with a Kinect?",
             f'<p>No. To reach a Kinect, the driver setup shipped with headtracking replaces Microsoft\'s official '
             f"Kinect driver with a generic WinUSB one. That breaks everything built on the Microsoft Kinect SDK, "
             f"BAM head tracking included, until you restore the original driver (Device Manager, the "
             f"&ldquo;Xbox NUI&rdquo; devices: uninstall the driver and scan for hardware changes, or reinstall the "
             f"Kinect SDK/runtime). There is no automatic way back. Webcam users are not affected.</p>"),
            ("Do I need a Kinect?",
             f'<p>No. A plain webcam works: the automatic calibration turns it into a 3D head tracker. A Kinect '
             f"measures the distance of your head directly and tracks in infrared, so it keeps working in a dark "
             f'game room where a webcam struggles. See <a href="{p("hardware")}">choosing a camera</a>.</p>'),
            ("Which Visual Pinball version do I need?",
             f"<p>Visual Pinball X 10.8.1 or later: headtracking is a plugin for the plugin system of VPX 10.8.1. "
             f"It is a Visual Pinball X plugin only.</p>"),
            ("Do I have to calibrate anything?",
             f"<p>There is no calibration routine: no checkerboard, no &ldquo;look here and press a key&rdquo;. The "
             f"plugin finds the camera position from the lockbar and side rails in the image. You do need to enter "
             f"two measurements in VPX (<kbd>F12</kbd> → Cabinet Settings): your <strong>lockbar width</strong> and "
             f'your <strong>screen inclination</strong>. See <a href="{p("download")}#setup">setting up the '
             f"view</a>.</p>"),
            ("The perspective feels too strong, too weak, or drifts diagonally.",
             f"<p>Check the two measurements first. A wrong lockbar width scales every distance by the same error, "
             f"so the effect feels too strong or too weak everywhere. A missing or wrong screen inclination mixes "
             f"up/down with closer/farther, so the view drifts diagonally when you move. After that, adjust the "
             f'gain and the smoothing on the <a href="{p("download")}#settings">settings page</a> of the plugin.</p>'),
            ("How do I recenter the view?",
             f"<p>Hold the lockbar button for 2 seconds during a game: the plugin takes your current head position "
             f"as the new neutral position and shows &ldquo;Head tracking recentered&rdquo;. At the start of each "
             f"game, the first stable position is captured, so stand where you normally play when the table "
             f"loads.</p>"),
            ("Nothing happens and no head tracking notification appears when the table starts.",
             f"<p>When the plugin runs, it shows a notification with the detected camera at game start, and the VPX "
             f"log gets <code>HeadTracking</code> lines. If a camera is missing, the plugin says so on screen. No "
             f"notification at all usually means a driver or permission problem: on Linux the Kinect needs udev "
             f"rules, on Windows it needs the WinUSB driver, and on macOS VPX must be allowed to use the camera. See "
             f'<a href="{p("download")}#linux">the per-system steps</a>.</p>'),
            ("My Kinect v2 is not detected.",
             f"<p>The Kinect v2 needs its power adapter (the Kinect Adapter) and a <strong>dedicated USB 3.0 port on "
             f"the back of the motherboard</strong>. It streams colour, infrared and depth at the same time, which "
             f"is close to what a USB 3.0 link can carry: no hub, no front-panel port, no cheap extension "
             f"cable.</p>"),
            ("The plugin says the Kinect's infrared stream is already in use.",
             f"<p>Something else is holding the Kinect, for example the headtracking demo or a capture tool. Close it "
             f"and restart the table.</p>"),
            ("The Kinect v2 picture freezes in the demo after a few seconds.",
             f"<p>Start the demo with the environment variable <code>HT_DEPTH_PIPELINE=cpu</code>. If the freeze goes "
             f"away, the GPU depth processing is the cause on your machine. It is a diagnosis, not a setting to keep: "
             f'send both logs on <a href="{DISCORD}">Discord</a>.</p>'),
            ("Does it work on an Intel Mac or on Windows ARM?",
             f"<p>Not at the moment. Releases are built for Linux (x86_64 and ARM 64-bit), Windows (64-bit x86) and "
             f"macOS on Apple silicon. The Intel Mac and Windows ARM builds are switched off in the release pipeline "
             f"because of problems in the machine-learning libraries.</p>"),
            ("Does it work on a minicab?",
             f'<p>Yes, but the short distance between the player and the camera decides which camera to pick. See '
             f'<a href="{p("hardware")}#minicab">the minicab notes</a>.</p>'),
            ("Is it free?",
             f'<p>Yes. headtracking is free software under the GNU GPL v3 or later. The source code is on '
             f'<a href="{REPO}">GitHub</a>.</p>'),
            ("Where do I get help?",
             f'<p>Bug reports, help and beta testing happen on <a href="{DISCORD}">Discord</a>. You can also open an '
             f'issue on <a href="{REPO}/issues">GitHub</a>. The detailed install guide is '
             f'<a href="{BLOB}docs/INSTALL.md">docs/INSTALL.md</a>.</p>'),
        ]
    return [
        ("headtracking est-il prêt à l'emploi ?",
         f"<p>C'est une préversion (preview). Le projet est en début de développement : toute la chaîne, de la caméra "
         f"jusqu'au point de vue qui suit la tête dans Visual Pinball X, fonctionne sur un vrai pincab sous Linux avec "
         f"une Kinect v2, une Kinect v1 et une webcam. Des versions Windows et macOS sont publiées, et les retours de "
         f"ceux qui les utilisent sont exactement ce dont le projet a besoin. Attendez-vous à quelques imperfections, "
         f'et signalez-les sur <a href="{DISCORD}">Discord</a>.</p>'),
        ("Est-ce une alternative à BAM ?",
         f"<p>Oui : headtracking est une alternative libre et multiplateforme à BAM pour Visual Pinball X. Il "
         f"fonctionne avec une simple webcam ou une Kinect v1 ou v2, n'a pas besoin du SDK Kinect de Microsoft, et "
         f"trouve tout seul la position de la caméra grâce à la lockbar et aux rails latéraux. Voir "
         f'<a href="{p("how-it-works")}">le fonctionnement</a>.</p>'),
        ("Puis-je garder BAM sur le même pincab Windows avec une Kinect ?",
         f"<p>Non. Pour accéder à la Kinect, l'installation de pilote fournie avec headtracking remplace le pilote "
         f"Kinect officiel de Microsoft par un pilote WinUSB générique. Cela casse tout ce qui repose sur le SDK "
         f"Kinect de Microsoft, y compris le head tracking de BAM, jusqu'à ce que vous remettiez le pilote d'origine "
         f"(Gestionnaire de périphériques, périphériques « Xbox NUI » : désinstaller le pilote puis rechercher les "
         f"modifications matérielles, ou réinstaller le SDK/runtime Kinect). Il n'y a pas de retour arrière "
         f"automatique. Les utilisateurs de webcam ne sont pas concernés.</p>"),
        ("Faut-il une Kinect ?",
         f"<p>Non. Une simple webcam fonctionne : la calibration automatique en fait un suivi de tête en 3D. Une "
         f"Kinect mesure directement la distance de votre tête et suit en infrarouge, elle continue donc de "
         f"fonctionner dans une salle de jeu sombre où une webcam peine. Voir "
         f'<a href="{p("hardware")}">choisir une caméra</a>.</p>'),
        ("Quelle version de Visual Pinball faut-il ?",
         f"<p>Visual Pinball X 10.8.1 ou plus récent : headtracking est un plugin pour le système de plugins de "
         f"VPX 10.8.1. Il ne fonctionne qu'avec Visual Pinball X.</p>"),
        ("Faut-il calibrer quelque chose ?",
         f"<p>Il n'y a pas de procédure de calibration : pas de damier, pas de « regardez ici et appuyez sur une "
         f"touche ». Le plugin trouve la position de la caméra grâce à la lockbar et aux rails latéraux visibles "
         f"dans l'image. Il faut en revanche saisir deux mesures dans VPX (<kbd>F12</kbd> → Cabinet Settings) : la "
         f"<strong>largeur de la lockbar</strong> et l'<strong>inclinaison de l'écran</strong>. Voir "
         f'<a href="{p("download")}#setup">le réglage de la vue</a>.</p>'),
        ("La perspective est trop forte, trop faible, ou dérive en diagonale.",
         f"<p>Vérifiez d'abord ces deux mesures. Une largeur de lockbar fausse applique la même erreur d'échelle à "
         f"toutes les distances : l'effet paraît partout trop fort ou trop faible. Une inclinaison d'écran absente "
         f"ou fausse mélange le haut/bas avec le proche/loin : la vue dérive en diagonale quand vous bougez. Ensuite "
         f'seulement, ajustez le gain et le lissage dans <a href="{p("download")}#settings">les réglages</a> du '
         f"plugin.</p>"),
        ("Comment recentrer la vue ?",
         f"<p>Maintenez le bouton de la lockbar enfoncé 2 secondes pendant une partie : le plugin prend la position "
         f"actuelle de votre tête comme nouvelle position neutre et affiche « Head tracking recentered ». Au début "
         f"de chaque partie, la première position stable est capturée : tenez-vous donc à votre place de jeu "
         f"habituelle pendant le chargement de la table.</p>"),
        ("Rien ne se passe et aucune notification de head tracking n'apparaît au lancement de la table.",
         f"<p>Quand le plugin tourne, il affiche au début de la partie une notification avec la caméra détectée, "
         f"et le journal de VPX reçoit des lignes <code>HeadTracking</code>. S'il ne trouve aucune caméra, le plugin "
         f"l'indique à l'écran. L'absence totale de notification vient en général d'un problème de pilote ou "
         f"d'autorisation : sous Linux la Kinect demande des règles udev, sous Windows le pilote WinUSB, et sous "
         f"macOS VPX doit avoir le droit d'utiliser la caméra. Voir "
         f'<a href="{p("download")}#linux">les étapes par système</a>.</p>'),
        ("Ma Kinect v2 n'est pas détectée.",
         f"<p>La Kinect v2 a besoin de son adaptateur secteur (le Kinect Adapter) et d'un <strong>port USB 3.0 dédié "
         f"à l'arrière de la carte mère</strong>. Elle transmet en même temps la couleur, l'infrarouge et la "
         f"profondeur, soit presque tout ce qu'un lien USB 3.0 peut faire passer : pas de hub, pas de port en "
         f"façade, pas de rallonge douteuse.</p>"),
        ("Le plugin indique que le flux infrarouge de la Kinect est déjà utilisé.",
         f"<p>Un autre programme occupe la Kinect, par exemple la démo headtracking ou un outil de capture. "
         f"Fermez-le et relancez la table.</p>"),
        ("L'image de la Kinect v2 se fige dans la démo au bout de quelques secondes.",
         f"<p>Lancez la démo avec la variable d'environnement <code>HT_DEPTH_PIPELINE=cpu</code>. Si le gel "
         f"disparaît, c'est le calcul de la profondeur sur la carte graphique qui pose problème sur votre machine. "
         f"C'est un diagnostic, pas un réglage à conserver : envoyez les deux journaux sur "
         f'<a href="{DISCORD}">Discord</a>.</p>'),
        ("Est-ce que ça marche sur un Mac Intel ou sous Windows ARM ?",
         f"<p>Pas pour l'instant. Les versions sont compilées pour Linux (x86_64 et ARM 64 bits), Windows (x86 "
         f"64 bits) et macOS sur Apple silicon. Les versions Mac Intel et Windows ARM sont désactivées dans la "
         f"chaîne de publication, à cause de problèmes dans les bibliothèques d'apprentissage automatique.</p>"),
        ("Est-ce que ça marche sur un minicab ?",
         f"<p>Oui, mais la faible distance entre le joueur et la caméra décide du choix de la caméra. Voir "
         f'<a href="{p("hardware")}#minicab">les notes sur les minicabs</a>.</p>'),
        ("Est-ce gratuit ?",
         f"<p>Oui. headtracking est un logiciel libre sous licence GNU GPL v3 ou ultérieure. Le code source est sur "
         f'<a href="{REPO}">GitHub</a>.</p>'),
        ("Où trouver de l'aide ?",
         f'<p>Les signalements de bugs, l\'aide et les tests des bêtas se passent sur <a href="{DISCORD}">Discord</a>. '
         f'Vous pouvez aussi ouvrir un ticket sur <a href="{REPO}/issues">GitHub</a>. Le guide d\'installation '
         f'détaillé (en anglais et en français) est <a href="{BLOB}docs/INSTALL.md">docs/INSTALL.md</a>.</p>'),
    ]


# ---------------------------------------------------------------- content

def content(page, lang, img):
    p = lambda name: href(name, lang, lang)  # noqa: E731
    en = lang == "en"

    if page == "index":
        if en:
            return ("headtracking – head tracking for Visual Pinball X with a webcam or Kinect",
                    "Free, open-source head tracking for Visual Pinball X (VPX): the table's perspective follows your "
                    "head (fish-tank VR) from a webcam or a Kinect v1/v2, self-calibrating from the lockbar. A BAM "
                    "alternative for Linux, Windows and macOS.",
                    f"""
<section class="hero split">
<div>
<p class="eyebrow">Head tracking for Visual Pinball X</p>
<h1>Move your head, the table follows</h1>
<p class="lead">headtracking gives Visual Pinball X (VPX) a point of view that follows your head, the
<em>fish-tank VR</em> effect that makes a flat screen look like a real pinball playfield under glass. It works from a
plain <strong>webcam</strong> or a <strong>Kinect v1 or v2</strong>, and it calibrates itself from your cabinet's
lockbar and side rails. Free, open source, for Linux, Windows and macOS.</p>
<p class="actions"><a class="btn big" href="{p('download')}">Download the preview</a>
<a class="btn big ghost" href="{DISCORD}">Join the Discord</a></p>
</div>
{fig(img, "setup.webp", 1280, 964, "A pinball cabinet running Visual Pinball X, with two Kinects and a webcam on top of the backbox",
     "A real pincab running VPX, with the cameras on top of the backbox.")}
</section>

<div class="note"><strong>Preview, early development.</strong> The whole chain, from the camera to a live point of
view inside a running VPX, works on a real Linux pincab with a Kinect v2, a Kinect v1 and a webcam. Windows and macOS
builds are published and need testers. Expect rough edges.</div>

<h2>What makes it different</h2>
<ul class="features">
<li><strong>No calibration routine.</strong> The cabinet is the calibration target: the lockbar and the two side rails
form a rectangle of known size. Seen in perspective, it is enough to work out where the camera is and how it sees,
from the image alone. No checkerboard, no &ldquo;look here and press a key&rdquo;. You only enter your lockbar width
and screen inclination in VPX.</li>
<li><strong>3D from a plain webcam.</strong> No depth sensor required. A Kinect measures the distance of your head
directly and tracks in the dark, but it is a bonus, not a requirement.</li>
<li><strong>Nothing else to install.</strong> One plugin: no Microsoft SDK, no Python. The Kinect libraries and the
neural-network runtime are built into it. A Kinect only needs a one-time USB access step on Linux and Windows, which
the demo app can do for you.</li>
<li><strong>Open source and cross-platform.</strong> Written in Rust, released under the GPL, for Linux, Windows and
macOS.</li>
</ul>

<div class="cards">
<div class="card"><h3>How it works</h3><p>The lockbar and rails give the camera position; a body model finds your
head; the plugin moves the VPX view every frame.</p><a class="more" href="{p('how-it-works')}">How it works →</a></div>
<div class="card"><h3>Download and install</h3><p>One archive per system with the plugin and a demo app, then a few
steps in VPX.</p><a class="more" href="{p('download')}">Install guide →</a></div>
<div class="card"><h3>Webcam or Kinect?</h3><p>What each camera brings, minimum distances, mounting on the backbox,
minicabs.</p><a class="more" href="{p('hardware')}">Choose a camera →</a></div>
<div class="card"><h3>Share your cabinet</h3><p>Two minutes, no coding: a capture of your pincab teaches the
calibration model new cabinets.</p><a class="more" href="{p('contribute')}">Contribute →</a></div>
</div>

<h2>On a real cabinet</h2>
<div class="gallery">
{fig(img, "setup-backbox.webp", 1280, 964, "A Kinect for Xbox 360, a Kinect for Xbox One and a small webcam on top of a pinball backbox",
     "The tracking rig on the backbox: a Kinect v1, a Kinect v2 and a webcam.")}
{fig(img, "anchor-check.webp", 1280, 720, "Camera view from the backbox with four traced lines along the lockbar and side rails",
     "Seen from the backbox: the lockbar edges and the side rails, the cabinet's reference frame.")}
</div>
{fig(img, "lockbar-detection.webp", 657, 179, "Lockbar outlined by a box and side rails marked by lines on a real cabinet",
     "What the detection finds on its own: the lockbar (box) and the two side rails.")}

<h2>The project needs you</h2>
<p>The calibration model learns what lockbars and rails look like from real cabinets, and it has seen very few so far.
If you own a pincab, <a href="{p('contribute')}">sending a capture</a> from the demo app takes two minutes and is the
most useful help there is. Rust developers, computer-vision people, VPX players and Windows or macOS testers are
welcome too.</p>

<h2>Free software, built in the open</h2>
<p>headtracking is written in Rust and released under the GNU GPL v3 or later. The source code, the releases and the
issue tracker are on <a href="{REPO}">GitHub</a>. Bug reports, help and beta testing happen on
<a href="{DISCORD}">Discord</a>.</p>
""")
        return ("headtracking – suivi de tête pour Visual Pinball X avec webcam ou Kinect",
                "Suivi de tête libre et gratuit pour le flipper virtuel Visual Pinball X (VPX) : la perspective suit "
                "votre tête (effet fish-tank VR) avec une webcam ou une Kinect v1/v2, calibration automatique par la "
                "lockbar. Alternative à BAM sous Linux, Windows et macOS.",
                f"""
<section class="hero split">
<div>
<p class="eyebrow">Suivi de tête pour Visual Pinball X</p>
<h1>Bougez la tête, la table suit</h1>
<p class="lead">headtracking donne à Visual Pinball X (VPX) un point de vue qui suit votre tête : l'effet
<em>fish-tank VR</em> qui fait d'un écran plat un vrai plateau de flipper sous sa vitre. Il fonctionne avec une simple
<strong>webcam</strong> ou une <strong>Kinect v1 ou v2</strong>, et se calibre tout seul grâce à la lockbar (la barre
avant du meuble) et aux rails latéraux de votre flipper virtuel. Libre, gratuit, pour Linux, Windows et macOS.</p>
<p class="actions"><a class="btn big" href="{p('download')}">Télécharger la préversion</a>
<a class="btn big ghost" href="{DISCORD}">Rejoindre le Discord</a></p>
</div>
{fig(img, "setup.webp", 1280, 964, "Un pincab qui fait tourner Visual Pinball X, avec deux Kinect et une webcam sur le fronton",
     "Un vrai pincab sous VPX, avec les caméras posées sur le fronton.")}
</section>

<div class="note"><strong>Préversion, en début de développement.</strong> Toute la chaîne, de la caméra jusqu'au point
de vue qui suit la tête dans VPX, fonctionne sur un vrai pincab sous Linux avec une Kinect v2, une Kinect v1 et une
webcam. Des versions Windows et macOS sont publiées et attendent des testeurs. Attendez-vous à quelques
imperfections.</div>

<h2>Ce qui le rend différent</h2>
<ul class="features">
<li><strong>Aucune procédure de calibration.</strong> C'est le meuble qui sert de mire : la lockbar et les deux rails
latéraux forment un rectangle de taille connue. Vu en perspective, il suffit à retrouver où se trouve la caméra et
comment elle voit, à partir de l'image seule. Pas de damier, pas de « regardez ici et appuyez sur une touche ». Vous
saisissez seulement la largeur de la lockbar et l'inclinaison de l'écran dans VPX.</li>
<li><strong>De la 3D avec une simple webcam.</strong> Aucun capteur de profondeur n'est nécessaire. Une Kinect mesure
directement la distance de votre tête et suit dans le noir, mais c'est un bonus, pas une obligation.</li>
<li><strong>Rien d'autre à installer.</strong> Un seul plugin : pas de SDK Microsoft, pas de Python. Les bibliothèques
Kinect et le moteur de réseaux de neurones sont intégrés. Une Kinect demande seulement, sous Linux et Windows, une
étape unique d'accès USB, que l'application de démonstration peut faire pour vous.</li>
<li><strong>Libre et multiplateforme.</strong> Écrit en Rust, publié sous licence GPL, pour Linux, Windows et
macOS.</li>
</ul>

<div class="cards">
<div class="card"><h3>Fonctionnement</h3><p>La lockbar et les rails donnent la position de la caméra ; un modèle du
corps trouve votre tête ; le plugin déplace la vue de VPX à chaque image.</p><a class="more" href="{p('how-it-works')}">Le fonctionnement →</a></div>
<div class="card"><h3>Télécharger et installer</h3><p>Une archive par système, avec le plugin et une application de
démonstration, puis quelques étapes dans VPX.</p><a class="more" href="{p('download')}">Guide d'installation →</a></div>
<div class="card"><h3>Webcam ou Kinect ?</h3><p>Ce qu'apporte chaque caméra, distances minimales, fixation sur le
fronton, minicabs.</p><a class="more" href="{p('hardware')}">Choisir une caméra →</a></div>
<div class="card"><h3>Partagez votre meuble</h3><p>Deux minutes, sans coder : un relevé de votre pincab apprend de
nouveaux meubles au modèle de calibration.</p><a class="more" href="{p('contribute')}">Contribuer →</a></div>
</div>

<h2>Sur un vrai meuble</h2>
<div class="gallery">
{fig(img, "setup-backbox.webp", 1280, 964, "Une Kinect pour Xbox 360, une Kinect pour Xbox One et une petite webcam sur le haut du fronton d'un flipper",
     "Les caméras sur le fronton : une Kinect v1, une Kinect v2 et une webcam.")}
{fig(img, "anchor-check.webp", 1280, 720, "Vue de la caméra depuis le fronton, avec quatre lignes tracées le long de la lockbar et des rails latéraux",
     "Vu depuis le fronton : les bords de la lockbar et les rails latéraux, le repère du meuble.")}
</div>
{fig(img, "lockbar-detection.webp", 657, 179, "Lockbar encadrée et rails latéraux marqués par des lignes sur un vrai meuble",
     "Ce que la détection trouve toute seule : la lockbar (le cadre) et les deux rails latéraux.")}

<h2>Le projet a besoin de vous</h2>
<p>Le modèle de calibration apprend à reconnaître lockbars et rails à partir de vrais meubles, et il en a encore vu
très peu. Si vous avez un pincab, <a href="{p('contribute')}">envoyer un relevé</a> depuis l'application de
démonstration prend deux minutes et c'est l'aide la plus utile qui soit. Développeurs Rust, spécialistes de vision
par ordinateur, joueurs de VPX et testeurs Windows ou macOS sont aussi les bienvenus.</p>

<h2>Un logiciel libre, développé ouvertement</h2>
<p>headtracking est écrit en Rust et publié sous licence GNU GPL v3 ou ultérieure. Le code source, les versions
publiées et les tickets sont sur <a href="{REPO}">GitHub</a>. Signalements de bugs, aide et tests des bêtas se passent
sur le <a href="{DISCORD}">Discord</a>.</p>
""")

    if page == "how-it-works":
        if en:
            return ("How VPX head tracking works: auto-calibration from lockbar and rails – headtracking",
                    "How headtracking gives Visual Pinball X a head-tracked point of view: camera position from the "
                    "lockbar and side rails, head found by a body model, distance from a Kinect or a webcam, live "
                    "view in the VPX plugin.",
                    f"""
<h1>How headtracking works</h1>
<p class="lead">A camera on the cabinet watches the player. The plugin works out where that camera is from the
cabinet itself, finds your head in each image, turns it into a position in millimetres, and moves the Visual Pinball
X point of view to match.</p>

<h2>The fish-tank effect</h2>
<p>When you move your head in front of a real pinball machine, you see the playfield from a slightly different angle:
near objects shift more than far ones. headtracking reproduces that on a screen. Visual Pinball X has a
<strong>Window</strong> view layout designed for this: the screen becomes a fixed window into the cabinet, and as your
head moves, the plugin moves the viewpoint so the 3D table stays in place behind the glass.</p>

<h2 id="calibration">Calibration from the cabinet itself</h2>
<p>To turn a head seen in an image into a real position, a program needs to know where the camera is and how it sees
(its focal length). Most setups ask you to measure or to go through a calibration routine. headtracking uses what
every cabinet already has: the <strong>lockbar</strong> and the <strong>two side rails</strong>.</p>
{fig(img, "lockbar-detection.webp", 657, 179, "Lockbar outlined by a box and side rails marked by lines on a real cabinet",
     "The detection on a real cabinet: lockbar (box) and side rails.")}
<ul>
<li>A neural network called the <em>anchor</em> model finds the lockbar and the rails in the camera image. There is
one model for colour images and one for infrared images.</li>
<li>Together they form a rectangle of known shape. Seen in perspective, it gives the camera's focal length and its
position relative to the table, using vanishing points and a plane homography. The maths have been checked to within
0 to 3&nbsp;% of a tape measure.</li>
<li>The real size comes from your <strong>lockbar width</strong>, which you enter in VPX. Your
<strong>screen inclination</strong> tells the plugin how the playfield is tilted. These are the only two values to
enter; <a href="{p('download')}#setup">measure them, do not guess</a>.</li>
<li>When a table starts, the plugin looks for the cabinet for a while, keeps its best detection, and shows the camera
position it found in a VPX notification. If it recognizes nothing, it still tracks your head relative to your starting
position, just without that notification.</li>
</ul>
{fig(img, "anchor-check.webp", 1280, 720, "Camera view from the backbox with four traced lines along the lockbar and side rails and their intersection points",
     "The reference frame traced on a capture: the lockbar's two edges, the two side rails and their six intersection points.")}

<h2>Finding your head</h2>
<p>The head is found by <strong>BlazePose</strong>, a body-pose model that runs in a few milliseconds per image. It
finds the head, the shoulders and the wrists at once, including on infrared images. It is a body detector, not a face
detector: a face detector loses track as soon as you lean over the playfield and the camera only sees the top of your
head. The flip side is that your bust (head and shoulders) must be in the picture.</p>

<h2>Measuring the distance</h2>
<ul>
<li><strong>Kinect v2 and Kinect v1</strong>: the depth sensor measures how far your head is, in millimetres. Both
track on their infrared stream, lit by the sensor itself, so they keep working in a dark game room. The Kinect v2
holds 30 images per second in infrared.</li>
<li><strong>Webcam</strong>: the distance is worked out from the width of your shoulders, using the focal length
recovered from the cabinet. That is what turns a single webcam into a 3D head tracker.</li>
</ul>
<p>See <a href="{p('hardware')}">choosing a camera</a> for the practical differences.</p>

<h2>Inside Visual Pinball X</h2>
<ul>
<li>headtracking is a plugin for the plugin system of <a href="{VPX}">Visual Pinball X</a> 10.8.1 and later. It is
enabled and set up from the in-game menu, <kbd>F12</kbd> → Plugin Settings → Head Tracking.</li>
<li>The tracking runs on its own thread; VPX reads the latest head position each frame without waiting, and the
plugin updates the view.</li>
<li>The first stable head position of a game becomes the neutral position. Holding the <strong>lockbar button for 2
seconds</strong> recenters on your current position.</li>
<li>Smoothing presets (Stable, Normal, Reactive, or Custom) filter the tremor out of the movement, and every setting
can be changed live while you play. See <a href="{p('download')}#settings">the settings</a>.</li>
<li>Problems are shown on screen, not only in a log: for example when no camera is found, or when another program is
already using the Kinect.</li>
</ul>

<h2 id="demo">The demo app</h2>
<p>Each release also contains <strong>headtracking-demo</strong>, a standalone program that needs no VPX. It shows the
camera picture with what the detection sees, lets you set the lockbar width and the playfield incline, and opens a 3D
parallax window where moving your head moves the scene, a quick way to judge the tracking. It also offers a one-click
fix when a Kinect is plugged in but cannot be opened, and the <strong>🎁 Contribute</strong> button that
<a href="{p('contribute')}">shares a capture</a> of your cabinet.</p>

<h2>Everything in one plugin</h2>
<p>The Kinect libraries (libfreenect and libfreenect2), libjpeg-turbo and the ONNX runtime that runs the neural
networks are built into the plugin, and the models are embedded in the program. There is no Microsoft Kinect SDK and
no Python to install. The whole project is written in Rust; <a href="{BLOB}CLAUDE.md">CLAUDE.md</a> describes the
architecture for developers.</p>
""")
        return ("Fonctionnement du suivi de tête pour VPX : calibration automatique par la lockbar – headtracking",
                "Comment headtracking donne à Visual Pinball X un point de vue qui suit la tête : position de la "
                "caméra par la lockbar et les rails, tête trouvée par un modèle du corps, distance par Kinect ou "
                "webcam, vue en direct dans le plugin VPX.",
                f"""
<h1>Le fonctionnement de headtracking</h1>
<p class="lead">Une caméra posée sur le meuble regarde le joueur. Le plugin déduit la position de cette caméra à
partir du meuble lui-même, trouve votre tête dans chaque image, la convertit en position en millimètres, et déplace le
point de vue de Visual Pinball X en conséquence.</p>

<h2>L'effet « fish-tank »</h2>
<p>Quand vous bougez la tête devant un vrai flipper, vous voyez le plateau sous un angle légèrement différent : les
objets proches se décalent plus que les objets lointains. headtracking reproduit cet effet sur un écran. Visual Pinball
X propose une disposition de vue <strong>Window</strong> conçue pour cela : l'écran devient une fenêtre fixe sur le
meuble, et quand votre tête bouge, le plugin déplace le point de vue pour que la table 3D reste en place derrière la
vitre.</p>

<h2 id="calibration">Une calibration tirée du meuble lui-même</h2>
<p>Pour transformer une tête vue dans une image en position réelle, un programme doit savoir où se trouve la caméra et
comment elle voit (sa focale). La plupart des solutions vous demandent des mesures ou une procédure de calibration.
headtracking se sert de ce que tout meuble possède déjà : la <strong>lockbar</strong> et les <strong>deux rails
latéraux</strong>.</p>
{fig(img, "lockbar-detection.webp", 657, 179, "Lockbar encadrée et rails latéraux marqués par des lignes sur un vrai meuble",
     "La détection sur un vrai meuble : la lockbar (le cadre) et les rails latéraux.")}
<ul>
<li>Un réseau de neurones, le modèle <em>anchor</em>, repère la lockbar et les rails dans l'image de la caméra. Il
existe un modèle pour les images couleur et un autre pour les images infrarouges.</li>
<li>Ensemble, ils forment un rectangle de forme connue. Vu en perspective, il donne la focale de la caméra et sa
position par rapport à la table, grâce aux points de fuite et à une homographie plane. Les calculs ont été vérifiés à
0 à 3&nbsp;% près d'un mètre ruban.</li>
<li>La taille réelle vient de la <strong>largeur de la lockbar</strong>, que vous saisissez dans VPX.
L'<strong>inclinaison de l'écran</strong> indique au plugin comment le plateau est penché. Ce sont les deux seules
valeurs à saisir ; <a href="{p('download')}#setup">mesurez-les, ne les devinez pas</a>.</li>
<li>Au lancement d'une table, le plugin cherche le meuble pendant un moment, garde sa meilleure détection, et affiche
dans une notification de VPX la position de caméra trouvée. S'il ne reconnaît rien, il suit quand même votre tête par
rapport à votre position de départ, simplement sans cette notification.</li>
</ul>
{fig(img, "anchor-check.webp", 1280, 720, "Vue de la caméra depuis le fronton, avec quatre lignes tracées le long de la lockbar et des rails, et leurs points d'intersection",
     "Le repère tracé sur un relevé : les deux bords de la lockbar, les deux rails latéraux et leurs six points d'intersection.")}

<h2>Trouver votre tête</h2>
<p>La tête est trouvée par <strong>BlazePose</strong>, un modèle de pose du corps qui tourne en quelques millisecondes
par image. Il repère d'un coup la tête, les épaules et les poignets, y compris sur les images infrarouges. C'est un
détecteur de corps, pas de visage : un détecteur de visage décroche dès que vous vous penchez sur le plateau et que la
caméra ne voit plus que le haut de votre crâne. En contrepartie, votre buste (tête et épaules) doit être dans
l'image.</p>

<h2>Mesurer la distance</h2>
<ul>
<li><strong>Kinect v2 et Kinect v1</strong> : le capteur de profondeur mesure l'éloignement de votre tête en
millimètres. Les deux suivent sur leur flux infrarouge, éclairé par le capteur lui-même, et continuent donc de
fonctionner dans une salle de jeu sombre. La Kinect v2 tient 30 images par seconde en infrarouge.</li>
<li><strong>Webcam</strong> : la distance est déduite de la largeur de vos épaules, grâce à la focale retrouvée à
partir du meuble. C'est ce qui fait d'une simple webcam un suivi de tête en 3D.</li>
</ul>
<p>Voir <a href="{p('hardware')}">choisir une caméra</a> pour les différences en pratique.</p>

<h2>Dans Visual Pinball X</h2>
<ul>
<li>headtracking est un plugin pour le système de plugins de <a href="{VPX}">Visual Pinball X</a> 10.8.1 et plus
récent. Il s'active et se règle depuis le menu en jeu, <kbd>F12</kbd> → Plugin Settings → Head Tracking.</li>
<li>Le suivi tourne dans son propre fil d'exécution ; VPX lit la dernière position de la tête à chaque image sans
attendre, et le plugin met la vue à jour.</li>
<li>La première position de tête stable d'une partie devient la position neutre. Maintenir le <strong>bouton de la
lockbar pendant 2 secondes</strong> recentre sur votre position actuelle.</li>
<li>Des préréglages de lissage (Stable, Normal, Reactive ou Custom) filtrent les tremblements, et chaque réglage se
modifie en direct pendant la partie. Voir <a href="{p('download')}#settings">les réglages</a>.</li>
<li>Les problèmes s'affichent à l'écran, pas seulement dans un journal : par exemple quand aucune caméra n'est trouvée,
ou quand un autre programme utilise déjà la Kinect.</li>
</ul>

<h2 id="demo">L'application de démonstration</h2>
<p>Chaque version contient aussi <strong>headtracking-demo</strong>, un programme autonome qui n'a pas besoin de VPX.
Il affiche l'image de la caméra avec ce que voit la détection, permet de régler la largeur de la lockbar et
l'inclinaison du plateau, et ouvre une fenêtre de parallaxe en 3D où bouger la tête déplace la scène : un moyen rapide
de juger le suivi. Il propose aussi une correction en un clic quand une Kinect est branchée mais inaccessible, et le
bouton <strong>🎁 Contribute</strong> qui <a href="{p('contribute')}">partage un relevé</a> de votre meuble.</p>

<h2>Tout dans un seul plugin</h2>
<p>Les bibliothèques Kinect (libfreenect et libfreenect2), libjpeg-turbo et le moteur ONNX qui fait tourner les
réseaux de neurones sont intégrés au plugin, et les modèles sont embarqués dans le programme. Il n'y a ni SDK Kinect de
Microsoft ni Python à installer. Tout le projet est écrit en Rust ; <a href="{BLOB}CLAUDE.md">CLAUDE.md</a> décrit
l'architecture pour les développeurs.</p>
""")

    if page == "download":
        if en:
            return ("Download headtracking: VPX head tracking plugin for Linux, Windows and macOS",
                    "Download and install headtracking, the free head tracking plugin for Visual Pinball X 10.8.1: "
                    "Linux, Windows and macOS archives, VPX plugin folder, F12 settings, Kinect drivers and udev rules.",
                    f"""
<h1>Download and install headtracking</h1>
<p class="lead">headtracking is published as a <strong>preview</strong> on GitHub. Each release has one archive per
system, containing the VPX plugin and the demo app. The file names include the version number, so pick the file for
your system on the releases page.</p>
<p class="actions"><a class="btn big" href="{RELEASES}">Open the releases page</a></p>
{downloads("en")}
<p>There is currently no build for Intel Macs or Windows on ARM. The Windows files are digitally signed, and the macOS
programs are signed and notarized by Apple.</p>

<h2>What is in the archive</h2>
<ul>
<li>The plugin: <code>headtracking.dll</code> (Windows), <code>libheadtracking.so</code> (Linux) or
<code>libheadtracking.dylib</code> (macOS), with its <code>plugin.cfg</code>.</li>
<li><code>headtracking-demo</code>, the <a href="{p('how-it-works')}#demo">demo app</a>: test your camera without VPX,
and <a href="{p('contribute')}">share a capture</a> of your cabinet.</li>
<li>On Windows, a <code>setup</code> folder with the Kinect driver installer.</li>
<li>The licence and the README.</li>
</ul>

<h2 id="install">Install the plugin in Visual Pinball X</h2>
<p>You need <a href="{VPX}">Visual Pinball X</a> 10.8.1 or later.</p>
<ol class="steps">
<li>Create a <code>headtracking</code> folder in the <code>plugins</code> folder of your VPX installation, and copy the
plugin library <strong>and</strong> <code>plugin.cfg</code> into it. Both must be in the same folder.
<pre><code>{PLUGIN_TREE}</code></pre></li>
<li>Start VPX once: it finds the new plugin by itself.</li>
<li>Load any table, press <kbd>F12</kbd> → <strong>Plugin Settings</strong> → <strong>Head Tracking</strong> and tick
<strong>Enable</strong>. Quit and reload the table: tracking starts with the game.</li>
</ol>

<h2 id="setup">Set up the view</h2>
<p>The plugin reminds you of these points in a notification when a game starts.</p>
<ol class="steps">
<li>In <kbd>F12</kbd> → <strong>Cabinet Settings</strong>, enter your real <strong>lockbar width</strong> and
<strong>screen inclination</strong>. The automatic calibration and the head position are based on them.</li>
<li>For the table's point of view, choose the <strong>Window</strong> view layout with <strong>rotation 0</strong>, and
turn on <strong>cabinet autofit</strong>. Window is the layout designed for head tracking.</li>
<li>Stand in your normal playing position while the table loads: the first stable head position becomes the neutral
position. Later, hold the lockbar button for 2 seconds to recenter.</li>
</ol>
<div class="note"><p><strong>Measure those two values, do not guess them.</strong> The plugin trusts them
completely.</p>
<ul>
<li>A wrong lockbar width scales every distance by the same error: the effect feels too strong or too weak
everywhere.</li>
<li>A missing or wrong screen inclination mixes up/down with closer/farther: the view drifts diagonally when you move,
and it feels wrong with no obvious cause.</li>
</ul></div>

<h2 id="settings">Settings</h2>
<p>Every setting is on the same <kbd>F12</kbd> → Plugin Settings → Head Tracking page and applies live while you play,
except <strong>Backend</strong> and <strong>Camera</strong>, which are read when the game starts (reload the table after
changing them).</p>
{settings_table("en")}
<p>They are stored in <code>VPinballX.ini</code>, in the <code>[Plugin.HeadTracking]</code> section. If you edit that
file by hand, do it with VPX closed: VPX rewrites the whole file when it exits.</p>

<h2 id="linux">Linux</h2>
<ul>
<li><strong>Webcam</strong>: your user must be allowed to use the camera. On most distributions it already is (the
<code>video</code> group); otherwise run <code>sudo usermod -aG video "$USER"</code> and log out and back in.</li>
<li><strong>Kinect</strong>: without udev rules, only the administrator can open the Kinect, and VPX fails silently.
If a Kinect is plugged in but missing from its camera list, the demo app offers an <strong>Install udev rule</strong>
button, which asks for your password. You can also add the rules yourself, then unplug and replug the Kinect:
<details><summary>udev rules for Kinect v2 and Kinect v1</summary>
<pre><code>{html.escape(UDEV)}</code></pre></details></li>
<li>No other library to install: only system libraries present by default on Debian, Ubuntu, Fedora and Arch are
used.</li>
</ul>

<h2 id="windows">Windows</h2>
<div class="note danger"><p><strong>Windows + Kinect: read this before installing the driver.</strong> To reach a
Kinect, the bundled driver setup <strong>replaces Microsoft's official Kinect driver</strong> with a generic WinUSB
one. This breaks everything built on the Microsoft Kinect SDK, <strong>including a working BAM head-tracking
setup</strong>, until you restore the original driver (Device Manager, the &ldquo;Xbox NUI&rdquo; devices: uninstall
the driver and scan for hardware changes, or reinstall the Kinect SDK/runtime). Only run it if you accept that trade.
<strong>Webcam users are not affected</strong> and have nothing to install.</p></div>
<p>Out of the box, Windows installs no usable driver for a Kinect v1 or v2: it shows up as an unknown device. To
install one:</p>
<ol class="steps">
<li>Plug in the Kinect and start <code>headtracking-demo.exe</code>. If no usable driver is found, a yellow banner
offers <strong>Install Kinect drivers (UAC prompt)</strong>. You can also double-click
<code>setup\\setup-kinect.cmd</code>.</li>
<li>Accept the administrator prompt. The PowerShell window lists what will change and asks you to type
<code>yes</code>; anything else cancels without touching the system.</li>
<li>Wait for the script to finish (about 10 to 30 seconds), click <strong>rescan</strong> in the demo, then restart
VPX.</li>
</ol>
<p>The script is signed in release archives. It works for both Kinect v1 and v2. If you prefer not to run it, you can
bind WinUSB by hand with <a href="{ZADIG}">Zadig</a>; the steps are in the
<a href="{BLOB}docs/INSTALL.md">detailed install guide</a>. On Windows 7, install Microsoft Security Advisory 3033929
first.</p>
<p>A Kinect v2 also needs a <strong>dedicated USB 3.0 port</strong> on the back of the motherboard, not a hub.</p>

<h2 id="macos">macOS</h2>
<p>A Kinect needs no extra driver on macOS. For a webcam, allow VPX in <strong>System Settings → Privacy &amp; Security →
Camera</strong> the first time. The build runs natively on Apple silicon.</p>

<h2 id="source">Build from source</h2>
<p>You need a recent stable Rust, <code>cmake</code> 3.20 or later and <code>libclang</code>. The native libraries
are included as submodules and built statically.</p>
<pre><code>git clone --recurse-submodules https://github.com/Le-Syl21/headtracking
cd headtracking
cargo build --release</code></pre>
<p>Full details, including every Windows and Linux troubleshooting step: <a href="{BLOB}docs/INSTALL.md">docs/INSTALL.md</a>.</p>
""")
        return ("Télécharger headtracking : plugin de suivi de tête VPX pour Linux, Windows et macOS",
                "Téléchargez et installez headtracking, le plugin libre de suivi de tête pour Visual Pinball X 10.8.1 : "
                "archives Linux, Windows et macOS, dossier des plugins VPX, réglages F12, pilotes Kinect et règles udev.",
                f"""
<h1>Télécharger et installer headtracking</h1>
<p class="lead">headtracking est publié en <strong>préversion</strong> sur GitHub. Chaque version propose une archive
par système, avec le plugin VPX et l'application de démonstration. Les noms de fichiers contiennent le numéro de
version : choisissez le fichier de votre système sur la page des versions.</p>
<p class="actions"><a class="btn big" href="{RELEASES}">Ouvrir la page des versions</a></p>
{downloads("fr")}
<p>Il n'existe pas pour l'instant de version pour les Mac Intel ni pour Windows sur ARM. Les fichiers Windows sont
signés numériquement, et les programmes macOS sont signés et notariés auprès d'Apple.</p>

<h2>Contenu de l'archive</h2>
<ul>
<li>Le plugin : <code>headtracking.dll</code> (Windows), <code>libheadtracking.so</code> (Linux) ou
<code>libheadtracking.dylib</code> (macOS), avec son <code>plugin.cfg</code>.</li>
<li><code>headtracking-demo</code>, l'<a href="{p('how-it-works')}#demo">application de démonstration</a> : testez
votre caméra sans VPX, et <a href="{p('contribute')}">partagez un relevé</a> de votre meuble.</li>
<li>Sous Windows, un dossier <code>setup</code> avec l'installation du pilote Kinect.</li>
<li>La licence et le README.</li>
</ul>

<h2 id="install">Installer le plugin dans Visual Pinball X</h2>
<p>Il faut <a href="{VPX}">Visual Pinball X</a> 10.8.1 ou plus récent.</p>
<ol class="steps">
<li>Créez un dossier <code>headtracking</code> dans le dossier <code>plugins</code> de votre installation de VPX, et
copiez-y la bibliothèque du plugin <strong>et</strong> <code>plugin.cfg</code>. Les deux doivent être dans le même
dossier.
<pre><code>{PLUGIN_TREE}</code></pre></li>
<li>Lancez VPX une fois : il trouve tout seul le nouveau plugin.</li>
<li>Chargez une table, appuyez sur <kbd>F12</kbd> → <strong>Plugin Settings</strong> → <strong>Head Tracking</strong> et
cochez <strong>Enable</strong>. Quittez et rechargez la table : le suivi démarre avec la partie.</li>
</ol>

<h2 id="setup">Régler la vue</h2>
<p>Le plugin rappelle ces points dans une notification au début de chaque partie.</p>
<ol class="steps">
<li>Dans <kbd>F12</kbd> → <strong>Cabinet Settings</strong>, saisissez la vraie <strong>largeur de votre lockbar</strong>
et l'<strong>inclinaison de l'écran</strong>. La calibration automatique et la position de la tête reposent sur ces
valeurs.</li>
<li>Pour le point de vue de la table, choisissez la disposition <strong>Window</strong> avec une <strong>rotation de
0</strong>, et activez le mode <strong>cabinet autofit</strong>. Window est la disposition conçue pour le suivi de
tête.</li>
<li>Tenez-vous à votre place de jeu habituelle pendant le chargement de la table : la première position de tête stable
devient la position neutre. Ensuite, maintenez le bouton de la lockbar 2 secondes pour recentrer.</li>
</ol>
<div class="note"><p><strong>Mesurez ces deux valeurs, ne les devinez pas.</strong> Le plugin leur fait entièrement
confiance.</p>
<ul>
<li>Une largeur de lockbar fausse applique la même erreur d'échelle à toutes les distances : l'effet paraît partout
trop fort ou trop faible.</li>
<li>Une inclinaison d'écran absente ou fausse mélange le haut/bas avec le proche/loin : la vue dérive en diagonale
quand vous bougez, et « quelque chose cloche » sans cause évidente.</li>
</ul></div>

<h2 id="settings">Réglages</h2>
<p>Tous les réglages se trouvent sur la même page <kbd>F12</kbd> → Plugin Settings → Head Tracking et s'appliquent en
direct pendant la partie, sauf <strong>Backend</strong> et <strong>Camera</strong>, lus au lancement de la partie
(rechargez la table après les avoir changés). Les noms ci-dessous sont ceux affichés, en anglais, dans VPX.</p>
{settings_table("fr")}
<p>Ils sont enregistrés dans <code>VPinballX.ini</code>, section <code>[Plugin.HeadTracking]</code>. Si vous modifiez
ce fichier à la main, faites-le avec VPX fermé : VPX réécrit tout le fichier en quittant.</p>

<h2 id="linux">Linux</h2>
<ul>
<li><strong>Webcam</strong> : votre utilisateur doit avoir le droit d'utiliser la caméra. C'est déjà le cas sur la
plupart des distributions (groupe <code>video</code>) ; sinon lancez <code>sudo usermod -aG video "$USER"</code> puis
fermez et rouvrez votre session.</li>
<li><strong>Kinect</strong> : sans règles udev, seul l'administrateur peut ouvrir la Kinect, et VPX échoue sans rien
dire. Si une Kinect est branchée mais absente de la liste des caméras, l'application de démonstration propose un
bouton <strong>Install udev rule</strong>, qui demande votre mot de passe. Vous pouvez aussi ajouter les règles
vous-même, puis débrancher et rebrancher la Kinect :
<details><summary>Règles udev pour Kinect v2 et Kinect v1</summary>
<pre><code>{html.escape(UDEV)}</code></pre></details></li>
<li>Aucune autre bibliothèque à installer : seules des bibliothèques système présentes par défaut sur Debian, Ubuntu,
Fedora et Arch sont utilisées.</li>
</ul>

<h2 id="windows">Windows</h2>
<div class="note danger"><p><strong>Windows + Kinect : à lire avant d'installer le pilote.</strong> Pour accéder à la
Kinect, l'installation de pilote fournie <strong>remplace le pilote Kinect officiel de Microsoft</strong> par un pilote
WinUSB générique. Cela casse tout ce qui repose sur le SDK Kinect de Microsoft, <strong>y compris un head tracking BAM
qui fonctionne</strong>, jusqu'à ce que vous remettiez le pilote d'origine (Gestionnaire de périphériques,
périphériques « Xbox NUI » : désinstaller le pilote puis rechercher les modifications matérielles, ou réinstaller le
SDK/runtime Kinect). Ne le lancez que si vous acceptez cet échange. <strong>Les utilisateurs de webcam ne sont pas
concernés</strong> et n'ont rien à installer.</p></div>
<p>D'origine, Windows n'installe aucun pilote utilisable pour une Kinect v1 ou v2 : elle apparaît comme périphérique
inconnu. Pour en installer un :</p>
<ol class="steps">
<li>Branchez la Kinect et lancez <code>headtracking-demo.exe</code>. S'il ne trouve pas de pilote utilisable, un
bandeau jaune propose <strong>Install Kinect drivers (UAC prompt)</strong>. Vous pouvez aussi double-cliquer sur
<code>setup\\setup-kinect.cmd</code>.</li>
<li>Acceptez la demande d'administrateur. La fenêtre PowerShell résume ce qui va changer et vous demande de taper
<code>yes</code> ; toute autre réponse annule sans rien toucher.</li>
<li>Attendez la fin du script (10 à 30 secondes environ), cliquez sur <strong>rescan</strong> dans la démo, puis
relancez VPX.</li>
</ol>
<p>Le script est signé dans les archives publiées. Il fonctionne pour la Kinect v1 comme pour la v2. Si vous préférez
ne pas le lancer, vous pouvez associer WinUSB à la main avec <a href="{ZADIG}">Zadig</a> ; les étapes sont dans le
<a href="{BLOB}docs/INSTALL.md">guide d'installation détaillé</a>. Sous Windows 7, installez d'abord le Microsoft
Security Advisory 3033929.</p>
<p>Une Kinect v2 demande en plus un <strong>port USB 3.0 dédié</strong> à l'arrière de la carte mère, pas un hub.</p>

<h2 id="macos">macOS</h2>
<p>Une Kinect ne demande aucun pilote supplémentaire sous macOS. Pour une webcam, autorisez VPX dans <strong>Réglages
Système → Confidentialité et sécurité → Caméra</strong> la première fois. La version tourne nativement sur Apple
silicon.</p>

<h2 id="source">Compiler depuis les sources</h2>
<p>Il faut un Rust stable récent, <code>cmake</code> 3.20 ou plus récent et <code>libclang</code>. Les bibliothèques
natives sont incluses en sous-modules et compilées en statique.</p>
<pre><code>git clone --recurse-submodules https://github.com/Le-Syl21/headtracking
cd headtracking
cargo build --release</code></pre>
<p>Tous les détails, dont chaque étape de dépannage Windows et Linux :
<a href="{BLOB}docs/INSTALL.md">docs/INSTALL.md</a>.</p>
""")

    if page == "hardware":
        if en:
            return ("Webcam or Kinect for pincab head tracking: cameras and mounting – headtracking",
                    "Which camera for Visual Pinball X head tracking: Kinect v2, Kinect v1 or webcam. Infrared tracking "
                    "in the dark, minimum distances, USB 3.0, mounting on the backbox, minicab advice.",
                    f"""
<h1>Webcam or Kinect?</h1>
<p class="lead">Every camera works with headtracking. The best value is a second-hand Kinect v2, but a webcam you
already own costs nothing, and the automatic calibration gives it real 3D too.</p>
{cameras_table("en")}

<h2 id="kinect-v2">Kinect v2 (Xbox One): the sweet spot</h2>
{fig(img, "kinect-v2.webp", 700, 358, "Kinect v2 (Xbox One) sensor bar", "Kinect v2 (Xbox One).", "side")}
<ul>
<li><strong>Cheap second-hand</strong>, since Microsoft discontinued it: from about €20 on classifieds such as
<a href="{html.escape(LEBONCOIN)}">leboncoin</a> or <a href="{html.escape(EBAY)}">eBay</a>.</li>
<li>It tracks on its <strong>infrared stream</strong>: the sensor lights the scene itself, so you get 30 images per
second in a pitch-dark game room where a webcam struggles.</li>
<li>Its <strong>depth sensor</strong> measures the distance of your head in millimetres instead of estimating it.</li>
<li><strong>Budget for the Kinect Adapter</strong> (power supply and USB 3.0, about €29 new) unless the listing
includes it: check before buying.</li>
<li>Plug it into a <strong>dedicated USB 3.0 port</strong> on the back of the motherboard. It will not show up behind
a shared hub.</li>
</ul>
<div class="note clear"><strong>The Kinect v2 uses a lot of USB bandwidth.</strong> It sends colour (1920×1080 at 30
images per second), infrared and depth (512×424 at 30 images per second, uncompressed) and its four microphones, all
at once. That is close to filling a USB 3.0 link on its own. Before blaming the camera or the adapter, make sure it is
on a dedicated USB 3.0 root port: no hub, no front-panel port, no cheap extension cable.</div>

<h2>Kinect v1 (Xbox 360)</h2>
<p>Even cheaper and field-tested too. Its resolution is lower, but it is perfectly usable: it uses colour to find the
cabinet, then tracks in infrared with its depth sensor. Its minimum distance of about 0.8 m can be a problem on small
cabinets.</p>

<h2>Webcam</h2>
<p>A plain webcam works: the calibration from the lockbar and rails recovers its focal length, and the distance of
your head comes from your shoulder width. It tracks on the colour image, so a dark game room is harder for it than for a Kinect. Head
and shoulders must fit in the picture.</p>

<h2 id="mounting">Mounting</h2>
{fig(img, "setup-backbox.webp", 1280, 964, "A Kinect for Xbox 360, a Kinect for Xbox One and a small webcam on top of a pinball backbox",
     "Cameras on top of the backbox of a real pincab: Kinect v1, Kinect v2 and a webcam.")}
<ul>
<li>Put the camera on top of the backbox (backglass) or the topper, facing the player. It must see the
<strong>lockbar and the side rails</strong> for the calibration, and your <strong>head and shoulders</strong> for the
tracking.</li>
<li>A 1/4"-20 screw assortment kit is all it takes to fix the sensor. The project keeps a ready-made
<a href="{AMAZON_LIST}">Amazon &ldquo;Headtracking&rdquo; list</a> with the Kinect Adapter and a screw kit.</li>
<li>Mounting the camera high and angling it down increases the distance to the player's head, which helps with the
minimum distances above.</li>
<li>If left and right (or up and down) come out reversed because of an unusual mounting, use the Invert settings of
the plugin.</li>
</ul>

<h2 id="minicab">Minicabs and small cabinets</h2>
<p>Head tracking works on a minicab, but the short distance between the player and the camera decides everything.</p>
<ul>
<li>Player <strong>60 cm or more</strong> from the camera: the second-hand Kinect v2 stays the best pick.</li>
<li>Player <strong>closer than about 60 cm</strong>: a <strong>wide-angle webcam</strong> is the better option. It has
no hard distance floor, and minicabs often live in lit rooms where the Kinect's own infrared light matters less.</li>
<li>The <strong>Kinect v1 is risky</strong> on a minicab: its 0.8 m floor is often more than the whole distance from
the player to the backbox.</li>
<li>Enter your <strong>real lockbar width</strong> in VPX: a minicab's lockbar is much narrower than a full-size one,
and the calibration is based on that value. Enter the screen inclination as on a full-size cabinet.</li>
<li>The detection of the lockbar and rails works at any cabinet size, and a close camera even sees them bigger. The
model has never seen a minicab, so <a href="{p('contribute')}">a capture of yours</a> is especially welcome.</li>
</ul>
<p>More details: <a href="{BLOB}docs/MINICAB.md">docs/MINICAB.md</a>.</p>
""")
        return ("Webcam ou Kinect pour le suivi de tête d'un pincab : caméras et fixation – headtracking",
                "Quelle caméra pour le suivi de tête dans Visual Pinball X : Kinect v2, Kinect v1 ou webcam. Suivi "
                "infrarouge dans le noir, distances minimales, USB 3.0, fixation sur le fronton, conseils minicab.",
                f"""
<h1>Webcam ou Kinect ?</h1>
<p class="lead">Toutes les caméras fonctionnent avec headtracking. Le meilleur rapport qualité-prix est une Kinect v2
d'occasion, mais une webcam que vous avez déjà ne coûte rien, et la calibration automatique lui donne aussi une vraie
3D.</p>
{cameras_table("fr")}

<h2 id="kinect-v2">Kinect v2 (Xbox One) : le meilleur choix</h2>
{fig(img, "kinect-v2.webp", 700, 358, "Barre de capteurs Kinect v2 (Xbox One)", "Kinect v2 (Xbox One).", "side")}
<ul>
<li><strong>Peu chère d'occasion</strong> depuis son arrêt par Microsoft : à partir d'environ 20 € sur les petites
annonces, par exemple <a href="{html.escape(LEBONCOIN)}">leboncoin</a> ou <a href="{html.escape(EBAY)}">eBay</a>.</li>
<li>Elle suit sur son <strong>flux infrarouge</strong> : le capteur éclaire lui-même la scène, ce qui donne 30 images
par seconde dans une salle de jeu plongée dans le noir, là où une webcam peine.</li>
<li>Son <strong>capteur de profondeur</strong> mesure la distance de votre tête en millimètres au lieu de
l'estimer.</li>
<li><strong>Prévoyez l'adaptateur Kinect</strong> (alimentation et USB 3.0, environ 29 € neuf) si l'annonce ne
l'inclut pas : vérifiez avant d'acheter.</li>
<li>Branchez-la sur un <strong>port USB 3.0 dédié</strong> à l'arrière de la carte mère. Elle n'apparaît pas derrière
un hub partagé.</li>
</ul>
<div class="note clear"><strong>La Kinect v2 consomme beaucoup de bande passante USB.</strong> Elle envoie en même temps
la couleur (1920×1080 à 30 images par seconde), l'infrarouge et la profondeur (512×424 à 30 images par seconde, non
compressés) et ses quatre micros. À elle seule, elle remplit presque un lien USB 3.0. Avant d'accuser la caméra ou
l'adaptateur, vérifiez qu'elle est sur un port USB 3.0 racine dédié : pas de hub, pas de port en façade, pas de
rallonge douteuse.</div>

<h2>Kinect v1 (Xbox 360)</h2>
<p>Encore moins chère et éprouvée elle aussi. Sa résolution est plus faible, mais elle reste tout à fait utilisable :
elle se sert de la couleur pour repérer le meuble, puis suit en infrarouge avec son capteur de profondeur. Sa distance
minimale d'environ 0,8 m peut poser problème sur les petits meubles.</p>

<h2>Webcam</h2>
<p>Une simple webcam fonctionne : la calibration par la lockbar et les rails retrouve sa focale, et la distance de
votre tête est déduite de la largeur de vos épaules. Elle suit sur l'image couleur : une salle de jeu sombre lui
est donc plus difficile qu'à une Kinect. La tête et les épaules doivent tenir dans l'image.</p>

<h2 id="mounting">Fixation</h2>
{fig(img, "setup-backbox.webp", 1280, 964, "Une Kinect pour Xbox 360, une Kinect pour Xbox One et une petite webcam sur le haut du fronton d'un flipper",
     "Des caméras sur le fronton d'un vrai pincab : Kinect v1, Kinect v2 et une webcam.")}
<ul>
<li>Placez la caméra sur le haut du fronton (backbox) ou du topper, tournée vers le joueur. Elle doit voir la
<strong>lockbar et les rails latéraux</strong> pour la calibration, et votre <strong>tête et vos épaules</strong> pour
le suivi.</li>
<li>Un kit d'assortiment de vis 1/4"-20 suffit pour fixer le capteur. Le projet tient à jour une
<a href="{AMAZON_LIST}">liste Amazon « Headtracking »</a> avec l'adaptateur Kinect et un kit de vis.</li>
<li>Monter la caméra haut et l'incliner vers le bas l'éloigne de la tête du joueur, ce qui aide face aux distances
minimales ci-dessus.</li>
<li>Si la gauche et la droite (ou le haut et le bas) sont inversées à cause d'un montage inhabituel, utilisez les
réglages Invert du plugin.</li>
</ul>

<h2 id="minicab">Minicabs et petits meubles</h2>
<p>Le suivi de tête fonctionne sur un minicab, mais la faible distance entre le joueur et la caméra décide de tout.</p>
<ul>
<li>Joueur à <strong>60 cm ou plus</strong> de la caméra : la Kinect v2 d'occasion reste le meilleur choix.</li>
<li>Joueur à <strong>moins d'environ 60 cm</strong> : une <strong>webcam grand-angle</strong> est la meilleure option.
Elle n'a pas de distance plancher stricte, et un minicab vit souvent dans une pièce éclairée, où l'éclairage
infrarouge de la Kinect compte moins.</li>
<li>La <strong>Kinect v1 est risquée</strong> sur un minicab : son plancher de 0,8 m dépasse souvent toute la distance
entre le joueur et le fronton.</li>
<li>Saisissez la <strong>vraie largeur de votre lockbar</strong> dans VPX : celle d'un minicab est bien plus étroite que
celle d'un flipper classique, et la calibration repose sur cette valeur. Saisissez l'inclinaison de l'écran comme sur
un meuble classique.</li>
<li>La détection de la lockbar et des rails fonctionne quelle que soit la taille du meuble, et une caméra proche les
voit même plus grands. Le modèle n'a encore jamais vu de minicab : <a href="{p('contribute')}">votre relevé</a> est
particulièrement bienvenu.</li>
</ul>
<p>Plus de détails : <a href="{BLOB}docs/MINICAB.md">docs/MINICAB.md</a>.</p>
""")

    if page == "contribute":
        if en:
            return ("Contribute a capture of your pincab – headtracking for Visual Pinball X",
                    "Help the open-source VPX head tracking project: share a capture of your pinball cabinet from the "
                    "demo app in two minutes, annotate photos, or test on Windows and macOS.",
                    f"""
<h1>Teach it your cabinet</h1>
<p class="lead">The automatic calibration learns what lockbars and side rails look like from real cabinets: every wood
tone, every lighting, every camera angle. It has seen very few so far. This is the project's biggest bottleneck, and
anyone with a pincab can help, in two minutes, without coding.</p>

<h2 id="capture">Send a capture</h2>
<ol class="steps">
<li>Download <strong>headtracking-demo</strong> from the <a href="{RELEASES}">releases page</a> (it is in the same
archive as the plugin). Nothing to install.</li>
<li>Point your webcam or Kinect at the playfield and select it in the demo.</li>
<li>Click <strong>🎁 Contribute</strong>, read the terms, tick the consent box and confirm.</li>
</ol>
<p>Empty cabinet or mid-game, day or night, every variation helps. The detector learns the cabinet, so you do not need
to stand in the picture, but captures with a player at the cabinet are the rarest. Minicabs are exactly the kind of
cabinet the model has never seen.</p>
{fig(img, "anchor-check.webp", 1280, 720, "Camera view from the backbox with four traced lines along the lockbar and side rails and their intersection points",
     "What is extracted from a capture: the lockbar's two edges, the two side rails and their six intersection points.")}

<h2 id="privacy">What gets shared</h2>
<p>The demo shows everything below in its consent window before anything is sent.</p>
<ul>
<li><strong>The images</strong>: one image per stream your sensor has (colour, and infrared and depth on a Kinect),
with what the detector saw. They show your cabinet and whatever is around it, possibly people and their faces: check
the preview before accepting, and only share images you have the right to share.</li>
<li><strong>A diagnostics log</strong>: the app version, your operating system, the sensor model, its USB speed and
how many devices share its controller, the frame-rate measurements, your account name, your computer name, and a
random id created for this installation, so one cabinet can be followed from one release to the next.</li>
<li><strong>Sole use</strong>: training and improving the head-tracking model. Captures are never published, sold or
shared with third parties.</li>
<li><strong>Storage</strong>: uploads go to the maintainer's private, write-only server, without an account. You can
also keep your own copy in a folder of your choice.</li>
<li><strong>Removal</strong>: give the exact file name shown after the upload on <a href="{DISCORD}">Discord</a>.</li>
</ul>

<h2>If the upload cannot go through</h2>
<p>A firewall, a proxy or an antivirus that intercepts HTTPS can block the upload. The demo says so in red
<em>before</em> you capture, and keeps the capture in a folder it shows on screen. Send that folder on
<a href="{DISCORD}">Discord</a>: it is worth exactly as much as an upload. <code>headtracking-demo --upload-test</code>
tells you in one line whether this computer can reach the server.</p>

<h2>Other ways to help</h2>
<ul class="features">
<li><strong>Annotate photos</strong>: trace 4 lines per photo in a browser tool, in
<a href="{TREE}tools/anchor">tools/anchor</a>. More cabinets make a model that works everywhere.</li>
<li><strong>Test on Windows or macOS</strong>: builds are published for both, and real-world reports are what is
missing.</li>
<li><strong>VPX and pinball players</strong>: try the plugin on real tables, and tell us how the point of view feels
and what a good cabinet setup needs.</li>
<li><strong>Rust developers</strong>: the VPX plugin, the calibration decoder, the filtering and the mapping to the
VPX view.</li>
<li><strong>Computer vision and machine learning</strong>: the head and anchor models, the vanishing-point and
homography solver, webcam focal recovery.</li>
</ul>
<p>No permission needed: open an issue on <a href="{REPO}/issues">GitHub</a> or say hi on
<a href="{DISCORD}">Discord</a>. <a href="{BLOB}CLAUDE.md">CLAUDE.md</a> is a tour of the architecture, and issues
labelled <em>good first issue</em> are a soft landing.</p>
""")
        return ("Contribuer avec un relevé de votre pincab – headtracking pour Visual Pinball X",
                "Aidez le projet libre de suivi de tête pour VPX : partagez en deux minutes un relevé de votre flipper "
                "virtuel depuis l'application de démonstration, annotez des photos, ou testez sous Windows et macOS.",
                f"""
<h1>Apprenez-lui votre meuble</h1>
<p class="lead">La calibration automatique apprend à reconnaître lockbars et rails latéraux à partir de vrais meubles :
chaque bois, chaque éclairage, chaque angle de caméra. Elle en a encore vu très peu. C'est le principal goulot
d'étranglement du projet, et n'importe quel possesseur de pincab peut aider, en deux minutes, sans coder.</p>

<h2 id="capture">Envoyer un relevé</h2>
<ol class="steps">
<li>Téléchargez <strong>headtracking-demo</strong> depuis la <a href="{RELEASES}">page des versions</a> (il est dans la
même archive que le plugin). Rien à installer.</li>
<li>Pointez votre webcam ou votre Kinect vers le plateau et sélectionnez-la dans la démo.</li>
<li>Cliquez sur <strong>🎁 Contribute</strong>, lisez les conditions, cochez la case de consentement et
confirmez.</li>
</ol>
<p>Meuble vide ou en pleine partie, de jour comme de nuit, chaque variation aide. Le détecteur apprend le meuble :
inutile d'être dans l'image, même si les relevés avec un joueur devant le meuble sont les plus rares. Les minicabs sont
exactement le genre de meuble que le modèle n'a jamais vu.</p>
{fig(img, "anchor-check.webp", 1280, 720, "Vue de la caméra depuis le fronton, avec quatre lignes tracées le long de la lockbar et des rails, et leurs points d'intersection",
     "Ce qui est extrait d'un relevé : les deux bords de la lockbar, les deux rails latéraux et leurs six points d'intersection.")}

<h2 id="privacy">Ce qui est partagé</h2>
<p>La démo affiche tout ce qui suit dans sa fenêtre de consentement, avant le moindre envoi.</p>
<ul>
<li><strong>Les images</strong> : une image par flux de votre capteur (couleur, plus infrarouge et profondeur sur une
Kinect), avec ce que le détecteur a vu. Elles montrent votre meuble et ce qui l'entoure, éventuellement des personnes
et leur visage : vérifiez l'aperçu avant d'accepter, et ne partagez que des images que vous avez le droit de
diffuser.</li>
<li><strong>Un journal de diagnostic</strong> : la version de l'application, votre système d'exploitation, le modèle
du capteur, sa vitesse USB et le nombre d'appareils qui partagent son contrôleur, les mesures d'images par seconde, le
nom de votre compte, le nom de votre ordinateur, et un identifiant aléatoire créé pour cette installation, afin de
suivre un même meuble d'une version à l'autre.</li>
<li><strong>Usage unique</strong> : entraîner et améliorer le modèle de suivi de tête. Les relevés ne sont jamais
publiés, vendus ni partagés avec des tiers.</li>
<li><strong>Stockage</strong> : les envois arrivent sur le serveur privé du mainteneur, en écriture seule, sans compte.
Vous pouvez aussi garder votre propre copie dans le dossier de votre choix.</li>
<li><strong>Suppression</strong> : indiquez sur <a href="{DISCORD}">Discord</a> le nom exact du fichier affiché après
l'envoi.</li>
</ul>

<h2>Si l'envoi ne passe pas</h2>
<p>Un pare-feu, un proxy ou un antivirus qui intercepte le HTTPS peut bloquer l'envoi. La démo l'indique en rouge
<em>avant</em> la capture, et garde le relevé dans un dossier dont elle affiche le chemin. Envoyez ce dossier sur
<a href="{DISCORD}">Discord</a> : il vaut exactement autant qu'un envoi. <code>headtracking-demo --upload-test</code>
indique en une ligne si cet ordinateur arrive à joindre le serveur.</p>

<h2>Autres façons d'aider</h2>
<ul class="features">
<li><strong>Annoter des photos</strong> : tracer 4 lignes par photo dans un outil qui s'ouvre dans le navigateur, dans
<a href="{TREE}tools/anchor">tools/anchor</a>. Plus de meubles, c'est un modèle qui marche partout.</li>
<li><strong>Tester sous Windows ou macOS</strong> : des versions sont publiées pour les deux, et ce sont les retours
d'utilisation réelle qui manquent.</li>
<li><strong>Joueurs de VPX et de flipper</strong> : essayez le plugin sur de vraies tables, et dites-nous ce que vous
pensez du point de vue et ce qu'il faut pour bien régler un meuble.</li>
<li><strong>Développeurs Rust</strong> : le plugin VPX, le décodeur de calibration, le filtrage et la conversion vers la
vue de VPX.</li>
<li><strong>Vision par ordinateur et apprentissage automatique</strong> : les modèles de tête et d'ancre, le calcul des
points de fuite et de l'homographie, la récupération de la focale des webcams.</li>
</ul>
<p>Aucune permission à demander : ouvrez un ticket sur <a href="{REPO}/issues">GitHub</a> ou passez dire bonjour sur
le <a href="{DISCORD}">Discord</a>. <a href="{BLOB}CLAUDE.md">CLAUDE.md</a> fait visiter l'architecture, et les tickets
marqués <em>good first issue</em> permettent de démarrer en douceur.</p>
""")

    if page == "faq":
        items = faq_items(lang)
        body = "\n".join(f'<details class="faq"><summary><h2>{html.escape(q)}</h2></summary>{a}</details>'
                         for q, a in items)
        if en:
            return ("Head tracking pinball FAQ: BAM alternative, Kinect, webcam, VPX setup – headtracking",
                    "Answers about headtracking for Visual Pinball X: preview status, BAM alternative and Kinect "
                    "drivers, webcam or Kinect, calibration, recentering, detection problems, supported systems.",
                    f"""
<h1>Questions and answers</h1>
<p class="lead">Common questions about head tracking in Visual Pinball X with headtracking. Something missing? Ask on
<a href="{DISCORD}">Discord</a>.</p>
{body}
""")
        return ("FAQ suivi de tête flipper virtuel : alternative à BAM, Kinect, webcam, réglage de VPX – headtracking",
                "Réponses sur headtracking pour Visual Pinball X : état de préversion, alternative à BAM et pilotes "
                "Kinect, webcam ou Kinect, calibration, recentrage, problèmes de détection, systèmes pris en charge.",
                f"""
<h1>Questions et réponses</h1>
<p class="lead">Les questions fréquentes sur le suivi de tête dans Visual Pinball X avec headtracking. Il en manque
une ? Posez-la sur <a href="{DISCORD}">Discord</a>.</p>
{body}
""")
    raise KeyError(page)


# ---------------------------------------------------------------- layout

def structured_data(page, lang, title, description):
    app = {
        "@type": "SoftwareApplication",
        "@id": SITE + "#app",
        "name": "headtracking",
        "description": ("Open-source head tracking plugin for Visual Pinball X, from a webcam or a Kinect v1/v2, "
                        "self-calibrating from the cabinet's lockbar and side rails."
                        if lang == "en" else
                        "Plugin libre de suivi de tête pour Visual Pinball X, avec une webcam ou une Kinect v1/v2, "
                        "calibré automatiquement par la lockbar et les rails latéraux du meuble."),
        "applicationCategory": "GameApplication",
        "operatingSystem": "Linux, Windows, macOS",
        "softwareRequirements": "Visual Pinball X 10.8.1 or later",
        "url": SITE,
        "downloadUrl": RELEASES,
        "image": SITE + "img/og.jpg",
        "license": "https://www.gnu.org/licenses/gpl-3.0.html",
        "codeRepository": REPO,
        "isAccessibleForFree": True,
        "offers": {"@type": "Offer", "price": "0", "priceCurrency": "EUR"},
    }
    v = version()
    if v:
        app["softwareVersion"] = v
    graph = [app, {"@type": "WebPage", "name": title, "description": description, "url": url(page, lang),
                   "inLanguage": lang, "about": {"@id": SITE + "#app"}}]
    if page == "faq":
        graph.append({"@type": "FAQPage", "url": url(page, lang), "inLanguage": lang, "mainEntity": [
            {"@type": "Question", "name": q,
             "acceptedAnswer": {"@type": "Answer", "text": html.unescape(re.sub(r"<[^>]+>", "", a)).strip()}}
            for q, a in faq_items(lang)]})
    return {"@context": "https://schema.org", "@graph": graph}


def french_spacing(text):
    """Non-breaking spaces before French double punctuation and inside
    guillemets, so a colon or a closing guillemet never starts a line."""
    text = re.sub(r" ([:;?!»])", "\u00a0\\1", text)
    return text.replace("« ", "«\u00a0")


def render(page, lang):
    up = "../" if lang == "fr" else ""
    title, description, body = content(page, lang, up + "img/")
    u = UI[lang]
    other, other_label, other_code = u["other"]
    nav = "".join(
        f'<a href="{href(n, lang, lang)}"{" aria-current=\"page\"" if n == page else ""}>{u["nav"][n]}</a>'
        for n in PAGES)
    ld = json.dumps(structured_data(page, lang, title, description), ensure_ascii=False).replace("</", "<\\/")
    verification = ('<meta name="google-site-verification" content="' + GOOGLE_VERIFICATION + '">\n'
                    '<meta name="msvalidate.01" content="' + BING_VERIFICATION + '">\n'
                    if page == "index" else "")
    doc = f"""<!doctype html>
<html lang="{lang}">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)}</title>
<meta name="description" content="{html.escape(description)}">
{verification}<link rel="canonical" href="{url(page, lang)}">
<link rel="alternate" hreflang="en" href="{url(page, 'en')}">
<link rel="alternate" hreflang="fr" href="{url(page, 'fr')}">
<link rel="alternate" hreflang="x-default" href="{url(page, 'en')}">
<meta property="og:type" content="website">
<meta property="og:site_name" content="headtracking">
<meta property="og:title" content="{html.escape(title)}">
<meta property="og:description" content="{html.escape(description)}">
<meta property="og:url" content="{url(page, lang)}">
<meta property="og:image" content="{SITE}img/og.jpg">
<meta property="og:image:width" content="1200">
<meta property="og:image:height" content="630">
<meta property="og:image:alt" content="{'Un pincab sous Visual Pinball X avec des caméras de suivi de tête sur le fronton' if lang == 'fr' else 'A pincab running Visual Pinball X with head-tracking cameras on the backbox'}">
<meta property="og:locale" content="{'fr_FR' if lang == 'fr' else 'en_GB'}">
<meta property="og:locale:alternate" content="{'en_GB' if lang == 'fr' else 'fr_FR'}">
<meta name="twitter:card" content="summary_large_image">
<meta name="twitter:title" content="{html.escape(title)}">
<meta name="twitter:description" content="{html.escape(description)}">
<meta name="twitter:image" content="{SITE}img/og.jpg">
<meta name="color-scheme" content="light dark">
<link rel="icon" type="image/svg+xml" href="{up}img/favicon.svg">
<link rel="stylesheet" href="{up}style.css">
<script type="application/ld+json">{ld}</script>
</head>
<body>
<header class="site"><div class="wrap">
<a class="brand" href="{href('index', lang, lang)}"><img src="{up}img/favicon.svg" alt="" width="24" height="24"><span>head<b>tracking</b></span></a>
<nav class="main">{nav}</nav>
<a class="lang" href="{href(page, other, lang)}" hreflang="{other}" lang="{other}"><img src="{up}img/{other_code.lower()}.svg" alt="" width="21" height="14">{other_label}</a>
</div></header>
<main><div class="wrap">
{body.strip()}
</div></main>
<footer class="site"><div class="wrap">
<a href="{REPO}">{u["footer_src"]}</a>
<a href="{DISCORD}">{u["footer_chat"]}</a>
<span>{u["footer_note"]}</span>
</div></footer>
</body>
</html>
"""
    return french_spacing(doc) if lang == "fr" else doc


def main():
    (DOCS / "fr").mkdir(exist_ok=True)
    for lang in ("en", "fr"):
        out = DOCS / ("fr" if lang == "fr" else "")
        for page in PAGES:
            (out / ("index.html" if page == "index" else page + ".html")).write_text(render(page, lang), encoding="utf-8")
    urls = []
    for page in PAGES:
        alts = "".join(f'<xhtml:link rel="alternate" hreflang="{l}" href="{url(page, l)}"/>' for l in ("en", "fr"))
        alts += f'<xhtml:link rel="alternate" hreflang="x-default" href="{url(page, "en")}"/>'
        for lang in ("en", "fr"):
            urls.append(f"<url><loc>{url(page, lang)}</loc>{alts}</url>")
    (DOCS / "sitemap.xml").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9" xmlns:xhtml="http://www.w3.org/1999/xhtml">\n'
        + "\n".join(urls) + "\n</urlset>\n", encoding="utf-8")
    (DOCS / ".nojekyll").write_text("", encoding="utf-8")


if __name__ == "__main__":
    main()
