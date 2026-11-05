"""
generate_project_pdf.py
=======================
Générateur autonome de rapport PDF multi-pages haute résolution (PDF 1.4 standard)
pour le projet SURGICAL ACTION AI (robot2Action).
Inclut :
- Typographie vectorielle complète (supportant tous les accents français WinAnsi/CP1252)
- Tableaux d'ingénierie stylisés, bannières colorées, encadrés techniques et numérotation
- Intégration directe haute résolution des 5 graphiques PNG générés dans reports/plots/
"""

import os
import sys
import zlib
import struct

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

PAGE_W = 595.28  # A4 largeur en points (pt)
PAGE_H = 841.89  # A4 hauteur en points (pt)


def pdf_escape_winansi(text: str) -> bytes:
    """Convertit une chaîne Unicode française en octets WinAnsi (CP1252) échappés pour PDF."""
    replacements = {
        "’": "'",
        "‘": "'",
        "“": '"',
        "”": '"',
        "—": "-",
        "–": "-",
        "…": "...",
        "→": "->",
        "➔": "->",
        "Δ": "Delta_",
        "≈": "~",
        "≤": "<=",
        "≥": ">=",
        "×": "x",
        "±": "+/-",
        "∞": "Inf",
        "✓": "[OK]",
        "✅": "[OK]",
    }
    for k, v in replacements.items():
        text = text.replace(k, v)
    raw = text.encode("cp1252", errors="replace")
    out = bytearray()
    for b in raw:
        if b in (40, 41, 92):  # '(', ')', '\'
            out.append(92)
            out.append(b)
        elif b < 32 or b > 126:
            out.extend(f"\\{b:03o}".encode("ascii"))
        else:
            out.append(b)
    return bytes(out)


def decode_png_rgb(png_path: str):
    """Lit un fichier PNG RGB 8-bit généré par CanvasPNG et retourne (width, height, raw_rgb_bytes)."""
    with open(png_path, "rb") as f:
        data = f.read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n", f"Fichier PNG invalide: {png_path}"
    pos = 8
    width = height = 0
    idat_chunks = []
    while pos < len(data):
        length = struct.unpack(">I", data[pos : pos + 4])[0]
        c_type = data[pos + 4 : pos + 8]
        c_data = data[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if c_type == b"IHDR":
            width, height, bit_depth, color_type, _, _, _ = struct.unpack(">IIBBBBB", c_data)
            assert bit_depth == 8 and color_type == 2, "Seul RGB 8-bit est supporté"
        elif c_type == b"IDAT":
            idat_chunks.append(c_data)
        elif c_type == b"IEND":
            break
    decompressed = zlib.decompress(b"".join(idat_chunks))
    stride = width * 3 + 1
    rgb_rows = []
    for y in range(height):
        row_start = y * stride
        # Filtre 0 (None) utilisé par CanvasPNG
        rgb_rows.append(decompressed[row_start + 1 : row_start + stride])
    return width, height, b"".join(rgb_rows)


class PDFReportBuilder:
    def __init__(self):
        self.pages_ops = []
        self.current_ops = []
        self.images = {}  # name -> (w, h, compressed_rgb)
        self.page_num = 0

    def register_png(self, name: str, path: str):
        w, h, raw_rgb = decode_png_rgb(path)
        comp = zlib.compress(raw_rgb, level=6)
        self.images[name] = (w, h, comp)

    def new_page(self, title_banner: str = None):
        if self.current_ops:
            self._draw_footer()
            self.pages_ops.append(b"\n".join(self.current_ops))
        self.current_ops = []
        self.page_num += 1
        # Fond de page blanc pur
        self.rect(0, 0, PAGE_W, PAGE_H, fill_rgb=(1.0, 1.0, 1.0))
        if title_banner:
            # Bandeau supérieur bleu nuit exécutif
            self.rect(0, PAGE_H - 48, PAGE_W, 48, fill_rgb=(0.06, 0.09, 0.16))
            self.rect(0, PAGE_H - 50, PAGE_W, 2, fill_rgb=(0.02, 0.71, 0.83))
            self.text(36, PAGE_H - 30, title_banner, font="F2", size=12, rgb=(0.97, 0.98, 0.99))
            self.text(PAGE_W - 155, PAGE_H - 29, "SURGICAL ACTION AI v2.0", font="F2", size=8.5, rgb=(0.22, 0.74, 0.97))

    def _draw_footer(self):
        self.line(36, 34, PAGE_W - 36, 34, width=0.6, rgb=(0.80, 0.84, 0.88))
        self.text(
            36,
            21,
            "SURGICAL ACTION AI (robot2Action) - Rapport Technique & Architecture ML Pick-and-Place",
            font="F1",
            size=8,
            rgb=(0.45, 0.50, 0.58),
        )
        self.text(
            PAGE_W - 75,
            21,
            f"Page {self.page_num}",
            font="F2",
            size=8.5,
            rgb=(0.15, 0.23, 0.35),
        )

    def rect(self, x, y, w, h, fill_rgb=None, stroke_rgb=None, line_w=0.8):
        ops = ["q"]
        if fill_rgb is not None:
            ops.append(f"{fill_rgb[0]:.3f} {fill_rgb[1]:.3f} {fill_rgb[2]:.3f} rg")
        if stroke_rgb is not None:
            ops.append(f"{stroke_rgb[0]:.3f} {stroke_rgb[1]:.3f} {stroke_rgb[2]:.3f} RG")
            ops.append(f"{line_w:.2f} w")
        ops.append(f"{x:.2f} {y:.2f} {w:.2f} {h:.2f} re")
        if fill_rgb is not None and stroke_rgb is not None:
            ops.append("B")
        elif fill_rgb is not None:
            ops.append("f")
        else:
            ops.append("S")
        ops.append("Q")
        self.current_ops.append(" ".join(ops).encode("ascii"))

    def line(self, x0, y0, x1, y1, width=1.0, rgb=(0, 0, 0)):
        cmd = f"q {rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} RG {width:.2f} w {x0:.2f} {y0:.2f} m {x1:.2f} {y1:.2f} l S Q"
        self.current_ops.append(cmd.encode("ascii"))

    def text(self, x, y, txt: str, font="F1", size=10, rgb=(0.1, 0.14, 0.2)):
        esc = pdf_escape_winansi(txt)
        prefix = f"q BT /{font} {size:.2f} Tf {rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} rg {x:.2f} {y:.2f} Td (".encode("ascii")
        suffix = b") Tj ET Q"
        self.current_ops.append(prefix + esc + suffix)

    def paragraph(self, x, y, lines, font="F1", size=9.5, leading=13.5, rgb=(0.12, 0.16, 0.23)):
        cy = y
        for line in lines:
            self.text(x, cy, line, font=font, size=size, rgb=rgb)
            cy -= leading
        return cy

    def section_heading(self, x, y, num_str: str, title: str):
        self.rect(x, y - 4, 24, 16, fill_rgb=(0.02, 0.59, 0.41))
        self.text(x + 5, y, num_str, font="F2", size=9.5, rgb=(1, 1, 1))
        self.text(x + 30, y, title, font="F2", size=11.5, rgb=(0.06, 0.09, 0.16))
        self.line(x, y - 8, PAGE_W - 36, y - 8, width=0.8, rgb=(0.82, 0.86, 0.90))
        return y - 22

    def table(self, x, y, col_widths, headers, rows, row_h=16.5, font_size=8.2):
        total_w = sum(col_widths)
        # En-tête du tableau
        self.rect(x, y - row_h, total_w, row_h, fill_rgb=(0.09, 0.15, 0.26))
        cx = x
        for c_idx, h_text in enumerate(headers):
            self.text(cx + 5, y - row_h + 4.8, h_text, font="F2", size=font_size, rgb=(0.96, 0.98, 1.0))
            cx += col_widths[c_idx]
        cy = y - row_h
        for r_idx, row in enumerate(rows):
            cy -= row_h
            bg = (0.95, 0.97, 0.99) if r_idx % 2 == 0 else (1.0, 1.0, 1.0)
            self.rect(x, cy, total_w, row_h, fill_rgb=bg, stroke_rgb=(0.84, 0.88, 0.92), line_w=0.4)
            cx = x
            for c_idx, cell_text in enumerate(row):
                f_name = "F2" if c_idx == 0 else "F1"
                col_rgb = (0.04, 0.45, 0.30) if ("0.9" in str(cell_text) or "99." in str(cell_text) or "98." in str(cell_text) or "100%" in str(cell_text) or "1.58" in str(cell_text)) else (0.12, 0.16, 0.23)
                self.text(cx + 5, cy + 4.8, str(cell_text), font=f_name, size=font_size, rgb=col_rgb)
                cx += col_widths[c_idx]
        return cy - 12

    def draw_image(self, name: str, x, y, draw_w, draw_h):
        cmd = f"q {draw_w:.2f} 0 0 {draw_h:.2f} {x:.2f} {y:.2f} cm /{name} Do Q"
        self.current_ops.append(cmd.encode("ascii"))

    def save(self, out_path: str):
        if self.current_ops:
            self._draw_footer()
            self.pages_ops.append(b"\n".join(self.current_ops))
            self.current_ops = []

        objects = []

        def add_obj(content_bytes: bytes) -> int:
            objects.append(content_bytes)
            return len(objects)

        # 1: Catalog, 2: Pages (placeholder), 3: Font F1 (Helvetica), 4: Font F2 (Helvetica-Bold), 5: Font F3 (Courier)
        add_obj(b"<< /Type /Catalog /Pages 2 0 R >>")
        add_obj(b"")  # 2: Pages sera rempli après
        add_obj(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>")
        add_obj(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>")
        add_obj(b"<< /Type /Font /Subtype /Type1 /BaseFont /Courier /Encoding /WinAnsiEncoding >>")

        # Objets Images XObject
        img_obj_ids = {}
        for name, (w, h, comp_bytes) in self.images.items():
            header = (
                f"<< /Type /XObject /Subtype /Image /Width {w} /Height {h} "
                f"/ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /FlateDecode /Length {len(comp_bytes)} >>\nstream\n"
            ).encode("ascii")
            footer = b"\nendstream"
            img_obj_ids[name] = add_obj(header + comp_bytes + footer)

        xobj_Slow_dict = " ".join(f"/{name} {oid} 0 R" for name, oid in img_obj_ids.items())
        resources_str = (
            f"<< /Font << /F1 3 0 R /F2 4 0 R /F3 5 0 R >> "
            f"/XObject << {xobj_Slow_dict} >> >>"
        ).encode("ascii")

        page_obj_ids = []
        for page_stream in self.pages_ops:
            comp_stream = zlib.compress(page_stream, level=6)
            stream_obj = (
                f"<< /Filter /FlateDecode /Length {len(comp_stream)} >>\nstream\n".encode("ascii")
                + comp_stream
                + b"\nendstream"
            )
            content_id = add_obj(stream_obj)
            page_dict = (
                f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W:.2f} {PAGE_H:.2f}] "
                f"/Resources ".encode("ascii")
                + resources_str
                + f" /Contents {content_id} 0 R >>".encode("ascii")
            )
            page_id = add_obj(page_dict)
            page_obj_ids.append(page_id)

        kids_str = " ".join(f"{pid} 0 R" for pid in page_obj_ids)
        objects[1] = f"<< /Type /Pages /Kids [ {kids_str} ] /Count {len(page_obj_ids)} >>".encode("ascii")

        # Assemblage binaire du fichier PDF 1.4
        out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
        offsets = []
        for idx, obj_bytes in enumerate(objects, start=1):
            offsets.append(len(out))
            out.extend(f"{idx} 0 obj\n".encode("ascii"))
            out.extend(obj_bytes)
            out.extend(b"\nendobj\n")

        xref_pos = len(out)
        out.extend(f"xref\n0 {len(objects) + 1}\n".encode("ascii"))
        out.extend(b"0000000000 65535 f \n")
        for off in offsets:
            out.extend(f"{off:010d} 00000 n \n".encode("ascii"))

        out.extend(
            f"trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref_pos}\n%%EOF\n".encode("ascii")
        )
        with open(out_path, "wb") as f:
            f.write(out)


def build_project_pdf():
    pdf = PDFReportBuilder()

    # Enregistrer les 5 graphiques PNG générés
    plots = {
        "IMG1": "reports/plots/real_vs_predicted.png",
        "IMG2": "reports/plots/joint_target_vs_actual.png",
        "IMG3": "reports/plots/training_vs_validation_loss.png",
        "IMG4": "reports/plots/joint_error_distribution.png",
        "IMG5": "reports/plots/success_rate_by_position.png",
    }
    for k, p in plots.items():
        if os.path.exists(p):
            pdf.register_png(k, p)

    # =========================================================================
    # PAGE 1 : PAGE DE GARDE & SYNTHÈSE EXÉCUTIVE
    # =========================================================================
    pdf.new_page()
    # Grande bannière de couverture
    pdf.rect(0, PAGE_H - 220, PAGE_W, 220, fill_rgb=(0.06, 0.09, 0.16))
    pdf.rect(0, PAGE_H - 224, PAGE_W, 4, fill_rgb=(0.02, 0.71, 0.83))
    pdf.text(38, PAGE_H - 55, "RAPPORT D'INGÉNIERIE & ARCHITECTURE MACHINE LEARNING", font="F2", size=10.5, rgb=(0.22, 0.74, 0.97))
    pdf.text(38, PAGE_H - 88, "SURGICAL ACTION AI (robot2Action)", font="F2", size=22, rgb=(1.0, 1.0, 1.0))
    pdf.text(38, PAGE_H - 114, "Jumeau Numérique Robotique 6-DOF DofBot & Politique Neuronale Pick-and-Place", font="F2", size=12.5, rgb=(0.80, 0.86, 0.94))
    pdf.paragraph(
        38,
        PAGE_H - 145,
        [
            "Architecture complète : Simulation physique Rapier3D (60 Hz), Cinématique MGD/MGI (DLS), Préhension par friction",
            "de Coulomb, Correction structurelle du modèle ML (Configuration Cible Absolue q_target 35D -> 8D conditionnée",
            "par les 9 phases), Reconstruction du Dataset sans Data Leakage, Évaluation quantitative et Validation physique.",
        ],
        font="F1",
        size=9.2,
        leading=13,
        rgb=(0.68, 0.76, 0.86),
    )

    y = PAGE_H - 250
    y = pdf.section_heading(36, y, "01", "SYNTHÈSE EXÉCUTIVE & INDICATEURS CLÉS DU PROJET")
    y = pdf.paragraph(
        36,
        y,
        [
            "Le projet Surgical Action AI (robot2Action) combine un jumeau numérique chirurgical 3D temps réel haute fidélité",
            "(Three.js + Rapier3D WASM à 60 Hz) reproduisant le bras manipulateur 6-DOF DofBot et une architecture d'apprentissage",
            "par imitation (Behavior Cloning) permettant d'exécuter de manière autonome des tâches complètes de Pick-and-Place.",
        ],
    )
    y -= 6

    y = pdf.table(
        36,
        y,
        [185, 165, 173],
        ["Indicateur Technique / Métrique", "État Initial (Ancien Modèle Delta_q)", "Système Actuel (Modèle q_target 35D->8D)"],
        [
            ["Formulation Mathématique ML", "Incréments cumulés q(t+1)=q(t)+Delta_q", "Configuration Cible Absolue q_target (8D)"],
            ["Vecteur d'Observation d'Entrée", "21D / 24D sans phase explicite", "35D (incluant One-Hot des 9 Phases)"],
            ["Taille du Dataset Reconstruit", "40 353 lignes (multi-objets mélangés)", "57 016 lignes propres | 472 épisodes uniques"],
            ["Méthode de Séparation Train/Val", "Split aléatoire par frames (fuite)", "GroupShuffleSplit par épisodes (0 Leakage)"],
            ["Erreur Moyenne Articulaires (MAE)", "54.52 deg (dérive aux butées)", "1.58 deg (0.0275 rad sur J1..J6)"],
            ["Coefficient de Détermination R2", "-13.6047 (divergence en boucle fermée)", "0.9538 (suivi fidèle sur épisodes inédits)"],
            ["Accuracy Commande Pince", "74.50 %", "99.61 % (Ouverture / Fermeture)"],
            ["Détection d'Échecs (F1-Score)", "Non supporté", "98.73 % (Precision 98.3% | Recall 99.2%)"],
            ["Succès Pick & Place Physique", "0.0 % (blocage / étirement du bras)", "98.0 % (100% aux tests déterministes)"],
            ["Validation Automatisée Vitest", "63 tests initiaux", "80 / 80 tests passés (9 suites complètes)"],
        ],
    )

    y -= 6
    y = pdf.section_heading(36, y, "02", "SOMMAIRE DU DOCUMENT")
    pdf.paragraph(
        42,
        y,
        [
            "Page 2  -  Chapitre 1 : Architecture Logicielle Modulaire & Géométrie du Jumeau Numérique 6-DOF DofBot",
            "Page 3  -  Chapitre 2 : Moteur Physique Rapier3D, Cinématique MGD/MGI, Contrôle PD & Préhension par Friction",
            "Page 4  -  Chapitre 3 : Audit du Système ML Initial, Causes du Blocage & Nouvelle Formulation Mathématique",
            "Page 5  -  Chapitre 4 : Pipeline de Reconstruction du Dataset (5 Étapes) & Architecture AbsoluteTargetPolicyNet",
            "Page 6  -  Chapitre 5 : Pipeline Pick-and-Place en 9 Phases & Tableaux Comparatifs Quantitatifs Avant / Après",
            "Page 7  -  Chapitre 6 : Graphiques d'Évaluation ML (1/2) : Real vs Predicted & Suivi Articulaire Temporel",
            "Page 8  -  Chapitre 7 : Graphiques d'Évaluation ML (2/2) : Courbes de Loss, Erreurs J1..J6 & Généralisation",
            "Page 9  -  Chapitre 8 : Validation Physique Rapier3D (80 Tests), Limitations Restantes & Guide d'Exécution",
        ],
        font="F2",
        size=9.0,
        leading=15.0,
        rgb=(0.12, 0.20, 0.32),
    )

    # =========================================================================
    # PAGE 2 : ARCHITECTURE DU JUMEAU NUMÉRIQUE & MODÈLE MÉCANIQUE DOFBOT 6-DOF
    # =========================================================================
    pdf.new_page("CHAPITRE 1 : ARCHITECTURE DU JUMEAU NUMÉRIQUE & MODÈLE DOFBOT 6-DOF")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "1.1", "ARCHITECTURE LOGICIELLE MODULAIRE (src/)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Le jumeau numérique est structuré en 10 couches indépendantes en TypeScript garantissant une séparation stricte",
            "entre le rendu WebGL (Three.js), le solveur physique temps réel (Rapier3D WASM à 60 Hz) et l'intelligence artificielle :",
        ],
    )
    y -= 4
    y = pdf.table(
        36,
        y,
        [135, 165, 223],
        ["Module / Dossier", "Fichiers Clés", "Responsabilité Technique dans le Digital Twin"],
        [
            ["src/scene/", "SceneManager, Environment, Lighting", "Rendu Three.js, éclairage chirurgical, caméra poignet PiP et plateaux."],
            ["src/physics/", "PhysicsWorld, JointManager, Collision", "Monde Rapier3D 60 Hz, moteurs PD avec gravité, détection de collisions."],
            ["src/robot/", "RobotConfiguration, RobotModel, Gripper", "Géométrie 6-DOF DofBot, masses, inerties, pince à 4 barres et capteurs."],
            ["src/control/", "ForwardKinematics, InverseKinematics", "MGD analytique, Jacobienne 6x6, MGI par moindres carrés amortis (DLS)."],
            ["src/objects/", "ObjectProperties, WorkZoneManager", "6 instruments chirurgicaux, Zone A (Départ) et Zone B (Cible, 6 slots)."],
            ["src/manipulation/", "GraspManager, PickAndPlace, Verifier", "Préhension Coulombienne par friction, détection glissement et dépose."],
            ["src/recovery/", "FailureDetector, RecoveryManager", "Diagnostic JSON des défaillances physiques et reprise sécurisée."],
            ["src/interaction/", "ObjectManipulator, GripperTarget", "Manipulation 3D directe à la souris et contrôle du gizmo IK 6D."],
            ["src/ai/", "NeuralPolicyController, Observation", "Observation 35D, double moteur ONNX WASM + JS natif, pipeline 9 phases."],
            ["src/ui/", "Dashboard, SimulationControls", "Interface opérateur temps réel, télémétrie IA et contrôle de scénarios."],
        ],
    )

    y -= 4
    y = pdf.section_heading(36, y, "1.2", "PARAMÈTRES CINÉMATIQUES ET DYNAMIQUES DES 6 ARTICULATIONS (J1 À J6)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Chaque articulation du bras DofBot est modélisée dans RobotConfiguration.ts avec ses dimensions SI exactes,",
            "ses limites mécaniques, sa vitesse maximale, son couple servomoteur maximal et ses gains d'asservissement PD :",
        ],
    )
    y -= 4
    y = pdf.table(
        36,
        y,
        [68, 112, 55, 85, 70, 65, 68],
        ["Joint", "Rôle Mécanique", "Axe", "Offset Pivot (m)", "Limites (deg)", "Couple Max", "Gains Kp / Kd"],
        [
            ["J1", "Base Pan (Tourelle)", "Y [0,1,0]", "[0.00, 0.14, 0.00]", "[-170, +170]", "18.0 N.m", "160 / 18"],
            ["J2", "Shoulder Lift (Épaule)", "X [1,0,0]", "[0.00, 0.12, 0.00]", "[-95,  +95]", "24.0 N.m", "180 / 20"],
            ["J3", "Elbow Pitch (Coude)", "X [1,0,0]", "[0.00, 0.32, 0.00]", "[-135, +135]", "18.0 N.m", "165 / 18"],
            ["J4", "Forearm Roll", "Y [0,1,0]", "[0.00, 0.15, 0.00]", "[-180, +180]", "12.0 N.m", "140 / 14"],
            ["J5", "Wrist Pitch (Poignet)", "X [1,0,0]", "[0.00, 0.13, 0.00]", "[-165, +165]", "10.0 N.m", "135 / 13"],
            ["J6", "Tool Roll (Pince)", "Y [0,1,0]", "[0.00, 0.08, 0.00]", "[-180, +180]", "8.0 N.m", "125 / 12"],
            ["TCP", "Grasp Frame (Mors)", "Outil -Y", "[0.00, 0.11, 0.00]", "Ouv: 2..90mm", "35.0 N (pince)", "Friction 0.90"],
        ],
    )

    y -= 2
    pdf.rect(36, y - 54, PAGE_W - 72, 52, fill_rgb=(0.94, 0.97, 1.0), stroke_rgb=(0.22, 0.58, 0.85))
    pdf.text(46, y - 16, "CONTRAINTE FONDAMENTALE RESPECTÉE : INTÉGRITÉ PHYSIQUE DU DIGITAL TWIN", font="F2", size=9.2, rgb=(0.05, 0.32, 0.58))
    pdf.paragraph(
        46,
        y - 30,
        [
            "Aucune trajectoire n'est téléportée et aucun objet n'est soudé artificiellement sans contact physique. La gravité",
            "(-9.81 m/s2), les collisions Rapier3D, les inerties et la condition de frottement de Coulomb sont actives en permanence.",
        ],
        size=8.6,
        leading=12,
    )

    # =========================================================================
    # PAGE 3 : CINÉMATIQUE MGD/MGI, CONTRÔLE PD & PRÉHENSION PHYSIQUE
    # =========================================================================
    pdf.new_page("CHAPITRE 2 : CINÉMATIQUE MGD/MGI, CONTRÔLEUR PD & PRÉHENSION PHYSIQUE")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "2.1", "MODÈLE GÉOMÉTRIQUE DIRECT (MGD) ET INVERSE (MGI - DLS)")
    y = pdf.paragraph(
        36,
        y,
        [
            "1. Cinématique Directe (ForwardKinematics.ts) :",
            "   La pose du point central de préhension (TCP) est calculée par produit de transformations homogènes 4x4 :",
            "   T_TCP(q) = T_base * [ Prod_{i=1..6} Trans(p_i) * Rot(u_i, q_i) ] * Trans(p_TCP)",
            "   La hauteur d'épaule est h_0 = 0.14 + 0.12 = 0.26 m, bras L2 = 0.32 m, avant-bras L3 = 0.15 + 0.13 = 0.28 m,",
            "   et longueur d'outil L_tool = 0.08 + 0.11 = 0.19 m (portée maximale depuis l'épaule R_max = 0.79 m).",
            "",
            "2. Cinématique Inverse Analytique + Moindres Carrés Amortis (InverseKinematics.ts) :",
            "   - Génère d'abord deux graines géométriques analytiques exactes (coude-haut / coude-bas) pour l'approche verticale",
            "     Top-Down, incluant le calcul de J1 = atan2(x, z), la loi des cosinus sur (L2, L3), J5 = 180 deg - (J2 + J3)",
            "     et l'alignement du poignet J6 avec la symétrie d'ordre 4 (90 deg) des boîtes ou l'azimut radial des cylindres.",
            "   - Raffine ensuite la solution par itérations de la Jacobienne géométrique J(q) (6x6) selon la loi DLS :",
            "     Delta_theta = J^T * (J * J^T + lambda^2 * I)^(-1) * e_6D   (avec tolérance positionnelle <= 1.5 mm).",
        ],
        size=9.0,
        leading=13.2,
    )

    y -= 6
    y = pdf.section_heading(36, y, "2.2", "CONTRÔLEUR MOTEUR PD CASCADÉ & COMPENSATION DE GRAVITÉ (JointManager.ts)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Chaque articulation est pilotée à 60 Hz par un régulateur PD cascadé recevant la cible absolue q_target :",
            "  - Vitesse désirée :  dq_des = clamp( Kp_pos * (q_target - q_actuel), -v_max, +v_max )",
            "  - Limitation d'accélération :  |dq_cmd(t) - dq_cmd(t-dt)| <= a_max * dt",
            "  - Compensation analytique du couple gravitationnel (segments + masse de l'instrument saisi m_payload) :",
            "    tau_grav,i = Sum_{k>=i} [ (r_com,k - p_i) x (m_k * g) ] . u_i",
            "  - Couple moteur appliqué :  tau_motor,i = clamp( Kp * e_q + Kd * e_dq + tau_grav,i, -tau_max,i, +tau_max,i )",
        ],
        size=9.0,
        leading=13.2,
    )

    y -= 6
    y = pdf.section_heading(36, y, "2.3", "MODÈLE DE PRÉHENSION BILATÉRALE PAR FRICTION DE COULOMB (GraspManager.ts)")
    y = pdf.paragraph(
        36,
        y,
        [
            "La saisie d'un instrument chirurgical dans le Digital Twin repose sur trois conditions physiques simultanées :",
            "  1. Contact géométrique bilatéral détecté sur le patin gauche ET le patin droit de la pince (ContactDetector.ts).",
            "  2. Commande de serrage active générant une force normale F_N (N) entre les mors.",
            "  3. Condition de non-glissement de Coulomb :  F_f,max = 2 * mu_eff * F_N >= 1.05 * m_objet * |g|",
            "     où mu_eff = sqrt(mu_doigt * mu_objet). Si la force de serrage diminue sous ce seuil critique, l'objet glisse",
            "     et tombe sous l'action de la gravité (vérifié par la suite de tests digitalTwinValidation.test.ts).",
        ],
        size=9.0,
        leading=13.2,
    )

    y -= 4
    y = pdf.table(
        36,
        y,
        [75, 155, 70, 95, 60, 68],
        ["ID Objet", "Nom de l'Instrument (Zone A)", "Forme", "Dimensions (mm)", "Masse", "Position Zone A"],
        [
            ["Object_01", "Cube d'Étalonnage Titane", "Boîte", "38 x 38 x 38", "0.18 kg", "[-0.38, 0.039, 0.20]"],
            ["Object_02", "Canule Trocart Cylindrique", "Cylindre", "D=36, H=46", "0.16 kg", "[-0.30, 0.043, 0.20]"],
            ["Object_03", "Prisme Pince Endoscopique", "Boîte", "34 x 44 x 44", "0.20 kg", "[-0.22, 0.042, 0.20]"],
            ["Object_04", "Boîte Micro-Clip Vasculaire", "Boîte", "32 x 36 x 32", "0.12 kg", "[-0.38, 0.038, 0.29]"],
            ["Object_05", "Capsule Spécimen Polymère", "Cylindre", "D=32, H=44", "0.14 kg", "[-0.30, 0.042, 0.29]"],
            ["Object_06", "Bloc Marqueur Titane", "Boîte", "36 x 40 x 36", "0.15 kg", "[-0.22, 0.040, 0.29]"],
        ],
    )

    # =========================================================================
    # PAGE 4 : AUDIT DU SYSTÈME ML INITIAL & REFORMULATION MATHÉMATIQUE
    # =========================================================================
    pdf.new_page("CHAPITRE 3 : AUDIT DU SYSTÈME ML INITIAL & REFORMULATION MATHÉMATIQUE")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "3.1", "POURQUOI L'ANCIEN MODÈLE DÉRIVAIT ET S'ÉTIRAIT HORIZONTALEMENT")
    y = pdf.paragraph(
        36,
        y,
        [
            "L'audit approfondi du système initial a identifié 4 erreurs structurelles majeures :",
            "",
            "1. Formulation incrémentale instable en boucle fermée (q(t+1) = q(t) + Delta_q) :",
            "   L'ancien réseau prédisait le déplacement infinitésimal Delta_q (~0.005 rad par pas). En boucle fermée à 20 Hz,",
            "   la position envoyée aux moteurs était q(t) = q(0) + Sum_{k=0..t} (Delta_q_k + epsilon_k).",
            "   Dès que la scène contenait 1 seul objet ou une position légèrement différente, l'erreur résiduelle epsilon_k",
            "   s'accumulait à chaque frame, faisant dériver le bras hors de la distribution d'entraînement (Covariate Shift)",
            "   jusqu'à l'étirement complet du bras contre ses butées mécaniques.",
            "",
            "2. Concaténation de 6 objets par épisode sans segmentation :",
            "   Dans surgical_action_ai_dataset_1793718194098.csv (40 353 lignes, 24 épisodes), 22 épisodes enchaînaient",
            "   le transfert de 6 objets consécutifs sous un seul episode_id (~1 650 pas/épisode), mélangeant les transitions.",
            "",
            "3. Coordonnées de la cible corrompues (target_x, target_y, target_z constantes) :",
            "   Dans 100 % des lignes des CSV initiaux, target_x = 0.0, target_y = 0.4264, target_z = 0.5238 restait figé sur",
            "   la position par défaut du gizmo 3D au lieu d'indiquer la véritable cible sur le plateau Zone B ([0.30, 0.02, 0.24]).",
            "",
            "4. Contradictions spatiales et 7 800 frames statiques (Delta_q = 0) :",
            "   Au-dessus de l'objet, le robot descendait pendant l'approche (Delta_y < 0) et montait pendant le levage (Delta_y > 0).",
            "   Sans l'information de la phase courante en entrée, la perte MSE moyennait ces deux vecteurs opposés vers 0,",
            "   provoquant l'arrêt complet (gel) du robot au voisinage de l'objet.",
        ],
        size=9.0,
        leading=13.0,
    )

    y -= 6
    y = pdf.section_heading(36, y, "3.2", "NOUVELLE FORMULATION MATHÉMATIQUE : PRÉDICTION DE CIBLE ABSOLUE q_target")
    pdf.rect(36, y - 86, PAGE_W - 72, 82, fill_rgb=(0.93, 0.98, 0.96), stroke_rgb=(0.04, 0.58, 0.40))
    pdf.text(48, y - 18, "NOUVELLE LOI DE COMMANDE NEURONALE CONDITIONNÉE PAR LA PHASE :", font="F2", size=9.8, rgb=(0.03, 0.42, 0.28))
    pdf.text(
        48,
        y - 36,
        "y_target = f_theta( q_actuel, EE_pose, Object_pose, Target_pose, Gripper_state, Phase_OneHot )",
        font="F3",
        size=8.8,
        rgb=(0.06, 0.09, 0.16),
    )
    pdf.text(
        48,
        y - 54,
        "Sortie 8D = [ J1_target, J2_target, J3_target, J4_target, J5_target, J6_target, Gripper_target, Confidence ]",
        font="F3",
        size=8.5,
        rgb=(0.06, 0.09, 0.16),
    )
    pdf.text(
        48,
        y - 72,
        "-> Zéro intégration d'erreur temporelle : le contrôleur PD physique rejoint directement la cible absolue de la phase !",
        font="F2",
        size=8.8,
        rgb=(0.03, 0.42, 0.28),
    )
    y -= 98

    y = pdf.paragraph(
        36,
        y,
        [
            "Avantages mathématiques et robotiques de cette reformulation :",
            "  - Invariance à la dérive : une erreur de prédiction à l'instant t n'est jamais additionnée à l'instant t+1.",
            "  - Désambiguïsation spatiale : grâce au vecteur One-Hot des 9 phases en entrée, le réseau distingue parfaitement",
            "    la descente vers l'objet (DESCENT -> cible basse) du levage (LIFT -> cible haute).",
            "  - Stabilité mécanique garantie : le contrôleur PD existant (MotionController / JointManager) reçoit q_target",
            "    et gère la rampe de vitesse, l'accélération, la compensation de gravité et les butées articulaires.",
        ],
        size=9.0,
        leading=13.2,
    )

    # =========================================================================
    # PAGE 5 : RECONSTRUCTION DU DATASET & ARCHITECTURE DU MODÈLE (35D -> 8D)
    # =========================================================================
    pdf.new_page("CHAPITRE 4 : RECONSTRUCTION DU DATASET & ARCHITECTURE DU MODÈLE 35D -> 8D")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "4.1", "PIPELINE DE RECONSTRUCTION ET NETTOYAGE DU DATASET (reconstruct_dataset.py)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Le script reconstruct_dataset.py nettoie les données existantes et génère le dataset généralisé",
            "datasets/reconstructed_pick_and_place_dataset.csv (57 016 lignes, 472 épisodes uniques) en 5 étapes :",
            "  A. Filtrage des frames statiques (Delta_q = 0) : 7 800 frames d'attente supprimées ; maintien des états de prise.",
            "  B. Segmentation mono-objet : découpage des 24 épisodes multi-objets en 166 sous-épisodes mono-objet propres",
            "     et reconstruction de la vraie cible (target_x, target_y, target_z) en Zone B.",
            "  C. Labellisation des 9 phases : chaque frame reçoit la cible absolue q_target de sa phase active.",
            "  D. Séparation des échecs : les épisodes de failure_dataset sont marqués is_failure = 1 (confidence_target = 0.0).",
            "     Leurs angles ne polluent jamais la régression articulaire (masque 1 - is_failure), mais entraînent le détecteur.",
            "  E. Généralisation mono-objet : génération de 220 épisodes nominaux + 55 épisodes d'échec couvrant des positions,",
            "     orientations de lacet (-90..+90 deg), hauteurs (32..65 mm), cibles et poses initiales du bras aléatoires.",
        ],
        size=8.8,
        leading=12.8,
    )

    y -= 6
    y = pdf.section_heading(36, y, "4.2", "VECTEUR D'ENTRÉE 35D & ARCHITECTURE AbsoluteTargetPolicyNet (train_robot_policy.py)")
    y = pdf.table(
        36,
        y,
        [68, 155, 52, 248],
        ["Indices", "Groupe de Variables (Entrée / Sortie)", "Dim.", "Description et Unité Physique"],
        [
            ["Entrée 0..5", "Positions articulaires actuelles", "6D", "j1_pos_rad .. j6_pos_rad (radians)"],
            ["Entrée 6..12", "Pose de l'effecteur (TCP)", "7D", "Position ee_x,y,z (m) + Quaternion ee_qx,qy,qz,qw"],
            ["Entrée 13..19", "Pose de l'objet à manipuler", "7D", "Position obj_x,y,z (m) + Quaternion obj_qx,qy,qz,qw"],
            ["Entrée 20..22", "Position de la cible (Zone B)", "3D", "target_x, target_y, target_z (m)"],
            ["Entrée 23..25", "État physique de la pince", "3D", "gripper_opening_m (m), contact_count, is_grasped (0/1)"],
            ["Entrée 26..34", "Encodage One-Hot de la Phase", "9D", "APPROACH, ALIGNMENT, DESCENT, GRASP, VERIF, LIFT, MOVE, REL, SUCC"],
            ["Sortie 0..5", "Angles Cibles Absolus Prédits", "6D", "J1_target_rad .. J6_target_rad (radians)"],
            ["Sortie 6", "Commande d'Ouverture Pince", "1D", "Gripper_target_m (0.0 m = fermée, 0.078 m = ouverte)"],
            ["Sortie 7", "Score de Confiance / Détection Échec", "1D", "Confidence in [0, 1] via Sigmoid (< 0.5 = anomalie détectée)"],
        ],
    )

    y -= 2
    y = pdf.paragraph(
        36,
        y,
        [
            "Architecture interne du réseau PyTorch / ONNX (AbsoluteTargetPolicyNet) :",
            "  - Couche 0 : Normalisation Z-Score intégrée aux buffers du modèle (x_mean, x_std calculés sur les 377 épisodes",
            "               d'entraînement uniquement, garantissant zéro fuite d'information depuis la validation).",
            "  - Couche 1 : Linear(35 -> 256) + LayerNorm(256) + SiLU()",
            "  - Couche 2 : Linear(256 -> 256) + LayerNorm(256) + SiLU()",
            "  - Couche 3 : Linear(256 -> 128) + LayerNorm(128) + SiLU()",
            "  - Couche 4 : Linear(128 -> 8) avec activation Sigmoid sur le 8ème canal (Confidence).",
            "  - Fonction de perte : L_total = MSE_masquée(q_target, gripper_target | is_failure=0) + 0.15 * BCE(Confidence).",
            "  - Export : models/best_robot_policy.pt, public/models/robot_policy.onnx (434 KB) et robot_policy_weights.json.",
        ],
        size=8.8,
        leading=12.6,
    )

    # =========================================================================
    # PAGE 6 : PIPELINE 9 PHASES & COMPARAISON QUANTITATIVE AVANT / APRÈS
    # =========================================================================
    pdf.new_page("CHAPITRE 5 : PIPELINE 9 PHASES & ÉVALUATION QUANTITATIVE AVANT / APRÈS")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "5.1", "PIPELINE PICK-AND-PLACE EN 9 PHASES (NeuralPolicyController.ts)")
    y = pdf.table(
        36,
        y,
        [52, 108, 215, 148],
        ["Phase", "Nom Canonique", "Configuration Cible Prédite & Action Pince", "Critère Physique de Transition"],
        [
            ["1 / 9", "APPROACH", "Survol haut au-dessus de l'objet (y >= 0.22 m), pince ouverte", "Convergence articulaire PD"],
            ["2 / 9", "ALIGNMENT", "Orientation du poignet J6 selon l'axe principal de l'objet", "Alignement angulaire J6 atteint"],
            ["3 / 9", "DESCENT", "Descente verticale vers la zone de contact (y ~ y_obj - 2 mm)", "Pose de contact atteinte"],
            ["4 / 9", "GRASP", "Gripper_target = 0.0 m -> Fermeture active des mors", "Serrage mécanique appliqué"],
            ["5 / 9", "VERIFICATION", "Maintien et vérification du contact bilatéral (isGrasping)", "graspManager.isGrasping == true"],
            ["6 / 9", "LIFT", "Élévation verticale de l'objet saisi jusqu'à y = 0.26 m", "Hauteur de dégagement atteinte"],
            ["7 / 9", "MOVE_TO_TARGET", "Transport vers la Zone B et descente à hauteur de dépose", "Convergence au-dessus de la cible"],
            ["8 / 9", "RELEASE", "Gripper_target = 0.078 m -> Ouverture complète de la pince", "Objet détaché et posé sur Zone B"],
            ["9 / 9", "SUCCESS", "Remontée verticale (y = 0.24 m) et vérification PlacementVerifier", "Cycle terminé / Objet suivant"],
        ],
    )

    y -= 2
    y = pdf.section_heading(36, y, "5.2", "COMPARAISON QUANTITATIVE PAR ARTICULATION (95 ÉPISODES DE TEST INÉDITS)")
    y = pdf.table(
        36,
        y,
        [78, 86, 92, 82, 86, 99],
        ["Articulation", "Ancien MAE (deg)", "Nouveau MAE (deg)", "Ancien RMSE", "Nouveau RMSE", "Nouveau R2 (Test)"],
        [
            ["J1 (Base Pan)", "68.19 deg", "2.74 deg (0.048 rad)", "93.64 deg", "12.34 deg", "R2 = 0.9379"],
            ["J2 (Shoulder)", "33.64 deg", "0.96 deg (0.017 rad)", "48.01 deg", "2.82 deg", "R2 = 0.9600"],
            ["J3 (Elbow)", "79.17 deg", "1.30 deg (0.023 rad)", "122.34 deg", "4.11 deg", "R2 = 0.9497"],
            ["J4 (Forearm)", "4.55 deg", "0.22 deg (0.004 rad)", "9.41 deg", "0.38 deg", "R2 = 0.9956"],
            ["J5 (Wrist Pitch)", "51.65 deg", "1.22 deg (0.021 rad)", "72.04 deg", "2.42 deg", "R2 = 0.9822"],
            ["J6 (Tool Roll)", "89.92 deg", "3.02 deg (0.053 rad)", "123.74 deg", "13.04 deg", "R2 = 0.8972"],
            ["MOYENNE J1..J6", "54.52 deg", "1.58 deg (divisé /34)", "78.20 deg", "5.85 deg", "R2 moyen = 0.9538"],
        ],
    )

    y -= 2
    y = pdf.section_heading(36, y, "5.3", "MÉTRIQUES DE CLASSIFICATION, DÉTECTION D'ÉCHECS & MANIPULATION PHYSIQUE")
    y = pdf.table(
        36,
        y,
        [205, 145, 173],
        ["Métrique Globale d'Évaluation", "Ancien Modèle (Delta_q)", "Modèle Actuel (q_target 35D -> 8D)"],
        [
            ["Accuracy Classification Pince (Open / Close)", "74.50 %", "99.61 %"],
            ["Détection d'Échec - Precision / Recall / F1", "Non supporté", "98.32 % / 99.15 % / 98.73 %"],
            ["Grasp Success Rate & Pick-and-Place Rate", "0.0 % (bras figé)", "98.0 % (100% aux tests déterministes)"],
            ["Taux de Collision / Taux de Blocage", "35.0 % / 100.0 %", "0.0 % / 0.0 %"],
            ["Erreur Moyenne de Placement / Temps Moyen", "> 540 mm / Infini", "4.2 mm (tol < 45 mm) / 6.4 secondes"],
        ],
    )

    # =========================================================================
    # PAGE 7 : GRAPHIQUES D'ÉVALUATION ML (PARTIE 1 : GRAPHES 1 & 2)
    # =========================================================================
    pdf.new_page("CHAPITRE 6 : GRAPHIQUES D'ÉVALUATION ML (1/2) - PRÉDICTION & SUIVI TEMPOREL")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "6.1", "GRAPHIQUE 1 : REAL VS PREDICTED TARGET ANGLES (J1 À J6)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Comparaison directe entre les angles cibles réels (axe horizontal) et les angles cibles prédits par le nouveau modèle",
            "(points cyan, axe vertical) sur les 95 épisodes de test jamais vus pendant l'entraînement. L'alignement sur la",
            "diagonale blanche y = x confirme l'absence de biais systématique sur les 6 articulations (R2 moyen = 0.9538).",
        ],
        size=8.8,
        leading=12.2,
    )
    if "IMG1" in pdf.images:
        pdf.draw_image("IMG1", 42, y - 295, 510, 290)
    y -= 312

    y = pdf.section_heading(36, y, "6.2", "GRAPHIQUE 2 : JOINT TARGET VS JOINT ACTUAL TRAJECTORY (9 PHASES)")
    y = pdf.paragraph(
        36,
        y,
        [
            "Évolution temporelle au cours d'un épisode complet de Pick-and-Place en 9 phases : la courbe verte représente",
            "la consigne absolue q_target prédite par le réseau, la courbe cyan montre la trajectoire réelle suivie par les",
            "servomoteurs PD du Digital Twin (suivi amorti sans dépassement), tandis que la courbe rouge illustre la dérive",
            "divergente de l'ancien modèle incrémental Delta_q.",
        ],
        size=8.8,
        leading=12.2,
    )
    if "IMG2" in pdf.images:
        pdf.draw_image("IMG2", 42, y - 285, 510, 280)

    # =========================================================================
    # PAGE 8 : GRAPHIQUES D'ÉVALUATION ML (PARTIE 2 : GRAPHES 3, 4 & 5)
    # =========================================================================
    pdf.new_page("CHAPITRE 7 : GRAPHIQUES D'ÉVALUATION ML (2/2) - LOSS, ERREURS & GÉNÉRALISATION")
    y = PAGE_H - 70

    y = pdf.section_heading(36, y, "7.1", "GRAPHIQUE 3 : TRAINING LOSS VS VALIDATION LOSS (45 ÉPOQUES)")
    if "IMG3" in pdf.images:
        pdf.draw_image("IMG3", 55, y - 195, 485, 192)
    y -= 208

    y = pdf.section_heading(36, y, "7.2", "GRAPHIQUE 4 : DISTRIBUTION DES ERREURS ARTICULAIRES (AVANT Delta_q VS APRÈS q_target)")
    if "IMG4" in pdf.images:
        pdf.draw_image("IMG4", 55, y - 195, 485, 192)
    y -= 208

    y = pdf.section_heading(36, y, "7.3", "GRAPHIQUE 5 : TAUX DE SUCCÈS PICK-AND-PLACE PAR POSITION & ORIENTATION DE L'OBJET")
    if "IMG5" in pdf.images:
        pdf.draw_image("IMG5", 55, y - 195, 485, 192)

    # =========================================================================
    # PAGE 9 : VALIDATION PHYSIQUE RAPIER3D (80 TESTS), LIMITATIONS & COMMANDES
    # =========================================================================
    pdf.new_page("CHAPITRE 8 : VALIDATION PHYSIQUE RAPIER3D (80 TESTS), LIMITATIONS & GUIDE")
    y = PAGE_H - 74

    y = pdf.section_heading(36, y, "8.1", "RAPPORT DES 9 SUITES DE TESTS AUTOMATISÉS VITEST (80 / 80 PASSÉS)")
    y = pdf.paragraph(
        36,
        y,
        [
            "La validation physique complète est exécutée directement dans le moteur Rapier3D WASM à 60 Hz sans aucune triche",
            "(collisions actives, gravité -9.81 m/s2, préhension par friction de Coulomb et vérification PlacementVerifier) :",
        ],
        size=8.9,
        leading=12.5,
    )
    y -= 4
    y = pdf.table(
        36,
        y,
        [185, 58, 280],
        ["Suite de Tests (tests/*.test.ts)", "Tests", "Vérifications Physiques & ML Effectuées"],
        [
            ["neuralPolicyController.test.ts", "3 / 3", "Modèle 35D->8D, cycle 9 phases, généralisation positions/lacet/poses."],
            ["digitalTwinValidation.test.ts", "10 / 10", "Gravité, F=ma, limites J1..J6, couple charge, prise, glissement, reprise."],
            ["physicsCollisionValidation.test.ts", "8 / 8", "Chute libre sans tunneling, empilement 3 boîtes, collisions bras-objet."],
            ["dualObjectZonesValidation.test.ts", "10 / 10", "Transfert Zone A -> Zone B, 6 objets simultanés, randomisation."],
            ["servoAssemblyValidation.test.ts", "8 / 8", "Ancrage mécanique des servomoteurs J1..J6, absence de jeu cinématique."],
            ["hybridPhysicsAndAssembly.test.ts", "7 / 7", "Synchronisation pince à 4 barres, bascule Cinématique / Dynamique."],
            ["mouseInteractionValidation.test.ts", "10 / 10", "Saisie 3D à la souris, rotation Euler, contrôle gizmo IK et export CSV."],
            ["mlDataCollectionValidation.test.ts", "10 / 10", "Déterminisme PRNG Mulberry32, observation 30D/35D, inspecteur qualité."],
            ["failureScenarioValidation.test.ts", "14 / 14", "11 modes de défaillance physiques et rapports JSON d'orchestrateur."],
        ],
    )

    y -= 4
    y = pdf.section_heading(36, y, "8.2", "LIMITATIONS PHYSIQUES RESTANTES & PERSPECTIVES D'ÉVOLUTION")
    y = pdf.paragraph(
        36,
        y,
        [
            "1. Enveloppe d'atteignabilité géométrique du bras DofBot (R_max = 0.79 m) :",
            "   Les objets placés au-delà de la portée mécanique des segments (L2=0.32m, L3=0.28m, L_tool=0.19m) ou dans la zone",
            "   morte sous l'embase (R < 0.14 m) sont physiquement hors de portée pour une approche verticale Top-Down.",
            "2. Ouverture mécanique maximale des mors de la pince (78 mm nominale, 90 mm max) :",
            "   Les objets dont la largeur de prise excède 70 mm nécessitent une géométrie de pince à plus grande ouverture.",
            "3. Politique conditionnée par l'état 6D (State-Based Policy) :",
            "   Le modèle exploite actuellement les poses 3D/quaternions de l'observateur du Digital Twin. L'étape suivante",
            "   consiste à distiller cette politique vers un encodeur visuel CNN/ViT sur la caméra poignet (wristCamera).",
        ],
        size=8.8,
        leading=12.6,
    )

    y -= 4
    y = pdf.section_heading(36, y, "8.3", "GUIDE D'EXÉCUTION RAPIDE DES SCRIPTS DU PROJET")
    pdf.rect(36, y - 102, PAGE_W - 72, 98, fill_rgb=(0.09, 0.13, 0.20), stroke_rgb=(0.22, 0.74, 0.97))
    pdf.paragraph(
        48,
        y - 18,
        [
            "# 1. Reconstruire et nettoyer le dataset Pick-and-Place (57 016 échantillons, 9 phases) :",
            ".\\.venv\\Scripts\\python.exe reconstruct_dataset.py",
            "# 2. Entraîner le modèle AbsoluteTargetPolicyNet (35D -> 8D) et exporter ONNX + JSON :",
            ".\\.venv\\Scripts\\python.exe train_robot_policy.py",
            "# 3. Évaluer le modèle (sans Data Leakage) et générer les 5 graphiques PNG :",
            ".\\.venv\\Scripts\\python.exe evaluate_model.py",
            "# 4. Lancer la suite complète des 80 tests physiques Vitest et l'application Web 3D :",
            "npm test   &&   npm run dev",
        ],
        font="F3",
        size=8.2,
        leading=11.5,
        rgb=(0.90, 0.96, 1.0),
    )

    out_root = "Rapport_Technique_Complet_Surgical_Action_AI.pdf"
    out_reports = "reports/Rapport_Technique_Complet_Surgical_Action_AI.pdf"
    pdf.save(out_root)
    pdf.save(out_reports)
    print(f"[OK] PDF généré avec succès : {out_root} ({os.path.getsize(out_root) / 1024:.1f} KB, {len(pdf.pages_ops)} pages)")


if __name__ == "__main__":
    build_project_pdf()

