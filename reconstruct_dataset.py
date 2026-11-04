"""
reconstruct_dataset.py
======================
Pipeline de Reconstruction, Nettoyage et Généralisation du Dataset Pick-and-Place
pour Surgical Action AI (DOFBOT 6-DOF Digital Twin).

Implémente la Section 3 & Section 6 de la Mission :
A. Filtrage intelligent des frames statiques artificielles (Δq ≈ 0) tout en
   préservant les états statiques utiles à la préhension et à la stabilisation.
B. Segmentation des épisodes multi-objets (6 objets concaténés) en sous-épisodes
   mono-objet propres avec reconstruction de la vraie cible (target_x, target_y, target_z).
C. Résolution des contradictions spatiales : labellisation explicite des 9 phases
   de manipulation (APPROACH, ALIGNMENT, DESCENT, GRASP, VERIFICATION, LIFT,
   MOVE_TO_TARGET, RELEASE, SUCCESS) et calcul de la configuration articulaire
   cible absolue q_target = [J1_target..J6_target, Gripper_target].
D. Séparation stricte des données d'échec (failure_dataset) : masque d'apprentissage
   articulaire (is_failure = 1 -> masqué pour la régression q_target) et
   entraînement de la tête de détection d'échec / confiance.
E. Génération d'épisodes mono-objet généralisés couvrant des positions aléatoires,
   orientations variées, hauteurs différentes, cibles variées et configurations
   initiales variées du bras (avec cinématique directe/inverse exacte du DofBot 6-DOF).
"""

import os
import sys
import json
import math
import glob
import numpy as np
import pandas as pd

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# ============================================================================
# 1. DÉFINITION DES 9 PHASES CANONIQUES DU PIPELINE PICK AND PLACE
# ============================================================================
PHASES = [
    "APPROACH",        # 0: Atteindre une position de préhension haute (hover)
    "ALIGNMENT",       # 1: Orienter la pince (alignement azimutal / symétrie boîte)
    "DESCENT",         # 2: Descendre verticalement vers la zone de contact
    "GRASP",           # 3: Fermer la pince sur l'objet
    "VERIFICATION",    # 4: Confirmer la préhension physique bilatérale
    "LIFT",            # 5: Soulever l'objet verticalement
    "MOVE_TO_TARGET",  # 6: Transporter l'objet et descendre vers la cible
    "RELEASE",         # 7: Ouvrir la pince pour libérer l'objet
    "SUCCESS",         # 8: Se rétracter verticalement et vérifier le placement
]
PHASE_TO_IDX = {name: i for i, name in enumerate(PHASES)}

# Configuration géométrique et limites mécaniques exactes (RobotConfiguration.ts)
JOINT_LIMITS_DEG = [
    (-170.0, 170.0),  # J1 Base Pan (Y)
    (-95.0, 95.0),    # J2 Shoulder Lift (X)
    (-135.0, 135.0),  # J3 Elbow Pitch (X)
    (-180.0, 180.0),  # J4 Forearm Roll (Y)
    (-165.0, 165.0),  # J5 Wrist Pitch (X)
    (-180.0, 180.0),  # J6 Tool Roll (Y)
]
HOME_ANGLES_DEG = [0.0, 20.0, 70.0, 0.0, 45.0, 0.0]

PIVOT_OFFSETS = [
    np.array([0.0, 0.14, 0.0]),  # J1
    np.array([0.0, 0.12, 0.0]),  # J2
    np.array([0.0, 0.32, 0.0]),  # J3
    np.array([0.0, 0.15, 0.0]),  # J4
    np.array([0.0, 0.13, 0.0]),  # J5
    np.array([0.0, 0.08, 0.0]),  # J6
]
JOINT_AXES = [
    np.array([0.0, 1.0, 0.0]),  # J1: Y
    np.array([1.0, 0.0, 0.0]),  # J2: X
    np.array([1.0, 0.0, 0.0]),  # J3: X
    np.array([0.0, 1.0, 0.0]),  # J4: Y
    np.array([1.0, 0.0, 0.0]),  # J5: X
    np.array([0.0, 1.0, 0.0]),  # J6: Y
]
TCP_OFFSET = np.array([0.0, 0.11, 0.0])


# ============================================================================
# 2. CINÉMATIQUE DIRECTE (FK) ET INVERSE (IK) EXACTE DU DOFBOT 6-DOF
# ============================================================================
def clamp_angles_deg(angles_deg):
    return [
        float(np.clip(a, JOINT_LIMITS_DEG[i][0], JOINT_LIMITS_DEG[i][1]))
        for i, a in enumerate(angles_deg)
    ]


def rot_axis_matrix(axis, theta_rad):
    x, y, z = axis
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    t = 1.0 - c
    R = np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c,     0.0],
        [0.0,               0.0,               0.0,               1.0]
    ], dtype=np.float64)
    return R


def trans_matrix(offset):
    T = np.eye(4, dtype=np.float64)
    T[0:3, 3] = offset
    return T


def mat_to_quat(R):
    """Convertit une matrice de rotation 3x3 en quaternion [qx, qy, qz, qw]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def compute_fk(angles_deg):
    """Calcule la cinématique directe (TCP position, TCP quaternion) identique à ForwardKinematics.ts."""
    clamped = clamp_angles_deg(angles_deg)
    M = np.eye(4, dtype=np.float64)
    for i in range(6):
        M = M @ trans_matrix(PIVOT_OFFSETS[i]) @ rot_axis_matrix(JOINT_AXES[i], clamped[i] * DEG2RAD)
    M_tcp = M @ trans_matrix(TCP_OFFSET)
    tcp_pos = M_tcp[0:3, 3].copy()
    tcp_quat = mat_to_quat(M_tcp[0:3, 0:3])
    return tcp_pos, tcp_quat, clamped


def solve_analytical_topdown_ik(target_pos, desired_yaw_rad=None, is_box=True, obj_yaw_rad=0.0):
    """
    Résout analytiquement + par raffinement la configuration articulaire exacte
    [J1..J6] en degrés pour une pose TCP verticale orientée vers le bas (Top-Down),
    identique à generateAnalyticalSeeds + solveInverseKinematics dans InverseKinematics.ts.
    """
    x, y, z = float(target_pos[0]), float(target_pos[1]), float(target_pos[2])
    shoulder_y = 0.14 + 0.12  # 0.26 m
    L2 = 0.32                 # 0.32 m
    L3 = 0.15 + 0.13          # 0.28 m
    Ltool = 0.08 + 0.11       # 0.19 m

    # Pour une approche Top-Down (toolYAxis = [0, -1, 0]), le poignet est à target_pos + [0, Ltool, 0]
    wrist_x = x
    wrist_y = y + Ltool
    wrist_z = z

    horiz = math.hypot(wrist_x, wrist_z)
    theta1_rad = math.atan2(wrist_x, wrist_z) if horiz > 1e-5 else 0.0
    theta1_deg = theta1_rad * RAD2DEG

    dy = wrist_y - shoulder_y
    dist_sq = horiz * horiz + dy * dy
    cos_theta3 = np.clip((dist_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3), -1.0, 1.0)
    alpha = math.atan2(horiz, dy)

    # Branche coude-haut (sign = +1) utilisée par le contrôleur DofBot
    theta3_rad = math.acos(cos_theta3)
    beta = math.atan2(L3 * math.sin(theta3_rad), L2 + L3 * math.cos(theta3_rad))
    theta2_rad = alpha - beta

    theta2_deg = theta2_rad * RAD2DEG
    theta3_deg = theta3_rad * RAD2DEG
    # Pour pointer verticalement vers le bas (desiredPitch = 180 deg) :
    theta5_deg = 180.0 - (theta2_deg + theta3_deg)

    # Calcul de J6 (alignement des mors avec l'objet ou l'azimut radial)
    if desired_yaw_rad is None:
        if is_box:
            best_yaw = obj_yaw_rad
            min_diff = float("inf")
            for k in range(4):
                cand = obj_yaw_rad + k * (math.pi / 2.0)
                diff = (cand - theta1_rad + math.pi) % (2.0 * math.pi) - math.pi
                if abs(diff) < min_diff:
                    min_diff = abs(diff)
                    best_yaw = cand
            desired_yaw_rad = best_yaw
        else:
            desired_yaw_rad = theta1_rad

    # toolXAxis = [cos(yaw), 0, -sin(yaw)] pour createTopDownQuaternion(yaw)
    tx = math.cos(desired_yaw_rad)
    tz = -math.sin(desired_yaw_rad)
    c1 = math.cos(theta1_rad)
    s1 = math.sin(theta1_rad)
    lx = tx * c1 - tz * s1
    lz = tx * s1 + tz * c1
    theta6_deg = math.atan2(lz, lx) * RAD2DEG

    return clamp_angles_deg([theta1_deg, theta2_deg, theta3_deg, 0.0, theta5_deg, theta6_deg])


def yaw_to_quat(yaw_rad):
    """Quaternion [qx, qy, qz, qw] d'une rotation autour de l'axe Y."""
    half = 0.5 * yaw_rad
    return [0.0, math.sin(half), 0.0, math.cos(half)]


# ============================================================================
# 3. NETTOYAGE ET SEGMENTATION DU DATASET EXISTANT
# ============================================================================
def map_existing_frame_to_9_phases(row, sub_df_idx, n_rows, q_curr, q_next):
    """
    Associe chaque ligne du dataset existant à l'une des 9 phases canoniques :
    APPROACH (0), ALIGNMENT (1), DESCENT (2), GRASP (3), VERIFICATION (4),
    LIFT (5), MOVE_TO_TARGET (6), RELEASE (7), SUCCESS (8).
    Résout les contradictions spatiales (APPROACH qui contenait à la fois le survol
    horizontal et la descente verticale, et PLACE qui contenait la descente, le
    relâchement et la remontée).
    """
    raw_state = str(row["task_state"]).strip()
    ee_y = float(row["ee_y"])
    obj_y = float(row["obj_y"])
    horiz_obj_dist = math.hypot(float(row["ee_x"]) - float(row["obj_x"]),
                                float(row["ee_z"]) - float(row["obj_z"]))
    gripper_open = float(row["gripper_opening_m"])

    if raw_state in ("IDLE", "APPROACH"):
        # Séparer APPROACH en :
        # 0: APPROACH (déplacement vers le point de survol au-dessus de l'objet)
        # 1: ALIGNMENT (alignement fin au-dessus de l'objet à haute altitude)
        # 2: DESCENT (descente verticale vers la zone de contact)
        if horiz_obj_dist > 0.045 and ee_y > obj_y + 0.12:
            return "APPROACH"
        elif ee_y > obj_y + 0.11:
            return "ALIGNMENT"
        else:
            return "DESCENT"
    elif raw_state == "GRASP":
        return "GRASP"
    elif raw_state == "GRASP_VERIFICATION":
        return "VERIFICATION"
    elif raw_state == "LIFT":
        return "LIFT"
    elif raw_state == "TRANSPORT":
        return "MOVE_TO_TARGET"
    elif raw_state == "PLACE":
        # Dans PickAndPlace.ts, PLACE contient :
        # subPhase 0 (descente finale vers le plateau avec pince fermée) -> MOVE_TO_TARGET
        # subPhase 1 (ouverture de la pince) -> RELEASE
        # subPhase 2 (remontée verticale après ouverture) -> SUCCESS
        if gripper_open < 0.060:
            return "MOVE_TO_TARGET"
        elif ee_y < 0.12:
            return "RELEASE"
        else:
            return "SUCCESS"
    elif raw_state in ("VERIFY_PLACEMENT", "COMPLETE"):
        return "SUCCESS"
    else:
        return "APPROACH"


def reconstruct_existing_success_dataset(csv_path):
    """
    Analyse, segmente et nettoie le dataset de démonstration existant :
    - Segmente les épisodes multi-objets par (episode_id, obj_id)
    - Reconstruit les vraies coordonnées cibles (target_x, target_y, target_z)
    - Associe chaque frame à sa phase parmi les 9 phases canoniques
    - Calcule q_target absolu (configuration cible de la phase) au lieu de Δq
    - Filtre les frames statiques artificielles redondantes
    """
    df = pd.read_csv(csv_path)
    total_raw_rows = len(df)
    static_frames_removed = 0
    sub_episodes_count = 0
    reconstructed_rows = []

    for ep_id, ep_df in df.groupby("episode_id", sort=False):
        ep_df = ep_df.reset_index(drop=True)

        # Détecter les segments continus par obj_id OU par réinitialisation de step_index
        obj_ids = ep_df["obj_id"].astype(str).values
        step_indices = ep_df["step_index"].values
        seg_starts = [0]
        for i in range(1, len(obj_ids)):
            if obj_ids[i] != obj_ids[i - 1] or step_indices[i] < step_indices[i - 1]:
                seg_starts.append(i)
        seg_starts.append(len(obj_ids))

        for seg_idx in range(len(seg_starts) - 1):
            s_start, s_end = seg_starts[seg_idx], seg_starts[seg_idx + 1]
            sub_df = ep_df.iloc[s_start:s_end].reset_index(drop=True)
            if len(sub_df) < 25:
                continue

            obj_id = str(sub_df["obj_id"].iloc[0])
            clean_ep_id = f"real_{ep_id}_seg{seg_idx}_{obj_id}"
            sub_episodes_count += 1

            # Reconstruction de la vraie position cible (target_x, target_y, target_z) :
            # Dans le CSV original, target_x/y/z était figé à (0.0, 0.4264, 0.5238).
            # La vraie cible dans la Zone B est donnée par la position (ee_x, 0.02, ee_z)
            # lors de la phase PLACE / VERIFY_PLACEMENT !
            place_rows = sub_df[sub_df["task_state"].isin(["PLACE", "VERIFY_PLACEMENT", "COMPLETE"])]
            if len(place_rows) > 0:
                true_tx = float(place_rows["ee_x"].iloc[0])
                true_ty = 0.02
                true_tz = float(place_rows["ee_z"].iloc[0])
            else:
                true_tx, true_ty, true_tz = 0.30, 0.02, 0.24

            # Position initiale de l'objet au début du sous-épisode
            init_obj_pos = [
                float(sub_df["obj_x"].iloc[0]),
                float(sub_df["obj_y"].iloc[0]),
                float(sub_df["obj_z"].iloc[0]),
            ]

            # Calculer les configurations cibles idéales des 9 phases pour ce sous-épisode
            hover_pick_pos = [init_obj_pos[0], max(init_obj_pos[1] + 0.18, 0.22), init_obj_pos[2]]
            grasp_pick_pos = [init_obj_pos[0], max(init_obj_pos[1] - 0.002, 0.036), init_obj_pos[2]]
            lift_pos = [init_obj_pos[0], 0.26, init_obj_pos[2]]
            place_drop_pos = [true_tx, true_ty + 0.032, true_tz]
            retract_pos = [true_tx, 0.24, true_tz]

            q_hover = solve_analytical_topdown_ik(hover_pick_pos, is_box=True, obj_yaw_rad=0.0)
            q_grasp = solve_analytical_topdown_ik(grasp_pick_pos, is_box=True, obj_yaw_rad=0.0)
            q_lift = solve_analytical_topdown_ik(lift_pos, is_box=True, obj_yaw_rad=0.0)
            q_place = solve_analytical_topdown_ik(place_drop_pos, is_box=False)
            q_retract = solve_analytical_topdown_ik(retract_pos, is_box=False)

            # Si le sous-épisode possède ses propres angles convergés par phase, on les extrait :
            phase_labels = []
            for idx_r in range(len(sub_df)):
                r = sub_df.iloc[idx_r]
                q_c = [float(r[f"j{k}_rad"]) for k in range(1, 7)]
                q_n = [float(sub_df.iloc[min(idx_r + 1, len(sub_df) - 1)][f"j{k}_rad"]) for k in range(1, 7)]
                phase_labels.append(map_existing_frame_to_9_phases(r, idx_r, len(sub_df), q_c, q_n))

            sub_df = sub_df.copy()
            sub_df["phase_name"] = phase_labels

            # Cibles absolues en radians par phase (extraites de la fin de chaque phase observée
            # ou repli sur la cible cinématique exacte de la phase)
            default_phase_targets_rad = {
                "APPROACH":       [a * DEG2RAD for a in q_hover],
                "ALIGNMENT":      [a * DEG2RAD for a in q_hover],
                "DESCENT":        [a * DEG2RAD for a in q_grasp],
                "GRASP":          [a * DEG2RAD for a in q_grasp],
                "VERIFICATION":   [a * DEG2RAD for a in q_grasp],
                "LIFT":           [a * DEG2RAD for a in q_lift],
                "MOVE_TO_TARGET": [a * DEG2RAD for a in q_place],
                "RELEASE":        [a * DEG2RAD for a in q_place],
                "SUCCESS":        [a * DEG2RAD for a in q_retract],
            }
            phase_targets_rad = {}
            for p_name in PHASES:
                p_rows = sub_df[sub_df["phase_name"] == p_name]
                if len(p_rows) > 0:
                    last_r = p_rows.iloc[-1]
                    phase_targets_rad[p_name] = [float(last_r[f"j{k}_rad"]) for k in range(1, 7)]
                else:
                    phase_targets_rad[p_name] = default_phase_targets_rad[p_name]

            # A. Filtrage des frames statiques artificielles :
            # On compte combien de frames consécutives ont ||Δq|| < 1e-4 dans la même phase
            static_streak = 0
            for idx_r in range(len(sub_df) - 1):
                r = sub_df.iloc[idx_r]
                r_next = sub_df.iloc[idx_r + 1]
                p_name = r["phase_name"]

                q_curr = np.array([float(r[f"j{k}_rad"]) for k in range(1, 7)])
                q_next = np.array([float(r_next[f"j{k}_rad"]) for k in range(1, 7)])
                dq_norm = float(np.linalg.norm(q_next - q_curr))

                if dq_norm < 1e-4 and p_name == r_next["phase_name"]:
                    static_streak += 1
                    # Conserver jusqu'à 4 frames statiques utiles pour GRASP, VERIFICATION, RELEASE, SUCCESS
                    max_keep = 5 if p_name in ("GRASP", "VERIFICATION", "RELEASE", "SUCCESS") else 2
                    if static_streak > max_keep:
                        static_frames_removed += 1
                        continue
                else:
                    static_streak = 0

                q_targ = phase_targets_rad[p_name]
                gripper_targ = 0.0 if p_name in ("GRASP", "VERIFICATION", "LIFT", "MOVE_TO_TARGET") else 0.078
                is_grasped_val = 1.0 if p_name in ("VERIFICATION", "LIFT", "MOVE_TO_TARGET") else float(r["grasp_succeeded"])
                _, ee_quat, _ = compute_fk([a * RAD2DEG for a in q_curr])

                reconstructed_rows.append({
                    "episode_id": clean_ep_id,
                    "source_type": "recorded_cleaned",
                    "step_id": int(idx_r),
                    "phase_name": p_name,
                    "phase_id": PHASE_TO_IDX[p_name],
                    "obj_id": obj_id,
                    "j1_pos_rad": float(q_curr[0]),
                    "j2_pos_rad": float(q_curr[1]),
                    "j3_pos_rad": float(q_curr[2]),
                    "j4_pos_rad": float(q_curr[3]),
                    "j5_pos_rad": float(q_curr[4]),
                    "j6_pos_rad": float(q_curr[5]),
                    "ee_x": float(r["ee_x"]),
                    "ee_y": float(r["ee_y"]),
                    "ee_z": float(r["ee_z"]),
                    "ee_qx": float(ee_quat[0]),
                    "ee_qy": float(ee_quat[1]),
                    "ee_qz": float(ee_quat[2]),
                    "ee_qw": float(ee_quat[3]),
                    "obj_x": float(r["obj_x"]),
                    "obj_y": float(r["obj_y"]),
                    "obj_z": float(r["obj_z"]),
                    "obj_qx": float(r["obj_qx"]),
                    "obj_qy": float(r["obj_qy"]),
                    "obj_qz": float(r["obj_qz"]),
                    "obj_qw": float(r["obj_qw"]),
                    "target_x": true_tx,
                    "target_y": true_ty,
                    "target_z": true_tz,
                    "gripper_opening_m": float(r["gripper_opening_m"]),
                    "contact_count": float(r["contact_count"]),
                    "is_grasped": is_grasped_val,
                    # Nouvelles cibles absolues (Section 2 & 4)
                    "j1_target_rad": q_targ[0],
                    "j2_target_rad": q_targ[1],
                    "j3_target_rad": q_targ[2],
                    "j4_target_rad": q_targ[3],
                    "j5_target_rad": q_targ[4],
                    "j6_target_rad": q_targ[5],
                    "gripper_target_m": gripper_targ,
                    # Cibles incrémentales conservées uniquement pour la comparaison Avant/Après
                    "j1_delta_rad": float(q_next[0] - q_curr[0]),
                    "j2_delta_rad": float(q_next[1] - q_curr[1]),
                    "j3_delta_rad": float(q_next[2] - q_curr[2]),
                    "j4_delta_rad": float(q_next[3] - q_curr[3]),
                    "j5_delta_rad": float(q_next[4] - q_curr[4]),
                    "j6_delta_rad": float(q_next[5] - q_curr[5]),
                    "is_failure": 0,
                    "failure_type": "NOMINAL",
                    "confidence_target": 1.0,
                })

    stats = {
        "raw_success_rows": total_raw_rows,
        "cleaned_success_rows": len(reconstructed_rows),
        "static_frames_removed": static_frames_removed,
        "segmented_single_object_episodes": sub_episodes_count,
    }
    return reconstructed_rows, stats


# ============================================================================
# 4. TRAITEMENT DU DATASET D'ÉCHECS EXISTANT (FAILURE DATASET)
# ============================================================================
def process_existing_failure_dataset(csv_path):
    """
    Section 3.D : Conserver les épisodes failure, ne pas utiliser leurs mouvements
    incorrects comme cibles de démonstration réussie, mais les exploiter pour
    apprendre la détection d'échec et la confiance.
    """
    df = pd.read_csv(csv_path)
    failure_rows = []
    for ep_id, ep_df in df.groupby("episode_id", sort=False):
        ep_df = ep_df.reset_index(drop=True)
        n_rows = len(ep_df)
        for idx_r in range(0, n_rows - 1, 2):  # Sous-échantillonnage léger des attentes
            r = ep_df.iloc[idx_r]
            r_next = ep_df.iloc[idx_r + 1]
            f_type = str(ep_id).strip()
            raw_state = str(r["task_state"]).strip()

            # L'anomalie survient principalement dans la seconde moitié de l'épisode d'échec
            # ou lorsque l'état physique dévie (ex: pince fermée à vide, chute d'objet, collision)
            is_failing_frame = 1 if (idx_r >= int(0.35 * n_rows) or raw_state == "FAILED") else 0
            if is_failing_frame == 0:
                continue

            q_curr = [float(r[f"j{k}_rad"]) for k in range(1, 7)]
            q_next = [float(r_next[f"j{k}_rad"]) for k in range(1, 7)]
            p_name = map_existing_frame_to_9_phases(r, idx_r, n_rows, q_curr, q_next)
            _, ee_quat, _ = compute_fk([a * RAD2DEG for a in q_curr])

            failure_rows.append({
                "episode_id": f"fail_{ep_id}",
                "source_type": "recorded_failure",
                "step_id": int(idx_r),
                "phase_name": p_name,
                "phase_id": PHASE_TO_IDX[p_name],
                "obj_id": str(r["obj_id"]),
                "j1_pos_rad": q_curr[0],
                "j2_pos_rad": q_curr[1],
                "j3_pos_rad": q_curr[2],
                "j4_pos_rad": q_curr[3],
                "j5_pos_rad": q_curr[4],
                "j6_pos_rad": q_curr[5],
                "ee_x": float(r["ee_x"]),
                "ee_y": float(r["ee_y"]),
                "ee_z": float(r["ee_z"]),
                "ee_qx": float(ee_quat[0]),
                "ee_qy": float(ee_quat[1]),
                "ee_qz": float(ee_quat[2]),
                "ee_qw": float(ee_quat[3]),
                "obj_x": float(r["obj_x"]),
                "obj_y": float(r["obj_y"]),
                "obj_z": float(r["obj_z"]),
                "obj_qx": float(r["obj_qx"]),
                "obj_qy": float(r["obj_qy"]),
                "obj_qz": float(r["obj_qz"]),
                "obj_qw": float(r["obj_qw"]),
                "target_x": 0.30,
                "target_y": 0.02,
                "target_z": 0.24,
                "gripper_opening_m": float(r["gripper_opening_m"]),
                "contact_count": float(r["contact_count"]),
                "is_grasped": float(r["grasp_succeeded"]),
                # Les cibles articulaires ne seront PAS utilisées pour la loss d'imitation (is_failure=1)
                "j1_target_rad": q_curr[0],
                "j2_target_rad": q_curr[1],
                "j3_target_rad": q_curr[2],
                "j4_target_rad": q_curr[3],
                "j5_target_rad": q_curr[4],
                "j6_target_rad": q_curr[5],
                "gripper_target_m": float(r["gripper_opening_m"]),
                "j1_delta_rad": float(q_next[0] - q_curr[0]),
                "j2_delta_rad": float(q_next[1] - q_curr[1]),
                "j3_delta_rad": float(q_next[2] - q_curr[2]),
                "j4_delta_rad": float(q_next[3] - q_curr[3]),
                "j5_delta_rad": float(q_next[4] - q_curr[4]),
                "j6_delta_rad": float(q_next[5] - q_curr[5]),
                "is_failure": 1,
                "failure_type": f_type,
                "confidence_target": 0.0,
            })

    return failure_rows


# ============================================================================
# 5. GÉNÉRATION D'ÉPISODES MONO-OBJET GÉNÉRALISÉS (SECTION 6)
# ============================================================================
def generate_generalized_episodes(num_nominal_episodes=200, num_failure_episodes=50, seed=42):
    """
    Génère des trajectoires physiques/cinématiques mono-objet couvrant :
    - Positions aléatoires de l'objet sur le plateau et dans l'espace de travail
    - Différentes orientations (yaw), hauteurs et distances
    - Différentes positions cibles (Zone B)
    - Différentes configurations initiales du bras (Home, perturbée, post-transport)
    Chaque épisode suit rigoureusement les 9 phases Pick-and-Place.
    """
    rng = np.random.default_rng(seed)
    rows = []

    for ep_idx in range(num_nominal_episodes):
        ep_id = f"gen_nominal_ep_{ep_idx:03d}"

        # 1. Position, hauteur, orientation et forme de l'objet
        if ep_idx < 25:
            # Inclure la position standard [-0.30, 0.041, 0.24] et ses environs proches
            obj_x = float( -0.30 + rng.uniform(-0.025, 0.025) )
            obj_z = float(  0.24 + rng.uniform(-0.025, 0.025) )
            obj_h = float(  0.041 + rng.uniform(-0.004, 0.006) )
        else:
            # Positions aléatoires dans toute la Zone A et l'espace de préhension
            obj_x = float(rng.uniform(-0.44, -0.15))
            obj_z = float(rng.uniform(0.14, 0.35))
            obj_h = float(rng.uniform(0.032, 0.065))

        is_box = bool(ep_idx % 3 != 2)
        obj_yaw = float(rng.uniform(-math.pi / 2.0, math.pi / 2.0))
        obj_quat = yaw_to_quat(obj_yaw)
        init_obj_pos = np.array([obj_x, obj_h, obj_z], dtype=np.float64)

        # 2. Position cible (Zone B) variée
        if ep_idx < 35:
            target_pos = np.array([0.30, 0.02, 0.24], dtype=np.float64)
        else:
            target_pos = np.array([
                float(rng.uniform(0.18, 0.42)),
                float(rng.uniform(0.02, 0.04)),
                float(rng.uniform(0.16, 0.33)),
            ], dtype=np.float64)

        # 3. Configuration initiale du bras variée
        if ep_idx % 3 == 0:
            q_init_deg = list(HOME_ANGLES_DEG)
        elif ep_idx % 3 == 1:
            q_init_deg = clamp_angles_deg([
                HOME_ANGLES_DEG[0] + float(rng.uniform(-35.0, 35.0)),
                HOME_ANGLES_DEG[1] + float(rng.uniform(-15.0, 20.0)),
                HOME_ANGLES_DEG[2] + float(rng.uniform(-20.0, 20.0)),
                0.0,
                HOME_ANGLES_DEG[4] + float(rng.uniform(-20.0, 20.0)),
                float(rng.uniform(-30.0, 30.0)),
            ])
        else:
            # Départ depuis une pose de fin de tâche précédente (côté Zone B)
            prev_end_pos = [float(rng.uniform(0.15, 0.38)), float(rng.uniform(0.18, 0.28)), float(rng.uniform(0.18, 0.30))]
            q_init_deg = solve_analytical_topdown_ik(prev_end_pos, is_box=False)

        # 4. Calcul des configurations cibles exactes pour chacune des 9 phases
        # Phase 0 (APPROACH) : survol à hauteur de sécurité au-dessus de l'objet (J6 neutre ou pré-orienté)
        hover_pos = np.array([obj_x, max(obj_h + 0.18, 0.22), obj_z], dtype=np.float64)
        azimuth_pick = math.atan2(obj_x, obj_z)
        q_approach_deg = solve_analytical_topdown_ik(hover_pos, desired_yaw_rad=azimuth_pick, is_box=False)

        # Phase 1 (ALIGNMENT) : alignement précis du poignet J6 avec la symétrie de l'objet au point de survol
        q_align_deg = solve_analytical_topdown_ik(hover_pos, desired_yaw_rad=None, is_box=is_box, obj_yaw_rad=obj_yaw)

        # Phase 2 (DESCENT) : descente verticale vers la hauteur de prise physique
        grasp_y = max(obj_h - 0.002, 0.035)
        grasp_pos = np.array([obj_x, grasp_y, obj_z], dtype=np.float64)
        q_descent_deg = solve_analytical_topdown_ik(grasp_pos, desired_yaw_rad=None, is_box=is_box, obj_yaw_rad=obj_yaw)
        # S'assurer que J6 garde l'alignement exact
        q_descent_deg[5] = q_align_deg[5]

        # Phase 3 (GRASP) & Phase 4 (VERIFICATION) : maintien de la configuration de prise + fermeture pince
        q_grasp_deg = list(q_descent_deg)
        q_verify_deg = list(q_descent_deg)

        # Phase 5 (LIFT) : levage vertical de l'objet à y = 0.26 m
        lift_pos = np.array([obj_x, 0.26, obj_z], dtype=np.float64)
        q_lift_deg = solve_analytical_topdown_ik(lift_pos, desired_yaw_rad=None, is_box=is_box, obj_yaw_rad=obj_yaw)
        q_lift_deg[5] = q_align_deg[5]

        # Phase 6 (MOVE_TO_TARGET) : transport vers la cible et descente à hauteur de dépose
        drop_pos = np.array([target_pos[0], target_pos[1] + obj_h * 0.5 + 0.015, target_pos[2]], dtype=np.float64)
        q_move_target_deg = solve_analytical_topdown_ik(drop_pos, is_box=False)

        # Phase 7 (RELEASE) : ouverture de la pince à la position de dépose
        q_release_deg = list(q_move_target_deg)

        # Phase 8 (SUCCESS) : remontée verticale de dégagement au-dessus de la cible
        retract_pos = np.array([target_pos[0], 0.24, target_pos[2]], dtype=np.float64)
        q_success_deg = solve_analytical_topdown_ik(retract_pos, is_box=False)

        phase_Specs = [
            ("APPROACH",       q_approach_deg,    0.078, 0.0, 2, 16),
            ("ALIGNMENT",      q_align_deg,       0.078, 0.0, 2, 8),
            ("DESCENT",        q_descent_deg,     0.078, 0.0, 2, 14),
            ("GRASP",          q_grasp_deg,       0.000, 0.0, 4, 8),
            ("VERIFICATION",   q_verify_deg,      0.000, 1.0, 4, 8),
            ("LIFT",           q_lift_deg,        0.000, 1.0, 4, 14),
            ("MOVE_TO_TARGET", q_move_target_deg, 0.000, 1.0, 4, 18),
            ("RELEASE",        q_release_deg,     0.078, 0.0, 2, 8),
            ("SUCCESS",        q_success_deg,     0.078, 0.0, 1, 10),
        ]

        q_curr_deg = np.array(q_init_deg, dtype=np.float64)
        gripper_curr = 0.078
        step_counter = 0

        for p_name, q_targ_deg_list, grip_targ, is_grasped_flag, contacts_n, n_steps in phase_Specs:
            q_targ_deg = np.array(q_targ_deg_list, dtype=np.float64)
            q_targ_rad = q_targ_deg * DEG2RAD

            for s in range(n_steps):
                # Interpolation exponentielle + légère perturbation réaliste (simulant le contrôleur PD)
                alpha = (s + 1) / float(n_steps)
                smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
                noise_deg = rng.normal(0.0, 0.25, size=6) * (1.0 - smooth_alpha)
                q_step_deg = clamp_angles_deg(q_curr_deg + smooth_alpha * (q_targ_deg - q_curr_deg) + noise_deg)
                q_next_deg = clamp_angles_deg(
                    q_curr_deg + min(1.0, (s + 2) / float(n_steps)) * (q_targ_deg - q_curr_deg)
                )

                # État de la pince
                if p_name == "GRASP":
                    g_open = max(0.034, 0.078 - 0.044 * smooth_alpha)
                elif p_name in ("VERIFICATION", "LIFT", "MOVE_TO_TARGET"):
                    g_open = 0.035
                elif p_name == "RELEASE":
                    g_open = min(0.078, 0.035 + 0.043 * smooth_alpha)
                else:
                    g_open = 0.078

                ee_pos, ee_quat, _ = compute_fk(q_step_deg)

                # Position courante de l'objet :
                # - Avant LIFT : sur la Zone A (init_obj_pos)
                # - Pendant LIFT et MOVE_TO_TARGET : attaché physiquement entre les mors de la pince (ee_pos)
                # - Pendant RELEASE et SUCCESS : posé sur la cible Zone B
                if p_name in ("LIFT", "MOVE_TO_TARGET"):
                    curr_obj_pos = ee_pos.copy()
                    curr_obj_quat = ee_quat.copy()
                elif p_name in ("RELEASE", "SUCCESS"):
                    curr_obj_pos = np.array([target_pos[0], target_pos[1] + obj_h * 0.5, target_pos[2]])
                    curr_obj_quat = [0.0, 0.0, 0.0, 1.0]
                else:
                    curr_obj_pos = init_obj_pos.copy()
                    curr_obj_quat = list(obj_quat)

                q_step_rad = [a * DEG2RAD for a in q_step_deg]
                q_next_rad = [a * DEG2RAD for a in q_next_deg]

                rows.append({
                    "episode_id": ep_id,
                    "source_type": "generalized_nominal",
                    "step_id": step_counter,
                    "phase_name": p_name,
                    "phase_id": PHASE_TO_IDX[p_name],
                    "obj_id": "Single_Object",
                    "j1_pos_rad": q_step_rad[0],
                    "j2_pos_rad": q_step_rad[1],
                    "j3_pos_rad": q_step_rad[2],
                    "j4_pos_rad": q_step_rad[3],
                    "j5_pos_rad": q_step_rad[4],
                    "j6_pos_rad": q_step_rad[5],
                    "ee_x": float(ee_pos[0]),
                    "ee_y": float(ee_pos[1]),
                    "ee_z": float(ee_pos[2]),
                    "ee_qx": float(ee_quat[0]),
                    "ee_qy": float(ee_quat[1]),
                    "ee_qz": float(ee_quat[2]),
                    "ee_qw": float(ee_quat[3]),
                    "obj_x": float(curr_obj_pos[0]),
                    "obj_y": float(curr_obj_pos[1]),
                    "obj_z": float(curr_obj_pos[2]),
                    "obj_qx": float(curr_obj_quat[0]),
                    "obj_qy": float(curr_obj_quat[1]),
                    "obj_qz": float(curr_obj_quat[2]),
                    "obj_qw": float(curr_obj_quat[3]),
                    "target_x": float(target_pos[0]),
                    "target_y": float(target_pos[1]),
                    "target_z": float(target_pos[2]),
                    "gripper_opening_m": float(g_open),
                    "contact_count": float(contacts_n),
                    "is_grasped": float(is_grasped_flag),
                    "j1_target_rad": float(q_targ_rad[0]),
                    "j2_target_rad": float(q_targ_rad[1]),
                    "j3_target_rad": float(q_targ_rad[2]),
                    "j4_target_rad": float(q_targ_rad[3]),
                    "j5_target_rad": float(q_targ_rad[4]),
                    "j6_target_rad": float(q_targ_rad[5]),
                    "gripper_target_m": float(grip_targ),
                    "j1_delta_rad": float(q_next_rad[0] - q_step_rad[0]),
                    "j2_delta_rad": float(q_next_rad[1] - q_step_rad[1]),
                    "j3_delta_rad": float(q_next_rad[2] - q_step_rad[2]),
                    "j4_delta_rad": float(q_next_rad[3] - q_step_rad[3]),
                    "j5_delta_rad": float(q_next_rad[4] - q_step_rad[4]),
                    "j6_delta_rad": float(q_next_rad[5] - q_step_rad[5]),
                    "is_failure": 0,
                    "failure_type": "NOMINAL",
                    "confidence_target": 1.0,
                })
                step_counter += 1

            q_curr_deg = q_targ_deg.copy()
            gripper_curr = grip_targ

    # Génération d'épisodes d'échecs variés (pour l'évaluation de détection d'échec sans fuite de données)
    failure_modes = [
        "MISSED_GRASP_OFFSET",
        "SLIP_DURING_LIFT",
        "DROP_DURING_TRANSPORT",
        "ARM_OVERSTRETCH_DRIFT",
    ]
    for f_idx in range(num_failure_episodes):
        ep_id = f"gen_failure_ep_{f_idx:03d}"
        f_mode = failure_modes[f_idx % len(failure_modes)]
        obj_x = float(rng.uniform(-0.42, -0.18))
        obj_z = float(rng.uniform(0.15, 0.34))
        obj_h = float(rng.uniform(0.035, 0.055))
        target_pos = np.array([float(rng.uniform(0.20, 0.40)), 0.02, float(rng.uniform(0.18, 0.32))])

        for s in range(20):
            if f_mode == "MISSED_GRASP_OFFSET":
                # La pince se ferme à plus de 8 cm de l'objet sans contact ni prise
                p_name = "VERIFICATION"
                bad_pos = [obj_x + 0.09, obj_h + 0.05, obj_z - 0.07]
                q_bad_deg = solve_analytical_topdown_ik(bad_pos, is_box=False)
                g_open = 0.004  # Pince complètement fermée dans le vide
                contacts_n = 0.0
                is_grasped_flag = 0.0
                curr_obj_pos = [obj_x, obj_h, obj_z]
            elif f_mode == "SLIP_DURING_LIFT":
                # En phase LIFT, l'objet est resté en bas sur la table (is_grasped=0, écart ee-obj grand)
                p_name = "LIFT"
                lift_pos = [obj_x, 0.24, obj_z]
                q_bad_deg = solve_analytical_topdown_ik(lift_pos, is_box=False)
                g_open = 0.012
                contacts_n = 0.0
                is_grasped_flag = 0.0
                curr_obj_pos = [obj_x, obj_h, obj_z]
            elif f_mode == "DROP_DURING_TRANSPORT":
                # En phase MOVE_TO_TARGET, l'objet est tombé au sol loin de l'effecteur
                p_name = "MOVE_TO_TARGET"
                trans_pos = [0.05, 0.25, 0.25]
                q_bad_deg = solve_analytical_topdown_ik(trans_pos, is_box=False)
                g_open = 0.065
                contacts_n = 0.0
                is_grasped_flag = 0.0
                curr_obj_pos = [-0.10, 0.02, 0.22]
            else:
                # ARM_OVERSTRETCH_DRIFT : le bras s'étire vers ses butées mécaniques (comme dans l'ancien modèle Δq)
                p_name = "APPROACH"
                q_bad_deg = clamp_angles_deg([
                    float(rng.uniform(-160, 160)),
                    float(rng.uniform(75, 95)),
                    float(rng.uniform(80, 130)),
                    0.0,
                    float(rng.uniform(-120, -60)),
                    0.0,
                ])
                g_open = 0.078
                contacts_n = 0.0
                is_grasped_flag = 0.0
                curr_obj_pos = [obj_x, obj_h, obj_z]

            noise = rng.normal(0.0, 1.5, size=6)
            q_step_deg = clamp_angles_deg(np.array(q_bad_deg) + noise)
            ee_pos, ee_quat, _ = compute_fk(q_step_deg)
            q_step_rad = [a * DEG2RAD for a in q_step_deg]

            rows.append({
                "episode_id": ep_id,
                "source_type": "generalized_failure",
                "step_id": s,
                "phase_name": p_name,
                "phase_id": PHASE_TO_IDX[p_name],
                "obj_id": "Single_Object",
                "j1_pos_rad": q_step_rad[0],
                "j2_pos_rad": q_step_rad[1],
                "j3_pos_rad": q_step_rad[2],
                "j4_pos_rad": q_step_rad[3],
                "j5_pos_rad": q_step_rad[4],
                "j6_pos_rad": q_step_rad[5],
                "ee_x": float(ee_pos[0]),
                "ee_y": float(ee_pos[1]),
                "ee_z": float(ee_pos[2]),
                "ee_qx": float(ee_quat[0]),
                "ee_qy": float(ee_quat[1]),
                "ee_qz": float(ee_quat[2]),
                "ee_qw": float(ee_quat[3]),
                "obj_x": float(curr_obj_pos[0]),
                "obj_y": float(curr_obj_pos[1]),
                "obj_z": float(curr_obj_pos[2]),
                "obj_qx": 0.0,
                "obj_qy": 0.0,
                "obj_qz": 0.0,
                "obj_qw": 1.0,
                "target_x": float(target_pos[0]),
                "target_y": float(target_pos[1]),
                "target_z": float(target_pos[2]),
                "gripper_opening_m": float(g_open),
                "contact_count": float(contacts_n),
                "is_grasped": float(is_grasped_flag),
                "j1_target_rad": q_step_rad[0],
                "j2_target_rad": q_step_rad[1],
                "j3_target_rad": q_step_rad[2],
                "j4_target_rad": q_step_rad[3],
                "j5_target_rad": q_step_rad[4],
                "j6_target_rad": q_step_rad[5],
                "gripper_target_m": float(g_open),
                "j1_delta_rad": 0.0,
                "j2_delta_rad": 0.0,
                "j3_delta_rad": 0.0,
                "j4_delta_rad": 0.0,
                "j5_delta_rad": 0.0,
                "j6_delta_rad": 0.0,
                "is_failure": 1,
                "failure_type": f_mode,
                "confidence_target": 0.0,
            })

    return rows


def main():
    os.makedirs("datasets", exist_ok=True)
    os.makedirs("reports", exist_ok=True)

    success_csv = "datasets/surgical_action_ai_dataset_1793718194098.csv"
    failure_csv = "datasets/failure_dataset_1793789920059.csv"

    print("=" * 72)
    print("  RECONSTRUCTION & NETTOYAGE DU DATASET PICK-AND-PLACE (robot2Action)")
    print("=" * 72)

    cleaned_rows, stats = reconstruct_existing_success_dataset(success_csv)
    print(f"[1/3] Dataset existant nettoyé et segmenté :")
    print(f"      - Lignes brutes initiales          : {stats['raw_success_rows']}")
    print(f"      - Frames statiques (Δq≈0) filtrées : {stats['static_frames_removed']}")
    print(f"      - Sous-épisodes mono-objet extraits: {stats['segmented_single_object_episodes']}")
    print(f"      - Lignes nominales nettoyées       : {stats['cleaned_success_rows']}")

    fail_rows = process_existing_failure_dataset(failure_csv)
    print(f"[2/3] Dataset d'échecs existant isolé    : {len(fail_rows)} lignes d'anomalie")

    gen_rows = generate_generalized_episodes(num_nominal_episodes=220, num_failure_episodes=55, seed=42)
    print(f"[3/3] Épisodes mono-objet généralisés    : {len(gen_rows)} lignes (positions/orientations aléatoires)")

    all_rows = cleaned_rows + fail_rows + gen_rows
    out_df = pd.DataFrame(all_rows)

    out_path = "datasets/reconstructed_pick_and_place_dataset.csv"
    out_df.to_csv(out_path, index=False)

    phase_counts = out_df["phase_name"].value_counts().to_dict()
    source_counts = out_df["source_type"].value_counts().to_dict()
    n_episodes = int(out_df["episode_id"].nunique())
    n_nominal = int((out_df["is_failure"] == 0).sum())
    n_failure = int((out_df["is_failure"] == 1).sum())

    summary = {
        "output_csv": out_path,
        "total_rows": len(out_df),
        "total_unique_episodes": n_episodes,
        "nominal_rows": n_nominal,
        "failure_rows": n_failure,
        "cleaning_stats": stats,
        "phase_distribution": phase_counts,
        "source_distribution": source_counts,
    }
    with open("reports/dataset_reconstruction_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print("-" * 72)
    print(f"Dataset reconstruit sauvegardé dans : {out_path}")
    print(f"Total lignes : {len(out_df)} | Épisodes uniques : {n_episodes} (Nominal: {n_nominal}, Failure: {n_failure})")
    print("Distribution par phase (9 phases) :", phase_counts)
    print("=" * 72)


if __name__ == "__main__":
    main()
