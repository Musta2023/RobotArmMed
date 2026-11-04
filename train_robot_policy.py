"""
train_robot_policy.py
=====================
Script d'entraînement de la nouvelle architecture ML Pick-and-Place (Section 2, 4 & 7) :
- Formulation mathématique corrigée : Prédiction directe de la configuration
  articulaire cible absolue q_target = [J1_target..J6_target, Gripper_target, Confidence]
  conditionnée par l'observation complète (35D) incluant la phase de manipulation (9 phases).
- Séparation stricte par épisodes complets (GroupShuffleSplit sur episode_id) -> 0 Data Leakage.
- Masquage des épisodes d'échec pour la régression articulaire + apprentissage conjoint
  de la détection d'échec (Confidence / Failure score).
- Entraînement parallèle d'une référence "Ancien Modèle Δq" sur le même split par épisodes
  pour la comparaison quantitative Avant/Après (Section 7 & 9).
- Export triple : PyTorch (.pt), ONNX autonome (.onnx) et Poids Natifs TypeScript (.json).
"""

import os
import sys
import json
import math
import time
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import GroupShuffleSplit

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

SEED = 42
torch.manual_seed(SEED)
np.random.seed(SEED)

PHASES = [
    "APPROACH",
    "ALIGNMENT",
    "DESCENT",
    "GRASP",
    "VERIFICATION",
    "LIFT",
    "MOVE_TO_TARGET",
    "RELEASE",
    "SUCCESS",
]

BASE_FEATURE_COLS = [
    # Positions articulaires actuelles (6D)
    "j1_pos_rad", "j2_pos_rad", "j3_pos_rad", "j4_pos_rad", "j5_pos_rad", "j6_pos_rad",
    # Pose de l'effecteur (7D)
    "ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw",
    # Pose de l'objet (7D)
    "obj_x", "obj_y", "obj_z", "obj_qx", "obj_qy", "obj_qz", "obj_qw",
    # Pose de la cible (3D)
    "target_x", "target_y", "target_z",
    # État de la pince et contacts (3D)
    "gripper_opening_m", "contact_count", "is_grasped",
]
# + 9D One-Hot de la phase de manipulation = 35D au total
INPUT_DIM = 35
OUTPUT_DIM = 8  # [J1_target..J6_target (6), Gripper_target (1), Confidence (1)]

TARGET_JOINT_COLS = [
    "j1_target_rad", "j2_target_rad", "j3_target_rad",
    "j4_target_rad", "j5_target_rad", "j6_target_rad",
    "gripper_target_m",
]

DELTA_JOINT_COLS = [
    "j1_delta_rad", "j2_delta_rad", "j3_delta_rad",
    "j4_delta_rad", "j5_delta_rad", "j6_delta_rad",
    "gripper_target_m",
]


def build_feature_matrix_35d(df: pd.DataFrame) -> np.ndarray:
    """Construit la matrice de features 35D : 26 features continues + 9 one-hot phases."""
    base_feats = df[BASE_FEATURE_COLS].values.astype(np.float32)
    phase_ids = df["phase_id"].values.astype(np.int64)
    one_hot = np.zeros((len(df), len(PHASES)), dtype=np.float32)
    one_hot[np.arange(len(df)), np.clip(phase_ids, 0, len(PHASES) - 1)] = 1.0
    return np.hstack([base_feats, one_hot])


class PickAndPlaceDataset(Dataset):
    def __init__(self, X, y_abs, y_delta, is_failure, confidence):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y_abs = torch.tensor(y_abs, dtype=torch.float32)
        self.y_delta = torch.tensor(y_delta, dtype=torch.float32)
        self.is_failure = torch.tensor(is_failure, dtype=torch.float32)
        self.confidence = torch.tensor(confidence, dtype=torch.float32)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return (
            self.X[idx],
            self.y_abs[idx],
            self.y_delta[idx],
            self.is_failure[idx],
            self.confidence[idx],
        )


class AbsoluteTargetPolicyNet(nn.Module):
    """
    Nouvelle architecture conditionnée par la phase (35D -> 8D) :
    - Normalisation Z-score intégrée aux buffers du modèle (pour les 26 features continues,
      les 9 entrées one-hot de phase restant intactes).
    - Réseau MLP profond avec LayerNorm et SiLU.
    - Sortie 8D : [J1_target..J6_target (rad), Gripper_target (m), Confidence_score (0..1)].
    """
    def __init__(self, x_mean: np.ndarray, x_std: np.ndarray):
        super().__init__()
        self.register_buffer("x_mean", torch.tensor(x_mean, dtype=torch.float32))
        self.register_buffer("x_std", torch.tensor(x_std, dtype=torch.float32))

        self.net = nn.Sequential(
            nn.Linear(INPUT_DIM, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 128),
            nn.LayerNorm(128),
            nn.SiLU(),
            nn.Linear(128, OUTPUT_DIM),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_norm = (x - self.x_mean) / self.x_std
        raw = self.net(x_norm)
        # Appliquer Sigmoid sur la 8ème sortie (score de confiance / probabilité nominale)
        joints_and_grip = raw[:, :7]
        conf = torch.sigmoid(raw[:, 7:8])
        return torch.cat([joints_and_grip, conf], dim=-1)


class LegacyDeltaPolicyNet(nn.Module):
    """
    Ancien modèle prédisant Δq = q(t+1) - q(t) sans conditionnement de phase explicite
    (entraîné sur le même split par épisodes pour comparaison rigoureuse Avant/Après).
    """
    def __init__(self, in_dim: int = 26):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 128),
            nn.LayerNorm(128),
            nn.SiLU(),
            nn.Linear(128, 7),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x[:, :26])


def main():
    os.makedirs("models", exist_ok=True)
    os.makedirs("public/models", exist_ok=True)
    os.makedirs("reports", exist_ok=True)

    dataset_path = "datasets/reconstructed_pick_and_place_dataset.csv"
    print("=" * 72)
    print("  ENTRAÎNEMENT DU MODÈLE PICK-AND-PLACE (CONFIGURATION CIBLE ABSOLUE)")
    print("=" * 72)
    df = pd.read_csv(dataset_path)
    print(f"Dataset chargé : {len(df)} échantillons | {df['episode_id'].nunique()} épisodes uniques")

    # 1. Séparation stricte par ÉPISODES COMPLETS (0 Data Leakage)
    gss = GroupShuffleSplit(n_splits=1, test_size=0.20, random_state=SEED)
    groups = df["episode_id"].values
    train_idx, val_idx = next(gss.split(df, groups=groups))

    train_df = df.iloc[train_idx].reset_index(drop=True)
    val_df = df.iloc[val_idx].reset_index(drop=True)

    train_eps = set(train_df["episode_id"].unique())
    val_eps = set(val_df["episode_id"].unique())
    overlap = train_eps.intersection(val_eps)
    assert len(overlap) == 0, f"DATA LEAKAGE DÉTECTÉ : {overlap}"
    print(f"Split par épisodes vérifié (Overlap = {len(overlap)}) :")
    print(f"  - Train : {len(train_df)} lignes ({len(train_eps)} épisodes)")
    print(f"  - Val   : {len(val_df)} lignes ({len(val_eps)} épisodes)")

    # Sauvegarder les indices / épisodes de test pour evaluate_model.py
    with open("reports/episode_split.json", "w", encoding="utf-8") as f:
        json.dump({
            "train_episodes": sorted(list(train_eps)),
            "val_episodes": sorted(list(val_eps)),
            "train_rows": len(train_df),
            "val_rows": len(val_df),
        }, f, indent=2)

    X_train = build_feature_matrix_35d(train_df)
    X_val = build_feature_matrix_35d(val_df)

    # Calcul des statistiques de normalisation sur les 26 features continues (train uniquement)
    x_mean = np.zeros(INPUT_DIM, dtype=np.float32)
    x_std = np.ones(INPUT_DIM, dtype=np.float32)
    x_mean[:26] = X_train[:, :26].mean(axis=0)
    x_std[:26] = np.maximum(X_train[:, :26].std(axis=0), 1e-4)

    y_abs_train = train_df[TARGET_JOINT_COLS].values.astype(np.float32)
    y_abs_val = val_df[TARGET_JOINT_COLS].values.astype(np.float32)

    y_delta_train = train_df[DELTA_JOINT_COLS].values.astype(np.float32)
    y_delta_val = val_df[DELTA_JOINT_COLS].values.astype(np.float32)

    fail_train = train_df["is_failure"].values.astype(np.float32)
    fail_val = val_df["is_failure"].values.astype(np.float32)

    conf_train = train_df["confidence_target"].values.astype(np.float32)
    conf_val = val_df["confidence_target"].values.astype(np.float32)

    train_ds = PickAndPlaceDataset(X_train, y_abs_train, y_delta_train, fail_train, conf_train)
    val_ds = PickAndPlaceDataset(X_val, y_abs_val, y_delta_val, fail_val, conf_val)

    train_loader = DataLoader(train_ds, batch_size=256, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=512, shuffle=False)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = AbsoluteTargetPolicyNet(x_mean, x_std).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1.5e-3, weight_decay=1e-4)
    EPOCHS = 45
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS, eta_min=5e-5)

    bce_loss_fn = nn.BCELoss()
    history = {
        "epochs": [],
        "train_total_loss": [],
        "val_total_loss": [],
        "train_joint_loss": [],
        "val_joint_loss": [],
        "train_conf_loss": [],
        "val_conf_loss": [],
    }

    best_val_loss = float("inf")
    start_t = time.time()

    for epoch in range(1, EPOCHS + 1):
        model.train()
        tr_tot, tr_j, tr_c, n_batches = 0.0, 0.0, 0.0, 0
        for bx, by_abs, _, b_fail, b_conf in train_loader:
            bx, by_abs, b_fail, b_conf = (
                bx.to(device),
                by_abs.to(device),
                b_fail.to(device),
                b_conf.to(device),
            )
            optimizer.zero_grad()
            pred = model(bx)
            pred_joints = pred[:, :7]
            pred_conf = pred[:, 7]

            # Masque : seuls les exemples nominaux (is_failure == 0) supervisent q_target !
            nominal_mask = (1.0 - b_fail).unsqueeze(1)
            sq_err = (pred_joints - by_abs) ** 2 * nominal_mask
            denom = torch.clamp(nominal_mask.sum() * 7.0, min=1.0)
            loss_joints = sq_err.sum() / denom

            # Perte de détection d'échec / confiance sur tous les exemples
            loss_conf = bce_loss_fn(pred_conf, b_conf)
            loss = loss_joints + 0.15 * loss_conf

            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 2.0)
            optimizer.step()

            tr_tot += float(loss.item())
            tr_j += float(loss_joints.item())
            tr_c += float(loss_conf.item())
            n_batches += 1

        scheduler.step()

        # Validation
        model.eval()
        va_tot, va_j, va_c, v_batches = 0.0, 0.0, 0.0, 0
        with torch.no_grad():
            for bx, by_abs, _, b_fail, b_conf in val_loader:
                bx, by_abs, b_fail, b_conf = (
                    bx.to(device),
                    by_abs.to(device),
                    b_fail.to(device),
                    b_conf.to(device),
                )
                pred = model(bx)
                pred_joints = pred[:, :7]
                pred_conf = pred[:, 7]

                nominal_mask = (1.0 - b_fail).unsqueeze(1)
                sq_err = (pred_joints - by_abs) ** 2 * nominal_mask
                denom = torch.clamp(nominal_mask.sum() * 7.0, min=1.0)
                loss_joints = sq_err.sum() / denom
                loss_conf = bce_loss_fn(pred_conf, b_conf)
                loss = loss_joints + 0.15 * loss_conf

                va_tot += float(loss.item())
                va_j += float(loss_joints.item())
                va_c += float(loss_conf.item())
                v_batches += 1

        tr_tot /= n_batches
        tr_j /= n_batches
        tr_c /= n_batches
        va_tot /= v_batches
        va_j /= v_batches
        va_c /= v_batches

        history["epochs"].append(epoch)
        history["train_total_loss"].append(tr_tot)
        history["val_total_loss"].append(va_tot)
        history["train_joint_loss"].append(tr_j)
        history["val_joint_loss"].append(va_j)
        history["train_conf_loss"].append(tr_c)
        history["val_conf_loss"].append(va_c)

        if va_j < best_val_loss:
            best_val_loss = va_j
            torch.save(model.state_dict(), "models/best_robot_policy.pt")

        if epoch == 1 or epoch % 5 == 0 or epoch == EPOCHS:
            print(
                f"Epoch {epoch:02d}/{EPOCHS} | "
                f"Train MSE(q): {tr_j:.6f} | Val MSE(q): {va_j:.6f} | "
                f"Val BCE(conf): {va_c:.5f}"
            )

    print(f"Entraînement terminé en {time.time() - start_t:.1f}s | Best Val MSE(q_target): {best_val_loss:.6f}")

    # 2. Entraînement rapide du modèle de référence "Ancien Modèle Δq" (pour comparaison Avant/Après)
    print("-" * 72)
    print("Entraînement de la référence Ancien Modèle (Δq cumulatif) pour benchmark Avant/Après...")
    legacy_model = LegacyDeltaPolicyNet(in_dim=26).to(device)
    leg_opt = torch.optim.AdamW(legacy_model.parameters(), lr=1e-3)
    for ep in range(1, 21):
        legacy_model.train()
        for bx, _, by_delta, b_fail, _ in train_loader:
            bx, by_delta, b_fail = bx.to(device), by_delta.to(device), b_fail.to(device)
            nominal_mask = (1.0 - b_fail).unsqueeze(1)
            leg_opt.zero_grad()
            pred_d = legacy_model(bx)
            loss_d = ((pred_d - by_delta) ** 2 * nominal_mask).sum() / torch.clamp(nominal_mask.sum() * 7.0, min=1.0)
            loss_d.backward()
            leg_opt.step()
    torch.save(legacy_model.state_dict(), "models/legacy_delta_policy.pt")

    # 3. Export du meilleur modèle q_target vers ONNX et JSON (Native JS)
    model.load_state_dict(torch.load("models/best_robot_policy.pt", map_location="cpu"))
    model.eval()

    # Export des poids JSON pour le moteur TypeScript/JS natif (incluant x_mean et x_std)
    weights_dict = {
        "architecture": "AbsoluteTargetPolicyNet_35D_to_8D",
        "input_dim": INPUT_DIM,
        "output_dim": OUTPUT_DIM,
        "phases": PHASES,
    }
    for k, v in model.state_dict().items():
        weights_dict[k] = v.cpu().numpy().tolist()

    json_path = "public/models/robot_policy_weights.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(weights_dict, f)
    print(f"✅ Poids natifs TypeScript/JS exportés : {json_path}")

    # Export ONNX autonome (fichier unique sans .onnx.data externe)
    onnx_path = "public/models/robot_policy.onnx"
    dummy_in = torch.zeros((1, INPUT_DIM), dtype=torch.float32)
    torch.onnx.export(
        model,
        dummy_in,
        onnx_path,
        input_names=["observation_35d"],
        output_names=["target_config_8d"],
        opset_version=17,
    )
    import onnx
    onnx_model = onnx.load(onnx_path)
    onnx.save_model(onnx_model, onnx_path, save_as_external_data=False)
    data_file = onnx_path + ".data"
    if os.path.exists(data_file):
        os.remove(data_file)
    print(f"✅ Modèle ONNX autonome exporté       : {onnx_path} ({os.path.getsize(onnx_path) / 1024:.1f} KB)")

    with open("reports/training_history.json", "w", encoding="utf-8") as f:
        json.dump(history, f, indent=2)
    print("=" * 72)


if __name__ == "__main__":
    main()
