# SURGICAL ACTION AI — Jumeau Numérique Robotique 3D & Plateforme ML (`Three.js` + `Rapier3D` + `TypeScript`)

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](package.json)
[![Tests](https://img.shields.io/badge/tests-63%2F63%20passing-brightgreen.svg)](tests/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.6-blue.svg)](https://www.typescriptlang.org/)
[![Physics](https://img.shields.io/badge/Rapier3D-WASM%2060Hz-orange.svg)](https://rapier.rs/)
[![Deploy](https://img.shields.io/badge/Vercel-Live%20Production-black.svg)](https://surgical-action-ai.vercel.app)

Jumeau numérique robotique 3D haute fidélité interactif et basé sur la physique temps réel, développé avec **Three.js**, **TypeScript**, **Vite**, et le moteur physique WASM **Rapier3D** (`@dimforge/rapier3d-compat`) pour la plateforme **Surgical Action AI** (Robot-to-Action).

Le jumeau numérique reconstitue fidèlement le manipulateur articulé 6-DOF **DofBot** en aluminium anodisé vert (embases circulaires étagées, colonnes en laiton, carte contrôleur arrière avec antennes doubles, profilés en U usinés, caméra poignet embarquée et pince parallèle à parallélogramme à 4 barres avec engrenages de synchronisation), au sein d'une cellule chirurgicale inspirée de l'environnement NVIDIA Isaac Sim.

---

## 🌐 Liens & Déploiement en Ligne

- **Application Web en Production (Vercel) :**  
  👉 **[https://surgical-action-ai.vercel.app](https://surgical-action-ai.vercel.app)**
- **Partage Réseau Local (Wi-Fi) :**  
  `npm run share` *(accessible directement depuis n'importe quel PC, tablette ou smartphone du même réseau Wi-Fi)*
- **Tunnel Public Sécurisé (Cloudflare) :**  
  `npm run tunnel` *(génère un lien d'accès HTTPS mondial instantané sans configuration de port)*

---

## 1. Démarrage Rapide

```bash
# 1. Installation des dépendances
npm install

# 2. Lancement du serveur de développement local (Vite)
npm run dev

# 3. Lancement avec exposition réseau local (Wi-Fi / LAN)
npm run share

# 4. Lancement avec tunnel public sécurisé Cloudflare Quick Tunnel
npm run tunnel

# 5. Exécution de la suite complète des 63 tests automatisés (Vitest)
npm test

# 6. Compilation optimisée de production
npm run build
```

---

## 2. Architecture Modulaire (`src/`)

```text
src/
├── main.ts                         # Point d'entrée, initialisation de la cellule & boucle physique fixe (60 Hz)
├── style.css                       # Interface sombre executive-grade inspirée de NVIDIA Isaac Sim
├── scene/
│   ├── SceneManager.ts             # Scène Three.js, WebGLRenderer, tone mapping & gestion du viewport
│   ├── Lighting.ts                 # Éclairage chirurgical (key/fill/rim light) & ombres douces
│   ├── Environment.ts              # Colliders de sol, grille fluorescente orange, plateaux et obstacles
│   └── CameraManager.ts            # Caméra principale OrbitControls + Caméra poignet embarquée (retour PiP)
├── physics/
│   ├── PhysicsWorld.ts             # Monde Rapier3D WASM, unités SI, pas fixe 60 Hz & gravité standard
│   ├── RigidBodyManager.ts         # Corps dynamiques, cinématiques et fixes, tenseurs d'inertie & forces
│   ├── JointManager.ts             # Articulations RevoluteImpulseJoints, contrôleurs PD & compensation de gravité
│   ├── CollisionManager.ts         # Détection de collision en phase étroite et AABB orientées
│   ├── CollisionGroups.ts          # Masques de filtrage des groupes de collision Rapier3D
│   ├── MaterialProperties.ts       # Propriétés physiques (acier chirurgical, titane, silicone, caoutchouc)
│   ├── PhysicsModeManager.ts       # Gestionnaire du basculement entre Mode Cinématique et Mode Dynamique
│   └── PhysicsDebugger.ts          # Rendu filaire des colliders et points de contact physiques
├── robot/
│   ├── RobotConfiguration.ts       # Géométrie 6-DOF, masses réelles, centres de masse, limites et couples
│   ├── RobotState.ts               # Télémétrie temps réel (angles, vitesses, couples, TCP, état de pince)
│   ├── RobotJoints.ts              # Corps rigides Rapier3D des liaisons et actionneurs rotatifs
│   ├── Gripper.ts                  # Pince à parallélogramme à 4 barres, mors rigides, friction et palonnier J6
│   └── RobotModel.ts               # Modèle 3D DofBot haute fidélité, fixations servos, caméra et liaison TCP
├── control/
│   ├── ForwardKinematics.ts        # Modèle Géométrique Direct (MGD / FK) 6-DOF & Jacobienne géométrique 6×6
│   ├── InverseKinematics.ts        # Modèle Géométrique Inverse (MGI / IK) par moindres carrés amortis (DLS)
│   ├── JointController.ts          # Contrôleur asservi en position, vitesse et couple saturé
│   └── MotionController.ts         # Coordinateur de trajectoire IK-moteurs avec arrêt automatique sur collision
├── objects/
│   ├── ObjectProperties.ts         # Définitions physiques des 6 instruments/objets (masses, inerties, couleurs)
│   ├── InstrumentFactory.ts        # Générateur dynamique d'objets, application de forces et gestion Rapier
│   ├── WorkZoneManager.ts          # Gestionnaire des plateaux Zone A (Départ) et Zone B (Cible) avec 6 slots
│   └── ScenarioGenerator.ts        # Générateur déterministe de scénarios ML (Niveaux 1 à 5, PRNG Mulberry32)
├── manipulation/
│   ├── ContactDetector.ts          # Évaluation bilatérale du contact physique sur les patins des doigts
│   ├── GraspManager.ts             # Préhension Coulombienne (2 * mu_eff * Fn >= 1.05 * m * |g|) et glissement
│   ├── PlacementVerifier.ts        # Vérification du positionnement, de l'orientation verticale et de la stabilité
│   └── PickAndPlace.ts             # Machine d'état autonome en 9 étapes pour le transfert unitaire et multi-objets
├── recovery/
│   ├── FailureDetector.ts          # Détection des modes d'échec physiques & émission du rapport structuré
│   └── RecoveryManager.ts          # Freinage de sécurité, attente d'autorisation & procédure de reprise
├── interaction/
│   ├── ObjectManipulator.ts        # Manipulation directe 3D à la souris (prise, glissement, élévation, rotation)
│   ├── InteractionModeManager.ts   # Basculement entre Mode Objet et Mode Robot
│   └── GripperTargetController.ts  # Contrôle de la cible IK par gizmo 3D avec vérification de portée
├── ai/
│   ├── ObservationBuilder.ts       # Espace d'observation global & vecteur normalisé plat 30D (o_t in R^30)
│   ├── ActionInterface.ts          # Interface d'action normalisée 7-DOF (a_t in [-1, 1]^7) avec velocity clipping
│   ├── EpisodeValidator.ts         # Évaluateur en 7 étapes (Reach, Grasp, Lift, Transport, Placement, Release, Stability)
│   ├── DataQualityValidator.ts     # Inspection de qualité (NaNs, limites, sauts de téléportation, score de conformité)
│   └── EpisodeRecorder.ts          # Enregistreur cadencé à 20 Hz, anti-idling, exports consolidés JSON / CSV
└── ui/
    ├── Dashboard.ts                # Télémétrie en temps réel dans la barre supérieure & coordination des panneaux
    ├── RobotControls.ts            # Sliders des 6 servomoteurs, coordonnées IK XYZ et force/ouverture de pince
    ├── PhysicsControls.ts          # Réglages de gravité, friction, restitution, gains Kp/Kd et générateur d'objets
    └── SimulationControls.ts       # Sélecteur de scénarios ML, enregistrement continu, inspection qualité & exports
```

---

## 3. Pipeline Robot-to-Action & Machine Learning (ML-Ready)

Le jumeau numérique est entièrement instrumenté pour la collecte de démonstrations d'experts et l'entraînement de politiques robotiques autonomes (**Behavior Cloning**, **Diffusion Policy**, **ACT / ALOHA**, **Flow Matching VLA**, **HuggingFace LeRobot**, et **NVIDIA Isaac Lab**).

```
┌────────────────────────────────────────────────────────────────────────┐
│                   BOUCLE FERMÉE D'APPRENTISSAGE IA                     │
│                                                                        │
│   Simulateur 3D           Espace d'Observation      Politique Neuronale│
│   (Rapier3D 60 Hz)        Vecteur Normalisé 30D     (BC / Diffusion)   │
│   ┌──────────────┐        ┌───────────────────┐     ┌──────────────┐   │
│   │ Robot DofBot │───────>│ o_t ∈ ℝ^30        │────>│ π_θ(a_t|o_t) │   │
│   │ 6-DOF + Pince│        │ (qpos, qvel, TCP, │     └──────┬───────┘   │
│   └──────▲───────┘        │  obj, target...)  │            │           │
│          │                └───────────────────┘            │           │
│          │ Action Normalisée 7-DOF                         │           │
│          └─────────────────────────────────────────────────┘           │
│            a_t ∈ [-1, 1]^7 (Δq_vel clamped ≤ 1.6 rad/s, gripper)       │
└────────────────────────────────────────────────────────────────────────┘
```

### A. Générateur de Scénarios Déterministe (`ScenarioGenerator.ts`)
- **PRNG Déterministe Mulberry32** garantissant une reproductibilité stricte graine par graine (`seed`).
- **5 Niveaux de Difficulté ML :**
  1. **Niveau 1 (Fixe Canonique) :** 1 objet placé au centre nominal de l'espace de travail.
  2. **Niveau 2 (Position & Lacet Aléatoires) :** Variations de translation ($r \in [0.24, 0.40]\,\text{m}$) et rotation en lacet ($\theta_{\text{yaw}} \in [-\pi, \pi]$).
  3. **Niveau 3 (Limites Espace de Travail) :** Positions extrêmes aux frontières de portée cinématique ($r \in [0.42, 0.48]\,\text{m}$).
  4. **Niveau 4 (Grille Multi-Objets) :** 6 instruments chirurgicaux simultanés disposés sans collision ($\Delta r \ge 0.065\,\text{m}$).
  5. **Niveau 5 (Perturbations Physiques) :** Randomisation de domaine (masses $\pm 30\%$, friction $\mu \in [0.3, 0.9]$, bruits sensoriels).

### B. Espace d'Observation Global & Vecteur Normalisé 30D (`ObservationBuilder.ts`)
Chaque pas de temps produit un état complet structuré ainsi qu'un **vecteur plat normalisé $\mathbf{o}_t \in \mathbb{R}^{30}$** prêt pour PyTorch :

| Index | Dimensions | Description | Normalisation |
| :---: | :---: | :--- | :--- |
| `0..5` | 6 | Positions articulaires ($q_1 \dots q_6$) | Radians $\in [-\pi, \pi]$ |
| `6..11` | 6 | Vitesses articulaires ($\dot{q}_1 \dots \dot{q}_6$) | $\text{rad/s}$ clampées à $[-2.5, 2.5]$ |
| `12..14` | 3 | Position cartésienne TCP $(X, Y, Z)$ | Mètres (espace base robot) |
| `15..18` | 4 | Orientation TCP (Quaternion $q_x, q_y, q_z, q_w$) | Quaternion unitaire normalisé |
| `19` | 1 | Ouverture de la pince ($d_{\text{grip}}$) | Mètres $\in [0.0, 0.078\,\text{m}]$ |
| `20..22` | 3 | Position de l'objet manipulé $(X_o, Y_o, Z_o)$ | Mètres (espace monde/base) |
| `23..26` | 4 | Orientation de l'objet (Quaternion) | Quaternion unitaire |
| `27..29` | 3 | Position de la boîte cible $(X_t, Y_t, Z_t)$ | Mètres (espace récepteur) |

### C. Espace d'Action Normalisé 7-DOF (`ActionInterface.ts`)
- Commande d'action relative continue $\mathbf{a}_t \in [-1, 1]^7$ :
  - $a_1 \dots a_6$ : Vitesses articulaires relatives proportionnelles, écrêtées de manière sécurisée à $|\dot{q}_{i,\max}| \le 1{,}6\,\text{rad/s}$.
  - $a_7$ : Commande continue d'ouverture/fermeture de la pince ($[-1, 1]$ avec seuils d'activation stricts).
- Protection matérielle : Clamping automatique sur les butées articulaires réelles avec amortissement en bordure.

### D. Validation d'Épisode en 7 Étapes (`EpisodeValidator.ts`)
Chaque épisode de démonstration est inspecté et labellisé automatiquement selon 7 critères séquentiels :
1. **Approche (`reach`) :** Distance TCP $\le 0{,}025\,\text{m}$ de l'objet.
2. **Prise (`grasp`) :** Contact bilatéral des doigts + force de serrage $\ge 12\,\text{N}$.
3. **Élévation (`lift`) :** Altitude objet élevée d'$au\ moins\ 0{,}03\,\text{m}$ au-dessus du plateau.
4. **Transport (`transport`) :** Déplacement stable de l'objet vers la zone cible sans perte de contact.
5. **Dépose (`placement`) :** Objet amené à moins de $0{,}045\,\text{m}$ du récepteur cible.
6. **Relâchement (`release`) :** Pince ouverte au-delà de $0{,}055\,\text{m}$ sans choc résiduel.
7. **Stabilité (`stability`) :** Objet immobile et stable pendant $\ge 0{,}5\,\text{s}$ ($\|\mathbf{v}\| < 0{,}01\,\text{m/s}$).

### E. Contrôle Qualité des Données (`DataQualityValidator.ts`)
Un inspecteur automatique analyse les trajectoires avant entraînement :
- Détection des valeurs corrompues (`NaN`, `±Infinity`).
- Détection des sauts de téléportation spatiale ($\|\Delta \mathbf{x}\| > 0{,}25\,\text{m}$ en un pas).
- Détection de gigue temporelle ($\Delta t \le 0$ ou $\Delta t > 0{,}2\,\text{s}$).
- Émission d'un score de conformité global (0 à 100%).

### F. Enregistreur & Exportateur Découplé 20 Hz (`EpisodeRecorder.ts`)
- Enregistrement cadencé à **20 Hz** (décimation propre 1:3 par rapport aux 60 Hz de la physique).
- Détection de fin de tâche par front montant : élimination des frames inactives résiduelles lorsque le robot est à l'arrêt.
- Export unitaire ou consolidé multi-scénarios en **JSON** et **CSV** directement compatibles avec les chargeurs de données PyTorch.

---

## 4. Points Forts de la Physique & Précision Mécanique (Unités SI)

- **Système d'Unités Internationales (SI)** : Longueurs en mètres ($\text{m}$), masses en kilogrammes ($\text{kg}$), temps en secondes ($\text{s}$), angles en radians ($\text{rad}$), forces en Newtons ($\text{N}$), couples en $\text{N}\cdot\text{m}$.
- **Gravité & Tenseurs d'Inertie** : Gravité standard $g = -9{,}81\,\text{m/s}^2$. Chaque corps possède une masse explicite, un centre de masse local et un tenseur d'inertie analytique $(I_{xx}, I_{yy}, I_{zz})$.
- **Couples Articulaires & Compensation de Gravité** : Calcul analytique à 60 Hz :
  $$\tau_{\text{gravité}, i} = \sum (\mathbf{r} \times m\mathbf{g}) \cdot \hat{\mathbf{u}}_i$$
- **Préhension Physique Coulombienne Réelle** : Aucune téléportation ni soudure artificielle. Le `GraspManager` valide la friction :
  $$F_{f,\max} = 2 \mu_{\text{eff}} F_N \ge 1{,}05 \, m |g|$$
  Si le serrage $F_N$ ou la friction est insuffisant, l'instrument glisse physiquement entre les doigts.
- **Assemblage Zéro-Jeu** : Accouplement poignet-pince sans flottement, paliers concentriques, oreilles de montage bridées par vis CHC argentées M2.5/M3, et cinématique à parallélogramme déformable à 4 barres.

---

## 5. Suite de Validation Automatisée (7 Suites — 63 Tests)

L'intégralité du jumeau numérique et de la chaîne de données ML est couverte par **63 tests automatisés** exécutés via Vitest :

```bash
npm test
```

```text
 ✓ tests/mlDataCollectionValidation.test.ts (10 tests)
   ✓ TEST 1 — Physics Determinism: Identical joint targets produce repeatable kinematic trajectories
   ✓ TEST 2 — Scenario Determinism: Mulberry32 PRNG generates identical poses for the same seed
   ✓ TEST 3 — Non-overlapping Spawning: Multi-object generation maintains minimum distance (>= 0.065 m)
   ✓ TEST 4 — 30D Observation Vector: Complete proprioception, TCP pose, ground truth objects & targets
   ✓ TEST 5 — 7D Action Interface: Clamps continuous actions [-1, 1]^7 to safe velocity limits (<= 1.6 rad/s)
   ✓ TEST 6 — 7-Stage Episode Validation: Evaluates Reach, Grasp, Lift, Transport, Placement, Release, Stability
   ✓ TEST 7 — Failure Reason Attribution: Correctly diagnoses reach failure on unreachable objects
   ✓ TEST 8 — Data Quality Inspection: Detects NaNs, joint limit violations, spatial jumps and time jitter
   ✓ TEST 9 — Multi-Episode Dataset Export: Exports clean consolidated JSON bundle with complete metadata
   ✓ TEST 10 — Closed-loop Policy Action Application: Safely drives arm joints toward target positions

 ✓ tests/servoAssemblyValidation.test.ts (8 tests)
   ✓ Parent-child motor anchoring, rotation around J1..J6 axes, no loose components during multi-joint motion

 ✓ tests/digitalTwinValidation.test.ts (10 tests)
   ✓ Gravity, F=ma acceleration, joint limits, payload torque, collision stop, grasp & friction slip, 9-stage sequence

 ✓ tests/hybridPhysicsAndAssembly.test.ts (7 tests)
   ✓ Turret/base continuity, J1..J6 rotation axes, 4-bar gripper synchronization, Kinematic/Dynamic mode switching

 ✓ tests/dualObjectZonesValidation.test.ts (10 tests)
   ✓ Zone A & Zone B platforms, 6 simultaneous instruments, transfer sequence, live progress tracking, randomization

 ✓ tests/physicsCollisionValidation.test.ts (8 tests)
   ✓ Settling without tunneling, 3-box stacking stability, friction, collision avoidance, grasp & release

 ✓ tests/mouseInteractionValidation.test.ts (10 tests)
   ✓ Mouse grab, dragging without floor penetration, vertical stacking, IK gizmo target control, dataset export

Test Files  7 passed (7)
     Tests  63 passed (63)
  Duration  11.47s
```

---

## 6. Déploiement Cloud (Vercel)

Le projet est configuré pour un déploiement continu instantané sur Vercel :

```bash
# Déploiement direct en production
npx vercel --prod
```

Le fichier de configuration Vite optimise automatiquement le découpage des modules (Three.js, Rapier3D WASM) pour garantir un chargement ultra-rapide sous les 3 secondes.

---

## 7. Licence

Ce projet est sous licence **MIT** — développé pour la recherche en robotique chirurgicale, la simulation de jumeaux numériques et l'apprentissage de politiques d'action IA (**Robot-to-Action**).
