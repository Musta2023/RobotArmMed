# SURGICAL ACTION AI — Jumeau Numérique Robotique 3D (`Three.js` + `Rapier3D` + `TypeScript`)

Jumeau numérique robotique 3D interactif et basé sur la physique temps réel, développé avec **Three.js**, **TypeScript**, **Vite**, et le moteur physique WASM **Rapier3D** (`@dimforge/rapier3d-compat`) pour la plateforme **Surgical Action AI**.

Le jumeau numérique reconstitue fidèlement le manipulateur articulé 6-DOF **DofBot** en aluminium anodisé vert (avec embases circulaires étagées, colonnes en laiton, carte contrôleur arrière avec antennes doubles, profilés en U usinés, caméra embarquée au poignet, et pince parallèle à parallélogramme déformable à 4 barres avec engrenages de synchronisation), au sein d'une cellule robotique inspirée de l'environnement NVIDIA Isaac Sim.

---

## 1. Démarrage Rapide

```bash
# Installation des dépendances
npm install

# Lancement de l'application interactive 3D dans le navigateur
npm run dev

# Exécution de l'ensemble des 45 tests de validation automatisés
npm test

# Compilation de production TypeScript & Vite
npm run build
```

---

## 2. Architecture Modulaire (`src/`)

```text
src/
├── main.ts                         # Point d'entrée de l'application & boucle physique à pas fixe (60 Hz)
├── style.css                       # Feuille de style UI sombre inspirée de NVIDIA Isaac Sim
├── scene/
│   ├── SceneManager.ts             # Scène Three.js, WebGLRenderer, tone mapping & gestion du redimensionnement
│   ├── Lighting.ts                 # Éclairage chirurgical (key/fill/rim light) & cartes d'ombres douces
│   ├── Environment.ts              # Colliders de sol, grille fluorescente orange, plateaux et obstacles
│   └── CameraManager.ts            # Caméra principale OrbitControls + Caméra poignet embarquée avec retour PiP
├── physics/
│   ├── PhysicsWorld.ts             # Monde Rapier3D WASM, unités SI, pas de temps fixe 60 Hz & contrôle de gravité
│   ├── RigidBodyManager.ts         # Corps dynamiques, cinématiques et fixes, tenseurs d'inertie & forces externes
│   ├── JointManager.ts             # Articulations RevoluteImpulseJoints, contrôleurs PD & couples de gravité/charge
│   ├── CollisionManager.ts         # Détection de collision en phase étroite et AABB orientées
│   ├── CollisionGroups.ts          # Masques de filtrage des groupes de collision Rapier3D
│   ├── MaterialProperties.ts       # Propriétés de matériaux (acier chirurgical, caoutchouc, aluminium, plastique)
│   ├── PhysicsModeManager.ts       # Gestionnaire du basculement entre Mode Cinématique et Mode Dynamique
│   └── PhysicsDebugger.ts          # Rendu visuel filaire des colliders et contacts physiques
├── robot/
│   ├── RobotConfiguration.ts       # Géométrie 6-DOF configurable, masses, centres de masse, limites et moteurs
│   ├── RobotState.ts               # Télémétrie en temps réel (angles, vitesses, couples, TCP, état de pince)
│   ├── RobotJoints.ts              # Corps rigides Rapier3D des liaisons et actionneurs rotatifs
│   ├── Gripper.ts                  # Pince à parallélogramme à 4 barres, mors rigides, friction et palonnier J6
│   └── RobotModel.ts               # Modèle 3D DofBot haute fidélité, fixations servos, caméra poignet et liaison TCP
├── control/
│   ├── ForwardKinematics.ts        # Modèle Géométrique Direct (MGD / FK) 6-DOF & Jacobienne géométrique 6×6
│   ├── InverseKinematics.ts        # Modèle Géométrique Inverse (MGI / IK) par moindres carrés amortis (DLS)
│   ├── JointController.ts          # Contrôleur d'articulation asservi en position, vitesse et couple saturé
│   └── MotionController.ts         # Coordinateur de trajectoire IK-moteurs avec arrêt automatique sur collision
├── objects/
│   ├── ObjectProperties.ts         # Définitions physiques des 6 instruments/objets (masses, inerties, couleurs)
│   ├── InstrumentFactory.ts        # Générateur dynamique d'objets, application de forces et gestion Rapier
│   └── WorkZoneManager.ts          # Gestionnaire des plateaux Zone A (Départ) et Zone B (Cible) avec 6 slots
├── manipulation/
│   ├── ContactDetector.ts          # Évaluation bilatérale du contact physique sur les patins des doigts
│   ├── GraspManager.ts             # Préhension Coulombienne (2 * mu_eff * Fn >= 1.05 * m * |g|) et glissement
│   ├── PlacementVerifier.ts        # Vérification du positionnement, de l'orientation verticale et de la stabilité
│   └── PickAndPlace.ts             # Machine d'état autonome en 9 étapes pour le transfert unitaire et multi-objets
├── recovery/
│   ├── FailureDetector.ts          # Détection des modes d'échec physiques & émission du rapport JSON structuré
│   └── RecoveryManager.ts          # Freinage de sécurité, attente d'autorisation & procédure de reprise
├── interaction/
│   ├── ObjectManipulator.ts        # Manipulation directe 3D à la souris (prise, glissement au sol, élévation, rotation)
│   ├── InteractionModeManager.ts   # Basculement entre Mode Objet et Mode Robot
│   └── GripperTargetController.ts  # Contrôle de la cible IK par gizmo 3D avec vérification de portée
├── ai/
│   ├── ObservationBuilder.ts       # Vecteurs d'observation multimodaux (articulations, couples, poses, caméra)
│   ├── ActionInterface.ts          # Interface de commande de politique (cartésien, articulaire, couple, pince)
│   └── EpisodeRecorder.ts          # Enregistreur continu de trajectoires & export JSON / CSV pour l'apprentissage IA
└── ui/
    ├── Dashboard.ts                # Télémétrie en temps réel dans la barre supérieure & coordination des panneaux
    ├── RobotControls.ts            # Sliders des 6 servomoteurs, coordonnées IK XYZ et force/ouverture de pince
    ├── PhysicsControls.ts          # Réglages de gravité, friction, restitution, gains Kp/Kd et générateur d'objets
    └── SimulationControls.ts       # Contrôles de simulation, séquence Zone A ➔ B, visualiseur d'échec & Enregistreur IA
```

---

## 3. Points Forts de la Physique et de la Manipulation (Unités SI)

- **Système d'Unités Internationales (SI)** : Longueurs en mètres ($\text{m}$), masses en kilogrammes ($\text{kg}$), temps en secondes ($\text{s}$), angles en radians ($\text{rad}$), forces en Newtons ($\text{N}$), et couples en Newton-mètres ($\text{N}\cdot\text{m}$).
- **Gravité & Tenseurs d'Inertie** : Gravité terrestre standard $g = -9{,}81\,\text{m/s}^2$. Chaque instrument et corps articulé possède une masse explicite, un centre de masse local et un tenseur d'inertie analytique $(I_{xx}, I_{yy}, I_{zz})$.
- **Couples Articulaires & Compensation de Gravité** : À chaque sous-pas de 60 Hz, le couple de maintien gravitationnel et de charge utile est calculé analytiquement :
  $$\tau_{\text{gravité}, i} = \sum (\mathbf{r} \times m\mathbf{g}) \cdot \hat{\mathbf{u}}_i$$
  Ce couple est combiné au retour PD ($K_p e_\theta - K_d \dot{\theta}$) avec respect strict des limites de couple maximal (`maxMotorTorqueNm`).
- **Préhension Physique par Friction & Glissement Réel** : La fermeture de la pince ne verrouille jamais magiquement un objet. Le `ContactDetector` valide le contact physique bilatéral sur les deux mors, et le `GraspManager` vérifie l'inégalité de Coulomb :
  $$F_{f,\max} = 2 \mu_{\text{eff}} F_N \ge 1{,}05 \, m |g|$$
  Si la force de serrage $F_N$ ou le coefficient de friction $\mu_{\text{eff}}$ devient insuffisant, l'objet glisse naturellement entre les doigts sous l'action de la gravité Rapier3D.

---

## 4. Assemblage Mécanique Réaliste & Couplage Zéro-Jeu

- **Accouplement Rigide Poignet ➔ Pince (Zéro Flottement)** :
  - **Interface Stator (Link 4) :** Plaque d'extrémité en aluminium vert fermant le canal en U, bague statorique usinée argentée ($\varnothing = 0{,}034\,\text{m}$), douille centrale et 4 boulons périphériques.
  - **Flange Rotative J6 (Pince) :** Palonnier métallique circulaire usiné CNC monté à fleur contre le stator du poignet, avec 6 vis BTR concentriques.
  - **Embase Adaptatrice & Goussets :** Bloc d'embase anodisé noir reliant rigidement le palonnier rotatif aux plaques du châssis de la pince et au servomoteur de commande, éliminant tout espace d'air.
- **Fixation Réaliste de Tous les Servomoteurs aux Articulations** :
  - Chaque moteur servo (corps noir) intègre ses oreilles de montage latérales bridées par 4 vis CHC argentées M2.5/M3 dans les parois du bras.
  - L'arbre cannelé de sortie et le palonnier circulaire métallique sont montés de manière concentrique sur les axes d'articulation actifs (J1 à J6).
  - Un roulement passif / ergot de guidage assure le maintien mécanique opposé sur chaque chape articulée.

---

## 5. Fonctionnalités Avancées

### Double Plateau : Zone A (Départ) ➔ Zone B (Cible)
- **6 Objets distincts** simultanément présents sur le plateau de départ bleu (Zone A).
- **6 Emplacements cibles** dédiés sur le plateau d'arrivée vert (Zone B).
- Machine d'état séquentielle autonome capable de transférer les 6 objets l'un après l'autre avec suivi de progression en direct (`X / 6 (Y%)`, compteur d'objets restants et placés).
- Boutons dédiés : « Démarrer (6 Objets) », « Randomiser Zone A », « Reset Objets vers Zone A ».

### Interaction 3D Directe à la Souris & Gizmo
- Sélection directe au clic de n'importe quel instrument dans la scène 3D.
- Translation au sol avec détection de collision, élévation verticale et empilement naturel d'objets.
- **Rotation 3D complète des objets** à la souris (touche `E` ou bascule gizmo).
- Contrôle direct du point central outil (TCP) via gizmo 3D interactif avec résolution IK temps réel par moindres carrés amortis.

### Architecture Hybride : Mode Cinématique & Mode Dynamique
- Possibilité de basculer instantanément l'autorité physique entre :
  - **Mode Cinématique** : Idéal pour les trajectoires de haute précision, le contrôle IK stable et l'apprentissage de démonstrations.
  - **Mode Dynamique** : Simulation rigide complète soumise aux forces externes, impacts et gravité dans Rapier3D.

### Enregistreur IA Continu (Dataset Multi-Scénarios)
- L'Enregistreur IA ne s'interrompt pas automatiquement à la fin d'une tâche unitaire : il reste actif jusqu'à l'arrêt manuel par l'utilisateur.
- Permet de collecter plusieurs scénarios consécutifs (transferts multiples, perturbations, rattrapages d'échecs) dans un fichier unique.
- Export direct aux formats **CSV** et **JSON** prêts pour l'entraînement de modèles d'Imitation Learning, Diffusion Policy, Flow Matching ou Behavior Cloning.

### Interface 100% en Français
- L'intégralité de l'interface utilisateur, des métriques de télémétrie, des menus de configuration physique, des rapports d'échec et des alertes est localisée en français.

---

## 6. Suite de Validation Automatisée (6 Suites — 53 Tests)

L'intégralité de l'application est couverte par 53 tests automatisés exécutés via `vitest` :

1. **`tests/servoAssemblyValidation.test.ts` (8 tests)** :
   - Relations parent-enfant rigoureuses pour chaque servomoteur (boîtier fixé au lien parent, palonnier fixé au lien enfant).
   - Validation de l'articulation individuelle de chaque joint (`J1` à `J6`) vérifiant que le boîtier reste fixe pendant que le palonnier et le lien enfant tournent autour de l'axe.
   - Maintien continu de l'ancrage physique sans détachement ni composant flottant lors de mouvements multi-articulaires simultanés.
2. **`tests/digitalTwinValidation.test.ts` (10 tests)** :
   - Gravité, proportionnalité masse-accélération ($F=ma$), limites articulaires, couple de charge, arrêt sur collision, préhension physique, glissement par friction réduite, pipeline complet Pick-and-Place en 9 étapes, injection d'échec et reprise autorisée, collecte de données IA.
3. **`tests/hybridPhysicsAndAssembly.test.ts` (7 tests)** :
   - Continuité d'assemblage tourelle/embase, axes de rotation J1–J6, cinématique à 4 barres de la pince avec engrenages antagonistes, colliders composés de l'embase, commutation d'autorité Cinématique/Dynamique, télémétrie vérifiée et torsion axiale J4.
4. **`tests/dualObjectZonesValidation.test.ts` (10 tests)** :
   - Création et dimensions des plateaux Zone A et B, 6 emplacements distincts, placement simultané sans collision, géométrie des 6 objets chirurgicaux, cinématique de transfert, calcul de progression en direct, et randomisation dans les limites de zone.
5. **`tests/physicsCollisionValidation.test.ts` (8 tests)** :
   - Chute libre et stabilisation sans effet tunnel, empilement stable de 3 corps rigides, friction statique sur plan incliné, restitution de rebond, respect des collisions rigides du bras, préhension par FixedImpulseJoint, relâchement naturel et proximité multi-objets.
6. **`tests/mouseInteractionValidation.test.ts` (10 tests)** :
   - Saisie et relâchement à la souris, glissement au sol sans traverser le plancher, empilement vertical, contrôle du gizmo de pince avec IK, orientation du gizmo, saisie d'objet à position aléatoire, suivi de l'objet en préhension, et export des enregistrements de démonstration en JSON et CSV.

```bash
# Lancer les tests
npx vitest run
```
Résultat : **6 fichiers de test réussis, 53 tests validés à 100%**.

---

## 7. Licence

Ce projet est sous licence MIT — conçu pour la recherche en robotique chirurgicale, la simulation de jumeaux numériques et l'apprentissage de politiques d'action IA (Robot-to-Action).
