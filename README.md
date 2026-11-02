# SURGICAL ACTION AI — Robotic Digital Twin (`Three.js` + `Rapier3D` + `TypeScript`)

An interactive, physics-based 3D Robotic Digital Twin built with **Three.js**, **TypeScript**, **Vite**, and **Rapier3D** (`@dimforge/rapier3d-compat`) for the **Surgical Action AI** platform.

The Digital Twin reconstructs the 6-DOF green anodized aluminum **DofBot** articulated manipulator (with stacked circular base plates, brass standoffs, rear expansion controller PCB with dual antennas, green U-bracket links, top-mounted wrist camera lens, and dark grey 4-bar geared parallel gripper) inside an NVIDIA Isaac Sim-inspired surgical engineering workspace.

---

## 1. Quick Start

```bash
# Install dependencies
npm install

# Run the interactive 3D Digital Twin in browser
npm run dev

# Run the 10-test physics & manipulation validation suite (Section 12)
npm test

# Production TypeScript & Vite bundle build
npm run build
```

---

## 2. Modular Architecture (`src/`)

```text
src/
├── main.ts                         # Application entry point & fixed-timestep physics loop
├── style.css                       # NVIDIA Isaac Sim dark engineering UI stylesheet
├── scene/
│   ├── SceneManager.ts             # Three.js Scene, WebGLRenderer, tone mapping & resize handler
│   ├── Lighting.ts                 # Surgical workcell key/fill/rim lighting & shadow maps
│   ├── Environment.ts              # Floor collider, glowing orange Isaac grid, Pickup Pad, Target Tray & Obstacle
│   └── CameraManager.ts            # Main OrbitControls camera + End-Effector Wrist Camera & PiP preview
├── physics/
│   ├── PhysicsWorld.ts             # Rapier3D WASM world, SI units, fixed 60 Hz timestep & gravity control
│   ├── RigidBodyManager.ts         # Dynamic/kinematic/fixed bodies, principal inertia tensors & external forces
│   ├── JointManager.ts             # RevoluteImpulseJoints, PD motor controllers, gravity/payload moment torque
│   ├── CollisionManager.ts         # Narrow-phase & oriented AABB collision detection across links/obstacles
│   └── MaterialProperties.ts       # Material presets (rubber, surgical_steel, metal, plastic) & Coulomb mu_eff
├── robot/
│   ├── RobotConfiguration.ts       # Configurable 6-DOF geometry, link masses, COMs, joint limits & motor specs
│   ├── RobotState.ts               # Real-time telemetry snapshots (angles, velocities, torques, TCP, gripper)
│   ├── RobotJoints.ts              # Base & link Rapier3D RigidBodies and revolute joint motor state
│   ├── Gripper.ts                  # Geared 4-bar linkage parallel gripper, finger RigidBodies & normal force
│   └── RobotModel.ts               # DofBot 3D visual reconstruction, wrist camera & kinematic synchronization
├── control/
│   ├── ForwardKinematics.ts        # 6-DOF analytical Forward Kinematics & 6×6 Geometric Jacobian
│   ├── InverseKinematics.ts        # Damped Least Squares (DLS) IK solver outputting feasible motor targets
│   ├── JointController.ts          # Position, velocity & torque-limited joint motor controller
│   └── MotionController.ts         # IK-to-motor trajectory coordinator with obstacle collision arrest
├── objects/
│   ├── ObjectProperties.ts         # 4 surgical instrument presets (rectangular, cylindrical, forceps, clip)
│   └── InstrumentFactory.ts        # Dynamic instrument spawner, mass/friction/restitution & force API
├── manipulation/
│   ├── ContactDetector.ts          # Bilateral left/right finger pad physical contact evaluation
│   ├── GraspManager.ts             # Coulomb friction grasping (2 * mu_eff * F_n >= 1.05 * m * |g|) & slip physics
│   ├── PlacementVerifier.ts        # Verifies target tray position, upright alignment, detachment & stability
│   └── PickAndPlace.ts             # 9-state autonomous Pick-and-Place state machine
├── recovery/
│   ├── FailureDetector.ts          # Detects 8 physical failure modes & emits Section 8 JSON payload
│   └── RecoveryManager.ts          # Safely brakes motors, awaits authorization & executes recovery strategy
├── ai/
│   ├── ObservationBuilder.ts       # Builds multimodal observation vectors (joints, torques, poses, camera)
│   ├── ActionInterface.ts          # External policy command interface (Cartesian, joint-space, torque, gripper)
│   └── EpisodeRecorder.ts          # Demonstration trajectory recorder & JSON exporter for PyTorch / Isaac Lab
└── ui/
    ├── Dashboard.ts                # Top-bar real-time telemetry & dock coordinator
    ├── RobotControls.ts            # 6-DOF joint motor sliders, IK target XYZ & gripper force/opening controls
    ├── PhysicsControls.ts          # Gravity, mass, friction, restitution, motor Kp/Kd/torque & object spawner
    └── SimulationControls.ts       # Play/Pause/Step/Reset, 9-state pipeline, Failure JSON viewer & AI Recorder
```

---

## 3. Physics & Manipulation Highlights (SI Units)

- **Units**: Meters ($\text{m}$), kilograms ($\text{kg}$), seconds ($\text{s}$), radians ($\text{rad}$), Newtons ($\text{N}$), and Newton-meters ($\text{N}\cdot\text{m}$).
- **Gravity & Inertia**: Default Earth gravity $g = -9.81\,\text{m/s}^2$. Every surgical instrument and link defines an explicit mass, center of mass, and analytical principal inertia tensor $(I_{xx}, I_{yy}, I_{zz})$.
- **Joint Motor Torque & Payload Dynamics**: Each revolute joint motor computes the exact 3D gravitational + held-payload moment arm torque $\tau_{\text{gravity}, i} = \sum (\mathbf{r} \times m\mathbf{g}) \cdot \hat{\mathbf{u}}_i$ at every fixed substep, combines it with PD feedback ($K_p e_\theta - K_d \dot{\theta}$), and enforces torque saturation (`maxMotorTorqueNm`).
- **Physical Friction Grasping & Slip Dynamics**: Closing the gripper never attaches an object automatically. `ContactDetector` verifies bilateral physical contact on both finger pads, and `GraspManager` evaluates the Coulomb static friction capacity:
  $$F_{f,\max} = 2 \mu_{\text{eff}} F_N \ge 1.05 \, m |g|$$
  When gripping force $F_N$ or surface friction $\mu$ is reduced below the threshold, $F_{f,\max}$ cannot support $m|g|$ and the instrument physically slips and accelerates downward through the fingers under Rapier3D gravity.

---

## 4. Validation Suite (Section 12 — 10 Automated Tests)

All 10 mandatory validation tests are implemented in `tests/digitalTwinValidation.test.ts`:

1. **TEST 1 — Gravity**: Drops a surgical instrument and verifies free-fall acceleration and landing on the Pickup Station pad.
2. **TEST 2 — Mass**: Applies an identical external force to light ($0.10\,\text{kg}$) and heavy ($1.00\,\text{kg}$) instruments and verifies $F = ma$ acceleration scaling.
3. **TEST 3 — Joint Movement**: Drives all 6 joints (`J1`–`J6`) and verifies mechanical joint limits and safe input clamping.
4. **TEST 4 — Torque**: Increases held payload mass from $0.05\,\text{kg}$ to $1.50\,\text{kg}$ and verifies increased shoulder/elbow motor torque.
5. **TEST 5 — Collision**: Enables the workspace obstacle and verifies the manipulator cannot pass through solid obstacles.
6. **TEST 6 — Grasping**: Verifies bilateral finger contact, friction holding capacity, and physical lifting of an instrument.
7. **TEST 7 — Slipping**: Reduces gripping force and friction while holding a lifted instrument and verifies slip detection and downward fall under gravity.
8. **TEST 8 — Pick and Place**: Executes the complete 9-state manipulation pipeline (`IDLE` → `APPROACH` → `GRASP` → `GRASP_VERIFICATION` → `LIFT` → `TRANSPORT` → `PLACE` → `VERIFY_PLACEMENT` → `COMPLETE`) and verifies stable placement on the Target Tray.
9. **TEST 9 — Recovery**: Injects an intentional grasp failure, verifies the structured JSON failure report (`{ task_id: "surgical_001", status: "failure", error: "grasp_failed", retry_allowed: true, object_state: "not_secured" }`), and executes authorized recovery to completion.
10. **TEST 10 — Data Collection**: Records multimodal observations and actions during manipulation and exports the structured JSON dataset for Behavior Cloning, Imitation Learning, Diffusion Policy, and Flow Matching.
