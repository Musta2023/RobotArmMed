/**
 * @file main.ts
 * Application entry point for the SURGICAL ACTION AI — ROBOTIC DIGITAL TWIN.
 * Orchestrates Three.js rendering, Rapier3D real-time physics, 6-DOF DofBot
 * kinematics & motor control, friction grasping, 9-state Pick-and-Place,
 * failure recovery, and Imitation Learning dataset recording.
 */

import './style.css';
import { PhysicsWorld } from './physics/PhysicsWorld.js';
import { RigidBodyManager } from './physics/RigidBodyManager.js';
import { CollisionManager } from './physics/CollisionManager.js';
import { JointManager } from './physics/JointManager.js';
import { PhysicsModeManager } from './physics/PhysicsModeManager.js';
import { PhysicsDebugger } from './physics/PhysicsDebugger.js';
import { RobotJoints } from './robot/RobotJoints.js';
import { Gripper } from './robot/Gripper.js';
import { RobotModel } from './robot/RobotModel.js';
import { JointController } from './control/JointController.js';
import { MotionController } from './control/MotionController.js';
import { InstrumentFactory } from './objects/InstrumentFactory.js';
import { WorkZoneManager } from './objects/WorkZoneManager.js';
import { ScenarioGenerator } from './objects/ScenarioGenerator.js';
import { ContactDetector } from './manipulation/ContactDetector.js';
import { GraspManager } from './manipulation/GraspManager.js';
import { PlacementVerifier } from './manipulation/PlacementVerifier.js';
import { PickAndPlace } from './manipulation/PickAndPlace.js';
import { FailureDetector } from './recovery/FailureDetector.js';
import { RecoveryManager } from './recovery/RecoveryManager.js';
import { ObservationBuilder } from './ai/ObservationBuilder.js';
import { ActionInterface } from './ai/ActionInterface.js';
import { EpisodeRecorder } from './ai/EpisodeRecorder.js';
import { NeuralPolicyController } from './ai/NeuralPolicyController.js';
import { Environment } from './scene/Environment.js';
import { SceneManager } from './scene/SceneManager.js';
import { ObjectManipulator } from './interaction/ObjectManipulator.js';
import { GripperTargetController } from './interaction/GripperTargetController.js';
import { InteractionModeManager } from './interaction/InteractionModeManager.js';
import { RobotControls } from './ui/RobotControls.js';
import { PhysicsControls } from './ui/PhysicsControls.js';
import { SimulationControls } from './ui/SimulationControls.js';
import { Dashboard } from './ui/Dashboard.js';

export interface DigitalTwinAppOptions {
  viewportContainer?: HTMLElement | null;
  wristPipContainer?: HTMLElement | null;
  leftDockContainer?: HTMLElement | null;
  rightDockTopContainer?: HTMLElement | null;
  rightDockBottomContainer?: HTMLElement | null;
  headless?: boolean;
  sixObjects?: boolean;
}

export async function createDigitalTwinApp(options: DigitalTwinAppOptions = {}) {
  // 1. Initialize Rapier3D Physics World (SI units: m, kg, s, N, N·m)
  const physicsWorld = new PhysicsWorld({
    gravityY: -9.81,
    fixedTimestep: 1 / 60,
    solverIterations: 16
  });
  await physicsWorld.init();

  const bodyManager = new RigidBodyManager(physicsWorld);
  const collisionManager = new CollisionManager(physicsWorld, bodyManager);
  const jointManager = new JointManager(physicsWorld);

  // 2. Build 6-DOF DofBot Articulated Robot Model + Mechanical Gripper + Wrist Camera
  const robotJoints = new RobotJoints(bodyManager, jointManager);
  const gripper = new Gripper(bodyManager);
  const robotModel = new RobotModel(robotJoints, gripper);

  // 2b. Initialize Hybrid Physics Mode Manager (Kinematic vs Dynamic authority)
  const physicsModeManager = new PhysicsModeManager({
    robotJoints,
    robotModel,
    jointManager,
    physicsWorld,
    initialMode: 'KINEMATIC'
  });

  // 3. Initialize Surgical Environment, WorkZoneManager & Instrument Factory
  const environment = new Environment(bodyManager);
  const workZoneManager = new WorkZoneManager();
  const instrumentFactory = new InstrumentFactory(bodyManager);

  // In browser mode (or when explicitly requested), initialize all 6 distinct objects in Zone A
  const spawnSix = options.sixObjects ?? (!options.headless);
  if (spawnSix) {
    instrumentFactory.setupSixZoneObjects();
  } else {
    instrumentFactory.spawnInstrument('rectangular_instrument', {
      id: 'surgical_instrument_01'
    });
  }

  // 4. Initialize 3D SceneManager
  const sceneManager = new SceneManager(
    options.headless ? null : (options.viewportContainer ?? null),
    options.headless ? null : (options.wristPipContainer ?? null),
    robotModel,
    instrumentFactory
  );
  sceneManager.scene.add(environment.root);
  sceneManager.scene.add(robotModel.root);
  sceneManager.scene.add(instrumentFactory.sceneGroup);

  // 5. Initialize Control Layer (FK, DLS IK, Joint PD Motor Controller, MotionController)
  const jointController = new JointController(robotModel, jointManager);
  const motionController = new MotionController(jointController, collisionManager);

  // 6. Initialize Physical Manipulation, Failure Recovery & Physics Debugger
  const contactDetector = new ContactDetector(gripper, instrumentFactory);
  const graspManager = new GraspManager(gripper, contactDetector, physicsWorld);
  const placementVerifier = new PlacementVerifier();
  const failureDetector = new FailureDetector();
  const recoveryManager = new RecoveryManager(failureDetector, motionController, gripper);
  const physicsDebugger = new PhysicsDebugger(
    bodyManager,
    collisionManager,
    robotModel,
    sceneManager.scene
  );

  const pickAndPlace = new PickAndPlace({
    motionController,
    gripper,
    graspManager,
    placementVerifier,
    instrumentFactory,
    failureDetector,
    recoveryManager,
    workZoneManager
  });

  // 7. Initialize 3D Mouse Interaction & Manipulation Controllers (Modes A, B, C)
  const objectManipulator = new ObjectManipulator(instrumentFactory, graspManager);
  if (typeof window !== 'undefined' && sceneManager.renderer?.domElement) {
    objectManipulator.initTransformControls(
      sceneManager.scene,
      sceneManager.cameraManager.mainCamera,
      sceneManager.renderer.domElement,
      sceneManager.cameraManager.orbitControls
    );
  }
  const gripperTargetController = new GripperTargetController(
    robotModel,
    motionController,
    sceneManager.scene,
    sceneManager.cameraManager,
    sceneManager.renderer?.domElement ?? null
  );
  const interactionModeManager = new InteractionModeManager({
    objectManipulator,
    gripperTargetController,
    robotModel,
    instrumentFactory,
    pickAndPlace,
    cameraManager: sceneManager.cameraManager,
    domElement: sceneManager.renderer?.domElement ?? null
  });

  // 8. Initialize AI Observation, Action, Scenario Generator & Episode Dataset Recorder
  const observationBuilder = new ObservationBuilder(
    robotModel,
    instrumentFactory,
    collisionManager
  );
  const actionInterface = new ActionInterface(motionController, gripper);
  const episodeRecorder = new EpisodeRecorder();
  const scenarioGenerator = new ScenarioGenerator({
    instrumentFactory,
    workZoneManager,
    robotModel
  });
  const neuralPolicyController = new NeuralPolicyController({
    motionController,
    gripper,
    observationBuilder,
    collisionManager
  });

  // Simulation playback state
  let isPlaying = true;
  let heldPayloadMassKg = 0;

  /**
   * Executes one deterministic fixed physics substep (dt = 1/60 s).
   */
  const executeFixedSubstep = (dt: number = physicsWorld.fixedTimestep): void => {
    // a. Step autonomous neural policy OR rule-based Pick-and-Place state machine
    if (neuralPolicyController.isRunning) {
      neuralPolicyController.step(dt, pickAndPlace.targetPosition);
    } else if (
      pickAndPlace.state !== 'IDLE' &&
      pickAndPlace.state !== 'COMPLETE' &&
      pickAndPlace.state !== 'FAILED'
    ) {
      pickAndPlace.step(dt);
    }

    // a2. Step physics-based 3D mouse object dragging if active
    objectManipulator.step(dt);

    // b. Step robot joint PD motors & check obstacle / dynamic object collisions
    collisionManager.graspedInstrumentId = graspManager.isGrasping
      ? graspManager.graspedInstrumentId
      : null;
    const activePayload = graspManager.isGrasping && graspManager.graspedInstrument
      ? graspManager.graspedInstrument.config.massKg
      : heldPayloadMassKg;
    motionController.stepControlAndMotors(dt, activePayload);

    // c. Step physical friction grasping & slip dynamics
    const graspStatus = graspManager.stepGraspPhysics(dt);
    collisionManager.graspedInstrumentId = graspStatus.graspedInstrumentId;

    // d. Step Rapier3D physics solver
    physicsWorld.stepSingle();

    // e. Synchronize Three.js visual meshes from Rapier3D rigid bodies & update telemetry
    bodyManager.syncVisualMeshes();
    robotModel.captureStateSnapshot(
      physicsWorld.simulationTime,
      graspStatus.isGrasping,
      graspStatus.isSlipping,
      graspStatus.graspedInstrumentId,
      graspStatus.payloadMassKg,
      graspStatus.leftFingerContact,
      graspStatus.rightFingerContact,
      graspStatus.frictionCapacityNewtons
    );

    if (physicsDebugger.enabled) {
      physicsDebugger.update();
    }

    // f. Record AI Imitation Learning / Manual Demonstration sample if recording is active
    if (episodeRecorder.isRecording) {
      const obs = observationBuilder.buildObservation(pickAndPlace.targetPosition);
      const targetPose = gripperTargetController.getTargetPose();
      const activeInst = instrumentFactory.getActiveInstrument();

      episodeRecorder.recordStep({
        observation: obs,
        action: {
          target_position: [...targetPose.position],
          target_orientation: [...targetPose.quaternion],
          gripper_command: gripper.isClosingCommandActive ? 'close' : 'open',
          gripping_force_n: gripper.grippingForceNewtons
        },
        contacts: collisionManager.activeContacts,
        taskState: pickAndPlace.state,
        physicsMode: physicsModeManager.mode.toLowerCase() as 'kinematic' | 'dynamic',
        isGraspVerified: graspManager.isGrasping && !graspStatus.isSlipping,
        gripperTargetPose: {
          position: targetPose.position,
          orientation: targetPose.quaternion
        },
        objectMetadata: activeInst
          ? {
              id: activeInst.id,
              name: activeInst.config.name,
              dimensions: activeInst.getDimensions(),
              mass_kg: activeInst.getMass()
            }
          : undefined
      });
    }
  };

  /**
   * Helper to step the simulation deterministically N times (used by UI single-step & Vitest).
   */
  const stepSimulation = (steps: number = 1, dt: number = physicsWorld.fixedTimestep): void => {
    for (let i = 0; i < steps; i++) {
      executeFixedSubstep(dt);
    }
  };

  const setHeldPayloadMassKg = (massKg: number): void => {
    heldPayloadMassKg = Math.max(0, massKg);
  };

  const resetRobotOnly = (): void => {
    objectManipulator.releaseObject();
    graspManager.releaseGrasp();
    pickAndPlace.reset();
    motionController.moveToHomePose(true);
    gripperTargetController.syncToCurrentTCP();
    bodyManager.syncVisualMeshes();
    robotModel.captureStateSnapshot(physicsWorld.simulationTime, false, false, null, 0);
  };

  const resetObjectsOnly = (): void => {
    objectManipulator.releaseObject();
    graspManager.releaseGrasp();
    workZoneManager.resetSlotOccupancy();
    if (instrumentFactory.instruments.has('Object_01')) {
      instrumentFactory.resetAllInstrumentsToZoneA();
    } else {
      instrumentFactory.resetAllInstruments();
    }
    bodyManager.syncVisualMeshes();
  };

  const resetAll = (): void => {
    objectManipulator.releaseObject();
    graspManager.releaseGrasp();
    pickAndPlace.reset();
    workZoneManager.resetSlotOccupancy();
    failureDetector.clearActiveFailure();
    motionController.moveToHomePose(true);
    gripperTargetController.syncToCurrentTCP();
    if (instrumentFactory.instruments.has('Object_01')) {
      instrumentFactory.resetAllInstrumentsToZoneA();
    } else {
      instrumentFactory.resetAllInstruments();
    }
    physicsWorld.resetClock();
    bodyManager.syncVisualMeshes();
    robotModel.captureStateSnapshot(0, false, false, null, 0);
  };

  // 9. Mount Engineering UI Panels if DOM containers are provided
  const robotControls = new RobotControls(
    options.leftDockContainer ?? null,
    robotModel,
    motionController,
    interactionModeManager
  );

  const physicsControls = new PhysicsControls(
    options.rightDockTopContainer ?? null,
    physicsWorld,
    jointManager,
    instrumentFactory,
    environment,
    graspManager,
    physicsDebugger
  );

  const simulationControls = new SimulationControls(
    options.rightDockBottomContainer ?? null,
    pickAndPlace,
    episodeRecorder,
    robotModel,
    {
      isPlaying: () => isPlaying,
      setPlaying: (val: boolean) => {
        isPlaying = val;
      },
      stepOnce: () => stepSimulation(1),
      resetAll,
      resetRobotOnly,
      resetObjectsOnly
    },
    scenarioGenerator,
    neuralPolicyController
  );

  const dashboard = new Dashboard({
    physicsWorld,
    collisionManager,
    robotModel,
    instrumentFactory,
    pickAndPlace,
    robotControls,
    physicsControls,
    simulationControls
  });

  // Auto-preload trained Neural Policy in background
  if (typeof window !== 'undefined') {
    neuralPolicyController.loadModel().catch((e) => {
      console.warn('[NeuralPolicy] Background auto-load notice:', e);
    });
  }

  // 9. Start Real-Time Render & Fixed-Timestep Physics Loop if in browser
  let animFrameId: number | null = null;
  let lastWallTimeMs = performance.now();

  if (!options.headless && options.viewportContainer && typeof requestAnimationFrame !== 'undefined') {
    const animate = (nowMs: number) => {
      animFrameId = requestAnimationFrame(animate);
      const deltaSeconds = Math.min((nowMs - lastWallTimeMs) / 1000, 0.1);
      lastWallTimeMs = nowMs;

      try {
        if (isPlaying) {
          physicsWorld.step(deltaSeconds, (fixedDt) => {
            executeFixedSubstep(fixedDt);
          });
        }
      } catch (err) {
        console.error('[DigitalTwin] Error in physics step:', err);
      }

      try {
        sceneManager.render();
      } catch (err) {
        console.error('[DigitalTwin] Error in scene render:', err);
      }

      try {
        dashboard.update(nowMs);
      } catch (err) {
        console.error('[DigitalTwin] Error in dashboard update:', err);
      }
    };

    animFrameId = requestAnimationFrame(animate);
  }

  return {
    physicsWorld,
    bodyManager,
    collisionManager,
    jointManager,
    physicsDebugger,
    environment,
    sceneManager,
    robotJoints,
    physicsModeManager,
    gripper,
    robotModel,
    jointController,
    motionController,
    instrumentFactory,
    contactDetector,
    graspManager,
    placementVerifier,
    failureDetector,
    recoveryManager,
    workZoneManager,
    setupSixObjectsInZoneA: () => instrumentFactory.setupSixZoneObjects(),
    pickAndPlace,
    objectManipulator,
    gripperTargetController,
    interactionModeManager,
    observationBuilder,
    actionInterface,
    episodeRecorder,
    neuralPolicyController,
    dashboard,
    stepSimulation,
    setHeldPayloadMassKg,
    resetAll,
    resetRobotOnly,
    resetObjectsOnly,
    dispose: () => {
      if (animFrameId !== null && typeof cancelAnimationFrame !== 'undefined') {
        cancelAnimationFrame(animFrameId);
      }
    }
  };
}

// Auto-bootstrap when loaded in browser DOM
if (typeof document !== 'undefined') {
  const togglePipBtn = document.getElementById('btn-toggle-pip');
  const pipCard = document.getElementById('wrist-pip-card-widget');
  if (togglePipBtn && pipCard) {
    togglePipBtn.addEventListener('click', () => {
      pipCard.classList.toggle('minimized');
      togglePipBtn.textContent = pipCard.classList.contains('minimized') ? '□' : '_';
    });
  }

  const viewportEl = document.getElementById('viewport-container');
  if (viewportEl) {
    createDigitalTwinApp({
      viewportContainer: viewportEl,
      wristPipContainer: document.getElementById('wrist-pip-card-host'),
      leftDockContainer: document.getElementById('left-dock-content'),
      rightDockTopContainer: document.getElementById('right-dock-top'),
      rightDockBottomContainer: document.getElementById('right-dock-bottom'),
      headless: false
    }).catch((err) => {
      console.error('Failed to initialize Surgical Action AI Digital Twin:', err);
    });
  }
}
