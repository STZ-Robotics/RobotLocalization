// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A simulated Limelight camera: PhotonVision generates the detections and this class publishes
 * them using the NetworkTables schema of a real Limelight.
 *
 * <p>The idea is that production robot code does not change a single line. If your subsystem
 * reads {@code LimelightHelpers.getTX("limelight-front")} against real hardware, it will read
 * the same thing against this simulation.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * public class Robot extends TimedRobot {
 *     private final SimulatedLimelight frontCamera =
 *             new SimulatedLimelight(
 *                     "limelight-front",
 *                     new Transform3d(
 *                             new Translation3d(0.3, 0.0, 0.25),
 *                             new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0)),
 *                     LimelightSimConfig.limelight3G());
 *
 *     @Override
 *     public void simulationPeriodic() {
 *         frontCamera.periodic(drivetrain.getSimulatedPose());
 *     }
 * }
 * }</pre>
 *
 * <h2>On a real robot</h2>
 * The constructor detects {@code RobotBase.isReal()} and creates nothing: no
 * {@code VisionSystemSim}, no {@code SimDevice}, no NetworkTables entries.
 * {@link #periodic(Pose2d)} becomes an immediate return, so you can leave the instance
 * declared at no cost during competition.
 *
 * <h2>Out of scope for now</h2>
 * <ul>
 *   <li>The {@code json} key / {@code LimelightHelpers.getLatestResults}.</li>
 *   <li>Neural detector pipelines ({@code tclass}, {@code rawdetections}).</li>
 *   <li>Modelling a movable crosshair: {@code tx} and {@code txnc} come out equal.</li>
 * </ul>
 */
public class SimulatedLimelight implements AutoCloseable {

    private static final List<SimulatedLimelight> INSTANCES = new ArrayList<>();
    private static final Map<String, Integer> NEXT_INDEX = new HashMap<>();

    private final String name;
    private final LimelightSimConfig config;
    private final boolean enabled;

    private final VisionSimWorld world;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final LimelightTargetTranslator translator;
    private final LimelightNTPublisher publisher;
    private final LimelightSimDevice simDevice;
    private final Matrix<N3, N3> cameraMatrix;
    private final Matrix<N8, N1> distCoeffs;

    private Transform3d robotToCamera;
    private LimelightFrame lastFrame = LimelightFrame.empty(0.0, 0.0, 0.0);
    private long frameCounter = 0;

    /**
     * Creates a simulated camera with the default configuration (Limelight 3G profile, the
     * feature's default field).
     *
     * @param name          the camera's name; used as the NetworkTables table and as the
     *                      device name in the Sim GUI.
     * @param robotToCamera transform from the robot origin to the camera, in the WPILib
     *                      convention (X forward, Y left, Z up).
     */
    public SimulatedLimelight(String name, Transform3d robotToCamera) {
        this(name, robotToCamera, new LimelightSimConfig());
    }

    /**
     * Creates a simulated camera.
     *
     * <p>The {@code name} wins: it overwrites the configuration's {@code tableName}, so the name
     * you see in the Sim GUI and the NT table's name cannot diverge.
     *
     * @param name          the camera's name.
     * @param robotToCamera transform from the robot origin to the camera, in the WPILib convention.
     * @param config        configuration of optics, field and keys to publish.
     */
    public SimulatedLimelight(String name, Transform3d robotToCamera, LimelightSimConfig config) {
        this.name = name;
        this.config = config.withTableName(name);
        this.robotToCamera = robotToCamera;
        this.enabled = RobotBase.isSimulation();

        if (!enabled) {
            world = null;
            camera = null;
            cameraSim = null;
            translator = null;
            publisher = null;
            simDevice = null;
            cameraMatrix = null;
            distCoeffs = null;
            return;
        }

        world = VisionSimWorld.get(config.visionWorldName(), config.fieldLayout());
        camera = new PhotonCamera(name);
        // The layout is passed explicitly. PhotonCameraSim keeps its own to solve the multi-tag PnP,
        // and the two-argument constructor hands it AprilTagFields.kDefaultField --- which on WPILib
        // 2026.1.1 is the 2025 field. With the world on 2026 that means detecting tags with one
        // geometry and solving them against another: the pose comes out twelve metres from where the
        // robot is, with no error anywhere.
        cameraSim = new PhotonCameraSim(camera, config.cameraProperties(), config.fieldLayout());
        world.sim().addCamera(cameraSim, robotToCamera);

        // Photonlib does not limit the range by default, so the simulated camera sees any tag inside
        // the FOV however far away and straight through anything --- VisionSystemSim does not model
        // occlusion. Measured against the 2026 field: with no limit it saw seven tags, five of them
        // between 8 and 13 m with ambiguities from 0.6 to 0.99, and the multi-tag PnP mixed them with
        // the nearby ones and landed a metre and a half off the ground. A real Limelight does not
        // resolve a 16.5 cm tag at that distance.
        cameraSim.setMaxSightRange(config.maxSightRangeMeters());
        cameraSim.setMinTargetAreaPixels(config.minTargetAreaPixels());

        // The viewer. Off by default: rendering with OpenCV on every frame costs real CPU, and most
        // of the time the only thing needed is the NT keys.
        cameraSim.enableRawStream(config.rawStream());
        cameraSim.enableDrawWireframe(config.wireframe());
        cameraSim.enableProcessedStream(config.processedStream());
        if (config.publishDebugField()) {
            SmartDashboard.putData("VisionSim/" + config.visionWorldName(), world.debugField());
        }

        cameraMatrix = config.cameraProperties().getIntrinsics();
        distCoeffs = config.cameraProperties().getDistCoeffs();

        translator =
                new LimelightTargetTranslator(
                        config.fieldLayout(), robotToCamera, config.megaTag2HeadingScaleFactor());
        publisher = new LimelightNTPublisher(this.config);
        simDevice = config.createSimDevice() ? new LimelightSimDevice(name, allocateIndex(name)) : null;

        synchronized (INSTANCES) {
            INSTANCES.add(this);
        }
    }

    private static synchronized int allocateIndex(String name) {
        int index = NEXT_INDEX.getOrDefault(name, 0);
        NEXT_INDEX.put(name, index + 1);
        return index;
    }

    /**
     * Advances every simulated camera created so far with the same robot pose. A shortcut for the
     * normal case: one call in {@code simulationPeriodic()} and done.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public static void periodicAll(Pose2d simulatedRobotPose) {
        periodicAll(new Pose3d(simulatedRobotPose));
    }

    /**
     * Advances every simulated camera created so far with the same robot pose.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        List<SimulatedLimelight> snapshot;
        synchronized (INSTANCES) {
            snapshot = List.copyOf(INSTANCES);
        }
        for (SimulatedLimelight limelight : snapshot) {
            limelight.periodic(simulatedRobotPose);
        }
    }

    /**
     * Returns the active simulated cameras.
     *
     * @return an immutable list of instances.
     */
    public static List<SimulatedLimelight> instances() {
        synchronized (INSTANCES) {
            return List.copyOf(INSTANCES);
        }
    }

    /**
     * The camera's cycle. Call it from {@code simulationPeriodic()} with the pose your odometry or
     * your chassis simulation treats as the truth.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public void periodic(Pose2d simulatedRobotPose) {
        periodic(new Pose3d(simulatedRobotPose));
    }

    /**
     * The camera's cycle, with a 3D pose.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public void periodic(Pose3d simulatedRobotPose) {
        if (!enabled) {
            return;
        }

        LimelightNTInputs inputs = publisher.readInputs();
        applyCameraPoseOverride(inputs);

        world.update(simulatedRobotPose);

        PhotonPipelineResult result = latestAllowedResult(inputs);
        if (result == null) {
            return;
        }

        LimelightFrame frame =
                translator.translate(result, inputs, cameraMatrix, distCoeffs, config.publishMegaTag2());

        if (simDevice != null && simDevice.forceNoTargets()) {
            frame =
                    LimelightFrame.empty(
                            frame.pipelineLatencyMs(),
                            frame.captureLatencyMs(),
                            frame.captureTimestampSeconds());
        }

        publisher.publish(frame, inputs);
        if (simDevice != null) {
            simDevice.update(frame, inputs);
        }
        lastFrame = frame;
    }

    /**
     * Returns the most recent result that survives the throttle, or {@code null} if this cycle
     * brought no new frame.
     *
     * <p>Having no new frame is normal: the camera runs at its own FPS, not the robot loop's. In
     * that case NetworkTables is not touched and consumers keep seeing the last published frame,
     * exactly as with real hardware.
     */
    private PhotonPipelineResult latestAllowedResult(LimelightNTInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return null;
        }

        int throttle = config.honorThrottle() ? Math.max(0, inputs.throttle()) : 0;
        if (throttle == 0) {
            return results.get(results.size() - 1);
        }

        PhotonPipelineResult kept = null;
        for (PhotonPipelineResult result : results) {
            if (frameCounter % (throttle + 1) == 0) {
                kept = result;
            }
            frameCounter++;
        }
        return kept;
    }

    /** Applies, on the fly, the {@code camerapose_robotspace_set} the robot code wrote. */
    private void applyCameraPoseOverride(LimelightNTInputs inputs) {
        inputs.cameraPoseOverride()
                .filter(override -> !override.equals(robotToCamera))
                .ifPresent(this::setRobotToCamera);
    }

    /**
     * Moves the camera relative to the robot. Equivalent to
     * {@code LimelightHelpers.setCameraPose_RobotSpace}, but from simulation code.
     *
     * @param robotToCamera the new robot-&gt;camera transform.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        if (!enabled) {
            return;
        }
        translator.setRobotToCamera(robotToCamera);
        world.sim().adjustCamera(cameraSim, robotToCamera);
    }

    /** @return the camera's name, which is also the NT table's. */
    public String name() {
        return name;
    }

    /** @return true if the simulation is active (that is, we are not on a real robot). */
    public boolean isEnabled() {
        return enabled;
    }

    /** @return the robot-&gt;camera transform in effect. */
    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    /** @return the last translated frame, useful for telemetry or tests. */
    public LimelightFrame lastFrame() {
        return lastFrame;
    }

    /** @return the configuration this camera was created with. */
    public LimelightSimConfig config() {
        return config;
    }

    /** @return the shared simulated world, or {@code null} on a real robot. */
    public VisionSimWorld world() {
        return world;
    }

    /** @return the underlying {@code PhotonCamera}, or {@code null} on a real robot. */
    public PhotonCamera camera() {
        return camera;
    }

    /** @return the underlying {@code PhotonCameraSim}, or {@code null} on a real robot. */
    public PhotonCameraSim cameraSim() {
        return cameraSim;
    }

    /**
     * Releases the camera and its {@code SimDevice}. On a real robot there is nothing to release.
     *
     * <p>The shared simulated world survives: other cameras may still be using it. To wipe it
     * entirely use {@link VisionSimWorld#reset()}.
     */
    @Override
    public void close() {
        synchronized (INSTANCES) {
            INSTANCES.remove(this);
        }
        if (!enabled) {
            return;
        }
        if (simDevice != null) {
            simDevice.close();
        }
        world.sim().removeCamera(cameraSim);
        cameraSim.close();
        camera.close();
    }

    /** Closes every simulated camera. Meant for tests. */
    public static void closeAll() {
        List<SimulatedLimelight> snapshot;
        synchronized (INSTANCES) {
            snapshot = new ArrayList<>(INSTANCES);
        }
        Collections.reverse(snapshot);
        for (SimulatedLimelight limelight : snapshot) {
            limelight.close();
        }
        synchronized (SimulatedLimelight.class) {
            NEXT_INDEX.clear();
        }
    }
}
