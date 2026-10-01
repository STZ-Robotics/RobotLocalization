// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import gg.questnav.questnav.protos.generated.Commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * A Meta Quest running QuestNav, simulated.
 *
 * <p>It publishes to NetworkTables exactly what a real headset publishes, so production robot code
 * --- the code that uses {@code gg.questnav.questnav.QuestNav} --- works unchanged and without
 * hardware. It is the counterpart of {@code SimulatedLimelight}: there we stand in for the camera,
 * here we stand in for the headset.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * public class Robot extends TimedRobot {
 *     private final SimulatedQuestNav quest =
 *             new SimulatedQuestNav(
 *                     new Transform3d(0, -0.332, 0, new Rotation3d(0, 0, Math.toRadians(-90))),
 *                     new QuestNavSimConfig());
 *
 *     @Override
 *     public void simulationPeriodic() {
 *         quest.periodic(drivetrain.getSimulatedPose());
 *     }
 * }
 * }</pre>
 *
 * The {@code Transform3d} is the same {@code ROBOT_TO_QUEST} you pass to your real subsystem.
 *
 * <h2>What makes it different from a simulated camera</h2>
 * <ul>
 *   <li><b>It always reports something.</b> A Limelight either sees the tag or it does not; a Quest
 *       always gives a pose, just with error. That is why the real content lives in
 *       {@link VioErrorModel} and not in the geometry.</li>
 *   <li><b>Several frames per cycle.</b> The headset runs at 100 Hz and the robot at 50, so each
 *       {@code periodic()} publishes ~2 frames with timestamps 10 ms apart, interpolating the pose in
 *       between. That is what makes {@code getAllUnreadPoseFrames()} return what it would really
 *       return.</li>
 *   <li><b>An origin of its own.</b> The Quest does not report field poses until the robot code calls
 *       {@code setPose()}. See {@link QuestOriginFrame}.</li>
 * </ul>
 *
 * <h2>Mind {@code dataTimestamp()}</h2>
 * By default publication happens on the same NetworkTables instance the robot code consumes, and in
 * that case {@code PoseFrame.dataTimestamp()} is <b>useless</b>: it is a constant 1 microsecond. That
 * is not a bug in this class but in the topology --- that value comes from the {@code serverTime} the
 * NT server assigns on <i>receipt</i>, and ntcore marks everything published locally with the sentinel
 * {@code serverTime = 1}.
 *
 * <p>If your robot code fuses on timestamps, which is the normal case for a pose estimator, switch on
 * {@link QuestNavSimConfig#withNetworkClient(boolean)}: the simulation then publishes as an NT4 client
 * against the robot's server, just like the real headset, and the timestamps come out correct.
 * Everything else --- pose, tracking, battery, commands --- works the same in both modes.
 *
 * <h2>On a real robot</h2>
 * The constructor detects {@code RobotBase.isReal()} and creates nothing. {@link #periodic(Pose2d)} is
 * an immediate return.
 */
public class SimulatedQuestNav implements AutoCloseable {

    /** Cap on frames per cycle, so a stall in the loop does not set off an avalanche. */
    private static final int MAX_FRAMES_PER_LOOP = 20;

    private static final List<SimulatedQuestNav> INSTANCES = new ArrayList<>();
    private static final Map<String, Integer> NEXT_INDEX = new HashMap<>();

    private final String name;
    private final QuestNavSimConfig config;
    private final boolean enabled;

    /** Our own NetworkTables instance, only when publishing as a network client. */
    private final NetworkTableInstance ownedInstance;

    private final QuestNavNTBridge bridge;
    private final VioErrorModel errorModel;
    private final QuestOriginFrame originFrame;
    private final QuestNavSimDevice simDevice;
    private final QuestNavViewfinder viewfinder;

    private Transform3d robotToHeadset;

    private double startTimeSeconds = Double.NaN;
    private double lastPeriodicSeconds = Double.NaN;
    private double nextFrameSeconds = Double.NaN;
    private double nextDeviceDataSeconds = Double.NaN;
    private Pose3d previousTrueRobotPose;

    private int frameCount = 0;
    private boolean trackingLossLatch = false;
    private QuestNavFrame lastFrame;

    /**
     * Creates a simulated headset with the default configuration.
     *
     * @param robotToHeadset transform from the robot origin to the headset, in the WPILib convention.
     *                       It is the same value as your {@code ROBOT_TO_QUEST}.
     */
    public SimulatedQuestNav(Transform3d robotToHeadset) {
        this(robotToHeadset, new QuestNavSimConfig());
    }

    /**
     * Creates a simulated headset.
     *
     * @param robotToHeadset transform from the robot origin to the headset, in the WPILib convention.
     * @param config         configuration of the error model and of what gets published.
     */
    public SimulatedQuestNav(Transform3d robotToHeadset, QuestNavSimConfig config) {
        this.name = config.tableName();
        this.config = config;
        this.robotToHeadset = robotToHeadset;
        this.enabled = RobotBase.isSimulation();

        if (!enabled) {
            ownedInstance = null;
            bridge = null;
            errorModel = null;
            originFrame = null;
            simDevice = null;
            viewfinder = null;
            return;
        }

        ownedInstance = config.networkClient() ? createClientInstance(config) : null;
        bridge =
                ownedInstance != null
                        ? new QuestNavNTBridge(config, ownedInstance)
                        : new QuestNavNTBridge(config);
        errorModel = new VioErrorModel(config);
        originFrame =
                config.randomizedOrigin()
                        ? QuestOriginFrame.randomized(
                                config.randomOriginTranslationMeters(),
                                config.randomOriginRotationDegrees(),
                                new Random(config.randomSeed()))
                        : new QuestOriginFrame();
        simDevice =
                config.createSimDevice() ? new QuestNavSimDevice(name, allocateIndex(name)) : null;
        viewfinder =
                config.viewfinder() ? new QuestNavViewfinder(name, robotToHeadset, config) : null;

        synchronized (INSTANCES) {
            INSTANCES.add(this);
        }
    }

    /**
     * Creates the NetworkTables client instance that reproduces the real topology: the headset is an
     * NT4 client and the robot is the server. It is what makes {@code PoseFrame.dataTimestamp()} carry
     * a real {@code serverTime}. See {@link QuestNavSimConfig#withNetworkClient}.
     */
    private static NetworkTableInstance createClientInstance(QuestNavSimConfig config) {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startClient4(config.clientName());
        instance.setServer(config.serverAddress(), NetworkTableInstance.kDefaultPort4);
        return instance;
    }

    private static synchronized int allocateIndex(String name) {
        int index = NEXT_INDEX.getOrDefault(name, 0);
        NEXT_INDEX.put(name, index + 1);
        return index;
    }

    /**
     * Advances every simulated headset with the same robot pose.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public static void periodicAll(Pose2d simulatedRobotPose) {
        periodicAll(new Pose3d(simulatedRobotPose));
    }

    /**
     * Advances every simulated headset with the same robot pose.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        List<SimulatedQuestNav> snapshot;
        synchronized (INSTANCES) {
            snapshot = List.copyOf(INSTANCES);
        }
        for (SimulatedQuestNav quest : snapshot) {
            quest.periodic(simulatedRobotPose);
        }
    }

    /**
     * Returns the active simulated headsets.
     *
     * @return an immutable list of instances.
     */
    public static List<SimulatedQuestNav> instances() {
        synchronized (INSTANCES) {
            return List.copyOf(INSTANCES);
        }
    }

    /**
     * The headset's cycle. Call it from {@code simulationPeriodic()} with the pose your simulation
     * treats as the truth.
     *
     * @param simulatedRobotPose the robot's current simulated pose.
     */
    public void periodic(Pose2d simulatedRobotPose) {
        periodic(new Pose3d(simulatedRobotPose));
    }

    /**
     * The headset's cycle, with a 3D pose.
     *
     * <h2>Which pose to pass, and which not to</h2>
     * It has to be the <b>truth</b>: the pose from your physics simulation, or pure wheel odometry.
     * <b>Not</b> the one coming out of your pose estimator.
     *
     * <p>This is not a style preference. The physics feeds the sensor and the sensor feeds the
     * estimator; feeding the sensor the estimator's output closes a loop that does not exist on a real
     * robot. And it has a concrete symptom: a vision-corrected estimator <b>jumps</b> every cycle, and
     * those jumps enter the error model as accelerations of thousands of m/s2. The impact detector
     * mistakes them for collisions, the Quest loses tracking non-stop, and each re-acquisition adds its
     * offset until the pose wanders off the field.
     *
     * <p>The model filters isolated jumps with a median, but against periodic feedback no causal filter
     * will do. If you cannot give it a clean pose, switch the automatic detection off with
     * {@link QuestNavSimConfig#withImpactTrackingLoss(boolean)} and inject the losses by hand.
     *
     * @param simulatedRobotPose the robot's current simulated pose, with no vision corrections.
     */
    public void periodic(Pose3d simulatedRobotPose) {
        if (!enabled) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (Double.isNaN(startTimeSeconds)) {
            initialize(now, simulatedRobotPose);
            return;
        }

        applySimDeviceRequests(now);
        handleRobotRequests(simulatedRobotPose, now);

        boolean publishing = errorModel.isPoweredOn() && errorModel.isLinkUp(now);
        emitFrames(simulatedRobotPose, now, publishing);

        if (publishing && now >= nextDeviceDataSeconds) {
            bridge.publishDeviceData(errorModel.batteryPercent(), errorModel.trackingLostCounter());
            nextDeviceDataSeconds = now + 1.0 / config.deviceDataRateHz();
        }

        if (lastFrame != null) {
            if (config.publishTelemetry()) {
                bridge.publishTelemetry(
                        lastFrame.truePose(), lastFrame.questPose(), reportedRobotPose(lastFrame));
            }
            if (simDevice != null) {
                simDevice.update(lastFrame, errorModel, publishing);
            }
        }

        if (viewfinder != null) {
            viewfinder.update(simulatedRobotPose);
        }

        // When publishing over the network the buffer has to be pushed: otherwise this cycle's frames
        // sit waiting for the NetworkTables timer and arrive in bursts. The robot measures the
        // connection by how long ago the last frame arrived, so one burst every 100 ms leaves it
        // "disconnected" most of the time.
        if (ownedInstance != null && publishing) {
            bridge.flush();
        }

        previousTrueRobotPose = simulatedRobotPose;
        lastPeriodicSeconds = now;
    }

    private void initialize(double now, Pose3d robotPose) {
        startTimeSeconds = now;
        lastPeriodicSeconds = now;
        previousTrueRobotPose = robotPose;
        nextFrameSeconds = now;
        nextDeviceDataSeconds = now;
    }

    /** Translates the Sim GUI's switches into error model events. */
    private void applySimDeviceRequests(double now) {
        if (simDevice == null) {
            return;
        }

        // Level, not edge: for as long as the box is ticked, tracking stays lost.
        boolean requested = simDevice.forceTrackingLoss();
        if (requested) {
            errorModel.injectTrackingLoss(config.trackingLossSeconds(), now);
            trackingLossLatch = true;
        } else {
            trackingLossLatch = false;
        }

        // Pulse: one jump per tick, or the pose would hop every cycle.
        if (simDevice.forcePoseJump()) {
            errorModel.injectPoseDiscontinuity(
                    new Translation3d(config.reacquireOffsetMeters(), 0.0, 0.0));
            simDevice.clearPoseJumpRequest();
        }

        if (simDevice.forceLinkLoss()) {
            errorModel.injectLinkLoss(0.25, now);
        }
    }

    /** Services the commands the robot code published to {@code request}. */
    private void handleRobotRequests(Pose3d robotPose, double now) {
        Pose3d headsetPose = robotPose.transformBy(robotToHeadset);

        for (QuestNavNTBridge.Request request : bridge.readRequests()) {
            if (request.type() != Commands.QuestNavCommandType.POSE_RESET) {
                bridge.publishResponse(
                        request.commandId(), false, "Command not supported by the simulation");
                continue;
            }

            if (request.targetPose().isEmpty()) {
                bridge.publishResponse(request.commandId(), false, "POSE_RESET with no targetPose");
                continue;
            }

            originFrame.reset(headsetPose, request.targetPose().get());
            errorModel.resetDrift();
            bridge.publishResponse(request.commandId(), true, "");
        }
    }

    /**
     * Generates and publishes the frames due in this cycle.
     *
     * <p>The headset runs at its own rate, not the robot loop's, so it has to catch up: one frame is
     * emitted for each pending instant, interpolating the true pose between the previous cycle's and
     * the current one. Without that interpolation the intermediate frames would be copies of the same
     * value and the robot code would see a staircased pose the real hardware does not produce.
     */
    private void emitFrames(Pose3d robotPose, double now, boolean publishing) {
        double period = 1.0 / config.frameRateHz();
        double span = now - lastPeriodicSeconds;
        int emitted = 0;

        while (nextFrameSeconds <= now && emitted < MAX_FRAMES_PER_LOOP) {
            double frameTime = nextFrameSeconds;
            double fraction = span > 0 ? (frameTime - lastPeriodicSeconds) / span : 1.0;
            fraction = Math.max(0.0, Math.min(1.0, fraction));

            Pose3d trueRobotAtFrame = previousTrueRobotPose.interpolate(robotPose, fraction);
            Pose3d trueHeadsetAtFrame = trueRobotAtFrame.transformBy(robotToHeadset);

            Pose3d errored = errorModel.update(trueHeadsetAtFrame, frameTime, period);
            Pose3d questPose = originFrame.toQuestFrame(errored);

            QuestNavFrame frame =
                    new QuestNavFrame(
                            questPose,
                            trueRobotAtFrame,
                            errorModel.isTracking() && !trackingLossLatch,
                            ++frameCount,
                            frameTime - startTimeSeconds,
                            (long) (frameTime * 1e6));

            if (publishing) {
                bridge.publishFrame(frame);
            }
            lastFrame = frame;

            nextFrameSeconds += period;
            emitted++;
        }

        if (emitted >= MAX_FRAMES_PER_LOOP) {
            // The loop fell so far behind that catching up makes no sense: better to jump forward than
            // to carry a debt that never gets paid.
            nextFrameSeconds = now + period;
        }
    }

    private Pose3d reportedRobotPose(QuestNavFrame frame) {
        return frame.questPose().transformBy(robotToHeadset.inverse());
    }

    /**
     * Forces a tracking loss from code, without going through the Sim GUI.
     *
     * @param durationSeconds how long it lasts.
     */
    public void injectTrackingLoss(double durationSeconds) {
        if (enabled) {
            errorModel.injectTrackingLoss(durationSeconds, Timer.getFPGATimestamp());
        }
    }

    /**
     * Injects a pose jump without losing tracking, like the one the double-tap to passthrough causes.
     *
     * @param jump displacement in metres.
     */
    public void injectPoseDiscontinuity(Translation3d jump) {
        if (enabled) {
            errorModel.injectPoseDiscontinuity(jump);
        }
    }

    /**
     * Simulates a network drop: publishing stops and the robot sees {@code isConnected()} fall.
     *
     * @param durationSeconds how long the drop lasts.
     */
    public void injectLinkLoss(double durationSeconds) {
        if (enabled) {
            errorModel.injectLinkLoss(durationSeconds, Timer.getFPGATimestamp());
        }
    }

    /**
     * Changes the headset's mounting on the fly.
     *
     * @param robotToHeadset the new robot-&gt;headset transform.
     */
    public void setRobotToHeadset(Transform3d robotToHeadset) {
        this.robotToHeadset = robotToHeadset;
        if (viewfinder != null) {
            viewfinder.setRobotToHeadset(robotToHeadset);
        }
    }

    /** @return the headset's name, which is also the NT table's. */
    public String name() {
        return name;
    }

    /** @return true if the simulation is active. */
    public boolean isEnabled() {
        return enabled;
    }

    /** @return the robot-&gt;headset transform in effect. */
    public Transform3d robotToHeadset() {
        return robotToHeadset;
    }

    /** @return the last generated frame, or {@code null} if there is none yet. */
    public QuestNavFrame lastFrame() {
        return lastFrame;
    }

    /** @return the error model, to inspect or tune it from a test. */
    public VioErrorModel errorModel() {
        return errorModel;
    }

    /** @return the headset's origin frame. */
    public QuestOriginFrame originFrame() {
        return originFrame;
    }

    /** @return the configuration it was created with. */
    public QuestNavSimConfig config() {
        return config;
    }

    /** @return the viewfinder, or {@code null} if it is off. */
    public QuestNavViewfinder viewfinder() {
        return viewfinder;
    }

    /** Releases the resources. On a real robot there is nothing to release. */
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
        if (viewfinder != null) {
            viewfinder.close();
        }
        bridge.close();
        if (ownedInstance != null) {
            ownedInstance.stopClient();
            ownedInstance.close();
        }
    }

    /**
     * Says whether the simulation is delivering data to the robot.
     *
     * <p>In local mode it is always true. When publishing as a network client, it is false until the
     * connection to the robot's NT4 server is established: if it stays false, the robot is not bringing
     * its server up and not a single frame is going to arrive.
     *
     * @return true if the transport is ready.
     */
    public boolean isPublishing() {
        if (!enabled) {
            return false;
        }
        return ownedInstance == null || ownedInstance.isConnected();
    }

    /** Closes every simulated headset. Meant for tests. */
    public static void closeAll() {
        List<SimulatedQuestNav> snapshot;
        synchronized (INSTANCES) {
            snapshot = new ArrayList<>(INSTANCES);
        }
        Collections.reverse(snapshot);
        for (SimulatedQuestNav quest : snapshot) {
            quest.close();
        }
        synchronized (SimulatedQuestNav.class) {
            NEXT_INDEX.clear();
        }
    }
}
