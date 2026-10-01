// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

/**
 * Writes to NetworkTables the exact schema of keys a real Limelight publishes.
 *
 * <p>This is the piece that keeps the robot code from ever noticing it is in simulation:
 * {@code LimelightHelpers}, Glass, OutlineViewer or a direct NT read all see the same table
 * with the same names, units and array formats.
 *
 * <p>It also reads the keys the robot code writes towards the camera ({@code pipeline},
 * {@code robot_orientation_set}, {@code fiducial_id_filters_set}...) and exposes them as
 * {@link LimelightNTInputs}, so the simulation honours them.
 *
 * <h2>What this feature does not publish</h2>
 * The {@code json} key (the full dump {@code LimelightHelpers.getLatestResults} consumes) is
 * out of scope for this iteration: reproducing that whole schema adds nothing the individual
 * keys do not already give. If your robot code depends on {@code getLatestResults}, replace
 * it with the direct getters.
 */
public class LimelightNTPublisher {

    /** The limit at which a real Limelight restarts its heartbeat. */
    private static final long HEARTBEAT_ROLLOVER = 2_000_000_000L;

    private static final double[] EMPTY = new double[0];

    private final AprilTagFieldLayout layout;
    private final NetworkTable table;
    private final LimelightSimConfig config;

    private final NetworkTableEntry tvEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry txncEntry;
    private final NetworkTableEntry tyncEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tsEntry;
    private final NetworkTableEntry tidEntry;
    private final NetworkTableEntry tlEntry;
    private final NetworkTableEntry clEntry;
    private final NetworkTableEntry hbEntry;
    private final NetworkTableEntry t2dEntry;
    private final NetworkTableEntry getpipeEntry;
    private final NetworkTableEntry getpipetypeEntry;
    private final NetworkTableEntry rawFiducialsEntry;
    private final NetworkTableEntry tcornxyEntry;

    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry botposeBlueEntry;
    private final NetworkTableEntry botposeRedEntry;
    private final NetworkTableEntry botposeOrbEntry;
    private final NetworkTableEntry botposeOrbBlueEntry;
    private final NetworkTableEntry botposeOrbRedEntry;

    private final NetworkTableEntry targetPoseCameraEntry;
    private final NetworkTableEntry targetPoseRobotEntry;
    private final NetworkTableEntry cameraPoseTargetEntry;
    private final NetworkTableEntry botPoseTargetEntry;

    private final NetworkTableEntry pipelineEntry;
    private final NetworkTableEntry ledModeEntry;
    private final NetworkTableEntry throttleEntry;
    private final NetworkTableEntry robotOrientationEntry;
    private final NetworkTableEntry fiducialFiltersEntry;
    private final NetworkTableEntry cameraPoseSetEntry;

    private long heartbeat = 0;

    /**
     * Creates the publisher on the configured table.
     *
     * @param config the simulated camera's configuration.
     */
    public LimelightNTPublisher(LimelightSimConfig config) {
        this(config, NetworkTableInstance.getDefault());
    }

    /**
     * Creates the publisher on a specific NetworkTables instance. Meant for tests, where an
     * isolated instance is preferable to the global one.
     *
     * @param config   the simulated camera's configuration.
     * @param instance the NetworkTables instance to use.
     */
    public LimelightNTPublisher(LimelightSimConfig config, NetworkTableInstance instance) {
        this.config = config;
        this.layout = config.fieldLayout();
        this.table = instance.getTable(config.tableName());

        this.tvEntry = table.getEntry("tv");
        this.txEntry = table.getEntry("tx");
        this.tyEntry = table.getEntry("ty");
        this.txncEntry = table.getEntry("txnc");
        this.tyncEntry = table.getEntry("tync");
        this.taEntry = table.getEntry("ta");
        this.tsEntry = table.getEntry("ts");
        this.tidEntry = table.getEntry("tid");
        this.tlEntry = table.getEntry("tl");
        this.clEntry = table.getEntry("cl");
        this.hbEntry = table.getEntry("hb");
        this.t2dEntry = table.getEntry("t2d");
        this.getpipeEntry = table.getEntry("getpipe");
        this.getpipetypeEntry = table.getEntry("getpipetype");
        this.rawFiducialsEntry = table.getEntry("rawfiducials");
        this.tcornxyEntry = table.getEntry("tcornxy");

        this.botposeEntry = table.getEntry("botpose");
        this.botposeBlueEntry = table.getEntry("botpose_wpiblue");
        this.botposeRedEntry = table.getEntry("botpose_wpired");
        this.botposeOrbEntry = table.getEntry("botpose_orb");
        this.botposeOrbBlueEntry = table.getEntry("botpose_orb_wpiblue");
        this.botposeOrbRedEntry = table.getEntry("botpose_orb_wpired");

        this.targetPoseCameraEntry = table.getEntry("targetpose_cameraspace");
        this.targetPoseRobotEntry = table.getEntry("targetpose_robotspace");
        this.cameraPoseTargetEntry = table.getEntry("camerapose_targetspace");
        this.botPoseTargetEntry = table.getEntry("botpose_targetspace");

        this.pipelineEntry = table.getEntry("pipeline");
        this.ledModeEntry = table.getEntry("ledMode");
        this.throttleEntry = table.getEntry("throttle_set");
        this.robotOrientationEntry = table.getEntry("robot_orientation_set");
        this.fiducialFiltersEntry = table.getEntry("fiducial_id_filters_set");
        this.cameraPoseSetEntry = table.getEntry("camerapose_robotspace_set");
    }

    /** @return the NetworkTables table being published to. */
    public NetworkTable table() {
        return table;
    }

    /**
     * Reads the keys the robot code writes towards the camera.
     *
     * <p>{@code camerapose_robotspace_set} is interpreted as
     * {@code [forward, left, up, roll, pitch, yaw]} in the WPILib convention, in metres and
     * degrees. That is the interpretation that makes the override consistent with the
     * {@code Transform3d} you pass to {@link SimulatedLimelight}'s constructor.
     *
     * @return a snapshot of the inputs.
     */
    public LimelightNTInputs readInputs() {
        double[] orientation = robotOrientationEntry.getDoubleArray(EMPTY);
        Optional<Rotation2d> heading =
                orientation.length >= 1
                        ? Optional.of(Rotation2d.fromDegrees(orientation[0]))
                        : Optional.empty();

        double[] filters = fiducialFiltersEntry.getDoubleArray(EMPTY);
        int[] ids = new int[filters.length];
        for (int i = 0; i < filters.length; i++) {
            ids[i] = (int) filters[i];
        }

        double[] cameraPose = cameraPoseSetEntry.getDoubleArray(EMPTY);
        Optional<Transform3d> override =
                cameraPose.length >= 6
                        ? Optional.of(
                                new Transform3d(
                                        new Translation3d(cameraPose[0], cameraPose[1], cameraPose[2]),
                                        new Rotation3d(
                                                Units.degreesToRadians(cameraPose[3]),
                                                Units.degreesToRadians(cameraPose[4]),
                                                Units.degreesToRadians(cameraPose[5]))))
                        : Optional.empty();

        return new LimelightNTInputs(
                (int) pipelineEntry.getDouble(0.0),
                (int) ledModeEntry.getDouble(0.0),
                (int) throttleEntry.getDouble(0.0),
                heading,
                config.honorFiducialFilters() ? ids : new int[0],
                config.honorCameraPoseOverride() ? override : Optional.empty());
    }

    /**
     * Publishes a complete frame. Every key of a given frame is written in the same call, so a
     * consumer never sees {@code tv} from one cycle alongside {@code botpose} from another.
     *
     * @param frame  the already translated frame.
     * @param inputs the inputs read this cycle, for echoes such as {@code getpipe}.
     */
    public void publish(LimelightFrame frame, LimelightNTInputs inputs) {
        tvEntry.setDouble(frame.hasTarget() ? 1.0 : 0.0);
        txEntry.setDouble(frame.tx());
        tyEntry.setDouble(frame.ty());
        txncEntry.setDouble(frame.txnc());
        tyncEntry.setDouble(frame.tync());
        taEntry.setDouble(frame.ta());
        tsEntry.setDouble(frame.ts());
        tidEntry.setDouble(frame.tid());
        tlEntry.setDouble(frame.pipelineLatencyMs());
        clEntry.setDouble(frame.captureLatencyMs());

        heartbeat = (heartbeat + 1) % HEARTBEAT_ROLLOVER;
        hbEntry.setDouble(heartbeat);

        getpipeEntry.setDouble(inputs.pipeline());
        getpipetypeEntry.setString("pipe_fiducial");
        t2dEntry.setDoubleArray(buildT2d(frame));

        if (config.publishRawFiducials()) {
            rawFiducialsEntry.setDoubleArray(LimelightCoordinates.rawFiducialsArray(frame.fiducials()));
        }
        if (config.publishCorners()) {
            tcornxyEntry.setDoubleArray(frame.cornersXY());
        }
        if (config.publishBotpose()) {
            publishBotpose(frame);
        }
        if (config.publishTargetSpacePoses()) {
            publishTargetSpacePoses(frame);
        }
    }

    private void publishBotpose(LimelightFrame frame) {
        publishPoseFamily(
                frame,
                frame.megaTag1Pose(),
                botposeEntry,
                botposeBlueEntry,
                botposeRedEntry,
                frame.fiducials());

        if (config.publishMegaTag2()) {
            // Same fiducials as MegaTag1: LimelightHelpers parses the _orb keys with the same
            // 11 + 7n format, only the PoseEstimate's isMegaTag2 flag changes.
            publishPoseFamily(
                    frame,
                    frame.megaTag2Pose(),
                    botposeOrbEntry,
                    botposeOrbBlueEntry,
                    botposeOrbRedEntry,
                    frame.fiducials());
        }
    }

    private void publishPoseFamily(
            LimelightFrame frame,
            Optional<Pose3d> bluePose,
            NetworkTableEntry centerEntry,
            NetworkTableEntry blueEntry,
            NetworkTableEntry redEntry,
            LimelightFrame.RawFiducial[] fiducials) {

        boolean wantsCenter =
                config.publishesBotposeVariant(LimelightSimConfig.BotposeVariant.LEGACY_CENTER);
        boolean wantsBlue =
                config.publishesBotposeVariant(LimelightSimConfig.BotposeVariant.WPI_BLUE);
        boolean wantsRed =
                config.publishesBotposeVariant(LimelightSimConfig.BotposeVariant.WPI_RED);

        if (bluePose.isEmpty()) {
            // Disabled origins are not touched at all: publishing an empty array every cycle spends
            // bandwidth to say the same thing as publishing nothing.
            if (wantsCenter) {
                centerEntry.setDoubleArray(EMPTY);
            }
            if (wantsBlue) {
                blueEntry.setDoubleArray(EMPTY);
            }
            if (wantsRed) {
                redEntry.setDoubleArray(EMPTY);
            }
            return;
        }

        Pose3d blue = bluePose.get();
        if (wantsBlue) {
            blueEntry.setDoubleArray(LimelightCoordinates.botposeArray(blue, frame, fiducials));
        }
        if (wantsRed) {
            redEntry.setDoubleArray(
                    LimelightCoordinates.botposeArray(
                            LimelightCoordinates.blueToRed(blue, layout), frame, fiducials));
        }
        if (wantsCenter) {
            centerEntry.setDoubleArray(
                    LimelightCoordinates.botposeArray(
                            LimelightCoordinates.blueToFieldCenter(blue, layout), frame, fiducials));
        }
    }

    private void publishTargetSpacePoses(LimelightFrame frame) {
        Optional<Transform3d> cameraToTarget = frame.cameraToTarget();
        if (cameraToTarget.isEmpty()) {
            targetPoseCameraEntry.setDoubleArray(EMPTY);
            targetPoseRobotEntry.setDoubleArray(EMPTY);
            cameraPoseTargetEntry.setDoubleArray(EMPTY);
            botPoseTargetEntry.setDoubleArray(EMPTY);
            return;
        }

        Transform3d camToTarget = cameraToTarget.get();
        targetPoseCameraEntry.setDoubleArray(LimelightCoordinates.toLimelightCameraSpace(camToTarget));
        cameraPoseTargetEntry.setDoubleArray(
                LimelightCoordinates.toLimelightCameraSpace(camToTarget.inverse()));

        frame.robotToTarget()
                .ifPresent(
                        robotToTarget -> {
                            targetPoseRobotEntry.setDoubleArray(
                                    LimelightCoordinates.toLimelightCameraSpace(robotToTarget));
                            botPoseTargetEntry.setDoubleArray(
                                    LimelightCoordinates.toLimelightCameraSpace(robotToTarget.inverse()));
                        });
    }

    /**
     * Builds the {@code t2d} key, the array of 17 targeting metrics a Limelight publishes in one
     * go. {@code LimelightHelpers} itself fixes the order.
     */
    private double[] buildT2d(LimelightFrame frame) {
        LimelightFrame.PixelMetrics pixels = frame.targetPixelMetrics();
        return new double[] {
            frame.hasTarget() ? 1.0 : 0.0,
            frame.targetCount(),
            frame.pipelineLatencyMs(),
            frame.captureLatencyMs(),
            frame.tx(),
            frame.ty(),
            frame.txnc(),
            frame.tync(),
            frame.ta(),
            frame.tid(),
            -1.0,
            -1.0,
            pixels.longSidePixels(),
            pixels.shortSidePixels(),
            pixels.horizontalExtentPx(),
            pixels.verticalExtentPx(),
            frame.ts()
        };
    }
}
