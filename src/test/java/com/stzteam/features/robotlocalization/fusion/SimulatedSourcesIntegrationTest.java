// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.vision.LimelightBotpose;
import com.stzteam.features.robotlocalization.vision.sim.LimelightSimConfig;
import com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight;
import com.stzteam.features.robotlocalization.vision.sim.VisionSimTelemetry;
import com.stzteam.features.robotlocalization.vision.sim.VisionSimWorld;
import com.stzteam.features.robotlocalization.vio.sim.QuestNavSimConfig;
import com.stzteam.features.robotlocalization.vio.sim.SimulatedQuestNav;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * The feature's two halves, together: the simulators produce measurements and the fusion consumes
 * them, with not a single simulation-specific branch.
 *
 * <p>It is the test that justifies the two living in the same package. If somebody changes the
 * {@code botpose} format in the publisher and forgets the reader, this falls over.
 */
class SimulatedSourcesIntegrationTest {

    private static final String CAMERA = "limelight-fusion-test";
    private static final double LOOP_PERIOD = 0.02;
    private static final int TAG_ID = 1;
    private static final double DISTANCE_TO_TAG = 3.0;
    private static final double CAMERA_HEIGHT = 0.5;

    private AprilTagFieldLayout layout;
    private Pose2d robotPose;
    private SimulatedLimelight limelight;
    private PoseFusion fusion;
    private final List<PoseSample> accepted = new ArrayList<>();

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        PoseFusion.setDefaultFieldLayout(layout);
        robotPose = poseFacingTag();
        accepted.clear();
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        SimulatedQuestNav.closeAll();
        VisionSimTelemetry.reset();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    private Pose2d poseFacingTag() {
        return poseFacingTagAt(DISTANCE_TO_TAG);
    }

    private Pose2d poseFacingTagAt(double distanceMeters) {
        Pose3d tag = layout.getTagPose(TAG_ID).orElseThrow();
        Rotation2d tagYaw = tag.getRotation().toRotation2d();
        Translation2d offset = new Translation2d(distanceMeters, 0.0).rotateBy(tagYaw);
        return new Pose2d(
                tag.getTranslation().toTranslation2d().plus(offset), tagYaw.plus(Rotation2d.k180deg));
    }

    private Transform3d robotToCamera() {
        double tagHeight = layout.getTagPose(TAG_ID).orElseThrow().getZ();
        double pitch = Math.atan2(tagHeight - CAMERA_HEIGHT, DISTANCE_TO_TAG - 0.2);
        return new Transform3d(
                new Translation3d(0.2, 0.0, CAMERA_HEIGHT), new Rotation3d(0.0, -pitch, 0.0));
    }

    private void startLimelight() {
        startLimelight(LimelightPoseSource.megaTag1(CAMERA));
    }

    private void startLimelight(LimelightPoseSource source) {
        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        robotToCamera(),
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));

        fusion =
                new PoseFusion(
                                (pose, timestamp, stdDevs) ->
                                        accepted.add(new PoseSample(pose, timestamp, stdDevs)),
                                LocalizationInputs.of(() -> robotPose, ChassisSpeeds::new))
                        .setTelemetryTable("LocalizationIntegration")
                        .registerSource(source);
    }

    /** A source that does not filter on ambiguity, for the tests that are about something else. */
    private LimelightPoseSource permissiveSource() {
        return new LimelightPoseSource(
                        CAMERA,
                        LimelightPoseSource.Mode.MEGATAG_1,
                        new PoseSourceConfig().withMaxAmbiguity(Double.POSITIVE_INFINITY))
                .withStdDevsStrategy(StdDevStrategies.quadraticContinuous(0.08, 0.16));
    }

    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(robotPose);
            fusion.update();
        }
    }

    /** Moves the robot to another pose by interpolating, as if it were driving. */
    private void driveTo(Pose2d target, int steps) {
        Pose2d start = robotPose;
        for (int i = 1; i <= steps; i++) {
            robotPose = start.interpolate(target, (double) i / steps);
            runLoops(1);
        }
        robotPose = target;
    }

    @Test
    void aSimulatedLimelightFeedsTheFusionAllTheWayToTheEstimator() {
        startLimelight();

        runLoops(15);

        assertFalse(accepted.isEmpty(), "some measurement should have arrived");
        PoseSample last = accepted.get(accepted.size() - 1);
        assertEquals(robotPose.getX(), last.pose().getX(), 0.3);
        assertEquals(robotPose.getY(), last.pose().getY(), 0.3);
    }

    @Test
    void theCaptureTimestampIsInThePastAndNotTheReadTime() {
        startLimelight();

        runLoops(15);

        PoseSample last = accepted.get(accepted.size() - 1);
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        assertTrue(last.timestampSeconds() < now, "the capture happened before the read");
        assertTrue(now - last.timestampSeconds() < 0.5, "but not half a second ago");
    }

    @Test
    void theAmbiguityGateIsActuallyWiredIn() {
        // The gate existed but nobody called it: a measurement with ambiguity 0.9 sailed right through.
        // With the threshold at zero, any single-tag solution --- which always has two possible
        // interpretations, and therefore a strictly positive ambiguity --- gets discarded.
        startLimelight(
                new LimelightPoseSource(
                        CAMERA, LimelightPoseSource.Mode.MEGATAG_1,
                        new PoseSourceConfig().withMaxAmbiguity(0.0)));

        runLoops(15);

        assertEquals(
                1.0,
                NetworkTableInstance.getDefault().getTable(CAMERA).getEntry("tv").getDouble(-1),
                "the camera does see the tag");
        assertEquals(
                RejectionReason.AMBIGUITY,
                fusion.getSource(CAMERA).orElseThrow().getLastReason(),
                "but the filter discards it");
        assertTrue(accepted.isEmpty());
    }

    @Test
    void theStdDevsGrowWithDistanceThroughTheStrategy() {
        startLimelight(permissiveSource());
        runLoops(15);
        double closeStdDev = accepted.get(accepted.size() - 1).stdDevs().get(0, 0);

        // Back off by driving rather than teleporting: photonlib interpolates the robot pose to date
        // each frame, so an instantaneous jump produces a few cycles of measurements that describe
        // neither the old place nor the new one.
        Pose2d farPose = poseFacingTagAt(DISTANCE_TO_TAG + 2.0);
        driveTo(farPose, 25);
        accepted.clear();
        runLoops(15);

        assertFalse(
                accepted.isEmpty(),
                "at 5 m it should still be measuring; reason="
                        + fusion.getSource(CAMERA).orElseThrow().getLastReason());
        double farStdDev = accepted.get(accepted.size() - 1).stdDevs().get(0, 0);
        assertTrue(
                farStdDev > closeStdDev,
                "farther away should inspire less confidence: " + closeStdDev + " -> " + farStdDev);
    }

    @Test
    void blindingTheCameraFromTheSimGuiStopsTheMeasurements() {
        startLimelight();
        runLoops(15);
        assertFalse(accepted.isEmpty());

        accepted.clear();
        new SimDeviceSim("Limelight:" + CAMERA, 0).getBoolean("Force No Targets").set(true);
        runLoops(10);

        assertTrue(accepted.isEmpty(), "with no targets nothing should arrive");
        assertEquals(
                RejectionReason.NO_MEASUREMENT,
                fusion.getSource(CAMERA).orElseThrow().getLastReason());
    }

    @Test
    void theBotposeFormatRoundTripsBetweenThePublisherAndTheReader() {
        // The simulator writes the format and the fusion reads it. This is the tie between the two.
        startLimelight();
        runLoops(15);

        double[] raw =
                NetworkTableInstance.getDefault()
                        .getTable(CAMERA)
                        .getEntry("botpose_wpiblue")
                        .getDoubleArray(new double[0]);

        var parsed = LimelightBotpose.parse(raw, 1_000_000L, false).orElseThrow();

        assertTrue(parsed.hasTargets());
        assertEquals(
                LimelightBotpose.HEADER_LENGTH
                        + LimelightBotpose.VALUES_PER_FIDUCIAL * parsed.tagCount(),
                raw.length,
                "the length is what the LimelightHelpers parser demands");
        assertEquals(parsed.tagCount(), parsed.fiducials().size());
        assertTrue(parsed.worstAmbiguity().isPresent());
        assertTrue(parsed.avgTagDistMeters() > 0.0);
    }

    @Test
    void theVisionSimPublishesThe3dFieldForAdvantageScope() {
        startLimelight();
        VisionSimTelemetry telemetry = VisionSimTelemetry.forWorld("main");

        runLoops(15);
        telemetry.publish();

        var table = NetworkTableInstance.getDefault().getTable("VisionSim").getSubTable("main");

        Pose3d[] fieldTags =
                table.getStructArrayTopic("fieldTags", Pose3d.struct)
                        .subscribe(new Pose3d[0])
                        .get();
        Pose3d[] cameras =
                table.getStructArrayTopic("cameras", Pose3d.struct).subscribe(new Pose3d[0]).get();
        Pose3d[] visible =
                table.getStructArrayTopic("visionTargets", Pose3d.struct)
                        .subscribe(new Pose3d[0])
                        .get();

        assertEquals(layout.getTags().size(), fieldTags.length, "the whole field in 3D");
        assertEquals(1, cameras.length, "one mounted camera");
        assertTrue(visible.length >= 1, "it should be seeing at least one tag");
        assertTrue(fieldTags[0].getZ() > 0.0, "the tags have height, which is the point of being 3D");
    }

    @Test
    void aQuestNavSourceRefusesToUseTheSentinelTimestamp() {
        // Publishing on the same NetworkTables instance, dataTimestamp is 1 microsecond. Mis-dating a
        // measurement corrupts the estimator with no error raised, so the source prefers to contribute
        // nothing and say why.
        Transform3d robotToQuest =
                new Transform3d(
                        new Translation3d(0.0, -0.332, 0.0),
                        new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

        SimulatedQuestNav quest =
                new SimulatedQuestNav(robotToQuest, QuestNavSimConfig.perfect());
        QuestNavPoseSource source = QuestNavPoseSource.create("quest", robotToQuest);

        PoseFusion questFusion =
                new PoseFusion(
                                (pose, timestamp, stdDevs) ->
                                        accepted.add(new PoseSample(pose, timestamp, stdDevs)),
                                LocalizationInputs.of(() -> robotPose, ChassisSpeeds::new))
                        .setTelemetryTable("LocalizationQuest")
                        .registerSource(source);

        for (int i = 0; i < 15; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            quest.periodic(robotPose);
            source.periodic();
            questFusion.update();
        }

        assertTrue(accepted.isEmpty(), "no measurement is better than a mis-dated one");
        assertEquals(RejectionReason.NO_MEASUREMENT, source.getLastReason());
    }
}
