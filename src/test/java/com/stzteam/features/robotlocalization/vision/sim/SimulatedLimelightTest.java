// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * An end-to-end test: it places the robot in front of a real field AprilTag, runs a few
 * simulation cycles and checks that the NetworkTables table ends up the way a physical
 * Limelight would leave it.
 *
 * <p>It is the automated version of the validation checklist: if this passes, a subsystem
 * written against {@code LimelightHelpers} works untouched.
 */
class SimulatedLimelightTest {

    private static final String CAMERA = "limelight-under-test";
    private static final int TAG_ID = 1;
    private static final double LOOP_PERIOD = 0.02;
    private static final double DISTANCE_TO_TAG = 3.0;
    private static final double CAMERA_HEIGHT = 0.5;
    private static final double CAMERA_FORWARD = 0.2;

    private AprilTagFieldLayout layout;
    private Pose2d robotPose;
    private SimulatedLimelight limelight;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "could not initialise the HAL");
        // SimDevices are registered by name and HAL_CreateSimDevice returns null if one of the same
        // name already exists, so the registry has to be cleared between cases.
        SimDeviceSim.resetData();
        clearRobotInputs();
        SimHooks.pauseTiming();

        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        robotPose = poseFacingTag(TAG_ID, DISTANCE_TO_TAG);

        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        new Transform3d(
                                new Translation3d(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT),
                                new Rotation3d(0.0, -pitchTowardsTag(TAG_ID), 0.0)),
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));
    }

    /**
     * The tilt the camera needs to point at the tag from the test's distance. It is computed
     * rather than hardcoded so the test does not break when the field changes and the tags sit
     * at a different height.
     */
    private double pitchTowardsTag(int tagId) {
        double tagHeight = layout.getTagPose(tagId).orElseThrow().getZ();
        return Math.atan2(tagHeight - CAMERA_HEIGHT, DISTANCE_TO_TAG - CAMERA_FORWARD);
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    @Test
    void publishesACoherentLimelightTableWhileLookingAtATag() {
        runLoops(10);

        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);

        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9, "it should see the tag");
        assertEquals(TAG_ID, (int) table.getEntry("tid").getDouble(-1.0));
        assertTrue(table.getEntry("tl").getDouble(0.0) > 0.0, "the latency should be real");
        assertTrue(
                Math.abs(table.getEntry("tx").getDouble(99.0)) < 10.0,
                "the tag is nearly centred, so tx should be small");

        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        int tagCount = (int) botpose[7];
        assertEquals(11 + 7 * tagCount, botpose.length, "the length is what LimelightHelpers demands");
        assertEquals(robotPose.getX(), botpose[0], 0.3, "botpose X should come close to the real pose");
        assertEquals(robotPose.getY(), botpose[1], 0.3, "botpose Y should come close to the real pose");
    }

    @Test
    void redOriginKeyMirrorsTheBlueOriginKey() {
        runLoops(10);

        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        double[] blue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        double[] red = table.getEntry("botpose_wpired").getDoubleArray(new double[0]);

        assertEquals(layout.getFieldLength() - blue[0], red[0], 1e-6);
        assertEquals(layout.getFieldWidth() - blue[1], red[1], 1e-6);
    }

    @Test
    void appearsInTheSimGuiAsAnOtherDevice() {
        SimDeviceSim device = new SimDeviceSim(LimelightSimDevice.DEVICE_PREFIX + CAMERA, 0);

        runLoops(10);

        assertEquals(true, device.getBoolean("tv").get(), "the SimDevice should mirror tv");
        assertNotEquals(
                0.0, device.getDouble("Botpose X (m)").get(), "the SimDevice should mirror botpose");
    }

    @Test
    void forcingNoTargetsFromTheSimGuiBlindsTheCamera() {
        runLoops(10);
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9);

        new SimDeviceSim(LimelightSimDevice.DEVICE_PREFIX + CAMERA, 0)
                .getBoolean("Force No Targets")
                .set(true);
        runLoops(5);

        assertEquals(0.0, table.getEntry("tv").getDouble(-1.0), 1e-9);
        assertEquals(-1.0, table.getEntry("tid").getDouble(0.0), 1e-9);
        assertEquals(0, table.getEntry("botpose_wpiblue").getDoubleArray(new double[] {1.0}).length);
    }

    @Test
    void megaTag2AppearsOnceTheRobotPublishesItsOrientation() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);

        runLoops(10);
        assertEquals(
                0,
                table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {1.0}).length,
                "with no heading there should be no MegaTag2, just as on real hardware");

        // Equivalent to LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0).
        table.getEntry("robot_orientation_set")
                .setDoubleArray(
                        new double[] {robotPose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        runLoops(10);

        double[] orb = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        assertTrue(orb.length >= 11, "MegaTag2 should publish a complete pose");
        assertEquals(robotPose.getX(), orb[0], 0.3);
        assertEquals(robotPose.getY(), orb[1], 0.3);
    }

    @Test
    void fiducialFiltersFromRobotCodeHideTheTag() {
        runLoops(10);
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9);

        // Equivalent to LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[] {99}).
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[] {99.0});
        runLoops(5);

        assertEquals(0.0, table.getEntry("tv").getDouble(-1.0), 1e-9);
    }

    /**
     * Clears the keys the robot code writes to the camera. The tests share the global
     * NetworkTables instance, so without this a filter installed by one case would blind the
     * next.
     */
    private static void clearRobotInputs() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[0]);
        table.getEntry("robot_orientation_set").setDoubleArray(new double[0]);
        table.getEntry("camerapose_robotspace_set").setDoubleArray(new double[0]);
        table.getEntry("throttle_set").setDouble(0.0);
        table.getEntry("pipeline").setDouble(0.0);
    }

    /** Advances the simulated clock and runs the camera's cycle, as a TimedRobot would. */
    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(robotPose);
        }
    }

    /**
     * Builds a robot pose {@code distance} metres in front of the tag, facing it head on.
     */
    private Pose2d poseFacingTag(int tagId, double distance) {
        Pose3d tag = layout.getTagPose(tagId).orElseThrow();
        Rotation2d tagYaw = tag.getRotation().toRotation2d();
        Translation2d offset = new Translation2d(distance, 0.0).rotateBy(tagYaw);

        return new Pose2d(
                tag.getTranslation().toTranslation2d().plus(offset),
                tagYaw.plus(Rotation2d.k180deg));
    }
}
