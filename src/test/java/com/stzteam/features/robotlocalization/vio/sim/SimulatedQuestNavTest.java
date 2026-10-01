// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The test that matters: it instantiates the vendordep's <b>real</b> {@link QuestNav} against our
 * publisher and checks that it sees what it would see against a physical headset.
 *
 * <p>If this passes, a team's subsystem works without touching a line.
 */
class SimulatedQuestNavTest {

    private static final double LOOP_PERIOD = 0.02;

    /** STZ's real mount. Used on purpose instead of the identity: a mount rotated 90 degrees
     * uncovers composition errors the identity hides. */
    private static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                    new Translation3d(0.0, -0.332, 0.0),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

    private SimulatedQuestNav sim;
    private QuestNav robotSide;
    private Pose2d robotPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(35.0));

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "could not initialise the HAL");
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        // No NT4 server is brought up here, on purpose. It was tried so `serverTime` would be real,
        // but ntcore marks everything published on the same instance that consumes it with the
        // sentinel, server or no server --- which is exactly what
        // inLocalModeDataTimestampIsTheNtSentinelAndMustNotBeUsedForFusion pins down. The only thing it
        // contributed was clashing with port 5810 when a simulation is running.

        sim = new SimulatedQuestNav(ROBOT_TO_QUEST, QuestNavSimConfig.perfect());
        robotSide = new QuestNav();
    }

    @AfterEach
    void tearDown() {
        SimulatedQuestNav.closeAll();

        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    @Test
    void theRealVendorLibrarySeesAConnectedTrackingHeadset() {
        runLoops(10);

        assertTrue(robotSide.isConnected(), "the robot code should see the headset connected");
        assertTrue(robotSide.isTracking(), "and it should see valid tracking");
    }

    @Test
    void theConnectionDropsWhenWeStopPublishing() {
        runLoops(10);
        assertTrue(robotSide.isConnected());

        // isConnected() checks whether the last frameData arrived less than 120 ms ago.
        sim.injectLinkLoss(1.0);
        runLoops(10);

        assertFalse(robotSide.isConnected(), "with no new frames it should fall after 120 ms");

        runLoops(60);
        assertTrue(robotSide.isConnected(), "and it should recover when the link comes back");
    }

    @Test
    void severalFramesArriveEachLoopWithSpacedTimestamps() {
        // The headset runs at 100 Hz and the loop at 50: ~2 frames have to arrive each cycle, with
        // timestamps 10 ms apart. It is precisely the API teams most often get wrong.
        runLoops(5);
        robotSide.getAllUnreadPoseFrames();

        runLoops(1);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();

        assertEquals(2, frames.length, "a 20 ms cycle at 100 Hz is 2 frames");
        assertEquals(
                frames[0].frameCount() + 1,
                frames[1].frameCount(),
                "the frame counter should be consecutive");
        assertEquals(
                0.010,
                frames[1].appTimestamp() - frames[0].appTimestamp(),
                1e-9,
                "the headset's clock should advance 10 ms between frames");

    }

    @Test
    void inLocalModeDataTimestampIsTheNtSentinelAndMustNotBeUsedForFusion() {
        // Pins down a real limitation, so nobody runs into it by surprise: publishing on the same
        // NetworkTables instance the robot consumes, ntcore marks the values with serverTime = 1, and
        // PoseFrame.dataTimestamp() --- which comes from there --- is nailed to 1 microsecond. Fusing on
        // timestamps requires withNetworkClient(true).
        runLoops(10);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();

        assertEquals(1e-6, frames[0].dataTimestamp(), 1e-12);
        assertEquals(frames[0].dataTimestamp(), frames[frames.length - 1].dataTimestamp(), 1e-12);

        // The app's clock does advance, and it is the one that stays usable in local mode.
        assertTrue(
                frames[frames.length - 1].appTimestamp() > frames[0].appTimestamp(),
                "the headset's clock should advance even when NT's does not");
    }

    @Test
    void withoutErrorTheReportedRobotPoseMatchesTheTruth() {
        runLoops(10);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        assertTrue(frames.length > 0);

        // What the robot code does: undo the mount on what the headset reports.
        Pose3d reportedRobot =
                frames[frames.length - 1].questPose3d().transformBy(ROBOT_TO_QUEST.inverse());

        assertEquals(robotPose.getX(), reportedRobot.getX(), 1e-6);
        assertEquals(robotPose.getY(), reportedRobot.getY(), 1e-6);
        assertEquals(
                robotPose.getRotation().getRadians(), reportedRobot.getRotation().getZ(), 1e-6);
    }

    @Test
    void setPoseFromRobotCodeReanchorsTheHeadsetOrigin() {
        // No noise, so the reset can be checked exactly: what is being tested here is the origin
        // re-anchoring, not the error model.
        sim = replaceSimWith(QuestNavSimConfig.perfect().withRandomizedOrigin(true).withRandomSeed(3));
        runLoops(5);
        robotSide.getAllUnreadPoseFrames();

        // With the Quest's origin displaced, what it reports is NOT a field pose...
        Pose3d target = new Pose3d(robotPose).transformBy(ROBOT_TO_QUEST);
        robotSide.setPose(target);
        runLoops(5);

        // ...until the robot code re-anchors it, which is exactly the step most teams forget.
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        Pose3d reported = frames[frames.length - 1].questPose3d();

        assertEquals(target.getX(), reported.getX(), 1e-6);
        assertEquals(target.getY(), reported.getY(), 1e-6);
        assertEquals(target.getRotation().getZ(), reported.getRotation().getZ(), 1e-6);
    }

    @Test
    void forcingTrackingLossFromTheSimGuiIsVisibleToTheRobotCode() {
        runLoops(10);
        assertTrue(robotSide.isTracking());

        SimDeviceSim device = new SimDeviceSim(QuestNavSimDevice.DEVICE_PREFIX + "QuestNav", 0);
        device.getBoolean("Force Tracking Loss").set(true);

        // deviceData runs at 3 Hz, so the loss counter takes up to a third of a second to arrive. That
        // is the hardware's behaviour, not slowness in the simulation.
        runLoops(25);

        assertFalse(robotSide.isTracking(), "the robot code should see that tracking was lost");
        assertTrue(
                robotSide.getTrackingLostCounter().orElse(0) > 0,
                "and the loss counter should have gone up");
    }

    @Test
    void aPoseDiscontinuityIsInvisibleToTheRobotCode() {
        // The Quest's most treacherous failure: the double-tap to passthrough that chassis vibration
        // triggers moves the pose all at once without touching isTracking. The robot code has no signal
        // to look at, and a pose estimator trusting 2 cm swallows the whole thing.
        runLoops(10);
        robotSide.getAllUnreadPoseFrames();
        runLoops(5);
        Pose3d before = latestQuestPose();

        sim.injectPoseDiscontinuity(new Translation3d(0.4, 0.0, 0.0));
        runLoops(5);
        Pose3d after = latestQuestPose();

        assertEquals(
                0.4,
                after.getTranslation().getDistance(before.getTranslation()),
                1e-6,
                "the pose should have jumped");
        assertTrue(robotSide.isTracking(), "and nothing in the protocol gives it away");
    }

    @Test
    void theHeadsetAppearsInTheSimGuiAsAnOtherDevice() {
        SimDeviceSim device = new SimDeviceSim(QuestNavSimDevice.DEVICE_PREFIX + "QuestNav", 0);

        runLoops(10);

        assertTrue(device.getBoolean("isTracking").get());
        assertTrue(device.getDouble("Frame Count").get() > 0.0);
        assertEquals(100.0, device.getDouble("Battery (%)").get(), 1e-9);
    }

    @Test
    void telemetryPosesArePublishedForAdvantageScope() {
        runLoops(10);

        var telemetry = NetworkTableInstance.getDefault().getTable("QuestNav").getSubTable("sim");
        Pose3d truePose =
                telemetry.getStructTopic("truePose", Pose3d.struct).subscribe(Pose3d.kZero).get();

        assertEquals(robotPose.getX(), truePose.getX(), 1e-9);
        assertEquals(robotPose.getY(), truePose.getY(), 1e-9);
    }

    /** Recreates the simulator with another configuration, keeping the test's robot code. */
    private SimulatedQuestNav replaceSimWith(QuestNavSimConfig config) {
        SimulatedQuestNav.closeAll();
        SimDeviceSim.resetData();
        return new SimulatedQuestNav(ROBOT_TO_QUEST, config);
    }

    private Pose3d latestQuestPose() {
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        assertTrue(frames.length > 0, "there should be frames");
        return frames[frames.length - 1].questPose3d();
    }

    /** Advances the simulated clock and runs the cycle, as a TimedRobot would. */
    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            sim.periodic(robotPose);
            robotSide.commandPeriodic();
        }
    }
}
