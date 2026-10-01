// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The pose the camera solves against the <b>2026</b> field, in the exact scene from a live simulation
 * that was getting it wrong.
 *
 * <p>The other integration tests pin the 2025 field on purpose, so as not to depend on the default
 * layout changing. That is what kept them from seeing this.
 */
class Field2026SolveTest {

    private static final String CAMERA = "limelight-2026";
    private static final double LOOP_PERIOD = 0.02;

    /** Where the robot was in the simulation when the solution came out 12 m away. */
    private static final Pose2d ROBOT_POSE = new Pose2d(2.71, 2.56, Rotation2d.fromDegrees(1.4));

    /** The same mounting as the test project. */
    private static final Transform3d ROBOT_TO_CAMERA =
            new Transform3d(
                    new Translation3d(0.30, 0.0, 0.50),
                    new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));

    private AprilTagFieldLayout layout;
    private SimulatedLimelight limelight;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        layout = FieldLayouts.rebuilt2026Welded();
        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        ROBOT_TO_CAMERA,
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(ROBOT_POSE);
        }
    }

    @Test
    void theFieldLayoutIsThe2026One() {
        assertEquals(32, layout.getTags().size());
        assertEquals(16.541, layout.getFieldLength(), 1e-3);
    }

    @Test
    void theMegaTag1SolutionLandsOnTheRobot() {
        runLoops(20);

        LimelightFrame frame = limelight.lastFrame();
        assertTrue(frame.hasTarget(), "the camera should see tags");
        assertTrue(frame.megaTag1Pose().isPresent(), "there should be a MegaTag1 solution");

        Pose3d solved = frame.megaTag1Pose().get();
        assertEquals(
                ROBOT_POSE.getX(),
                solved.getX(),
                0.30,
                "the solution should land on the robot, and it landed at x=" + solved.getX());
        assertEquals(ROBOT_POSE.getY(), solved.getY(), 0.30);
    }

    @Test
    void theCameraCannotSeeTagsAcrossTheWholeField() {
        // VisionSystemSim does not model occlusion, so with no range limit the camera sees any tag
        // inside the FOV straight through the field walls. Without the 7 m default, seven tags came in
        // --- five of them between 8 and 13 m with ambiguities up to 0.99 --- and the multi-tag PnP
        // mixed them with the nearby ones and solved twelve metres from the robot.
        runLoops(20);

        LimelightFrame frame = limelight.lastFrame();
        for (LimelightFrame.RawFiducial fiducial : frame.fiducials()) {
            assertTrue(
                    fiducial.distToCamera() <= 7.5,
                    "tag " + fiducial.id() + " is " + fiducial.distToCamera()
                            + " m away and should not be visible");
        }
    }

    @Test
    void theSolutionStaysPutAsTagsComeAndGo() {
        // A solution that jumps from one side of the field to the other between frames is worse than
        // none: the pose estimator swallows it and drags the robot along with it.
        runLoops(20);
        Pose3d first = limelight.lastFrame().megaTag1Pose().orElseThrow();

        runLoops(40);
        Pose3d later = limelight.lastFrame().megaTag1Pose().orElseThrow();

        assertEquals(
                0.0,
                first.getTranslation().getDistance(later.getTranslation()),
                0.20,
                "the robot did not move, so neither should the solution");
    }
}
