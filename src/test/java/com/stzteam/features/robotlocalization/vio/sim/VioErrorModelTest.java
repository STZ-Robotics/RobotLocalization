// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.junit.jupiter.api.Test;

/**
 * Checks the VIO error model, which is the feature's real content. These are pure tests: neither HAL
 * nor NetworkTables is needed.
 */
class VioErrorModelTest {

    private static final double DT = 0.01;

    private static Pose3d at(double x) {
        return new Pose3d(x, 0.0, 0.0, Rotation3d.kZero);
    }

    @Test
    void withoutDriftOrNoiseTheReportedPoseIsTheTruth() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());

        Pose3d reported = model.update(at(3.0), 1.0, DT);

        assertEquals(3.0, reported.getX(), 1e-12);
        assertTrue(model.isTracking());
    }

    @Test
    void driftGrowsWithDistanceTravelledNotWithTime() {
        VioErrorModel still = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(3));
        VioErrorModel moving = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(3));

        for (int i = 0; i < 500; i++) {
            still.update(at(0.0), i * DT, DT);
            moving.update(at(i * 0.02), i * DT, DT);
        }

        assertEquals(0.0, still.driftTranslation().getNorm(), 1e-12, "standing still it should not drift");
        assertTrue(
                moving.driftTranslation().getNorm() > 0.0,
                "covering 10 m should accumulate drift");
    }

    @Test
    void driftMagnitudeGrowsWithDistanceOnAverage() {
        // The drift is a random walk, so one particular run can shrink: what grows with distance is
        // its *expected* magnitude. Asserting monotonicity on a single seed would be a flaky test that
        // sooner or later fails with nothing broken.
        double shortRun = averageDriftAfterSteps(20);
        double longRun = averageDriftAfterSteps(600);

        // In a random walk the deviation grows as the square root of the step count: 30 times more
        // steps is ~5.5 times more drift. 3 is required, to leave room for sampling.
        assertTrue(
                longRun > shortRun * 3.0,
                "covering 30 times more should drift considerably more: " + shortRun + " -> " + longRun);
    }

    /** Averages the accumulated drift over several seeds, so as not to depend on one realisation. */
    private static double averageDriftAfterSteps(int steps) {
        int seeds = 40;
        double total = 0.0;
        for (int seed = 0; seed < seeds; seed++) {
            VioErrorModel model = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(seed));
            for (int i = 0; i < steps; i++) {
                model.update(at(i * 0.05), i * DT, DT);
            }
            total += model.driftTranslation().getNorm();
        }
        return total / seeds;
    }

    @Test
    void whileTrackingIsLostThePoseFreezesAndTheCounterGoesUp() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());
        model.update(at(1.0), 0.0, DT);

        model.injectTrackingLoss(0.5, 1.0);
        Pose3d duringLoss = model.update(at(5.0), 1.1, DT);

        assertFalse(model.isTracking());
        assertEquals(1, model.trackingLostCounter());
        assertEquals(1.0, duringLoss.getX(), 1e-9, "without tracking the pose freezes");
    }

    @Test
    void regainingTrackingReacquiresAtAnOffset() {
        // It is the failure QuestNav documents and the one that breaks a pose estimator trusting 2 cm.
        QuestNavSimConfig config =
                QuestNavSimConfig.perfect().withReacquireOffsetMeters(0.25).withRandomSeed(1);
        VioErrorModel model = new VioErrorModel(config);
        model.update(at(1.0), 0.0, DT);

        model.injectTrackingLoss(0.5, 1.0);
        model.update(at(1.0), 1.1, DT);
        Pose3d afterRecovery = model.update(at(1.0), 2.0, DT);

        assertTrue(model.isTracking(), "it should have recovered tracking");
        double jump = afterRecovery.getTranslation().getDistance(new Translation3d(1.0, 0.0, 0.0));
        assertEquals(0.25, jump, 1e-6, "the re-acquisition should come back displaced");
    }

    @Test
    void aPoseDiscontinuityJumpsWithoutDroppingTracking() {
        // The vibration-induced double-tap to passthrough: isTracking stays true, which is the trap.
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());
        model.update(at(2.0), 0.0, DT);

        model.injectPoseDiscontinuity(new Translation3d(0.4, 0.0, 0.0));
        Pose3d afterJump = model.update(at(2.0), 0.1, DT);

        assertTrue(model.isTracking(), "the double-tap does not drop tracking");
        assertEquals(2.4, afterJump.getX(), 1e-9);
    }

    @Test
    void aPoseResetClearsTheAccumulatedDrift() {
        VioErrorModel model = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(9));
        for (int i = 0; i < 500; i++) {
            model.update(at(i * 0.05), i * DT, DT);
        }
        assertTrue(model.driftTranslation().getNorm() > 0.0);

        model.resetDrift();

        assertEquals(0.0, model.driftTranslation().getNorm(), 1e-12);
        assertEquals(0.0, model.driftYawDegrees(), 1e-12);
    }

    @Test
    void theHeadsetShutsDownWhenTheBatteryRunsOut() {
        VioErrorModel model =
                new VioErrorModel(
                        new QuestNavSimConfig()
                                .withInitialBatteryPercent(1.0)
                                .withBatteryDrainPercentPerMinute(120.0));

        assertTrue(model.isPoweredOn());
        for (int i = 0; i < 200; i++) {
            model.update(at(0.0), i * DT, DT);
        }

        assertFalse(model.isPoweredOn(), "the battery should have run out");
        assertEquals(0, model.batteryPercent());
    }

    @Test
    void anInjectedLinkLossBringsTheLinkDownForItsDuration() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());

        assertTrue(model.isLinkUp(1.0));
        model.injectLinkLoss(0.5, 1.0);

        assertFalse(model.isLinkUp(1.2));
        assertTrue(model.isLinkUp(1.6));
    }

    @Test
    void aHardImpactKnocksOutTrackingOnItsOwn() {
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(30.0, 2000.0));

        // Accelerating the way an FRC robot accelerates --- around 6 m/s2 --- breaks nothing...
        double x = 0.0;
        double v = 0.0;
        int steps = 50;
        for (int i = 0; i < steps; i++) {
            v = Math.min(3.0, v + 0.06);
            x += v * DT;
            model.update(at(x), i * DT, DT);
        }
        assertTrue(model.isTracking(), "normal acceleration should not drop tracking");

        // ...but stopping dead against the wall does: 3 m/s to 0 in one cycle is 300 m/s2.
        model.update(at(x), steps * DT, DT);
        model.update(at(x), (steps + 1) * DT, DT);

        assertFalse(model.isTracking(), "a hard stop should drop tracking");
    }

    @Test
    void aSinglePoseJumpIsNotMistakenForACrash() {
        // The case the live simulation uncovered. The pose feeding the model comes from the robot's
        // pose estimator, which JUMPS every time it accepts a vision measurement. A 20 cm jump in a
        // 10 ms frame is 2000 m/s2 apparent: looking at a single frame, every vision correction looked
        // like a collision and the Quest lost tracking on two cycles out of three.
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(40.0, 2000.0));

        double x = 0.0;
        for (int i = 0; i < 20; i++) {
            model.update(at(x), i * DT, DT);
        }

        // Vision correction: the pose jumps 20 cm from one frame to the next and then stays put.
        x += 0.20;
        for (int i = 20; i < 40; i++) {
            model.update(at(x), i * DT, DT);
        }

        assertTrue(model.isTracking(), "a vision correction is not a collision");
        assertEquals(0, model.trackingLostCounter());
    }

    @Test
    void feedingBackTheFusedEstimateDefeatsTheImpactDetector() {
        // This pins down a limitation, not desirable behaviour.
        //
        // If you hand the simulator the pose coming out of the pose estimator, and that estimator is
        // corrected by vision on every robot cycle, the pose jumps every other VIO frame. That is not
        // an isolated impulse --- which the median does filter --- but an oscillation at half the
        // sampling frequency, and no causal filter tells it apart from real motion.
        //
        // The conclusion is not to tune the detector: it is that feeding a simulated sensor the
        // estimator's output is a loop that does not exist in reality. The physics feeds the sensor
        // and the sensor feeds the estimator, never the other way round. See
        // SimulatedQuestNav#periodic.
        VioErrorModel model =
                new VioErrorModel(QuestNavSimConfig.perfect().withImpactTrackingLoss(true));

        double x = 0.0;
        for (int i = 0; i < 200; i++) {
            if (i % 2 == 0) {
                x += 0.15; // a "vision correction" at half the frame rate
            }
            model.update(at(x), i * DT, DT);
        }

        assertTrue(
                model.trackingLostCounter() > 0,
                "feeding the estimator back confuses the detector, and this documents that");
    }

    @Test
    void aSustainedDecelerationStillCountsAsACrash() {
        // And the converse has to keep working: a real hard stop lasts tens of milliseconds.
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(30.0, 2000.0));

        double x = 0.0;
        double v = 0.0;
        for (int i = 0; i < 50; i++) {
            v = Math.min(3.0, v + 0.06);
            x += v * DT;
            model.update(at(x), i * DT, DT);
        }
        assertTrue(model.isTracking());

        // Collision: the velocity collapses and stays at zero for several frames.
        for (int i = 50; i < 60; i++) {
            model.update(at(x), i * DT, DT);
        }

        assertFalse(model.isTracking(), "a sustained hard stop should drop tracking");
    }

    @Test
    void theSameSeedProducesTheSameRun() {
        VioErrorModel a = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(42));
        VioErrorModel b = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(42));

        for (int i = 0; i < 100; i++) {
            Pose3d pa = a.update(at(i * 0.05), i * DT, DT);
            Pose3d pb = b.update(at(i * 0.05), i * DT, DT);
            assertEquals(pa.getX(), pb.getX(), 1e-12);
        }
    }
}
