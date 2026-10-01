// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

/**
 * Arbitration between sources that contradict each other.
 *
 * <p>It used to be buried inside the update loop and there was no way to test it without standing up
 * sources and NetworkTables. Pulling it out into its own class was half the work; this is the other
 * half.
 */
class ConsensusGateTest {

    private static ConsensusGate.Candidate at(String name, double x, double timestamp) {
        return new ConsensusGate.Candidate(name, new Pose2d(x, 0.0, Rotation2d.kZero), timestamp);
    }

    private static ConsensusGate gate() {
        return new ConsensusGate().withToleranceMeters(0.5).withTimeWindowSeconds(0.1);
    }

    @Test
    void oneSourceNeverConflictsWithItself() {
        ConsensusGate.Result result = gate().evaluate(List.of(at("a", 0.0, 1.0)), 0.0);

        assertTrue(result.rejected().isEmpty());
        assertTrue(result.isUnanimous());
    }

    @Test
    void sourcesThatAgreeAllSurvive() {
        ConsensusGate.Result result =
                gate().evaluate(List.of(at("a", 0.0, 1.0), at("b", 0.3, 1.0)), 0.0);

        assertTrue(result.rejected().isEmpty());
        assertTrue(result.isUnanimous());
    }

    @Test
    void twoSourcesThatDisagreeAreBothRejected() {
        // With two sources there is nobody to break the tie: believing one is a gamble.
        ConsensusGate.Result result =
                gate().evaluate(List.of(at("a", 0.0, 1.0), at("b", 3.0, 1.0)), 0.0);

        assertEquals(Set.of(0, 1), result.rejected());
        assertFalse(result.isUnanimous());
        assertEquals(1, result.conflicts().size());
    }

    @Test
    void majorityVoteKeepsTheBestSupportedAndDropsTheOutlier() {
        ConsensusGate.Result result =
                gate().withMode(ConsensusGate.Mode.MAJORITY_VOTE)
                        .evaluate(
                                List.of(
                                        at("a", 0.0, 1.0),
                                        at("b", 0.2, 1.0),
                                        at("c", 0.3, 1.0),
                                        at("outlier", 5.0, 1.0)),
                                0.0);

        assertEquals(Set.of(3), result.rejected(), "only the dissenting one should fall");
    }

    @Test
    void majorityVoteRejectsEveryoneOnAnEvenSplit() {
        // Two against two: the evidence is not enough to pick a side.
        ConsensusGate.Result result =
                gate().withMode(ConsensusGate.Mode.MAJORITY_VOTE)
                        .evaluate(
                                List.of(
                                        at("a", 0.0, 1.0),
                                        at("b", 0.2, 1.0),
                                        at("c", 5.0, 1.0),
                                        at("d", 5.2, 1.0)),
                                0.0);

        assertEquals(Set.of(0, 1, 2, 3), result.rejected());
    }

    @Test
    void supportIsCountedAcrossAllSourcesNotJustTheDisputedOnes() {
        // 'a' only argues with 'far', but three sources that are not in dispute back it up. That
        // backing has to count: it is what having a majority means.
        ConsensusGate.Result result =
                gate().withMode(ConsensusGate.Mode.MAJORITY_VOTE)
                        .evaluate(
                                List.of(
                                        at("a", 0.0, 1.0),
                                        at("b", 0.2, 1.0),
                                        at("c", 0.2, 1.0),
                                        at("d", 0.2, 1.0),
                                        at("far", 1.5, 1.0)),
                                0.0);

        assertEquals(Set.of(4), result.rejected(), "the unsupported source is the odd one out");
    }

    @Test
    void samplesTooFarApartInTimeAreNotCompared() {
        // Half a second apart describes two different moments, not a disagreement.
        ConsensusGate.Result result =
                gate().evaluate(List.of(at("a", 0.0, 1.0), at("b", 3.0, 1.5)), 0.0);

        assertTrue(result.rejected().isEmpty());
        assertTrue(result.isUnanimous());
    }

    @Test
    void motionWidensTheToleranceByHowFarTheRobotCouldHaveMoved() {
        // 0.08 s apart and 0.6 m of difference: with the robot still that is a disagreement...
        List<ConsensusGate.Candidate> candidates = List.of(at("a", 0.0, 1.0), at("b", 0.6, 1.08));

        assertEquals(Set.of(0, 1), gate().evaluate(candidates, 0.0).rejected());

        // ...but at 4 m/s the robot covers 0.32 m in that time, so 0.6 m falls within what can be
        // explained and both sources survive.
        assertTrue(gate().evaluate(candidates, 4.0).rejected().isEmpty());
    }

    @Test
    void motionCompensationCanBeTurnedOff() {
        List<ConsensusGate.Candidate> candidates = List.of(at("a", 0.0, 1.0), at("b", 0.6, 1.08));

        ConsensusGate.Result result =
                gate().withMotionCompensation(false).evaluate(candidates, 4.0);

        assertEquals(Set.of(0, 1), result.rejected());
    }

    @Test
    void conflictsNameBothSourcesSoTheLogIsActionable() {
        ConsensusGate.Result result =
                gate().evaluate(List.of(at("limelight", 0.0, 1.0), at("quest", 3.0, 1.0)), 0.0);

        String conflict = result.conflicts().get(0);
        assertTrue(conflict.contains("limelight"), conflict);
        assertTrue(conflict.contains("quest"), conflict);
    }
}
