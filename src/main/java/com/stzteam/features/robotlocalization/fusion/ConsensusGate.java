// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Arbitrates between sources that contradict each other.
 *
 * <p>A single camera reporting an absurd pose passes every individual filter: it is geometrically
 * valid, the ambiguity is low and the robot is standing still. The only thing that gives it away is
 * that <b>another source says something different</b>. That is what this class looks at.
 *
 * <h2>Motion compensation</h2>
 * Two measurements taken {@code dt} apart describe different positions if the robot moved in
 * between, and that is not a disagreement. Instead of propagating one pose towards the other ---
 * which would require knowing whether the {@code ChassisSpeeds} come in robot or field frame --- the
 * tolerance is widened by the distance the robot could have covered in that time:
 *
 * <pre>{@code effective_tolerance = tolerance + speed * dt}</pre>
 *
 * It is conservative and does not depend on the reference frame. Without this, with the default
 * 0.1 s window and a robot at 4 m/s, two perfect sources differ by 0.4 m --- comparable to the whole
 * tolerance --- and the filter starts discarding good measurements exactly when they are needed
 * most.
 *
 * <h2>Majority vote</h2>
 * For each candidate, the number of others it agrees with is counted. The ones with the highest
 * count survive. On a tie, everything in dispute is discarded: a tie means the evidence is not
 * enough to choose, and feeding the estimator a pose another source contradicts is worse than
 * feeding it none.
 */
public class ConsensusGate {

    /** What to do when two sources contradict each other. */
    public enum Mode {
        /**
         * Discard both. This is the prudent choice with two sources, which is the normal case: with
         * no third one to break the tie, there is no way to tell which is lying.
         */
        REJECT_BOTH,

        /**
         * Keep the best-supported ones. This only makes sense with three or more sources; with two
         * there is always a tie and it behaves like {@link #REJECT_BOTH}.
         */
        MAJORITY_VOTE
    }

    /**
     * A candidate to arbitrate.
     *
     * @param name             source name, for the messages.
     * @param pose             the proposed pose.
     * @param timestampSeconds the instant of capture.
     */
    public record Candidate(String name, Pose2d pose, double timestampSeconds) {}

    /**
     * The outcome of the arbitration.
     *
     * @param rejected  indices of the discarded candidates.
     * @param conflicts a description of each disagreement, for telemetry or warnings.
     */
    public record Result(Set<Integer> rejected, List<String> conflicts) {

        /** A result with no discards and no conflicts. */
        static final Result CLEAN = new Result(Set.of(), List.of());

        /** @return true if every source agreed. */
        public boolean isUnanimous() {
            return conflicts.isEmpty();
        }
    }

    private Mode mode = Mode.REJECT_BOTH;
    private double toleranceMeters = 0.5;
    private double timeWindowSeconds = 0.1;
    private boolean motionCompensation = true;

    /**
     * What to do on a disagreement.
     *
     * @param newMode arbitration mode.
     * @return this same gate.
     */
    public ConsensusGate withMode(Mode newMode) {
        this.mode = newMode;
        return this;
    }

    /**
     * How far apart two poses may be before they count as contradictory.
     *
     * @param meters tolerance in metres.
     * @return this same gate.
     */
    public ConsensusGate withToleranceMeters(double meters) {
        this.toleranceMeters = meters;
        return this;
    }

    /**
     * Maximum time separation for two measurements to be compared. Beyond this window they
     * describe different moments and are not contrasted.
     *
     * @param seconds window in seconds.
     * @return this same gate.
     */
    public ConsensusGate withTimeWindowSeconds(double seconds) {
        this.timeWindowSeconds = seconds;
        return this;
    }

    /**
     * Controls whether the tolerance widens with the robot's speed.
     *
     * @param enabled true to compensate for motion.
     * @return this same gate.
     */
    public ConsensusGate withMotionCompensation(boolean enabled) {
        this.motionCompensation = enabled;
        return this;
    }

    /** @return the arbitration mode in effect. */
    public Mode mode() {
        return mode;
    }

    /** @return the tolerance in metres. */
    public double toleranceMeters() {
        return toleranceMeters;
    }

    /** @return the time window in seconds. */
    public double timeWindowSeconds() {
        return timeWindowSeconds;
    }

    /**
     * Arbitrates between the candidates.
     *
     * @param candidates            this cycle's candidates, in any order.
     * @param robotSpeedMetersPerSec robot speed, to compensate for motion. Pass 0 if you do not have
     *                               it.
     * @return which candidates to discard, and why.
     */
    public Result evaluate(List<Candidate> candidates, double robotSpeedMetersPerSec) {
        int n = candidates.size();
        if (n < 2) {
            return Result.CLEAN;
        }

        boolean[][] agrees = new boolean[n][n];
        Set<Integer> disputed = new HashSet<>();
        List<String> conflicts = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Candidate a = candidates.get(i);
                Candidate b = candidates.get(j);

                double dt = Math.abs(a.timestampSeconds() - b.timestampSeconds());
                if (dt > timeWindowSeconds) {
                    // Different moments: not comparable, neither to agree nor to disagree.
                    continue;
                }

                double distance = a.pose().getTranslation().getDistance(b.pose().getTranslation());
                double allowed = toleranceMeters;
                if (motionCompensation) {
                    allowed += Math.abs(robotSpeedMetersPerSec) * dt;
                }

                if (distance <= allowed) {
                    agrees[i][j] = true;
                    agrees[j][i] = true;
                } else {
                    disputed.add(i);
                    disputed.add(j);
                    conflicts.add(
                            String.format(
                                    "%s vs %s: %.2f m (limit %.2f m)",
                                    a.name(), b.name(), distance, allowed));
                }
            }
        }

        if (disputed.isEmpty()) {
            return Result.CLEAN;
        }

        if (mode == Mode.REJECT_BOTH) {
            return new Result(disputed, conflicts);
        }

        return new Result(majorityLosers(agrees, disputed, n), conflicts);
    }

    /**
     * Returns the disputed candidates with the least support.
     *
     * <p>Support is counted over <b>all</b> candidates, not only the disputed ones: a source is
     * corroborated by anyone who agrees with it, whether or not that one is arguing with somebody
     * else.
     */
    private static Set<Integer> majorityLosers(boolean[][] agrees, Set<Integer> disputed, int n) {
        int[] support = new int[n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && agrees[i][j]) {
                    support[i]++;
                }
            }
        }

        int best = Integer.MIN_VALUE;
        int worst = Integer.MAX_VALUE;
        for (int i : disputed) {
            best = Math.max(best, support[i]);
            worst = Math.min(worst, support[i]);
        }

        if (best == worst) {
            // Everyone arguing has the same support: there is no majority, so all of them go.
            return new HashSet<>(disputed);
        }

        Set<Integer> losers = new HashSet<>();
        for (int i : disputed) {
            if (support[i] < best) {
                losers.add(i);
            }
        }
        return losers;
    }
}
