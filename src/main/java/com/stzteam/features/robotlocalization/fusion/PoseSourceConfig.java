// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Filter thresholds for a {@link PoseSource}.
 *
 * <p>Fluent setters, like the rest of the feature's configuration objects:
 *
 * <pre>{@code
 * var config = PoseSourceConfig.forAprilTags()
 *         .withMaxAngularVelocity(DegreesPerSecond.of(540))
 *         .withMaxAmbiguity(0.25);
 * }</pre>
 *
 * <h2>How a filter is switched off</h2>
 * Every threshold starts at infinity, which is the way of saying "no limit": the comparison
 * {@code value <= INFINITY} always passes, so an unconfigured filter filters nothing. There are no
 * enable flags or sentinel values to memorise.
 */
public class PoseSourceConfig {

    private double maxAngularVelocityDegPerSec = Double.POSITIVE_INFINITY;
    private double maxPlanarVelocityMps = Double.POSITIVE_INFINITY;
    private double maxAccelerationGs = Double.POSITIVE_INFINITY;
    private double maxAmbiguity = Double.POSITIVE_INFINITY;
    private double maxMahalanobisSigmas = Double.POSITIVE_INFINITY;

    private double mahalanobisBaseStdDevMeters = 0.05;
    private double mahalanobisBaseStdDevRadians = 0.05;
    private double mahalanobisDriftVariancePerSecond = 0.0;

    /** Creates a configuration with no filter active. */
    public PoseSourceConfig() {}

    /**
     * A sensible preset for a camera looking at AprilTags.
     *
     * <p>The numbers are defensible starting points, not truths: 720 degrees per second is roughly
     * where motion blur starts to ruin PnP on a global-shutter camera, and an ambiguity of 0.3 is
     * the threshold PhotonVision recommends. Tune them against your own data.
     *
     * @return a new configuration.
     */
    public static PoseSourceConfig forAprilTags() {
        return new PoseSourceConfig()
                .withMaxAngularVelocity(DegreesPerSecond.of(720))
                .withMaxAmbiguity(0.3)
                .withMahalanobisGate(4.0, 0.05, 0.05, 0.02);
    }

    /**
     * A sensible preset for a visual-inertial odometry source such as QuestNav.
     *
     * <p>No angular velocity limit: a VIO does not do PnP, so turning does not affect it the way it
     * affects a camera. No ambiguity, because it reports none. The Mahalanobis gate is tighter
     * because a VIO should not jump, and when it does --- re-acquisition after losing tracking ---
     * that is exactly what needs catching.
     *
     * @return a new configuration.
     */
    public static PoseSourceConfig forVisualInertialOdometry() {
        return new PoseSourceConfig().withMahalanobisGate(3.0, 0.02, 0.02, 0.05);
    }

    /**
     * Angular velocity above which measurements are discarded.
     *
     * @param limit maximum angular velocity.
     * @return this same configuration.
     */
    public PoseSourceConfig withMaxAngularVelocity(AngularVelocity limit) {
        this.maxAngularVelocityDegPerSec =
                limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(DegreesPerSecond));
        return this;
    }

    /**
     * Linear velocity above which measurements are discarded.
     *
     * @param limit maximum linear velocity.
     * @return this same configuration.
     */
    public PoseSourceConfig withMaxPlanarVelocity(LinearVelocity limit) {
        this.maxPlanarVelocityMps =
                limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(MetersPerSecond));
        return this;
    }

    /**
     * Acceleration above which measurements are discarded. This mostly serves to ignore the moment
     * of a collision, when the camera moves faster than its shutter can take.
     *
     * @param limit maximum acceleration.
     * @return this same configuration.
     */
    public PoseSourceConfig withMaxAcceleration(LinearAcceleration limit) {
        this.maxAccelerationGs = limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(Gs));
        return this;
    }

    /**
     * PnP ambiguity above which the measurement is discarded. It only applies to sources that
     * report ambiguity; the ones that do not skip the filter.
     *
     * @param limit maximum ambiguity, from 0 to 1.
     * @return this same configuration.
     */
    public PoseSourceConfig withMaxAmbiguity(double limit) {
        this.maxAmbiguity = limit;
        return this;
    }

    /**
     * Configures the Mahalanobis filter, which discards measurements too far from what the
     * estimator already believes.
     *
     * <p>The uncertainty it compares against <b>grows with time</b> since the last accepted
     * measurement. That is what makes the filter useful rather than a padlock: if a source has
     * contributed nothing for ten seconds, the estimator may have wandered off, and the filter opens
     * up to let it be corrected.
     *
     * @param sigmas                maximum distance in sigmas. 3 is tight, 5 is permissive.
     * @param baseStdDevMeters      position uncertainty of a freshly corrected estimator.
     * @param baseStdDevRadians     heading uncertainty of a freshly corrected estimator.
     * @param driftVariancePerSecond how much variance the odometry adds per second without a
     *                               correction.
     * @return this same configuration.
     */
    public PoseSourceConfig withMahalanobisGate(
            double sigmas,
            double baseStdDevMeters,
            double baseStdDevRadians,
            double driftVariancePerSecond) {
        this.maxMahalanobisSigmas = sigmas;
        this.mahalanobisBaseStdDevMeters = baseStdDevMeters;
        this.mahalanobisBaseStdDevRadians = baseStdDevRadians;
        this.mahalanobisDriftVariancePerSecond = driftVariancePerSecond;
        return this;
    }

    /** @return maximum angular velocity in degrees per second. */
    public double maxAngularVelocityDegPerSec() {
        return maxAngularVelocityDegPerSec;
    }

    /** @return maximum linear velocity in metres per second. */
    public double maxPlanarVelocityMps() {
        return maxPlanarVelocityMps;
    }

    /** @return maximum acceleration in G. */
    public double maxAccelerationGs() {
        return maxAccelerationGs;
    }

    /** @return maximum ambiguity. */
    public double maxAmbiguity() {
        return maxAmbiguity;
    }

    /** @return maximum Mahalanobis distance, in sigmas. */
    public double maxMahalanobisSigmas() {
        return maxMahalanobisSigmas;
    }

    /** @return true if the Mahalanobis filter is active. */
    public boolean mahalanobisEnabled() {
        return Double.isFinite(maxMahalanobisSigmas);
    }

    /** @return the estimator's base position uncertainty, in metres. */
    public double mahalanobisBaseStdDevMeters() {
        return mahalanobisBaseStdDevMeters;
    }

    /** @return the estimator's base heading uncertainty, in radians. */
    public double mahalanobisBaseStdDevRadians() {
        return mahalanobisBaseStdDevRadians;
    }

    /** @return the variance the odometry adds per second without a correction. */
    public double mahalanobisDriftVariancePerSecond() {
        return mahalanobisDriftVariancePerSecond;
    }
}
