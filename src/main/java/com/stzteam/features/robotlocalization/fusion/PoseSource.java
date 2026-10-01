// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import com.stzteam.forgemini.io.NetworkIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BiFunction;

/**
 * A source of pose measurements: a camera, a VIO headset, anything that can say where the robot is.
 *
 * <p>Writing one takes only two methods: {@link #getMeasurement()}, which reads the hardware, and
 * {@link #isPoseValid(Pose2d)}, which decides whether the resulting pose makes sense. Everything
 * else --- motion filters, ambiguity, timestamps, telemetry, enabling and disabling --- is already
 * here.
 *
 * <h2>The filters</h2>
 * {@link PoseFusion} applies them in this order, and the first one that fails discards the
 * measurement:
 *
 * <ol>
 *   <li><b>Enabled</b>, from code or from the dashboard.</li>
 *   <li><b>Environment</b>: angular velocity, acceleration and linear velocity below their
 *       thresholds. This is evaluated before reading the hardware, so a source discarded on
 *       environment does not even spend the time of the read.</li>
 *   <li><b>Timestamp</b> strictly newer than that of the last accepted measurement.</li>
 *   <li><b>Valid pose</b>, according to the subclass.</li>
 *   <li><b>Ambiguity</b>, if the source reports it.</li>
 *   <li><b>Consensus</b> with the other sources; {@link ConsensusGate} arbitrates it.</li>
 *   <li><b>Mahalanobis</b> against what the estimator already believes.</li>
 * </ol>
 *
 * <h2>Standard deviations</h2>
 * There are two routes. A fixed one with {@link #setStdDevs(double, double, double)}, and a dynamic
 * one with {@link #setStdDevsStrategy(BiFunction)}, which receives the distance and the tag count
 * and returns the matching confidence. The second is the normal choice for vision, and
 * {@link StdDevStrategies} ships the ready-made strategies.
 */
public abstract class PoseSource {

    private final String name;

    private Matrix<N3, N1> stdDevs;
    private PoseSourceConfig config;
    private BiFunction<Double, Integer, Matrix<N3, N1>> stdDevsStrategy;

    /** Filled in by {@link PoseFusion} when the source is registered. */
    protected LocalizationInputs inputs = LocalizationInputs.NONE;

    private String telemetryTable = "Localization";

    private boolean enabled = true;
    private boolean lastPublishedEnabled = true;

    private double lastAcceptedTimestamp = Double.NaN;
    private RejectionReason lastReason = RejectionReason.NO_MEASUREMENT;

    /**
     * Creates a source.
     *
     * @param name            unique name; it is the key in NetworkTables and in the registry.
     * @param defaultStdDevs  default standard deviations, used when no strategy is set.
     * @param config          filter thresholds.
     */
    protected PoseSource(String name, Matrix<N3, N1> defaultStdDevs, PoseSourceConfig config) {
        this.name = name;
        this.stdDevs = defaultStdDevs;
        this.config = config;
    }

    // ------------------------------------------------------------------
    // What the subclass implements
    // ------------------------------------------------------------------

    /**
     * Reads the hardware and returns this cycle's measurement, if there is one.
     *
     * <p>It is only called when the source is enabled and the environment is valid, so none of that
     * needs to be checked again.
     *
     * @return the measurement, or empty if there is nothing new.
     */
    public abstract Optional<PoseSample> getMeasurement();

    /**
     * Decides whether a pose makes sense. The usual check is that it falls inside the field, with
     * {@link PoseFusion#isInField(Pose2d, double)}.
     *
     * @param pose pose to validate.
     * @return true if the pose is acceptable.
     */
    public abstract boolean isPoseValid(Pose2d pose);

    /**
     * Ambiguity of the last PnP solution, if the source reports it.
     *
     * <p>Returning empty --- which is the default behaviour --- means "this source has no such
     * concept", and the ambiguity filter is skipped. That is the right thing for a VIO.
     *
     * @return the ambiguity from 0 to 1, or empty.
     */
    public OptionalDouble getAmbiguity() {
        return OptionalDouble.empty();
    }

    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    /**
     * Sets constant standard deviations and drops any dynamic strategy.
     *
     * @param stdX     deviation in X, metres.
     * @param stdY     deviation in Y, metres.
     * @param stdTheta heading deviation, radians.
     * @return this same source.
     */
    public PoseSource setStdDevs(double stdX, double stdY, double stdTheta) {
        return setStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
    }

    /**
     * Sets constant standard deviations and drops any dynamic strategy.
     *
     * @param newStdDevs deviation vector.
     * @return this same source.
     */
    public PoseSource setStdDevs(Matrix<N3, N1> newStdDevs) {
        this.stdDevs = newStdDevs;
        this.stdDevsStrategy = null;
        return this;
    }

    /**
     * Installs a strategy that computes the deviations from the distance to the tag and the number
     * of visible tags. See {@link StdDevStrategies}.
     *
     * @param strategy the strategy, or null to go back to fixed deviations.
     * @return this same source.
     */
    public PoseSource setStdDevsStrategy(BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        this.stdDevsStrategy = strategy;
        return this;
    }

    /**
     * Computes the deviations for a measurement. Subclasses call this when building their
     * {@link PoseSample}.
     *
     * @param distanceMeters average distance to the tag; ignored when there is no strategy.
     * @param tagCount       number of tags used; ignored when there is no strategy.
     * @return the deviation vector.
     */
    public Matrix<N3, N1> computeStdDevs(double distanceMeters, int tagCount) {
        if (stdDevsStrategy == null) {
            return stdDevs;
        }
        return stdDevsStrategy.apply(distanceMeters, Math.max(1, tagCount));
    }

    /**
     * Changes the filter thresholds.
     *
     * @param newConfig the new configuration.
     * @return this same source.
     */
    public PoseSource setConfig(PoseSourceConfig newConfig) {
        this.config = newConfig;
        return this;
    }

    /**
     * NetworkTables table the telemetry is published to. Change it if you have two
     * {@link PoseFusion} instances on the same robot and do not want them to collide.
     *
     * @param table table name.
     * @return this same source.
     */
    public PoseSource setTelemetryTable(String table) {
        this.telemetryTable = table;
        return this;
    }

    // ------------------------------------------------------------------
    // Enabling and disabling
    // ------------------------------------------------------------------

    /**
     * Enables or disables the source from code.
     *
     * <p>It coexists with the dashboard switch without either of them getting lost: both are
     * authoritative, and whichever moved last wins. See {@link #syncEnabledWithDashboard()}.
     *
     * @param value true to enable it.
     */
    public void setEnabled(boolean value) {
        this.enabled = value;
    }

    /** @return true if the source is enabled. */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Reconciles the dashboard switch with the one in code.
     *
     * <p>The detail that matters: the NetworkTables value is compared against <b>the last value we
     * published ourselves</b>, not against the current state. If they differ, somebody moved it
     * from the dashboard and that value wins; if they match, nobody touched it and the code's value
     * wins.
     *
     * <p>Without that comparison, reading NetworkTables at the start of every cycle silently
     * overwrites any {@link #setEnabled(boolean)}, and disabling a source from code stops working.
     */
    void syncEnabledWithDashboard() {
        boolean fromDashboard = NetworkIO.get(telemetryTable, name + "/enabled", lastPublishedEnabled);
        if (fromDashboard != lastPublishedEnabled) {
            enabled = fromDashboard;
        }
    }

    // ------------------------------------------------------------------
    // Filters
    // ------------------------------------------------------------------

    /**
     * Checks that the robot is not moving in a way that invalidates the measurement.
     *
     * @return the rejection reason, or {@link RejectionReason#ACCEPTED} if the environment is valid.
     */
    RejectionReason checkEnvironment() {
        if (!enabled) {
            return RejectionReason.DISABLED;
        }

        OptionalDouble angular = inputs.angularSpeedDegreesPerSecond();
        if (angular.isPresent() && angular.getAsDouble() > config.maxAngularVelocityDegPerSec()) {
            return RejectionReason.ANGULAR_VELOCITY;
        }

        OptionalDouble acceleration = inputs.planarAccelerationGs();
        if (acceleration.isPresent() && acceleration.getAsDouble() > config.maxAccelerationGs()) {
            return RejectionReason.ACCELERATION;
        }

        OptionalDouble planar = inputs.planarSpeedMetersPerSecond();
        if (planar.isPresent() && planar.getAsDouble() > config.maxPlanarVelocityMps()) {
            return RejectionReason.PLANAR_VELOCITY;
        }

        return RejectionReason.ACCEPTED;
    }

    /**
     * Checks the measurement itself: that it is new, that the pose makes sense and that the
     * ambiguity is within the threshold.
     *
     * @param sample measurement to check.
     * @return the rejection reason, or {@link RejectionReason#ACCEPTED}.
     */
    RejectionReason checkMeasurement(PoseSample sample) {
        if (!Double.isNaN(lastAcceptedTimestamp)
                && sample.timestampSeconds() <= lastAcceptedTimestamp) {
            return RejectionReason.STALE_TIMESTAMP;
        }
        if (!isPoseValid(sample.pose())) {
            return RejectionReason.POSE_INVALID;
        }

        OptionalDouble ambiguity = getAmbiguity();
        if (ambiguity.isPresent() && ambiguity.getAsDouble() > config.maxAmbiguity()) {
            return RejectionReason.AMBIGUITY;
        }

        return RejectionReason.ACCEPTED;
    }

    /**
     * Mahalanobis filter: discards anything too far from what the estimator already believes,
     * measured in sigmas of the combined uncertainty.
     *
     * <p>The estimator's uncertainty grows with the time since the last accepted measurement, so
     * the filter opens up on its own once the source has been quiet for a while. That avoids the
     * classic failure of this kind of gate: staying shut forever because the estimator wandered off
     * and nothing looks reasonable to it any more.
     *
     * <p>The <b>first</b> measurement is always accepted. There is nothing to compare it against:
     * the estimator does not yet hold a belief worth defending.
     *
     * @param sample measurement to check.
     * @return the rejection reason, or {@link RejectionReason#ACCEPTED}.
     */
    RejectionReason checkMahalanobis(PoseSample sample) {
        if (!config.mahalanobisEnabled() || Double.isNaN(lastAcceptedTimestamp)) {
            return RejectionReason.ACCEPTED;
        }

        Optional<Pose2d> estimate = inputs.pose();
        if (estimate.isEmpty()) {
            return RejectionReason.ACCEPTED;
        }

        double elapsed = Math.max(0.0, sample.timestampSeconds() - lastAcceptedTimestamp);
        double drift = elapsed * config.mahalanobisDriftVariancePerSecond();

        double estimatorVarXY = square(config.mahalanobisBaseStdDevMeters()) + drift;
        double estimatorVarTheta = square(config.mahalanobisBaseStdDevRadians()) + drift;

        Pose2d current = estimate.get();
        double dx = sample.pose().getX() - current.getX();
        double dy = sample.pose().getY() - current.getY();
        double dtheta = sample.pose().getRotation().minus(current.getRotation()).getRadians();

        double varX = estimatorVarXY + square(sample.stdDevs().get(0, 0));
        double varY = estimatorVarXY + square(sample.stdDevs().get(1, 0));
        double varTheta = estimatorVarTheta + square(sample.stdDevs().get(2, 0));

        double distanceSquared = (dx * dx) / varX + (dy * dy) / varY + (dtheta * dtheta) / varTheta;

        return distanceSquared <= square(config.maxMahalanobisSigmas())
                ? RejectionReason.ACCEPTED
                : RejectionReason.MAHALANOBIS;
    }

    private static double square(double value) {
        return value * value;
    }

    // ------------------------------------------------------------------
    // State and telemetry
    // ------------------------------------------------------------------

    void markAccepted(double timestampSeconds) {
        this.lastAcceptedTimestamp = timestampSeconds;
        this.lastReason = RejectionReason.ACCEPTED;
    }

    void markRejected(RejectionReason reason) {
        this.lastReason = reason;
    }

    /**
     * A free-form detail the subclass can publish alongside the status.
     *
     * <p>It exists so {@link RejectionReason#NO_MEASUREMENT} is not a dead end: "the source
     * reported nothing" could mean the camera sees no tags, the headset is disconnected, or the
     * timestamp is unusable, and from the dashboard all three look the same.
     *
     * @return a short line, or the empty string if there is nothing to add.
     */
    protected String telemetryDetail() {
        return "";
    }

    void publishTelemetry() {
        NetworkIO.set(telemetryTable, name + "/enabled", enabled);
        NetworkIO.set(telemetryTable, name + "/status", lastReason.name());
        NetworkIO.set(telemetryTable, name + "/detail", telemetryDetail());
        NetworkIO.set(telemetryTable, name + "/accepted", lastReason == RejectionReason.ACCEPTED);
        NetworkIO.set(
                telemetryTable,
                name + "/lastAcceptedTimestamp",
                Double.isNaN(lastAcceptedTimestamp) ? -1.0 : lastAcceptedTimestamp);
        lastPublishedEnabled = enabled;
    }

    /** @return the source name. */
    public String getName() {
        return name;
    }

    /** @return the configuration in effect. */
    public PoseSourceConfig getConfig() {
        return config;
    }

    /** @return the fixed standard deviations, with no strategy applied. */
    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    /** @return why the last measurement was discarded --- or not. */
    public RejectionReason getLastReason() {
        return lastReason;
    }

    /**
     * The instant of the last accepted measurement.
     *
     * @return the instant, or empty if none has been accepted yet.
     */
    public OptionalDouble getLastAcceptedTimestamp() {
        return Double.isNaN(lastAcceptedTimestamp)
                ? OptionalDouble.empty()
                : OptionalDouble.of(lastAcceptedTimestamp);
    }

    void bindInputs(LocalizationInputs newInputs) {
        this.inputs = newInputs == null ? LocalizationInputs.NONE : newInputs;
    }
}
