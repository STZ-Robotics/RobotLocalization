// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import com.stzteam.forgemini.io.NetworkIO;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * The gatekeeper that decides which vision measurements reach the pose estimator.
 *
 * <p>It is not an estimator or a Kalman filter: WPILib already ships those. What is missing, and
 * what this class does, is everything that comes before --- registering several sources, discarding
 * the ones that lie, and handing over only what is left.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * private final PoseFusion fusion = new PoseFusion(
 *         poseEstimator::addVisionMeasurement,
 *         LocalizationInputs.of(this::getPose, this::getChassisSpeeds))
 *     .registerSource(new LimelightPoseSource("limelight-front", ROBOT_TO_CAMERA))
 *     .registerSource(new QuestNavPoseSource("quest", ROBOT_TO_QUEST));
 *
 * @Override
 * public void periodic() {
 *     fusion.update();
 * }
 * }</pre>
 *
 * <h2>Order matters</h2>
 * The filters run from cheap to expensive and from local to global. First what can be decided
 * without reading hardware --- is it enabled, is the robot not spinning like a top ---, then the
 * measurement itself, and last what needs to see every source at once: consensus and Mahalanobis. A
 * source discarded on environment does not even spend the read.
 *
 * <p>Every source publishes to NetworkTables why its last measurement was discarded. See
 * {@link RejectionReason}: without it, a badly calibrated filter and a broken camera look the same.
 */
public class PoseFusion {

    /**
     * Where the surviving measurements end up. The signature is that of
     * {@code SwerveDrivePoseEstimator.addVisionMeasurement}, so it can be passed as a method
     * reference and without boxing the timestamp into a {@code Double}.
     */
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        /**
         * Hands a measurement to the estimator.
         *
         * @param pose             the measured pose.
         * @param timestampSeconds the instant of capture.
         * @param stdDevs          confidence in the measurement.
         */
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    /** How often a disagreement warning repeats, in seconds. */
    private static final double WARNING_PERIOD_SECONDS = 2.0;

    private static AprilTagFieldLayout defaultFieldLayout = FieldLayouts.defaultField();

    private final VisionMeasurementConsumer consumer;
    private final LocalizationInputs inputs;
    private final Map<String, PoseSource> sources = new LinkedHashMap<>();
    private final ConsensusGate consensusGate = new ConsensusGate();

    private AprilTagFieldLayout fieldLayout = defaultFieldLayout;
    private String telemetryTable = "Localization";
    private boolean consensusEnabled = true;
    private double lastWarningSeconds = Double.NEGATIVE_INFINITY;
    private int acceptedThisCycle = 0;

    /**
     * Creates the fusion.
     *
     * @param consumer where to hand the accepted measurements, usually
     *                 {@code poseEstimator::addVisionMeasurement}.
     * @param inputs   the robot state the sources share.
     */
    public PoseFusion(VisionMeasurementConsumer consumer, LocalizationInputs inputs) {
        this.consumer = consumer;
        this.inputs = inputs == null ? LocalizationInputs.NONE : inputs;
    }

    /**
     * Changes the layout used by {@link #isInField(Pose2d)} and by any source that does not bring
     * its own.
     *
     * @param layout the field layout.
     */
    public static void setDefaultFieldLayout(AprilTagFieldLayout layout) {
        defaultFieldLayout = layout;
    }

    /** @return the feature's default layout. */
    public static AprilTagFieldLayout defaultFieldLayout() {
        return defaultFieldLayout;
    }

    /**
     * Checks that a pose falls inside the field, using the default layout.
     *
     * @param pose pose to check.
     * @return true if it is inside.
     */
    public static boolean isInField(Pose2d pose) {
        return isInField(pose, defaultFieldLayout, 0.0);
    }

    /**
     * Checks that a pose falls inside the field, with a margin.
     *
     * <p>A small margin --- on the order of half a metre --- avoids discarding good measurements
     * from a robot up against the wall, where the chassis centre can land outside the rectangle from
     * the uncertainty alone.
     *
     * @param pose          pose to check.
     * @param marginMeters  margin around the field.
     * @return true if it is inside.
     */
    public static boolean isInField(Pose2d pose, double marginMeters) {
        return isInField(pose, defaultFieldLayout, marginMeters);
    }

    /**
     * Checks that a pose falls inside the field of a specific layout.
     *
     * @param pose         pose to check.
     * @param layout       the field layout.
     * @param marginMeters margin around the field.
     * @return true if it is inside.
     */
    public static boolean isInField(Pose2d pose, AprilTagFieldLayout layout, double marginMeters) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -marginMeters
                && x <= layout.getFieldLength() + marginMeters
                && y >= -marginMeters
                && y <= layout.getFieldWidth() + marginMeters;
    }

    /**
     * Registers a source. The inputs and the telemetry table are injected into it.
     *
     * @param source source to register.
     * @return this same fusion, for chaining.
     */
    public PoseFusion registerSource(PoseSource source) {
        source.bindInputs(inputs);
        source.setTelemetryTable(telemetryTable);
        sources.put(source.getName(), source);
        return this;
    }

    /**
     * Looks a source up by name.
     *
     * @param name source name.
     * @return the source, or empty.
     */
    public Optional<PoseSource> getSource(String name) {
        return Optional.ofNullable(sources.get(name));
    }

    /**
     * Looks a source up by name and type.
     *
     * @param <T>  the concrete type expected.
     * @param name source name.
     * @param type the expected class.
     * @return the source, or empty if it does not exist or is of another type.
     */
    public <T extends PoseSource> Optional<T> getSource(String name, Class<T> type) {
        PoseSource source = sources.get(name);
        return type.isInstance(source) ? Optional.of(type.cast(source)) : Optional.empty();
    }

    /** @return every registered source, in registration order. */
    public Collection<PoseSource> getSources() {
        return sources.values();
    }

    /** @return the consensus gate, so it can be configured. */
    public ConsensusGate consensus() {
        return consensusGate;
    }

    /**
     * Enables or disables arbitration between sources.
     *
     * @param enabled true to arbitrate.
     * @return this same fusion.
     */
    public PoseFusion setConsensusEnabled(boolean enabled) {
        this.consensusEnabled = enabled;
        return this;
    }

    /** @return true if arbitration is active. */
    public boolean isConsensusEnabled() {
        return consensusEnabled;
    }

    /**
     * NetworkTables table for the telemetry. Change it before registering the sources.
     *
     * @param table table name.
     * @return this same fusion.
     */
    public PoseFusion setTelemetryTable(String table) {
        this.telemetryTable = table;
        sources.values().forEach(s -> s.setTelemetryTable(table));
        return this;
    }

    /** @return how many measurements were accepted in the last {@link #update()}. */
    public int getAcceptedCount() {
        return acceptedThisCycle;
    }

    /**
     * One full cycle. Call it from {@code periodic()}.
     *
     * <p>It is safe to call even with no source registered or with all of them disabled: in that
     * case it does nothing beyond publishing telemetry.
     */
    public void update() {
        acceptedThisCycle = 0;
        List<PoseSource> candidateSources = new ArrayList<>();
        List<PoseSample> candidateSamples = new ArrayList<>();

        for (PoseSource source : sources.values()) {
            source.syncEnabledWithDashboard();

            RejectionReason environment = source.checkEnvironment();
            if (environment != RejectionReason.ACCEPTED) {
                source.markRejected(environment);
                continue;
            }

            Optional<PoseSample> measurement;
            try {
                measurement = source.getMeasurement();
            } catch (RuntimeException e) {
                // A broken source cannot take the others down with it: the robot still needs to
                // localise with whatever is left.
                source.markRejected(RejectionReason.SOURCE_ERROR);
                warn("Source '" + source.getName() + "' failed: " + e);
                continue;
            }

            if (measurement.isEmpty()) {
                source.markRejected(RejectionReason.NO_MEASUREMENT);
                continue;
            }

            PoseSample sample = measurement.get();
            RejectionReason check = source.checkMeasurement(sample);
            if (check != RejectionReason.ACCEPTED) {
                source.markRejected(check);
                continue;
            }

            candidateSources.add(source);
            candidateSamples.add(sample);
        }

        ConsensusGate.Result consensus = arbitrate(candidateSources, candidateSamples);

        for (int i = 0; i < candidateSources.size(); i++) {
            PoseSource source = candidateSources.get(i);
            PoseSample sample = candidateSamples.get(i);

            if (consensus.rejected().contains(i)) {
                source.markRejected(RejectionReason.CONSENSUS);
                continue;
            }

            RejectionReason mahalanobis = source.checkMahalanobis(sample);
            if (mahalanobis != RejectionReason.ACCEPTED) {
                source.markRejected(mahalanobis);
                continue;
            }

            source.markAccepted(sample.timestampSeconds());
            consumer.accept(sample.pose(), sample.timestampSeconds(), sample.stdDevs());
            acceptedThisCycle++;
        }

        publishTelemetry();
    }

    private ConsensusGate.Result arbitrate(List<PoseSource> sourceList, List<PoseSample> sampleList) {
        if (!consensusEnabled || sourceList.size() < 2) {
            return new ConsensusGate.Result(java.util.Set.of(), List.of());
        }

        List<ConsensusGate.Candidate> candidates = new ArrayList<>(sourceList.size());
        for (int i = 0; i < sourceList.size(); i++) {
            candidates.add(
                    new ConsensusGate.Candidate(
                            sourceList.get(i).getName(),
                            sampleList.get(i).pose(),
                            sampleList.get(i).timestampSeconds()));
        }

        OptionalDouble speed = inputs.planarSpeedMetersPerSecond();
        ConsensusGate.Result result =
                consensusGate.evaluate(candidates, speed.orElse(0.0));

        for (String conflict : result.conflicts()) {
            warn("Consensus: " + conflict);
        }
        return result;
    }

    /**
     * Warns on the console with a rate limit.
     *
     * <p>A sustained disagreement repeats every cycle, and at 50 Hz that fills the Driver Station
     * log fast enough to bury every other message.
     */
    private void warn(String message) {
        double now = Timer.getFPGATimestamp();
        if (now - lastWarningSeconds < WARNING_PERIOD_SECONDS) {
            return;
        }
        lastWarningSeconds = now;
        DriverStation.reportWarning("[PoseFusion] " + message, false);
    }

    private void publishTelemetry() {
        for (PoseSource source : sources.values()) {
            source.publishTelemetry();
        }
        NetworkIO.set(telemetryTable, "acceptedThisCycle", acceptedThisCycle);
        NetworkIO.set(telemetryTable, "sourceCount", sources.size());
    }
}
