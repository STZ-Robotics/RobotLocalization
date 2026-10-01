// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import com.stzteam.features.robotlocalization.vision.LimelightBotpose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BiFunction;

/**
 * A Limelight camera as a pose source.
 *
 * <p>It reads NetworkTables directly, without {@code LimelightHelpers}. That class is a file each
 * team copies into its own repo, not a Maven artifact, so a library cannot depend on it --- and does
 * not need to: we know the whole schema because this feature's own simulator writes it.
 *
 * <p>As a useful consequence, this source works identically against a physical Limelight and against
 * {@code SimulatedLimelight}, with no simulation-specific branch anywhere.
 *
 * <h2>MegaTag1 and MegaTag2</h2>
 * MegaTag1 solves the pose from what it sees alone. MegaTag2 pins the heading with the robot's gyro
 * and solves the rest, which makes it far more stable while turning, but it <b>requires</b> that
 * somebody write the heading to it every cycle. This source does that on its own if you give it a
 * {@link LocalizationInputs#gyroHeading()}; it is the most frequently forgotten step and the one that
 * makes MegaTag2 return zeros without explaining why.
 *
 * <p>With MegaTag2 the heading that comes out is the one you fed in, so it carries no new
 * information: it is worth giving it an enormous heading sigma so the estimator ignores it. That is
 * what {@link #megaTag2(String)} does by default.
 */
public class LimelightPoseSource extends PoseSource {

    /** Which Limelight solution to consume. */
    public enum Mode {
        /** Pose solved from vision alone. Useful to initialise the heading. */
        MEGATAG_1,
        /** Pose with the heading pinned by the gyro. Far more stable while turning. */
        MEGATAG_2
    }

    /** The heading sigma that tells the estimator "ignore this component". */
    public static final double IGNORE_HEADING_STD_DEV = 9_999_999.0;

    private final Mode mode;
    private final NetworkTable table;
    private final DoubleArrayEntry botposeEntry;
    private final DoubleArrayPublisher orientationPublisher;

    private double fieldMarginMeters = 0.5;
    private boolean publishOrientation = true;
    private OptionalDouble lastAmbiguity = OptionalDouble.empty();
    private String detail = "";

    /**
     * Creates a MegaTag1 source with the recommended configuration for AprilTags.
     *
     * @param name the Limelight's table name, for example {@code "limelight-front"}.
     * @return the source.
     */
    public static LimelightPoseSource megaTag1(String name) {
        return new LimelightPoseSource(name, Mode.MEGATAG_1, PoseSourceConfig.forAprilTags())
                .withStdDevsStrategy(StdDevStrategies.quadraticContinuous(0.08, 0.16));
    }

    /**
     * Creates a MegaTag2 source with the recommended configuration.
     *
     * <p>No angular velocity filter --- MegaTag2 copes with turning, which is exactly its reason for
     * existing --- and with the heading sigma saturated so the estimator does not believe a heading it
     * supplied itself.
     *
     * @param name the Limelight's table name.
     * @return the source.
     */
    public static LimelightPoseSource megaTag2(String name) {
        return new LimelightPoseSource(
                        name,
                        Mode.MEGATAG_2,
                        PoseSourceConfig.forAprilTags().withMaxAngularVelocity(null))
                .withStdDevsStrategy(
                        StdDevStrategies.scaled(
                                StdDevStrategies.quadraticContinuous(0.04, 1.0),
                                1.0,
                                IGNORE_HEADING_STD_DEV));
    }

    /**
     * Creates a source on the global NetworkTables instance.
     *
     * @param name   the Limelight's table name.
     * @param mode   which solution to consume.
     * @param config filter thresholds.
     */
    public LimelightPoseSource(String name, Mode mode, PoseSourceConfig config) {
        this(name, mode, config, NetworkTableInstance.getDefault());
    }

    /**
     * Creates a source on a specific NetworkTables instance.
     *
     * @param name     the Limelight's table name.
     * @param mode     which solution to consume.
     * @param config   filter thresholds.
     * @param instance the NetworkTables instance.
     */
    public LimelightPoseSource(
            String name, Mode mode, PoseSourceConfig config, NetworkTableInstance instance) {
        super(name, VecBuilder.fill(0.5, 0.5, 1.0), config);
        this.mode = mode;
        this.table = instance.getTable(name);

        String key = mode == Mode.MEGATAG_2 ? "botpose_orb_wpiblue" : "botpose_wpiblue";
        // An entry rather than getEntry().getDoubleArray(): the value's atomic timestamp is needed
        // to date the capture, and only getAtomic() provides it.
        this.botposeEntry = table.getDoubleArrayTopic(key).getEntry(new double[0]);
        this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    }

    /**
     * Margin around the field within which a pose still counts as valid.
     *
     * @param meters margin in metres.
     * @return this same source.
     */
    public LimelightPoseSource withFieldMargin(double meters) {
        this.fieldMarginMeters = meters;
        return this;
    }

    /**
     * Controls whether the source writes the heading to the Limelight. Switch it off only if another
     * part of your code already does; without it, MegaTag2 produces nothing.
     *
     * @param enabled true to write it.
     * @return this same source.
     */
    public LimelightPoseSource withOrientationPublishing(boolean enabled) {
        this.publishOrientation = enabled;
        return this;
    }

    /**
     * Installs a deviation strategy and returns the concrete type, so calls can be chained.
     *
     * @param strategy a strategy from {@link StdDevStrategies}.
     * @return this same source.
     */
    public LimelightPoseSource withStdDevsStrategy(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        setStdDevsStrategy(strategy);
        return this;
    }

    /** @return this source's mode. */
    public Mode getMode() {
        return mode;
    }

    /** @return the NetworkTables table being read. */
    public NetworkTable getTable() {
        return table;
    }

    @Override
    public Optional<PoseSample> getMeasurement() {
        if (publishOrientation) {
            pushRobotOrientation();
        }

        TimestampedDoubleArray value = botposeEntry.getAtomic();
        lastAmbiguity = OptionalDouble.empty();

        Optional<LimelightBotpose.Estimate> parsed =
                LimelightBotpose.parse(value.value, value.timestamp, mode == Mode.MEGATAG_2);

        if (parsed.isEmpty() || !parsed.get().hasTargets()) {
            detail =
                    mode == Mode.MEGATAG_2 && inputs.heading().isEmpty()
                            ? "MegaTag2 has no data: nobody is feeding it the robot heading"
                            : "no tags in sight";
            return Optional.empty();
        }

        LimelightBotpose.Estimate estimate = parsed.get();

        // Ambiguity only means something with ONE tag in sight. With two or more, the solution comes
        // from a multi-tag PnP that is geometrically constrained, and the per-tag ambiguity Limelight
        // reports is that of each tag's individual solve --- exactly what the multi-tag solve
        // resolves. Applying the filter there discards the best measurements available: against the
        // simulation, three tags at twelve metres gave per-tag ambiguities of 0.97 and the gate ate
        // 100% of the frames.
        lastAmbiguity =
                estimate.tagCount() == 1 ? estimate.worstAmbiguity() : OptionalDouble.empty();

        detail =
                String.format(
                        "%d tag(s) at %.1f m%s",
                        estimate.tagCount(),
                        estimate.avgTagDistMeters(),
                        lastAmbiguity.isPresent()
                                ? String.format(", ambiguity %.2f", lastAmbiguity.getAsDouble())
                                : " (multi-tag, no ambiguity filter)");

        return Optional.of(
                new PoseSample(
                        estimate.pose(),
                        estimate.timestampSeconds(),
                        computeStdDevs(estimate.avgTagDistMeters(), estimate.tagCount())));
    }

    @Override
    protected String telemetryDetail() {
        return detail;
    }

    /**
     * Writes the robot heading to the Limelight, which is what MegaTag2 needs in order to solve.
     *
     * <p>Only the yaw is filled in. The other five values exist in the protocol but Limelight itself
     * documents them as unnecessary.
     */
    private void pushRobotOrientation() {
        inputs.heading()
                .ifPresent(
                        heading ->
                                orientationPublisher.set(
                                        new double[] {heading.getDegrees(), 0, 0, 0, 0, 0}));
    }

    @Override
    public boolean isPoseValid(Pose2d pose) {
        return PoseFusion.isInField(pose, fieldMarginMeters);
    }

    @Override
    public OptionalDouble getAmbiguity() {
        return lastAmbiguity;
    }
}
