// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * The format of a Limelight's {@code botpose} array, in one place.
 *
 * <p>The simulator writes it and the fusion reads it, so defining it twice would be asking for the
 * two to stop matching one day with nothing to warn you. A test makes the round trip between the two
 * halves of the feature so that cannot happen silently.
 *
 * <h2>The format</h2>
 * <pre>
 * [0..5]   x, y, z, roll, pitch, yaw      metres and degrees
 * [6]      total latency                  milliseconds
 * [7]      tag count
 * [8]      tag span                       metres
 * [9]      average camera-tag distance    metres
 * [10]     average area                   percentage of the image
 * [11+]    7 values per tag               id, txnc, tync, ta, distCamera, distRobot, ambiguity
 * </pre>
 *
 * <p>The tag block is only filled in if the length is <b>exactly</b> {@code 11 + 7 * tagCount}. It is
 * the same condition {@code LimelightHelpers} imposes, and the reason the published tag count has to
 * match the tags actually appended.
 */
public final class LimelightBotpose {

    /** Header values before the first tag. */
    public static final int HEADER_LENGTH = 11;

    /** Values each tag occupies. */
    public static final int VALUES_PER_FIDUCIAL = 7;

    private LimelightBotpose() {}

    /**
     * A visible tag, with the seven values Limelight publishes.
     *
     * @param id           fiducial id.
     * @param txnc         horizontal offset from the principal point, in degrees.
     * @param tync         vertical offset from the principal point, in degrees.
     * @param ta           tag area, as a percentage of the image.
     * @param distToCamera camera-to-tag distance, in metres.
     * @param distToRobot  robot-origin-to-tag distance, in metres.
     * @param ambiguity    ambiguity of the PnP solution, from 0 to 1.
     */
    public record Fiducial(
            int id,
            double txnc,
            double tync,
            double ta,
            double distToCamera,
            double distToRobot,
            double ambiguity) {}

    /**
     * A complete pose estimate, already parsed.
     *
     * @param pose              robot pose on the field.
     * @param timestampSeconds  the instant of capture, in the FPGA time base.
     * @param latencyMs         the frame's total latency.
     * @param tagCount          number of tags used.
     * @param tagSpanMeters     span between the tags used.
     * @param avgTagDistMeters  average distance to the tags.
     * @param avgTagAreaPercent average area of the tags.
     * @param fiducials         per-tag detail, empty if the array arrived without the block.
     * @param megaTag2          whether it comes from a {@code botpose_orb} key.
     */
    public record Estimate(
            Pose2d pose,
            double timestampSeconds,
            double latencyMs,
            int tagCount,
            double tagSpanMeters,
            double avgTagDistMeters,
            double avgTagAreaPercent,
            List<Fiducial> fiducials,
            boolean megaTag2) {

        /**
         * Ambiguity of the most ambiguous tag.
         *
         * <p>The worst is taken rather than the average because a badly solved tag is the one that
         * sends the robot to the other side of the field, and averaging it with good ones hides it.
         *
         * <p><b>Mind when you use it.</b> It only makes sense if {@link #tagCount()} is 1. With two or
         * more tags the pose comes from a geometrically constrained multi-tag PnP, while these numbers
         * are the ambiguity of each tag's individual solve --- precisely what the multi-tag solve
         * resolves. Filtering on them discards the best measurements available.
         *
         * @return the maximum ambiguity, or empty if there is no per-tag detail.
         */
        public OptionalDouble worstAmbiguity() {
            return fiducials.stream().mapToDouble(Fiducial::ambiguity).max();
        }

        /** @return true if there is a usable pose, that is, at least one tag. */
        public boolean hasTargets() {
            return tagCount > 0;
        }
    }

    /**
     * Parses a {@code botpose} array.
     *
     * <p>The capture timestamp comes from the NetworkTables clock minus the latency. It is worth
     * knowing that this is the <b>local</b> clock, not {@code serverTime}: that is why a Limelight
     * source is dated correctly even when the publisher lives in the same process, which is not the
     * case for QuestNav.
     *
     * @param raw                the array exactly as it comes out of NetworkTables.
     * @param ntTimestampMicros  the instant NetworkTables received the value, in microseconds.
     * @param megaTag2           whether it comes from a {@code botpose_orb} key.
     * @return the estimate, or empty if the array is empty or too short.
     */
    public static Optional<Estimate> parse(
            double[] raw, long ntTimestampMicros, boolean megaTag2) {
        if (raw == null || raw.length < HEADER_LENGTH) {
            return Optional.empty();
        }

        Pose2d pose =
                new Pose2d(
                        new Translation2d(raw[0], raw[1]),
                        Rotation2d.fromRadians(Units.degreesToRadians(raw[5])));

        double latencyMs = raw[6];
        int tagCount = (int) raw[7];
        double timestampSeconds = ntTimestampMicros / 1_000_000.0 - latencyMs / 1_000.0;

        return Optional.of(
                new Estimate(
                        pose,
                        timestampSeconds,
                        latencyMs,
                        tagCount,
                        raw[8],
                        raw[9],
                        raw[10],
                        parseFiducials(raw, tagCount),
                        megaTag2));
    }

    private static List<Fiducial> parseFiducials(double[] raw, int tagCount) {
        if (tagCount <= 0 || raw.length != HEADER_LENGTH + VALUES_PER_FIDUCIAL * tagCount) {
            return List.of();
        }

        List<Fiducial> fiducials = new ArrayList<>(tagCount);
        for (int i = 0; i < tagCount; i++) {
            int base = HEADER_LENGTH + i * VALUES_PER_FIDUCIAL;
            fiducials.add(
                    new Fiducial(
                            (int) raw[base],
                            raw[base + 1],
                            raw[base + 2],
                            raw[base + 3],
                            raw[base + 4],
                            raw[base + 5],
                            raw[base + 6]));
        }
        return fiducials;
    }
}
