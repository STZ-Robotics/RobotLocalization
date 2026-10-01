// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Optional;

/**
 * An immutable snapshot of one simulated-camera cycle, already translated into the
 * semantics of a real Limelight (degrees, percentages, sign conventions).
 *
 * <p>This is the feature's internal currency: the {@link LimelightTargetTranslator}
 * produces it from a {@code PhotonPipelineResult}, and both {@link LimelightNTPublisher}
 * and {@link LimelightSimDevice} consume it. Keeping a single frame per cycle guarantees
 * NetworkTables and the SimDevice never fall out of sync.
 *
 * @param hasTarget              equivalent to {@code tv != 0}.
 * @param tx                     horizontal offset to the crosshair in degrees, positive to the right
 *                               (Limelight convention; PhotonVision uses positive to the left).
 * @param ty                     vertical offset to the crosshair in degrees, positive upwards.
 * @param txnc                   horizontal offset from the principal point, in degrees.
 * @param tync                   vertical offset from the principal point, in degrees.
 * @param ta                     target area as a percentage of the image (0-100).
 * @param ts                     target skew/rotation in degrees.
 * @param tid                    id of the primary AprilTag, or -1 if there is no target.
 * @param pipelineLatencyMs      latency attributed to the pipeline (key {@code tl}).
 * @param captureLatencyMs       latency attributed to the capture (key {@code cl}).
 * @param captureTimestampSeconds the frame's capture instant, in the FPGA time base.
 * @param targetCount            number of visible fiducials after applying the id filters.
 * @param fiducials              per-visible-tag detail, in the order {@code rawfiducials} publishes.
 * @param megaTag1Pose           robot pose estimated without an external heading (the MegaTag1
 *                               equivalent), in the blue-origin field frame.
 * @param megaTag2Pose           robot pose estimated by pinning the heading the robot code wrote to
 *                               {@code robot_orientation_set} (the MegaTag2 equivalent).
 * @param tagSpanMeters          maximum distance between the tags used for the estimate.
 * @param avgTagDistMeters       average camera-to-tag distance of the tags used.
 * @param avgTagAreaPercent      average area of the tags used, as a percentage of the image.
 * @param cameraToTarget         camera -&gt; primary tag transform, in the WPILib convention.
 * @param robotToTarget          robot -&gt; primary tag transform, in the WPILib convention.
 * @param targetPixelMetrics     pixel metrics of the primary target, for the {@code t2d} key.
 * @param cornersXY              detected corners in pixels, flattened as {@code [x0,y0,x1,y1,...]}.
 */
public record LimelightFrame(
        boolean hasTarget,
        double tx,
        double ty,
        double txnc,
        double tync,
        double ta,
        double ts,
        int tid,
        double pipelineLatencyMs,
        double captureLatencyMs,
        double captureTimestampSeconds,
        int targetCount,
        RawFiducial[] fiducials,
        Optional<Pose3d> megaTag1Pose,
        Optional<Pose3d> megaTag2Pose,
        double tagSpanMeters,
        double avgTagDistMeters,
        double avgTagAreaPercent,
        Optional<Transform3d> cameraToTarget,
        Optional<Transform3d> robotToTarget,
        PixelMetrics targetPixelMetrics,
        double[] cornersXY) {

    /** A reusable empty array for frames with no detections. */
    private static final RawFiducial[] NO_FIDUCIALS = new RawFiducial[0];

    /** A reusable empty array for frames with no corners. */
    private static final double[] NO_CORNERS = new double[0];

    /**
     * A single AprilTag detection, with the same content and order as the 7 per-tag values
     * a real Limelight publishes to {@code rawfiducials}.
     *
     * @param id            fiducial id.
     * @param txnc          horizontal offset from the principal point, in degrees.
     * @param tync          vertical offset from the principal point, in degrees.
     * @param ta            tag area as a percentage of the image.
     * @param distToCamera  camera-to-tag distance in metres.
     * @param distToRobot   robot-origin-to-tag distance in metres.
     * @param ambiguity     ambiguity of the PnP solution (0 = no ambiguity).
     */
    public record RawFiducial(
            int id,
            double txnc,
            double tync,
            double ta,
            double distToCamera,
            double distToRobot,
            double ambiguity) {}

    /**
     * Pixel metrics of the primary target. They feed indices 12-15 of the {@code t2d} key;
     * a real Limelight exposes them for fine-grained 2D vision logic.
     *
     * @param longSidePixels       long side of the minimum-area rectangle.
     * @param shortSidePixels      short side of the minimum-area rectangle.
     * @param horizontalExtentPx   width of the axis-aligned bounding box.
     * @param verticalExtentPx     height of the axis-aligned bounding box.
     */
    public record PixelMetrics(
            double longSidePixels,
            double shortSidePixels,
            double horizontalExtentPx,
            double verticalExtentPx) {

        /** All-zero metrics, for frames with no target. */
        public static final PixelMetrics ZERO = new PixelMetrics(0, 0, 0, 0);
    }

    /**
     * A frame with no detections, carrying the current cycle's latency and timestamp. A real
     * Limelight keeps publishing at its FPS even when it sees nothing, so this is what gets
     * published when the pipeline returns zero targets.
     *
     * @param pipelineLatencyMs       pipeline latency in milliseconds.
     * @param captureLatencyMs        capture latency in milliseconds.
     * @param captureTimestampSeconds capture timestamp in FPGA seconds.
     * @return an empty but temporally coherent frame.
     */
    public static LimelightFrame empty(
            double pipelineLatencyMs, double captureLatencyMs, double captureTimestampSeconds) {
        return new LimelightFrame(
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -1,
                pipelineLatencyMs,
                captureLatencyMs,
                captureTimestampSeconds,
                0,
                NO_FIDUCIALS,
                Optional.empty(),
                Optional.empty(),
                0.0,
                0.0,
                0.0,
                Optional.empty(),
                Optional.empty(),
                PixelMetrics.ZERO,
                NO_CORNERS);
    }

    /**
     * The frame's total latency, which is what has to be subtracted from the read instant to
     * date a vision measurement.
     *
     * @return {@code tl + cl} in milliseconds.
     */
    public double totalLatencyMs() {
        return pipelineLatencyMs + captureLatencyMs;
    }
}
