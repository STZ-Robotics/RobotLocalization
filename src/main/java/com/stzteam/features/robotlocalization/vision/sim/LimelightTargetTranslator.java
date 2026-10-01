// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Translates a {@code PhotonPipelineResult} into the equivalent {@link LimelightFrame}.
 *
 * <p>All the semantics live here: the sign changes between conventions, the computation of
 * the metadata Limelight appends to {@code botpose} (tag count, span, average distance and
 * area) and the two pose estimates:
 *
 * <ul>
 *   <li><b>MegaTag1</b> ({@code botpose*}): multi-tag PnP if two or more tags are visible,
 *       and otherwise the least ambiguous tag. It uses no external information.</li>
 *   <li><b>MegaTag2</b> ({@code botpose_orb*}): constrained PnP pinning the yaw the robot
 *       code wrote to {@code robot_orientation_set}. If the robot does not write that key,
 *       this estimate does not exist, exactly as with real hardware.</li>
 * </ul>
 */
public class LimelightTargetTranslator {

    private final AprilTagFieldLayout layout;
    private final PhotonPoseEstimator estimator;
    private final double headingScaleFactor;

    private Transform3d robotToCamera;
    private Pose3d lastEstimatedPose = Pose3d.kZero;
    private boolean constrainedSolverAvailable = true;

    /**
     * Creates a translator bound to a field layout and a camera position.
     *
     * @param layout             the AprilTag layout poses are estimated against.
     * @param robotToCamera      transform from the robot origin to the camera, in the WPILib
     *                           convention.
     * @param headingScaleFactor weight of the heading in the MegaTag2 solver.
     */
    public LimelightTargetTranslator(
            AprilTagFieldLayout layout, Transform3d robotToCamera, double headingScaleFactor) {
        this.layout = layout;
        this.robotToCamera = robotToCamera;
        this.headingScaleFactor = headingScaleFactor;
        this.estimator = new PhotonPoseEstimator(layout, robotToCamera);

        // MegaTag2's constrained PnP drops down to OpenCV. The 2-argument constructor does not
        // force the native library to load, so we ask for it here rather than discovering it is
        // missing halfway through a cycle.
        try {
            OpenCvLoader.forceStaticLoad();
        } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
            constrainedSolverAvailable = false;
        }
    }

    /**
     * Updates the camera's position relative to the robot. Called when the robot code writes
     * {@code camerapose_robotspace_set}.
     *
     * @param robotToCamera the new robot-&gt;camera transform.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        estimator.setRobotToCameraTransform(robotToCamera);
    }

    /** @return the robot-&gt;camera transform in effect. */
    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    /**
     * Translates a PhotonVision result into the equivalent Limelight frame.
     *
     * @param result       the result just read from the simulated camera.
     * @param inputs       what the robot code wrote to the table this cycle.
     * @param cameraMatrix camera intrinsics, needed for MegaTag2.
     * @param distCoeffs   distortion coefficients, needed for MegaTag2.
     * @param megaTag2     whether the MegaTag2 estimate should be attempted.
     * @return the translated frame.
     */
    public LimelightFrame translate(
            PhotonPipelineResult result,
            LimelightNTInputs inputs,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            boolean megaTag2) {

        double pipelineLatencyMs = result.metadata.getLatencyMillis();
        double captureTimestamp = result.getTimestampSeconds();

        List<PhotonTrackedTarget> targets = filterTargets(result, inputs);
        if (targets.isEmpty()) {
            return LimelightFrame.empty(pipelineLatencyMs, 0.0, captureTimestamp);
        }

        PhotonTrackedTarget best = targets.get(0);
        PhotonPipelineResult filtered = withTargets(result, targets, inputs);

        LimelightFrame.RawFiducial[] fiducials = buildFiducials(targets);
        Optional<Pose3d> megaTag1Pose = estimateMegaTag1(filtered);
        Optional<Pose3d> megaTag2Pose =
                megaTag2
                        ? estimateMegaTag2(filtered, inputs, cameraMatrix, distCoeffs, megaTag1Pose)
                        : Optional.empty();

        megaTag2Pose.or(() -> megaTag1Pose).ifPresent(pose -> lastEstimatedPose = pose);

        Transform3d cameraToTarget = best.getBestCameraToTarget();

        return new LimelightFrame(
                true,
                LimelightCoordinates.yawToTx(best.getYaw()),
                LimelightCoordinates.pitchToTy(best.getPitch()),
                LimelightCoordinates.yawToTx(best.getYaw()),
                LimelightCoordinates.pitchToTy(best.getPitch()),
                best.getArea(),
                best.getSkew(),
                best.getFiducialId(),
                pipelineLatencyMs,
                0.0,
                captureTimestamp,
                targets.size(),
                fiducials,
                megaTag1Pose,
                megaTag2Pose,
                tagSpanMeters(targets),
                averageTagDistance(targets),
                averageTagArea(targets),
                Optional.of(cameraToTarget),
                Optional.of(robotToCamera.plus(cameraToTarget)),
                pixelMetrics(best),
                flattenCorners(targets));
    }

    /**
     * Applies the id filter the robot code installed. With no filter, it returns the targets as
     * they came, preserving photonlib's order (best first).
     */
    private List<PhotonTrackedTarget> filterTargets(
            PhotonPipelineResult result, LimelightNTInputs inputs) {
        if (!inputs.hasFiducialFilter()) {
            return result.getTargets();
        }
        List<PhotonTrackedTarget> kept = new ArrayList<>(result.getTargets().size());
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (inputs.acceptsFiducial(target.getFiducialId())) {
                kept.add(target);
            }
        }
        return kept;
    }

    /**
     * Rebuilds the result with the filtered list. The original multi-tag result is only kept if
     * every tag it used survived the filter; otherwise it is dropped so the estimator does not use
     * a contaminated solution.
     */
    private PhotonPipelineResult withTargets(
            PhotonPipelineResult original,
            List<PhotonTrackedTarget> targets,
            LimelightNTInputs inputs) {
        if (!inputs.hasFiducialFilter()) {
            return original;
        }

        var multitag = original.getMultiTagResult();
        boolean multitagStillValid =
                multitag.isPresent()
                        && multitag.get().fiducialIDsUsed.stream()
                                .allMatch(id -> inputs.acceptsFiducial(id.intValue()));

        return new PhotonPipelineResult(
                original.metadata, targets, multitagStillValid ? multitag : Optional.empty());
    }

    /** MegaTag1: multi-tag if there is one, otherwise the least ambiguous tag. */
    private Optional<Pose3d> estimateMegaTag1(PhotonPipelineResult result) {
        Optional<Pose3d> multiTag =
                estimator.estimateCoprocMultiTagPose(result).map(e -> e.estimatedPose);
        if (multiTag.isPresent()) {
            return multiTag;
        }
        return estimator.estimateLowestAmbiguityPose(result).map(e -> e.estimatedPose);
    }

    /**
     * MegaTag2: constrained PnP with the robot's yaw pinned. It returns empty if the robot did not
     * publish its orientation, which is the same behaviour a real Limelight has when nobody calls
     * {@code SetRobotOrientation}.
     */
    private Optional<Pose3d> estimateMegaTag2(
            PhotonPipelineResult result,
            LimelightNTInputs inputs,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            Optional<Pose3d> seed) {

        if (!constrainedSolverAvailable || inputs.robotHeading().isEmpty()) {
            return Optional.empty();
        }
        if (cameraMatrix == null || distCoeffs == null) {
            return Optional.empty();
        }

        Rotation2d heading = inputs.robotHeading().get();
        estimator.addHeadingData(result.getTimestampSeconds(), heading);

        try {
            return estimator
                    .estimateConstrainedSolvepnpPose(
                            result,
                            cameraMatrix,
                            distCoeffs,
                            seed.orElse(lastEstimatedPose),
                            false,
                            headingScaleFactor)
                    .map(e -> e.estimatedPose);
        } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
            // The constrained solver lives in JNI. If the native library is unavailable (headless
            // tests, for example) MegaTag2 is switched off instead of blowing up every cycle.
            constrainedSolverAvailable = false;
            System.err.println(
                    "[SimulatedLimelight] Constrained SolvePnP unavailable; "
                            + "botpose_orb is disabled: "
                            + e.getMessage());
            return Optional.empty();
        }
    }

    private LimelightFrame.RawFiducial[] buildFiducials(List<PhotonTrackedTarget> targets) {
        LimelightFrame.RawFiducial[] out = new LimelightFrame.RawFiducial[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            double distToCamera = cameraToTarget.getTranslation().getNorm();
            double distToRobot = robotToCamera.plus(cameraToTarget).getTranslation().getNorm();

            out[i] =
                    new LimelightFrame.RawFiducial(
                            target.getFiducialId(),
                            LimelightCoordinates.yawToTx(target.getYaw()),
                            LimelightCoordinates.pitchToTy(target.getPitch()),
                            target.getArea(),
                            distToCamera,
                            distToRobot,
                            target.getPoseAmbiguity());
        }
        return out;
    }

    /** Maximum distance between the visible tags, which is what Limelight calls the "tag span". */
    private double tagSpanMeters(List<PhotonTrackedTarget> targets) {
        List<Translation3d> positions = new ArrayList<>(targets.size());
        for (PhotonTrackedTarget target : targets) {
            layout.getTagPose(target.getFiducialId())
                    .ifPresent(pose -> positions.add(pose.getTranslation()));
        }

        double span = 0.0;
        for (int i = 0; i < positions.size(); i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                span = Math.max(span, positions.get(i).getDistance(positions.get(j)));
            }
        }
        return span;
    }

    private double averageTagDistance(List<PhotonTrackedTarget> targets) {
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return sum / targets.size();
    }

    private double averageTagArea(List<PhotonTrackedTarget> targets) {
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getArea();
        }
        return sum / targets.size();
    }

    /**
     * Pixel metrics of the primary target, derived from the minimum-area rectangle photonlib
     * already computes.
     */
    private LimelightFrame.PixelMetrics pixelMetrics(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        if (corners == null || corners.size() < 4) {
            return LimelightFrame.PixelMetrics.ZERO;
        }

        double sideA = distance(corners.get(0), corners.get(1));
        double sideB = distance(corners.get(1), corners.get(2));

        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (TargetCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        return new LimelightFrame.PixelMetrics(
                Math.max(sideA, sideB), Math.min(sideA, sideB), maxX - minX, maxY - minY);
    }

    private static double distance(TargetCorner a, TargetCorner b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private static double[] flattenCorners(List<PhotonTrackedTarget> targets) {
        List<TargetCorner> all = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            List<TargetCorner> corners = target.getDetectedCorners();
            if (corners != null) {
                all.addAll(corners);
            }
        }

        double[] out = new double[all.size() * 2];
        for (int i = 0; i < all.size(); i++) {
            out[i * 2] = all.get(i).x;
            out[i * 2 + 1] = all.get(i).y;
        }
        return out;
    }
}
