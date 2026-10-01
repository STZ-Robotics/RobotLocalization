// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import com.stzteam.features.robotlocalization.vision.LimelightBotpose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Conversions between the WPILib/PhotonVision conventions and the ones a real Limelight
 * publishes to NetworkTables.
 *
 * <p>This class exists so the signs and axis orderings live in one place, with tests,
 * instead of scattered through the translator. It is the easiest thing in the whole
 * feature to get wrong.
 *
 * <h2>Conventions used</h2>
 * <ul>
 *   <li><b>Field space</b> ({@code botpose*}): identical to WPILib. X+ along the field,
 *       Y+ to the left, Z+ up, angles in degrees. {@code botpose_wpiblue} uses the blue
 *       origin (the same one {@code AprilTagFieldLayout} uses by default),
 *       {@code botpose_wpired} the red origin, and plain {@code botpose} the legacy origin
 *       at the centre of the field.</li>
 *   <li><b>Camera / target space</b> ({@code targetpose_cameraspace} and family): X+ to the
 *       right, Y+ down, Z+ out of the camera or the tag. It is a right-handed system,
 *       related to WPILib's by a pure rotation.</li>
 * </ul>
 */
public final class LimelightCoordinates {

    /**
     * Change of basis from the WPILib convention (X forward, Y left, Z up) to Limelight's
     * camera/target convention (X right, Y down, Z forward).
     *
     * <p>The rows are Limelight's axes expressed in WPILib coordinates. The determinant is
     * +1, so this is a proper rotation and not a reflection.
     */
    private static final Matrix<N3, N3> WPILIB_TO_LIMELIGHT =
            MatBuilder.fill(Nat.N3(), Nat.N3(), 0, -1, 0, 0, 0, -1, 1, 0, 0);

    private LimelightCoordinates() {}

    /**
     * Converts PhotonVision's yaw (positive to the left) to Limelight's {@code tx}
     * (positive to the right).
     *
     * @param photonYawDegrees the yaw reported by {@code PhotonTrackedTarget#getYaw()}.
     * @return the equivalent {@code tx} value, in degrees.
     */
    public static double yawToTx(double photonYawDegrees) {
        return -photonYawDegrees;
    }

    /**
     * Converts PhotonVision's pitch to Limelight's {@code ty}. Both use positive upwards,
     * so this is the identity; it exists as a method so the translator documents the
     * conversion symmetrically to {@link #yawToTx(double)}.
     *
     * @param photonPitchDegrees the pitch reported by {@code PhotonTrackedTarget#getPitch()}.
     * @return the equivalent {@code ty} value, in degrees.
     */
    public static double pitchToTy(double photonPitchDegrees) {
        return photonPitchDegrees;
    }

    /**
     * Re-expresses a blue-origin field pose in the red origin, which is what a Limelight
     * publishes to {@code botpose_wpired}.
     *
     * @param bluePose the robot pose in the blue origin.
     * @param layout   the field layout, from which the length and width are taken.
     * @return the same physical pose, seen from the red origin.
     */
    public static Pose3d blueToRed(Pose3d bluePose, AprilTagFieldLayout layout) {
        Pose3d redOrigin =
                new Pose3d(
                        layout.getFieldLength(),
                        layout.getFieldWidth(),
                        0.0,
                        new Rotation3d(0.0, 0.0, Math.PI));
        return bluePose.relativeTo(redOrigin);
    }

    /**
     * Re-expresses a blue-origin field pose in the legacy field-centre origin, which is
     * what a Limelight publishes to the plain {@code botpose} key.
     *
     * @param bluePose the robot pose in the blue origin.
     * @param layout   the field layout, from which the length and width are taken.
     * @return the same physical pose, with the origin moved to the centre of the field.
     */
    public static Pose3d blueToFieldCenter(Pose3d bluePose, AprilTagFieldLayout layout) {
        return new Pose3d(
                bluePose.getX() - layout.getFieldLength() / 2.0,
                bluePose.getY() - layout.getFieldWidth() / 2.0,
                bluePose.getZ(),
                bluePose.getRotation());
    }

    /**
     * Flattens a pose into the 6-double format Limelight uses:
     * {@code [x, y, z, roll, pitch, yaw]}, in metres and degrees.
     *
     * @param pose the pose to flatten.
     * @return a 6-element array.
     */
    public static double[] poseToArray(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            Units.radiansToDegrees(pose.getRotation().getX()),
            Units.radiansToDegrees(pose.getRotation().getY()),
            Units.radiansToDegrees(pose.getRotation().getZ())
        };
    }

    /**
     * Converts a transform expressed in the WPILib convention (X forward, Y left, Z up) to
     * the 6 doubles Limelight publishes for camera space and target space (X right, Y down,
     * Z forward).
     *
     * @param transform the transform in the WPILib convention.
     * @return {@code [x, y, z, roll, pitch, yaw]} in metres and degrees, in Limelight's
     *         convention.
     */
    public static double[] toLimelightCameraSpace(Transform3d transform) {
        Translation3d t = transform.getTranslation();
        Matrix<N3, N3> rotated =
                WPILIB_TO_LIMELIGHT
                        .times(transform.getRotation().toMatrix())
                        .times(WPILIB_TO_LIMELIGHT.transpose());
        Rotation3d rotation = new Rotation3d(rotated);

        return new double[] {
            -t.getY(),
            -t.getZ(),
            t.getX(),
            Units.radiansToDegrees(rotation.getX()),
            Units.radiansToDegrees(rotation.getY()),
            Units.radiansToDegrees(rotation.getZ())
        };
    }

    /**
     * Builds the complete {@code botpose} array {@code LimelightHelpers} expects: 6 pose
     * values, 5 metadata values and 7 for each fiducial used.
     *
     * <p>{@code LimelightHelpers.getBotPoseEstimate} only fills in {@code rawFiducials} if
     * the length is exactly {@code 11 + 7 * tagCount}, so the number of fiducials passed
     * here has to match the published {@code tagCount}.
     *
     * @param pose      the robot pose, already in whichever origin the key calls for.
     * @param frame     the frame the latency, span, distance and average area are taken from.
     * @param fiducials the fiducials to append; their length defines the published
     *                  {@code tagCount}.
     * @return the array ready for {@code setDoubleArray}.
     */
    public static double[] botposeArray(
            Pose3d pose, LimelightFrame frame, LimelightFrame.RawFiducial[] fiducials) {
        double[] out =
                new double
                        [LimelightBotpose.HEADER_LENGTH
                                + LimelightBotpose.VALUES_PER_FIDUCIAL * fiducials.length];
        double[] posePart = poseToArray(pose);
        System.arraycopy(posePart, 0, out, 0, 6);

        out[6] = frame.totalLatencyMs();
        out[7] = fiducials.length;
        out[8] = frame.tagSpanMeters();
        out[9] = frame.avgTagDistMeters();
        out[10] = frame.avgTagAreaPercent();

        for (int i = 0; i < fiducials.length; i++) {
            LimelightFrame.RawFiducial f = fiducials[i];
            int base = LimelightBotpose.HEADER_LENGTH + i * LimelightBotpose.VALUES_PER_FIDUCIAL;
            out[base] = f.id();
            out[base + 1] = f.txnc();
            out[base + 2] = f.tync();
            out[base + 3] = f.ta();
            out[base + 4] = f.distToCamera();
            out[base + 5] = f.distToRobot();
            out[base + 6] = f.ambiguity();
        }
        return out;
    }

    /**
     * Flattens the fiducials into the format of the {@code rawfiducials} key: 7 doubles per
     * tag, with no header.
     *
     * @param fiducials the visible fiducials.
     * @return the flattened array.
     */
    public static double[] rawFiducialsArray(LimelightFrame.RawFiducial[] fiducials) {
        double[] out = new double[fiducials.length * 7];
        for (int i = 0; i < fiducials.length; i++) {
            LimelightFrame.RawFiducial f = fiducials[i];
            int base = i * 7;
            out[base] = f.id();
            out[base + 1] = f.txnc();
            out[base + 2] = f.tync();
            out[base + 3] = f.ta();
            out[base + 4] = f.distToCamera();
            out[base + 5] = f.distToRobot();
            out[base + 6] = f.ambiguity();
        }
        return out;
    }
}
