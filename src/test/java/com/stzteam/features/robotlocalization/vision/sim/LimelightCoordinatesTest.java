// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Optional;

/**
 * Fija las convenciones de la traduccion. Si alguien invierte un signo aqui, el robot code
 * escrito contra un Limelight real empieza a girar al reves en simulacion, asi que estos
 * tests son la red de seguridad de toda la feature.
 */
class LimelightCoordinatesTest {

    private static final double EPS = 1e-9;
    private static final double FIELD_LENGTH = 17.5;
    private static final double FIELD_WIDTH = 8.0;

    private static AprilTagFieldLayout layout() {
        return new AprilTagFieldLayout(
                List.of(new AprilTag(1, Pose3d.kZero)), FIELD_LENGTH, FIELD_WIDTH);
    }

    @Test
    void photonYawIsPositiveLeftAndLimelightTxIsPositiveRight() {
        assertEquals(-15.0, LimelightCoordinates.yawToTx(15.0), EPS);
        assertEquals(15.0, LimelightCoordinates.yawToTx(-15.0), EPS);
    }

    @Test
    void bothConventionsAgreeOnPitchBeingPositiveUp() {
        assertEquals(7.5, LimelightCoordinates.pitchToTy(7.5), EPS);
    }

    @Test
    void redOriginMirrorsTheFieldAndFlipsYaw() {
        Pose3d blue = new Pose3d(1.0, 2.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
        Pose3d red = LimelightCoordinates.blueToRed(blue, layout());

        assertEquals(FIELD_LENGTH - 1.0, red.getX(), 1e-9);
        assertEquals(FIELD_WIDTH - 2.0, red.getY(), 1e-9);
        assertEquals(180.0, Math.abs(Units.radiansToDegrees(red.getRotation().getZ())), 1e-9);
    }

    @Test
    void legacyBotposeOriginSitsAtTheFieldCenter() {
        Pose3d blue = new Pose3d(1.0, 2.0, 0.3, new Rotation3d(0.0, 0.0, Math.PI / 4));
        Pose3d centered = LimelightCoordinates.blueToFieldCenter(blue, layout());

        assertEquals(1.0 - FIELD_LENGTH / 2.0, centered.getX(), EPS);
        assertEquals(2.0 - FIELD_WIDTH / 2.0, centered.getY(), EPS);
        assertEquals(0.3, centered.getZ(), EPS);
        assertEquals(
                Units.radiansToDegrees(blue.getRotation().getZ()),
                Units.radiansToDegrees(centered.getRotation().getZ()),
                EPS);
    }

    @Test
    void poseArrayUsesMetersAndDegreesInLimelightOrder() {
        Pose3d pose =
                new Pose3d(
                        1.0,
                        2.0,
                        3.0,
                        new Rotation3d(
                                Units.degreesToRadians(10.0),
                                Units.degreesToRadians(20.0),
                                Units.degreesToRadians(30.0)));

        double[] array = LimelightCoordinates.poseToArray(pose);

        assertEquals(6, array.length);
        assertEquals(1.0, array[0], EPS);
        assertEquals(2.0, array[1], EPS);
        assertEquals(3.0, array[2], EPS);
        assertEquals(10.0, array[3], 1e-6);
        assertEquals(20.0, array[4], 1e-6);
        assertEquals(30.0, array[5], 1e-6);
    }

    @Test
    void cameraSpaceSwapsAxesToRightDownForward() {
        // 2 m adelante, 1 m a la izquierda y 0.5 m arriba, en convencion WPILib.
        Transform3d wpilib =
                new Transform3d(new Translation3d(2.0, 1.0, 0.5), Rotation3d.kZero);

        double[] limelight = LimelightCoordinates.toLimelightCameraSpace(wpilib);

        assertEquals(-1.0, limelight[0], EPS); // 1 m a la izquierda = -1 en el eje "derecha"
        assertEquals(-0.5, limelight[1], EPS); // 0.5 m arriba = -0.5 en el eje "abajo"
        assertEquals(2.0, limelight[2], EPS); // adelante se convierte en el eje "forward"
    }

    @Test
    void cameraSpaceTurnsWpilibYawIntoLimelightPitch() {
        // Girar +30 grados sobre el eje Z (arriba) de WPILib equivale a girar -30 grados
        // sobre el eje Y (abajo) de Limelight.
        Transform3d wpilib =
                new Transform3d(
                        Translation3d.kZero, new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)));

        double[] limelight = LimelightCoordinates.toLimelightCameraSpace(wpilib);

        assertEquals(0.0, limelight[3], 1e-6);
        assertEquals(-30.0, limelight[4], 1e-6);
        assertEquals(0.0, limelight[5], 1e-6);
    }

    @Test
    void botposeArrayMatchesTheLayoutLimelightHelpersParses() {
        LimelightFrame.RawFiducial[] fiducials = {
            new LimelightFrame.RawFiducial(7, 1.0, 2.0, 3.0, 4.0, 5.0, 0.06),
            new LimelightFrame.RawFiducial(8, 1.1, 2.1, 3.1, 4.1, 5.1, 0.07)
        };
        LimelightFrame frame = frameWith(fiducials, 12.5, 1.25, 3.5, 0.9);

        double[] array =
                LimelightCoordinates.botposeArray(new Pose3d(1.0, 2.0, 0.0, Rotation3d.kZero), frame, fiducials);

        // LimelightHelpers.getBotPoseEstimate solo rellena rawFiducials si el largo es exacto.
        assertEquals(11 + 7 * fiducials.length, array.length);
        assertEquals(12.5, array[6], EPS); // latencia total en ms
        assertEquals(2.0, array[7], EPS); // tag count
        assertEquals(1.25, array[8], EPS); // tag span
        assertEquals(3.5, array[9], EPS); // distancia promedio
        assertEquals(0.9, array[10], EPS); // area promedio

        assertEquals(7.0, array[11], EPS);
        assertEquals(0.06, array[17], EPS);
        assertEquals(8.0, array[18], EPS);
        assertEquals(0.07, array[24], EPS);
    }

    @Test
    void rawFiducialsArrayPacksSevenValuesPerTag() {
        LimelightFrame.RawFiducial[] fiducials = {
            new LimelightFrame.RawFiducial(3, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        };

        double[] array = LimelightCoordinates.rawFiducialsArray(fiducials);

        assertEquals(7, array.length);
        assertEquals(3.0, array[0], EPS);
        assertEquals(0.6, array[6], EPS);
    }

    private static LimelightFrame frameWith(
            LimelightFrame.RawFiducial[] fiducials,
            double totalLatencyMs,
            double tagSpan,
            double avgDist,
            double avgArea) {
        return new LimelightFrame(
                true,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                fiducials[0].id(),
                totalLatencyMs,
                0.0,
                0.0,
                fiducials.length,
                fiducials,
                Optional.empty(),
                Optional.empty(),
                tagSpan,
                avgDist,
                avgArea,
                Optional.empty(),
                Optional.empty(),
                LimelightFrame.PixelMetrics.ZERO,
                new double[0]);
    }
}
