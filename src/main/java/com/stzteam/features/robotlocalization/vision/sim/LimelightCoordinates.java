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
 * Conversiones entre las convenciones de WPILib/PhotonVision y las que publica un
 * Limelight real en NetworkTables.
 *
 * <p>Esta clase existe para que los signos y los ordenes de ejes vivan en un solo
 * sitio, con tests, en vez de repartidos por el traductor. Es el punto mas facil de
 * equivocar de toda la feature.
 *
 * <h2>Convenciones usadas</h2>
 * <ul>
 *   <li><b>Field space</b> ({@code botpose*}): identica a WPILib. X+ a lo largo del campo,
 *       Y+ a la izquierda, Z+ arriba, angulos en grados. {@code botpose_wpiblue} usa el
 *       origen azul (el mismo que {@code AprilTagFieldLayout} por defecto),
 *       {@code botpose_wpired} el origen rojo, y {@code botpose} el origen legacy en el
 *       centro del campo.</li>
 *   <li><b>Camera / target space</b> ({@code targetpose_cameraspace} y familia): X+ a la
 *       derecha, Y+ abajo, Z+ saliendo de la camara o del tag. Es un sistema derecho,
 *       relacionado con el de WPILib por una rotacion pura.</li>
 * </ul>
 */
public final class LimelightCoordinates {

    /**
     * Cambio de base de la convencion WPILib (X adelante, Y izquierda, Z arriba) a la
     * convencion de camara/target de Limelight (X derecha, Y abajo, Z adelante).
     *
     * <p>Las filas son los ejes de Limelight expresados en coordenadas WPILib. El
     * determinante es +1, asi que es una rotacion propia y no una reflexion.
     */
    private static final Matrix<N3, N3> WPILIB_TO_LIMELIGHT =
            MatBuilder.fill(Nat.N3(), Nat.N3(), 0, -1, 0, 0, 0, -1, 1, 0, 0);

    private LimelightCoordinates() {}

    /**
     * Convierte el yaw de PhotonVision (positivo a la izquierda) al {@code tx} de
     * Limelight (positivo a la derecha).
     *
     * @param photonYawDegrees yaw reportado por {@code PhotonTrackedTarget#getYaw()}.
     * @return el valor equivalente de {@code tx}, en grados.
     */
    public static double yawToTx(double photonYawDegrees) {
        return -photonYawDegrees;
    }

    /**
     * Convierte el pitch de PhotonVision al {@code ty} de Limelight. Ambos usan
     * positivo hacia arriba, asi que es identidad; existe como metodo para que el
     * traductor documente la conversion de forma simetrica a {@link #yawToTx(double)}.
     *
     * @param photonPitchDegrees pitch reportado por {@code PhotonTrackedTarget#getPitch()}.
     * @return el valor equivalente de {@code ty}, en grados.
     */
    public static double pitchToTy(double photonPitchDegrees) {
        return photonPitchDegrees;
    }

    /**
     * Reexpresa una pose de campo con origen azul en el origen rojo, que es lo que
     * un Limelight publica en {@code botpose_wpired}.
     *
     * @param bluePose pose del robot con origen azul.
     * @param layout   layout del campo, del que se toman largo y ancho.
     * @return la misma pose fisica, vista desde el origen rojo.
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
     * Reexpresa una pose de campo con origen azul en el origen legacy del centro del
     * campo, que es lo que un Limelight publica en la key {@code botpose} a secas.
     *
     * @param bluePose pose del robot con origen azul.
     * @param layout   layout del campo, del que se toman largo y ancho.
     * @return la misma pose fisica, con el origen trasladado al centro del campo.
     */
    public static Pose3d blueToFieldCenter(Pose3d bluePose, AprilTagFieldLayout layout) {
        return new Pose3d(
                bluePose.getX() - layout.getFieldLength() / 2.0,
                bluePose.getY() - layout.getFieldWidth() / 2.0,
                bluePose.getZ(),
                bluePose.getRotation());
    }

    /**
     * Aplana una pose al formato de 6 doubles que usa Limelight:
     * {@code [x, y, z, roll, pitch, yaw]}, con metros y grados.
     *
     * @param pose pose a aplanar.
     * @return un array de 6 elementos.
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
     * Convierte una transformada expresada en convencion WPILib (X adelante, Y izquierda,
     * Z arriba) a los 6 doubles que Limelight publica para camera space y target space
     * (X derecha, Y abajo, Z adelante).
     *
     * @param transform transformada en convencion WPILib.
     * @return {@code [x, y, z, roll, pitch, yaw]} en metros y grados, en convencion Limelight.
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
     * Construye el array de {@code botpose} completo que espera {@code LimelightHelpers}:
     * 6 valores de pose, 5 de metadatos y 7 por cada fiducial usado.
     *
     * <p>{@code LimelightHelpers.getBotPoseEstimate} solo rellena {@code rawFiducials} si
     * la longitud es exactamente {@code 11 + 7 * tagCount}, asi que el numero de
     * fiduciales pasados aqui tiene que coincidir con el {@code tagCount} publicado.
     *
     * @param pose      pose del robot, ya en el origen que corresponda a la key.
     * @param frame     frame del que se toman latencia, span, distancia y area promedio.
     * @param fiducials fiduciales a anexar; su longitud define el {@code tagCount} publicado.
     * @return el array listo para {@code setDoubleArray}.
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
     * Aplana los fiduciales al formato de la key {@code rawfiducials}: 7 doubles por tag,
     * sin cabecera.
     *
     * @param fiducials fiduciales visibles.
     * @return el array aplanado.
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
