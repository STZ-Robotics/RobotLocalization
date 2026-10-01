// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * El marco de origen del headset, y lo que le pasa cuando llega un POSE_RESET.
 *
 * <p>Este es el punto de esta feature que es facil de equivocar, el equivalente al signo de
 * {@code tx} en la Limelight simulada. Vive en su propia clase, con tests puros, por esa razon.
 *
 * <h2>El malentendido que evita</h2>
 * Un Quest <b>no publica la pose del robot en el campo</b>. Publica la pose del headset en un
 * origen propio y arbitrario, el sitio donde estaba cuando arranco la app. Que ese origen coincida
 * con el origen del campo es algo que el robot code tiene que <i>provocar</i>, llamando a
 * {@code setPose()}. Si la simulacion publicara directamente la pose de campo, estaria escondiendo
 * justo el paso que mas equipos se saltan.
 *
 * <h2>La cuenta</h2>
 * Si el origen del Quest esta en la pose {@code O} del campo y el headset esta fisicamente en
 * {@code H}, lo que se publica es {@code H} visto desde {@code O}:
 *
 * <pre>{@code publicado = H.relativeTo(O)}</pre>
 *
 * Un POSE_RESET a {@code T} pide que, sin mover el headset, lo publicado pase a valer {@code T}.
 * Despejando {@code O} de la ecuacion de arriba:
 *
 * <pre>{@code O = H ∘ T⁻¹}</pre>
 */
public class QuestOriginFrame {

    private Pose3d origin;

    /** Crea un marco cuyo origen coincide con el del campo. */
    public QuestOriginFrame() {
        this(Pose3d.kZero);
    }

    /**
     * Crea un marco con un origen concreto.
     *
     * @param origin pose del origen del Quest, en coordenadas de campo.
     */
    public QuestOriginFrame(Pose3d origin) {
        this.origin = origin;
    }

    /**
     * Crea un marco con un origen desplazado al azar, para forzar que el robot code haga bien su
     * seeding. Con el origen en identidad, un equipo que nunca llama a {@code setPose()} ve poses
     * de campo correctas por accidente y el bug no aparece hasta la competencia.
     *
     * @param translationMeters desplazamiento maximo en metros.
     * @param rotationDegrees   desviacion maxima de yaw en grados.
     * @param random            fuente de aleatoriedad, para que el test sea reproducible.
     * @return un marco con origen arbitrario.
     */
    public static QuestOriginFrame randomized(
            double translationMeters, double rotationDegrees, java.util.Random random) {
        double x = (random.nextDouble() * 2 - 1) * translationMeters;
        double y = (random.nextDouble() * 2 - 1) * translationMeters;
        double yaw = Math.toRadians((random.nextDouble() * 2 - 1) * rotationDegrees);
        return new QuestOriginFrame(
                new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d(0.0, 0.0, yaw)));
    }

    /**
     * Convierte una pose de campo del headset a lo que el Quest publicaria.
     *
     * @param headsetFieldPose pose real del headset en el campo.
     * @return la pose en el marco del Quest.
     */
    public Pose3d toQuestFrame(Pose3d headsetFieldPose) {
        return headsetFieldPose.relativeTo(origin);
    }

    /**
     * Reancla el origen para que un headset que esta fisicamente en {@code headsetFieldPose} pase a
     * reportar {@code targetPose}. Es lo que hace un POSE_RESET real.
     *
     * @param headsetFieldPose pose real del headset en el campo.
     * @param targetPose       pose que el robot code quiere que el Quest reporte.
     */
    public void reset(Pose3d headsetFieldPose, Pose3d targetPose) {
        origin =
                headsetFieldPose.transformBy(
                        new Transform3d(targetPose.getTranslation(), targetPose.getRotation()).inverse());
    }

    /** @return la pose del origen del Quest en coordenadas de campo. */
    public Pose3d origin() {
        return origin;
    }
}
