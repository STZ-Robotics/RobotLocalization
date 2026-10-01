// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Optional;

/**
 * Snapshot inmutable de un ciclo de la camara simulada, ya traducido a la
 * semantica de un Limelight real (grados, porcentajes, convenciones de signo).
 *
 * <p>Esta es la moneda de cambio interna de la feature: el
 * {@link LimelightTargetTranslator} la produce a partir de un
 * {@code PhotonPipelineResult}, y tanto {@link LimelightNTPublisher} como
 * {@link LimelightSimDevice} la consumen. Manteniendo un unico frame por ciclo
 * se garantiza que NetworkTables y el SimDevice nunca queden desincronizados.
 *
 * @param hasTarget              equivalente a {@code tv != 0}.
 * @param tx                     offset horizontal al crosshair en grados, positivo a la derecha
 *                               (convencion Limelight; PhotonVision usa positivo a la izquierda).
 * @param ty                     offset vertical al crosshair en grados, positivo hacia arriba.
 * @param txnc                   offset horizontal respecto al punto principal, en grados.
 * @param tync                   offset vertical respecto al punto principal, en grados.
 * @param ta                     area del target como porcentaje de la imagen (0-100).
 * @param ts                     skew/rotacion del target en grados.
 * @param tid                    id del AprilTag primario, o -1 si no hay target.
 * @param pipelineLatencyMs      latencia atribuida al pipeline (key {@code tl}).
 * @param captureLatencyMs       latencia atribuida a la captura (key {@code cl}).
 * @param captureTimestampSeconds instante de captura del frame, en la base de tiempo del FPGA.
 * @param targetCount            numero de fiduciales visibles tras aplicar filtros de id.
 * @param fiducials              detalle por tag visible, en el orden que publica {@code rawfiducials}.
 * @param megaTag1Pose           pose del robot estimada sin heading externo (equivalente a MegaTag1),
 *                               en campo con origen azul.
 * @param megaTag2Pose           pose del robot estimada fijando el heading que el robot code escribio
 *                               en {@code robot_orientation_set} (equivalente a MegaTag2).
 * @param tagSpanMeters          distancia maxima entre los tags usados para la estimacion.
 * @param avgTagDistMeters       distancia promedio camara-tag de los tags usados.
 * @param avgTagAreaPercent      area promedio de los tags usados, como porcentaje de la imagen.
 * @param cameraToTarget         transformada camara -&gt; tag primario, en convencion WPILib.
 * @param robotToTarget          transformada robot -&gt; tag primario, en convencion WPILib.
 * @param targetPixelMetrics     metricas en pixeles del target primario, para la key {@code t2d}.
 * @param cornersXY              esquinas detectadas en pixeles, aplanadas como {@code [x0,y0,x1,y1,...]}.
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

    /** Array vacio reutilizable para frames sin detecciones. */
    private static final RawFiducial[] NO_FIDUCIALS = new RawFiducial[0];

    /** Array vacio reutilizable para frames sin esquinas. */
    private static final double[] NO_CORNERS = new double[0];

    /**
     * Una deteccion individual de AprilTag, con el mismo contenido y orden que los
     * 7 valores por tag que un Limelight real publica en {@code rawfiducials}.
     *
     * @param id            id del fiducial.
     * @param txnc          offset horizontal respecto al punto principal, en grados.
     * @param tync          offset vertical respecto al punto principal, en grados.
     * @param ta            area del tag como porcentaje de la imagen.
     * @param distToCamera  distancia camara-tag en metros.
     * @param distToRobot   distancia origen del robot-tag en metros.
     * @param ambiguity     ambiguedad de la solucion PnP (0 = sin ambiguedad).
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
     * Metricas en pixeles del target primario. Alimentan los indices 12-15 de la key
     * {@code t2d}; un Limelight real las expone para logica de vision 2D fina.
     *
     * @param longSidePixels       lado largo del rectangulo de area minima.
     * @param shortSidePixels      lado corto del rectangulo de area minima.
     * @param horizontalExtentPx   ancho del bounding box alineado a los ejes.
     * @param verticalExtentPx     alto del bounding box alineado a los ejes.
     */
    public record PixelMetrics(
            double longSidePixels,
            double shortSidePixels,
            double horizontalExtentPx,
            double verticalExtentPx) {

        /** Metricas en cero, para frames sin target. */
        public static final PixelMetrics ZERO = new PixelMetrics(0, 0, 0, 0);
    }

    /**
     * Frame sin detecciones, con la latencia y el timestamp del ciclo actual.
     * Un Limelight real sigue publicando a su FPS aunque no vea nada, asi que esto
     * es lo que se publica cuando el pipeline devuelve cero targets.
     *
     * @param pipelineLatencyMs       latencia del pipeline en milisegundos.
     * @param captureLatencyMs        latencia de captura en milisegundos.
     * @param captureTimestampSeconds timestamp de captura en segundos del FPGA.
     * @return un frame vacio pero temporalmente coherente.
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
     * Latencia total del frame, que es lo que hay que restarle al instante de lectura
     * para fechar una medicion de vision.
     *
     * @return {@code tl + cl} en milisegundos.
     */
    public double totalLatencyMs() {
        return pipelineLatencyMs + captureLatencyMs;
    }
}
