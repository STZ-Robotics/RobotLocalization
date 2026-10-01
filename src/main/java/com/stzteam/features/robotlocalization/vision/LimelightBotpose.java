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
 * El formato del array {@code botpose} de un Limelight, en un solo sitio.
 *
 * <p>Lo escribe el simulador y lo lee la fusion, asi que definirlo dos veces seria pedir que un dia
 * dejen de coincidir sin que nada avise. Un test hace el viaje de ida y vuelta entre las dos mitades
 * de la feature para que eso no pueda pasar en silencio.
 *
 * <h2>El formato</h2>
 * <pre>
 * [0..5]   x, y, z, roll, pitch, yaw      metros y grados
 * [6]      latencia total                 milisegundos
 * [7]      numero de tags
 * [8]      separacion entre tags          metros
 * [9]      distancia promedio camara-tag  metros
 * [10]     area promedio                  porcentaje de la imagen
 * [11+]    7 valores por tag              id, txnc, tync, ta, distCamara, distRobot, ambiguedad
 * </pre>
 *
 * <p>El bloque de tags solo se rellena si el largo es <b>exactamente</b>
 * {@code 11 + 7 * numeroDeTags}. Es la misma condicion que impone {@code LimelightHelpers}, y el
 * motivo por el que el numero de tags que se publica tiene que cuadrar con los que se anexan.
 */
public final class LimelightBotpose {

    /** Valores de cabecera antes del primer tag. */
    public static final int HEADER_LENGTH = 11;

    /** Valores que ocupa cada tag. */
    public static final int VALUES_PER_FIDUCIAL = 7;

    private LimelightBotpose() {}

    /**
     * Un tag visible, con los siete valores que publica Limelight.
     *
     * @param id           id del fiducial.
     * @param txnc         offset horizontal respecto al punto principal, en grados.
     * @param tync         offset vertical respecto al punto principal, en grados.
     * @param ta           area del tag, en porcentaje de la imagen.
     * @param distToCamera distancia camara-tag, en metros.
     * @param distToRobot  distancia origen del robot-tag, en metros.
     * @param ambiguity    ambiguedad de la solucion de PnP, de 0 a 1.
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
     * Una estimacion de pose completa, ya interpretada.
     *
     * @param pose              pose del robot en el campo.
     * @param timestampSeconds  instante de la captura, en la base de tiempo del FPGA.
     * @param latencyMs         latencia total del frame.
     * @param tagCount          numero de tags usados.
     * @param tagSpanMeters     separacion entre los tags usados.
     * @param avgTagDistMeters  distancia promedio a los tags.
     * @param avgTagAreaPercent area promedio de los tags.
     * @param fiducials         detalle por tag, vacio si el array venia sin el bloque.
     * @param megaTag2          si viene de una key {@code botpose_orb}.
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
         * Ambiguedad del tag mas ambiguo.
         *
         * <p>Se toma el peor y no el promedio porque un tag mal resuelto es el que manda el robot al
         * otro lado del campo, y promediarlo con otros buenos lo esconde.
         *
         * <p><b>Ojo con cuando usarla.</b> Solo tiene sentido si {@link #tagCount()} es 1. Con dos o
         * mas tags la pose sale de un PnP multi-tag geometricamente restringido, mientras que estos
         * numeros son la ambiguedad del solve individual de cada tag --- precisamente lo que el
         * multi-tag resuelve. Filtrar por ellos descarta las mejores mediciones disponibles.
         *
         * @return la ambiguedad maxima, o vacio si no hay detalle de tags.
         */
        public OptionalDouble worstAmbiguity() {
            return fiducials.stream().mapToDouble(Fiducial::ambiguity).max();
        }

        /** @return true si hay una pose utilizable, es decir al menos un tag. */
        public boolean hasTargets() {
            return tagCount > 0;
        }
    }

    /**
     * Interpreta un array de {@code botpose}.
     *
     * <p>El timestamp de la captura sale del reloj de NetworkTables menos la latencia. Conviene
     * saber que es el reloj <b>local</b>, no el {@code serverTime}: por eso una fuente de Limelight
     * se fecha bien aunque el publicador viva en el mismo proceso, cosa que no pasa con QuestNav.
     *
     * @param raw                el array tal cual sale de NetworkTables.
     * @param ntTimestampMicros  instante en que NetworkTables recibio el valor, en microsegundos.
     * @param megaTag2           si viene de una key {@code botpose_orb}.
     * @return la estimacion, o vacio si el array esta vacio o es demasiado corto.
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
