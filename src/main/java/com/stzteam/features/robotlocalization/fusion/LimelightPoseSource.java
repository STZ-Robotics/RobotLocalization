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
 * Una camara Limelight como fuente de pose.
 *
 * <p>Lee NetworkTables directamente, sin {@code LimelightHelpers}. Esa clase es un archivo que cada
 * equipo copia en su repo, no un artefacto de Maven, asi que una libreria no puede depender de ella
 * --- y tampoco hace falta: el esquema lo conocemos entero porque el simulador de esta misma feature
 * lo escribe.
 *
 * <p>Como consecuencia util, esta fuente funciona igual contra un Limelight fisico que contra
 * {@code SimulatedLimelight}, sin ninguna rama de codigo para simulacion.
 *
 * <h2>MegaTag1 y MegaTag2</h2>
 * MegaTag1 resuelve la pose solo con lo que ve. MegaTag2 fija el rumbo con el giroscopio del robot y
 * resuelve el resto, lo que lo hace mucho mas estable girando, pero <b>exige</b> que alguien le
 * escriba el rumbo cada ciclo. Esta fuente lo hace sola si le das un
 * {@link LocalizationInputs#gyroHeading()}; es el paso que mas veces se olvida y el que hace que
 * MegaTag2 devuelva ceros sin explicar por que.
 *
 * <p>Con MegaTag2 el rumbo que sale es el que tu le metiste, asi que no aporta informacion nueva:
 * conviene darle una sigma de rumbo enorme para que el estimator lo ignore. Es lo que hace
 * {@link #megaTag2(String)} por defecto.
 */
public class LimelightPoseSource extends PoseSource {

    /** Que solucion de Limelight consumir. */
    public enum Mode {
        /** Pose resuelta solo con vision. Sirve para inicializar el rumbo. */
        MEGATAG_1,
        /** Pose con el rumbo fijado por el giroscopio. Mucho mas estable girando. */
        MEGATAG_2
    }

    /** Sigma de rumbo que le dice al estimator "ignora esta componente". */
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
     * Crea una fuente de MegaTag1 con la configuracion recomendada para AprilTags.
     *
     * @param name nombre de la tabla del Limelight, por ejemplo {@code "limelight-front"}.
     * @return la fuente.
     */
    public static LimelightPoseSource megaTag1(String name) {
        return new LimelightPoseSource(name, Mode.MEGATAG_1, PoseSourceConfig.forAprilTags())
                .withStdDevsStrategy(StdDevStrategies.quadraticContinuous(0.08, 0.16));
    }

    /**
     * Crea una fuente de MegaTag2 con la configuracion recomendada.
     *
     * <p>Sin filtro de velocidad angular --- MegaTag2 aguanta girar, que es justo su razon de ser ---
     * y con la sigma de rumbo saturada para que el estimator no se crea un rumbo que el mismo
     * proporciono.
     *
     * @param name nombre de la tabla del Limelight.
     * @return la fuente.
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
     * Crea una fuente sobre la instancia global de NetworkTables.
     *
     * @param name   nombre de la tabla del Limelight.
     * @param mode   que solucion consumir.
     * @param config umbrales de los filtros.
     */
    public LimelightPoseSource(String name, Mode mode, PoseSourceConfig config) {
        this(name, mode, config, NetworkTableInstance.getDefault());
    }

    /**
     * Crea una fuente sobre una instancia concreta de NetworkTables.
     *
     * @param name     nombre de la tabla del Limelight.
     * @param mode     que solucion consumir.
     * @param config   umbrales de los filtros.
     * @param instance instancia de NetworkTables.
     */
    public LimelightPoseSource(
            String name, Mode mode, PoseSourceConfig config, NetworkTableInstance instance) {
        super(name, VecBuilder.fill(0.5, 0.5, 1.0), config);
        this.mode = mode;
        this.table = instance.getTable(name);

        String key = mode == Mode.MEGATAG_2 ? "botpose_orb_wpiblue" : "botpose_wpiblue";
        // Una entry y no getEntry().getDoubleArray(): hace falta el timestamp atomico del valor para
        // fechar la captura, y eso solo lo da getAtomic().
        this.botposeEntry = table.getDoubleArrayTopic(key).getEntry(new double[0]);
        this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    }

    /**
     * Margen alrededor del campo dentro del cual una pose sigue considerandose valida.
     *
     * @param meters margen en metros.
     * @return esta misma fuente.
     */
    public LimelightPoseSource withFieldMargin(double meters) {
        this.fieldMarginMeters = meters;
        return this;
    }

    /**
     * Controla si la fuente le escribe el rumbo al Limelight. Apagalo solo si ya lo hace otra parte
     * de tu codigo; sin el, MegaTag2 no produce nada.
     *
     * @param enabled true para escribirlo.
     * @return esta misma fuente.
     */
    public LimelightPoseSource withOrientationPublishing(boolean enabled) {
        this.publishOrientation = enabled;
        return this;
    }

    /**
     * Pone una estrategia de desviaciones y devuelve el tipo concreto, para poder encadenar.
     *
     * @param strategy estrategia de {@link StdDevStrategies}.
     * @return esta misma fuente.
     */
    public LimelightPoseSource withStdDevsStrategy(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        setStdDevsStrategy(strategy);
        return this;
    }

    /** @return el modo de esta fuente. */
    public Mode getMode() {
        return mode;
    }

    /** @return la tabla de NetworkTables que se esta leyendo. */
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
                            ? "MegaTag2 sin datos: nadie le esta dando el rumbo del robot"
                            : "sin tags a la vista";
            return Optional.empty();
        }

        LimelightBotpose.Estimate estimate = parsed.get();

        // La ambiguedad solo significa algo con UN tag a la vista. Con dos o mas, la solucion sale
        // de un PnP multi-tag que esta geometricamente restringido, y la ambiguedad que Limelight
        // reporta por tag es la del solve individual de cada uno --- justo lo que el multi-tag
        // resuelve. Aplicar el filtro ahi descarta las mejores mediciones que hay: contra la
        // simulacion, tres tags a doce metros daban ambiguedades de 0.97 por tag y el gate se
        // comia el 100% de los frames.
        lastAmbiguity =
                estimate.tagCount() == 1 ? estimate.worstAmbiguity() : OptionalDouble.empty();

        detail =
                String.format(
                        "%d tag(s) a %.1f m%s",
                        estimate.tagCount(),
                        estimate.avgTagDistMeters(),
                        lastAmbiguity.isPresent()
                                ? String.format(", ambiguedad %.2f", lastAmbiguity.getAsDouble())
                                : " (multi-tag, sin filtro de ambiguedad)");

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
     * Le escribe al Limelight el rumbo del robot, que es lo que MegaTag2 necesita para resolver.
     *
     * <p>Solo se rellena el yaw. Los otros cinco valores existen en el protocolo pero el propio
     * Limelight los documenta como innecesarios.
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
