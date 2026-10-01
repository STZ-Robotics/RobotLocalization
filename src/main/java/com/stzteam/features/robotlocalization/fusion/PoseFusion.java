// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import com.stzteam.forgemini.io.NetworkIO;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * El portero que decide que mediciones de vision llegan al pose estimator.
 *
 * <p>No es un estimador ni un filtro de Kalman: esos ya los trae WPILib. Lo que falta, y lo que hace
 * esta clase, es todo lo de antes --- registrar varias fuentes, descartar las que mienten, y
 * entregar solo lo que queda.
 *
 * <h2>Uso</h2>
 * <pre>{@code
 * private final PoseFusion fusion = new PoseFusion(
 *         poseEstimator::addVisionMeasurement,
 *         LocalizationInputs.of(this::getPose, this::getChassisSpeeds))
 *     .registerSource(new LimelightPoseSource("limelight-front", ROBOT_TO_CAMERA))
 *     .registerSource(new QuestNavPoseSource("quest", ROBOT_TO_QUEST));
 *
 * @Override
 * public void periodic() {
 *     fusion.update();
 * }
 * }</pre>
 *
 * <h2>El orden importa</h2>
 * Los filtros van de barato a caro y de local a global. Primero lo que se decide sin leer hardware
 * --- esta encendida, el robot no esta girando como una peonza ---, luego la medicion en si, y al
 * final lo que necesita ver a todas las fuentes juntas: consenso y Mahalanobis. Una fuente
 * descartada por entorno ni siquiera gasta la lectura.
 *
 * <p>Cada fuente publica en NetworkTables por que se descarto su ultima medicion. Ver
 * {@link RejectionReason}: sin eso, un filtro mal calibrado y una camara rota se ven igual.
 */
public class PoseFusion {

    /**
     * Donde acaban las mediciones que sobreviven. La firma es la de
     * {@code SwerveDrivePoseEstimator.addVisionMeasurement}, asi que se pasa como referencia a
     * metodo y sin envolver el timestamp en un {@code Double}.
     */
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        /**
         * Entrega una medicion al estimator.
         *
         * @param pose             pose medida.
         * @param timestampSeconds instante de la captura.
         * @param stdDevs          confianza en la medicion.
         */
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    /** Cada cuanto se repite un aviso de desacuerdo, en segundos. */
    private static final double WARNING_PERIOD_SECONDS = 2.0;

    private static AprilTagFieldLayout defaultFieldLayout = FieldLayouts.defaultField();

    private final VisionMeasurementConsumer consumer;
    private final LocalizationInputs inputs;
    private final Map<String, PoseSource> sources = new LinkedHashMap<>();
    private final ConsensusGate consensusGate = new ConsensusGate();

    private AprilTagFieldLayout fieldLayout = defaultFieldLayout;
    private String telemetryTable = "Localization";
    private boolean consensusEnabled = true;
    private double lastWarningSeconds = Double.NEGATIVE_INFINITY;
    private int acceptedThisCycle = 0;

    /**
     * Crea la fusion.
     *
     * @param consumer donde entregar las mediciones aceptadas, normalmente
     *                 {@code poseEstimator::addVisionMeasurement}.
     * @param inputs   estado del robot que comparten las fuentes.
     */
    public PoseFusion(VisionMeasurementConsumer consumer, LocalizationInputs inputs) {
        this.consumer = consumer;
        this.inputs = inputs == null ? LocalizationInputs.NONE : inputs;
    }

    /**
     * Cambia el layout que usan {@link #isInField(Pose2d)} y las fuentes que no traigan el suyo.
     *
     * @param layout layout del campo.
     */
    public static void setDefaultFieldLayout(AprilTagFieldLayout layout) {
        defaultFieldLayout = layout;
    }

    /** @return el layout por defecto de la feature. */
    public static AprilTagFieldLayout defaultFieldLayout() {
        return defaultFieldLayout;
    }

    /**
     * Comprueba que una pose caiga dentro del campo, con el layout por defecto.
     *
     * @param pose pose a comprobar.
     * @return true si esta dentro.
     */
    public static boolean isInField(Pose2d pose) {
        return isInField(pose, defaultFieldLayout, 0.0);
    }

    /**
     * Comprueba que una pose caiga dentro del campo, con un margen.
     *
     * <p>Un margen pequeno --- del orden de medio metro --- evita descartar mediciones buenas de un
     * robot pegado a la pared, donde el centro del chasis puede quedar fuera del rectangulo por la
     * propia incertidumbre.
     *
     * @param pose          pose a comprobar.
     * @param marginMeters  margen alrededor del campo.
     * @return true si esta dentro.
     */
    public static boolean isInField(Pose2d pose, double marginMeters) {
        return isInField(pose, defaultFieldLayout, marginMeters);
    }

    /**
     * Comprueba que una pose caiga dentro del campo de un layout concreto.
     *
     * @param pose         pose a comprobar.
     * @param layout       layout del campo.
     * @param marginMeters margen alrededor del campo.
     * @return true si esta dentro.
     */
    public static boolean isInField(Pose2d pose, AprilTagFieldLayout layout, double marginMeters) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -marginMeters
                && x <= layout.getFieldLength() + marginMeters
                && y >= -marginMeters
                && y <= layout.getFieldWidth() + marginMeters;
    }

    /**
     * Registra una fuente. Se le inyectan las entradas y la tabla de telemetria.
     *
     * @param source fuente a registrar.
     * @return esta misma fusion, para encadenar.
     */
    public PoseFusion registerSource(PoseSource source) {
        source.bindInputs(inputs);
        source.setTelemetryTable(telemetryTable);
        sources.put(source.getName(), source);
        return this;
    }

    /**
     * Busca una fuente por nombre.
     *
     * @param name nombre de la fuente.
     * @return la fuente, o vacio.
     */
    public Optional<PoseSource> getSource(String name) {
        return Optional.ofNullable(sources.get(name));
    }

    /**
     * Busca una fuente por nombre y tipo.
     *
     * @param <T>  tipo concreto esperado.
     * @param name nombre de la fuente.
     * @param type clase esperada.
     * @return la fuente, o vacio si no existe o si es de otro tipo.
     */
    public <T extends PoseSource> Optional<T> getSource(String name, Class<T> type) {
        PoseSource source = sources.get(name);
        return type.isInstance(source) ? Optional.of(type.cast(source)) : Optional.empty();
    }

    /** @return todas las fuentes registradas, en orden de registro. */
    public Collection<PoseSource> getSources() {
        return sources.values();
    }

    /** @return la puerta de consenso, para configurarla. */
    public ConsensusGate consensus() {
        return consensusGate;
    }

    /**
     * Enciende o apaga el arbitraje entre fuentes.
     *
     * @param enabled true para arbitrar.
     * @return esta misma fusion.
     */
    public PoseFusion setConsensusEnabled(boolean enabled) {
        this.consensusEnabled = enabled;
        return this;
    }

    /** @return true si el arbitraje esta activo. */
    public boolean isConsensusEnabled() {
        return consensusEnabled;
    }

    /**
     * Tabla de NetworkTables para la telemetria. Cambiala antes de registrar las fuentes.
     *
     * @param table nombre de la tabla.
     * @return esta misma fusion.
     */
    public PoseFusion setTelemetryTable(String table) {
        this.telemetryTable = table;
        sources.values().forEach(s -> s.setTelemetryTable(table));
        return this;
    }

    /** @return cuantas mediciones se aceptaron en el ultimo {@link #update()}. */
    public int getAcceptedCount() {
        return acceptedThisCycle;
    }

    /**
     * Un ciclo completo. Llamalo desde {@code periodic()}.
     *
     * <p>Es seguro llamarlo aunque no haya ninguna fuente registrada o aunque todas esten apagadas:
     * en ese caso no hace nada mas que publicar telemetria.
     */
    public void update() {
        acceptedThisCycle = 0;
        List<PoseSource> candidateSources = new ArrayList<>();
        List<PoseSample> candidateSamples = new ArrayList<>();

        for (PoseSource source : sources.values()) {
            source.syncEnabledWithDashboard();

            RejectionReason environment = source.checkEnvironment();
            if (environment != RejectionReason.ACCEPTED) {
                source.markRejected(environment);
                continue;
            }

            Optional<PoseSample> measurement;
            try {
                measurement = source.getMeasurement();
            } catch (RuntimeException e) {
                // Una fuente rota no puede tumbar a las demas: el robot sigue necesitando
                // localizarse con lo que quede.
                source.markRejected(RejectionReason.SOURCE_ERROR);
                warn("La fuente '" + source.getName() + "' fallo: " + e);
                continue;
            }

            if (measurement.isEmpty()) {
                source.markRejected(RejectionReason.NO_MEASUREMENT);
                continue;
            }

            PoseSample sample = measurement.get();
            RejectionReason check = source.checkMeasurement(sample);
            if (check != RejectionReason.ACCEPTED) {
                source.markRejected(check);
                continue;
            }

            candidateSources.add(source);
            candidateSamples.add(sample);
        }

        ConsensusGate.Result consensus = arbitrate(candidateSources, candidateSamples);

        for (int i = 0; i < candidateSources.size(); i++) {
            PoseSource source = candidateSources.get(i);
            PoseSample sample = candidateSamples.get(i);

            if (consensus.rejected().contains(i)) {
                source.markRejected(RejectionReason.CONSENSUS);
                continue;
            }

            RejectionReason mahalanobis = source.checkMahalanobis(sample);
            if (mahalanobis != RejectionReason.ACCEPTED) {
                source.markRejected(mahalanobis);
                continue;
            }

            source.markAccepted(sample.timestampSeconds());
            consumer.accept(sample.pose(), sample.timestampSeconds(), sample.stdDevs());
            acceptedThisCycle++;
        }

        publishTelemetry();
    }

    private ConsensusGate.Result arbitrate(List<PoseSource> sourceList, List<PoseSample> sampleList) {
        if (!consensusEnabled || sourceList.size() < 2) {
            return new ConsensusGate.Result(java.util.Set.of(), List.of());
        }

        List<ConsensusGate.Candidate> candidates = new ArrayList<>(sourceList.size());
        for (int i = 0; i < sourceList.size(); i++) {
            candidates.add(
                    new ConsensusGate.Candidate(
                            sourceList.get(i).getName(),
                            sampleList.get(i).pose(),
                            sampleList.get(i).timestampSeconds()));
        }

        OptionalDouble speed = inputs.planarSpeedMetersPerSecond();
        ConsensusGate.Result result =
                consensusGate.evaluate(candidates, speed.orElse(0.0));

        for (String conflict : result.conflicts()) {
            warn("Consenso: " + conflict);
        }
        return result;
    }

    /**
     * Avisa por consola con un limite de tasa.
     *
     * <p>Un desacuerdo sostenido se repite en cada ciclo, y a 50 Hz eso llena el log de la Driver
     * Station lo bastante rapido como para tapar el resto de los mensajes.
     */
    private void warn(String message) {
        double now = Timer.getFPGATimestamp();
        if (now - lastWarningSeconds < WARNING_PERIOD_SECONDS) {
            return;
        }
        lastWarningSeconds = now;
        DriverStation.reportWarning("[PoseFusion] " + message, false);
    }

    private void publishTelemetry() {
        for (PoseSource source : sources.values()) {
            source.publishTelemetry();
        }
        NetworkIO.set(telemetryTable, "acceptedThisCycle", acceptedThisCycle);
        NetworkIO.set(telemetryTable, "sourceCount", sources.size());
    }
}
