// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import com.stzteam.forgemini.io.NetworkIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BiFunction;

/**
 * Una fuente de mediciones de pose: una camara, un headset de VIO, lo que sea que pueda decir donde
 * esta el robot.
 *
 * <p>Para escribir una solo hacen falta dos metodos: {@link #getMeasurement()}, que lee el hardware,
 * y {@link #isPoseValid(Pose2d)}, que decide si la pose resultante tiene sentido. Todo el resto ---
 * filtros de movimiento, ambiguedad, timestamps, telemetria, encendido y apagado --- ya esta aqui.
 *
 * <h2>Los filtros</h2>
 * {@link PoseFusion} los aplica en este orden, y el primero que falle descarta la medicion:
 *
 * <ol>
 *   <li><b>Encendida</b>, desde codigo o desde el dashboard.</li>
 *   <li><b>Entorno</b>: velocidad angular, aceleracion y velocidad lineal por debajo de sus
 *       umbrales. Se evalua antes de leer el hardware, asi que una fuente descartada por entorno ni
 *       siquiera gasta el tiempo de la lectura.</li>
 *   <li><b>Timestamp</b> estrictamente mas nuevo que el de la ultima medicion aceptada.</li>
 *   <li><b>Pose valida</b>, segun la subclase.</li>
 *   <li><b>Ambiguedad</b>, si la fuente la reporta.</li>
 *   <li><b>Consenso</b> con las demas fuentes; lo arbitra {@link ConsensusGate}.</li>
 *   <li><b>Mahalanobis</b> contra lo que el estimator ya cree.</li>
 * </ol>
 *
 * <h2>Desviaciones estandar</h2>
 * Hay dos caminos. Uno fijo con {@link #setStdDevs(double, double, double)}, y uno dinamico con
 * {@link #setStdDevsStrategy(BiFunction)}, que recibe distancia y numero de tags y devuelve la
 * confianza que toque. El segundo es lo normal para vision, y {@link StdDevStrategies} trae las
 * estrategias hechas.
 */
public abstract class PoseSource {

    private final String name;

    private Matrix<N3, N1> stdDevs;
    private PoseSourceConfig config;
    private BiFunction<Double, Integer, Matrix<N3, N1>> stdDevsStrategy;

    /** Rellenado por {@link PoseFusion} al registrar la fuente. */
    protected LocalizationInputs inputs = LocalizationInputs.NONE;

    private String telemetryTable = "Localization";

    private boolean enabled = true;
    private boolean lastPublishedEnabled = true;

    private double lastAcceptedTimestamp = Double.NaN;
    private RejectionReason lastReason = RejectionReason.NO_MEASUREMENT;

    /**
     * Crea una fuente.
     *
     * @param name            nombre unico; es la clave en NetworkTables y en el registro.
     * @param defaultStdDevs  desviaciones estandar por defecto, si no se pone una estrategia.
     * @param config          umbrales de los filtros.
     */
    protected PoseSource(String name, Matrix<N3, N1> defaultStdDevs, PoseSourceConfig config) {
        this.name = name;
        this.stdDevs = defaultStdDevs;
        this.config = config;
    }

    // ------------------------------------------------------------------
    // Lo que implementa la subclase
    // ------------------------------------------------------------------

    /**
     * Lee el hardware y devuelve la medicion de este ciclo, si hay alguna.
     *
     * <p>Solo se llama cuando la fuente esta encendida y el entorno es valido, asi que no hace falta
     * volver a comprobar nada de eso.
     *
     * @return la medicion, o vacio si no hay nada nuevo.
     */
    public abstract Optional<PoseSample> getMeasurement();

    /**
     * Decide si una pose tiene sentido. Lo habitual es comprobar que caiga dentro del campo con
     * {@link PoseFusion#isInField(Pose2d, double)}.
     *
     * @param pose pose a validar.
     * @return true si la pose es aceptable.
     */
    public abstract boolean isPoseValid(Pose2d pose);

    /**
     * Ambiguedad de la ultima solucion de PnP, si la fuente la reporta.
     *
     * <p>Devolver vacio --- que es el comportamiento por defecto --- significa "esta fuente no tiene
     * ese concepto", y el filtro de ambiguedad se salta. Es lo correcto para un VIO.
     *
     * @return la ambiguedad de 0 a 1, o vacio.
     */
    public OptionalDouble getAmbiguity() {
        return OptionalDouble.empty();
    }

    // ------------------------------------------------------------------
    // Configuracion
    // ------------------------------------------------------------------

    /**
     * Fija desviaciones estandar constantes y descarta cualquier estrategia dinamica.
     *
     * @param stdX     desviacion en X, metros.
     * @param stdY     desviacion en Y, metros.
     * @param stdTheta desviacion de rumbo, radianes.
     * @return esta misma fuente.
     */
    public PoseSource setStdDevs(double stdX, double stdY, double stdTheta) {
        return setStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
    }

    /**
     * Fija desviaciones estandar constantes y descarta cualquier estrategia dinamica.
     *
     * @param newStdDevs vector de desviaciones.
     * @return esta misma fuente.
     */
    public PoseSource setStdDevs(Matrix<N3, N1> newStdDevs) {
        this.stdDevs = newStdDevs;
        this.stdDevsStrategy = null;
        return this;
    }

    /**
     * Pone una estrategia que calcula las desviaciones a partir de la distancia al tag y del numero
     * de tags visibles. Ver {@link StdDevStrategies}.
     *
     * @param strategy estrategia, o null para volver a las desviaciones fijas.
     * @return esta misma fuente.
     */
    public PoseSource setStdDevsStrategy(BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        this.stdDevsStrategy = strategy;
        return this;
    }

    /**
     * Calcula las desviaciones de una medicion. Las subclases lo llaman al construir su
     * {@link PoseSample}.
     *
     * @param distanceMeters distancia promedio al tag; se ignora si no hay estrategia.
     * @param tagCount       numero de tags usados; se ignora si no hay estrategia.
     * @return el vector de desviaciones.
     */
    public Matrix<N3, N1> computeStdDevs(double distanceMeters, int tagCount) {
        if (stdDevsStrategy == null) {
            return stdDevs;
        }
        return stdDevsStrategy.apply(distanceMeters, Math.max(1, tagCount));
    }

    /**
     * Cambia los umbrales de los filtros.
     *
     * @param newConfig nueva configuracion.
     * @return esta misma fuente.
     */
    public PoseSource setConfig(PoseSourceConfig newConfig) {
        this.config = newConfig;
        return this;
    }

    /**
     * Tabla de NetworkTables donde se publica la telemetria. Cambiala si tienes dos
     * {@link PoseFusion} en el mismo robot y no quieres que se pisen.
     *
     * @param table nombre de la tabla.
     * @return esta misma fuente.
     */
    public PoseSource setTelemetryTable(String table) {
        this.telemetryTable = table;
        return this;
    }

    // ------------------------------------------------------------------
    // Encendido y apagado
    // ------------------------------------------------------------------

    /**
     * Enciende o apaga la fuente desde codigo.
     *
     * <p>Convive con el interruptor del dashboard sin que ninguno de los dos se pierda: mandan los
     * dos, y el ultimo en moverse gana. Ver {@link #syncEnabledWithDashboard()}.
     *
     * @param value true para encenderla.
     */
    public void setEnabled(boolean value) {
        this.enabled = value;
    }

    /** @return true si la fuente esta encendida. */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Reconcilia el interruptor del dashboard con el del codigo.
     *
     * <p>El detalle importante: se compara el valor de NetworkTables contra <b>lo ultimo que
     * publicamos nosotros</b>, no contra el estado actual. Si difieren, lo movio una persona desde
     * el dashboard y ese valor manda; si coinciden, nadie lo toco y manda el del codigo.
     *
     * <p>Sin esa comparacion, leer NetworkTables al principio de cada ciclo sobrescribe en silencio
     * cualquier {@link #setEnabled(boolean)}, y apagar una fuente desde codigo deja de funcionar.
     */
    void syncEnabledWithDashboard() {
        boolean fromDashboard = NetworkIO.get(telemetryTable, name + "/enabled", lastPublishedEnabled);
        if (fromDashboard != lastPublishedEnabled) {
            enabled = fromDashboard;
        }
    }

    // ------------------------------------------------------------------
    // Filtros
    // ------------------------------------------------------------------

    /**
     * Comprueba que el robot no se este moviendo de forma que invalide la medicion.
     *
     * @return el motivo del descarte, o {@link RejectionReason#ACCEPTED} si el entorno es valido.
     */
    RejectionReason checkEnvironment() {
        if (!enabled) {
            return RejectionReason.DISABLED;
        }

        OptionalDouble angular = inputs.angularSpeedDegreesPerSecond();
        if (angular.isPresent() && angular.getAsDouble() > config.maxAngularVelocityDegPerSec()) {
            return RejectionReason.ANGULAR_VELOCITY;
        }

        OptionalDouble acceleration = inputs.planarAccelerationGs();
        if (acceleration.isPresent() && acceleration.getAsDouble() > config.maxAccelerationGs()) {
            return RejectionReason.ACCELERATION;
        }

        OptionalDouble planar = inputs.planarSpeedMetersPerSecond();
        if (planar.isPresent() && planar.getAsDouble() > config.maxPlanarVelocityMps()) {
            return RejectionReason.PLANAR_VELOCITY;
        }

        return RejectionReason.ACCEPTED;
    }

    /**
     * Comprueba la medicion en si: que sea nueva, que la pose tenga sentido y que la ambiguedad este
     * dentro del umbral.
     *
     * @param sample medicion a comprobar.
     * @return el motivo del descarte, o {@link RejectionReason#ACCEPTED}.
     */
    RejectionReason checkMeasurement(PoseSample sample) {
        if (!Double.isNaN(lastAcceptedTimestamp)
                && sample.timestampSeconds() <= lastAcceptedTimestamp) {
            return RejectionReason.STALE_TIMESTAMP;
        }
        if (!isPoseValid(sample.pose())) {
            return RejectionReason.POSE_INVALID;
        }

        OptionalDouble ambiguity = getAmbiguity();
        if (ambiguity.isPresent() && ambiguity.getAsDouble() > config.maxAmbiguity()) {
            return RejectionReason.AMBIGUITY;
        }

        return RejectionReason.ACCEPTED;
    }

    /**
     * Filtro de Mahalanobis: descarta lo que este demasiado lejos de lo que el estimator ya cree,
     * medido en sigmas de la incertidumbre combinada.
     *
     * <p>La incertidumbre del estimator crece con el tiempo desde la ultima medicion aceptada, asi
     * que el filtro se abre solo cuando la fuente lleva rato callada. Eso evita el fallo clasico de
     * este tipo de puerta: quedarse cerrada para siempre porque el estimator se fue de sitio y ya
     * nada le parece razonable.
     *
     * <p>La <b>primera</b> medicion se acepta siempre. No hay con que compararla: el estimator
     * todavia no tiene una creencia que defender.
     *
     * @param sample medicion a comprobar.
     * @return el motivo del descarte, o {@link RejectionReason#ACCEPTED}.
     */
    RejectionReason checkMahalanobis(PoseSample sample) {
        if (!config.mahalanobisEnabled() || Double.isNaN(lastAcceptedTimestamp)) {
            return RejectionReason.ACCEPTED;
        }

        Optional<Pose2d> estimate = inputs.pose();
        if (estimate.isEmpty()) {
            return RejectionReason.ACCEPTED;
        }

        double elapsed = Math.max(0.0, sample.timestampSeconds() - lastAcceptedTimestamp);
        double drift = elapsed * config.mahalanobisDriftVariancePerSecond();

        double estimatorVarXY = square(config.mahalanobisBaseStdDevMeters()) + drift;
        double estimatorVarTheta = square(config.mahalanobisBaseStdDevRadians()) + drift;

        Pose2d current = estimate.get();
        double dx = sample.pose().getX() - current.getX();
        double dy = sample.pose().getY() - current.getY();
        double dtheta = sample.pose().getRotation().minus(current.getRotation()).getRadians();

        double varX = estimatorVarXY + square(sample.stdDevs().get(0, 0));
        double varY = estimatorVarXY + square(sample.stdDevs().get(1, 0));
        double varTheta = estimatorVarTheta + square(sample.stdDevs().get(2, 0));

        double distanceSquared = (dx * dx) / varX + (dy * dy) / varY + (dtheta * dtheta) / varTheta;

        return distanceSquared <= square(config.maxMahalanobisSigmas())
                ? RejectionReason.ACCEPTED
                : RejectionReason.MAHALANOBIS;
    }

    private static double square(double value) {
        return value * value;
    }

    // ------------------------------------------------------------------
    // Estado y telemetria
    // ------------------------------------------------------------------

    void markAccepted(double timestampSeconds) {
        this.lastAcceptedTimestamp = timestampSeconds;
        this.lastReason = RejectionReason.ACCEPTED;
    }

    void markRejected(RejectionReason reason) {
        this.lastReason = reason;
    }

    /**
     * Detalle libre que la subclase puede publicar junto al estado.
     *
     * <p>Existe para que {@link RejectionReason#NO_MEASUREMENT} no sea un callejon sin salida: "la
     * fuente no reporto nada" puede ser que la camara no vea tags, que el headset este
     * desconectado, o que el timestamp no sirva, y desde el dashboard los tres se ven igual.
     *
     * @return una linea corta, o cadena vacia si no hay nada que anadir.
     */
    protected String telemetryDetail() {
        return "";
    }

    void publishTelemetry() {
        NetworkIO.set(telemetryTable, name + "/enabled", enabled);
        NetworkIO.set(telemetryTable, name + "/status", lastReason.name());
        NetworkIO.set(telemetryTable, name + "/detail", telemetryDetail());
        NetworkIO.set(telemetryTable, name + "/accepted", lastReason == RejectionReason.ACCEPTED);
        NetworkIO.set(
                telemetryTable,
                name + "/lastAcceptedTimestamp",
                Double.isNaN(lastAcceptedTimestamp) ? -1.0 : lastAcceptedTimestamp);
        lastPublishedEnabled = enabled;
    }

    /** @return nombre de la fuente. */
    public String getName() {
        return name;
    }

    /** @return configuracion vigente. */
    public PoseSourceConfig getConfig() {
        return config;
    }

    /** @return desviaciones estandar fijas, sin aplicar ninguna estrategia. */
    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    /** @return por que se descarto --- o no --- la ultima medicion. */
    public RejectionReason getLastReason() {
        return lastReason;
    }

    /**
     * Instante de la ultima medicion aceptada.
     *
     * @return el instante, o vacio si todavia no se acepto ninguna.
     */
    public OptionalDouble getLastAcceptedTimestamp() {
        return Double.isNaN(lastAcceptedTimestamp)
                ? OptionalDouble.empty()
                : OptionalDouble.of(lastAcceptedTimestamp);
    }

    void bindInputs(LocalizationInputs newInputs) {
        this.inputs = newInputs == null ? LocalizationInputs.NONE : newInputs;
    }
}
