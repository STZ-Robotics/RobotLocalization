// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Umbrales de los filtros de una {@link PoseSource}.
 *
 * <p>Setters fluidos, igual que el resto de configuraciones de la feature:
 *
 * <pre>{@code
 * var config = PoseSourceConfig.forAprilTags()
 *         .withMaxAngularVelocity(DegreesPerSecond.of(540))
 *         .withMaxAmbiguity(0.25);
 * }</pre>
 *
 * <h2>Como se apaga un filtro</h2>
 * Todo umbral arranca en infinito, que es la forma de decir "sin limite": la comparacion
 * {@code valor <= INFINITY} siempre pasa, asi que un filtro sin configurar no filtra nada. No hay
 * banderas de activacion ni valores centinela que memorizar.
 */
public class PoseSourceConfig {

    private double maxAngularVelocityDegPerSec = Double.POSITIVE_INFINITY;
    private double maxPlanarVelocityMps = Double.POSITIVE_INFINITY;
    private double maxAccelerationGs = Double.POSITIVE_INFINITY;
    private double maxAmbiguity = Double.POSITIVE_INFINITY;
    private double maxMahalanobisSigmas = Double.POSITIVE_INFINITY;

    private double mahalanobisBaseStdDevMeters = 0.05;
    private double mahalanobisBaseStdDevRadians = 0.05;
    private double mahalanobisDriftVariancePerSecond = 0.0;

    /** Crea una configuracion sin ningun filtro activo. */
    public PoseSourceConfig() {}

    /**
     * Preset razonable para una camara que mira AprilTags.
     *
     * <p>Los numeros son puntos de partida defendibles, no verdades: 720 grados por segundo es
     * aproximadamente donde el desenfoque de movimiento empieza a arruinar el PnP en una camara de
     * obturador global, y 0.3 de ambiguedad es el umbral que recomienda PhotonVision. Ajustalos
     * contra tus propios datos.
     *
     * @return una configuracion nueva.
     */
    public static PoseSourceConfig forAprilTags() {
        return new PoseSourceConfig()
                .withMaxAngularVelocity(DegreesPerSecond.of(720))
                .withMaxAmbiguity(0.3)
                .withMahalanobisGate(4.0, 0.05, 0.05, 0.02);
    }

    /**
     * Preset razonable para una fuente de odometria visual-inercial tipo QuestNav.
     *
     * <p>Sin limite de velocidad angular: un VIO no hace PnP, asi que girar no le afecta como a una
     * camara. Sin ambiguedad porque no reporta ninguna. El gate de Mahalanobis va mas apretado
     * porque un VIO no deberia dar saltos, y cuando los da --- re-adquisicion tras perder el
     * tracking --- es justo lo que hay que atrapar.
     *
     * @return una configuracion nueva.
     */
    public static PoseSourceConfig forVisualInertialOdometry() {
        return new PoseSourceConfig().withMahalanobisGate(3.0, 0.02, 0.02, 0.05);
    }

    /**
     * Velocidad angular por encima de la cual se descartan las mediciones.
     *
     * @param limit velocidad angular maxima.
     * @return esta misma configuracion.
     */
    public PoseSourceConfig withMaxAngularVelocity(AngularVelocity limit) {
        this.maxAngularVelocityDegPerSec =
                limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(DegreesPerSecond));
        return this;
    }

    /**
     * Velocidad lineal por encima de la cual se descartan las mediciones.
     *
     * @param limit velocidad lineal maxima.
     * @return esta misma configuracion.
     */
    public PoseSourceConfig withMaxPlanarVelocity(LinearVelocity limit) {
        this.maxPlanarVelocityMps =
                limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(MetersPerSecond));
        return this;
    }

    /**
     * Aceleracion por encima de la cual se descartan las mediciones. Sirve sobre todo para ignorar
     * el instante de un choque, cuando la camara se mueve mas rapido de lo que su obturador aguanta.
     *
     * @param limit aceleracion maxima.
     * @return esta misma configuracion.
     */
    public PoseSourceConfig withMaxAcceleration(LinearAcceleration limit) {
        this.maxAccelerationGs = limit == null ? Double.POSITIVE_INFINITY : Math.abs(limit.in(Gs));
        return this;
    }

    /**
     * Ambiguedad de PnP por encima de la cual se descarta la medicion. Solo aplica a las fuentes que
     * reportan ambiguedad; las que no, se saltan el filtro.
     *
     * @param limit ambiguedad maxima, de 0 a 1.
     * @return esta misma configuracion.
     */
    public PoseSourceConfig withMaxAmbiguity(double limit) {
        this.maxAmbiguity = limit;
        return this;
    }

    /**
     * Configura el filtro de Mahalanobis, que descarta las mediciones demasiado lejos de lo que el
     * estimator ya cree.
     *
     * <p>La incertidumbre con la que se compara <b>crece con el tiempo</b> desde la ultima medicion
     * aceptada. Eso es lo que hace al filtro util y no un candado: si una fuente lleva diez segundos
     * sin aportar nada, el estimator pudo haberse ido de sitio, y el filtro se abre para dejar que
     * lo corrija.
     *
     * @param sigmas                distancia maxima en sigmas. 3 es apretado, 5 es permisivo.
     * @param baseStdDevMeters      incertidumbre de posicion del estimator recien corregido.
     * @param baseStdDevRadians     incertidumbre de rumbo del estimator recien corregido.
     * @param driftVariancePerSecond cuanta varianza suma la odometria por segundo sin correccion.
     * @return esta misma configuracion.
     */
    public PoseSourceConfig withMahalanobisGate(
            double sigmas,
            double baseStdDevMeters,
            double baseStdDevRadians,
            double driftVariancePerSecond) {
        this.maxMahalanobisSigmas = sigmas;
        this.mahalanobisBaseStdDevMeters = baseStdDevMeters;
        this.mahalanobisBaseStdDevRadians = baseStdDevRadians;
        this.mahalanobisDriftVariancePerSecond = driftVariancePerSecond;
        return this;
    }

    /** @return velocidad angular maxima en grados por segundo. */
    public double maxAngularVelocityDegPerSec() {
        return maxAngularVelocityDegPerSec;
    }

    /** @return velocidad lineal maxima en metros por segundo. */
    public double maxPlanarVelocityMps() {
        return maxPlanarVelocityMps;
    }

    /** @return aceleracion maxima en G. */
    public double maxAccelerationGs() {
        return maxAccelerationGs;
    }

    /** @return ambiguedad maxima. */
    public double maxAmbiguity() {
        return maxAmbiguity;
    }

    /** @return distancia de Mahalanobis maxima, en sigmas. */
    public double maxMahalanobisSigmas() {
        return maxMahalanobisSigmas;
    }

    /** @return true si el filtro de Mahalanobis esta activo. */
    public boolean mahalanobisEnabled() {
        return Double.isFinite(maxMahalanobisSigmas);
    }

    /** @return incertidumbre base de posicion del estimator, en metros. */
    public double mahalanobisBaseStdDevMeters() {
        return mahalanobisBaseStdDevMeters;
    }

    /** @return incertidumbre base de rumbo del estimator, en radianes. */
    public double mahalanobisBaseStdDevRadians() {
        return mahalanobisBaseStdDevRadians;
    }

    /** @return varianza que suma la odometria por segundo sin correccion. */
    public double mahalanobisDriftVariancePerSecond() {
        return mahalanobisDriftVariancePerSecond;
    }
}
