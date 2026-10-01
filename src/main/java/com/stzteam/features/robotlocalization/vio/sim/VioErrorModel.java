// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.Random;

/**
 * El modelo de error de la odometria visual-inercial. Es el contenido real de esta feature.
 *
 * <p>Una Limelight ve el tag o no lo ve. Un Quest <b>siempre</b> reporta una pose; lo que cambia es
 * cuanto miente. Simular el protocolo sin simular la mentira daria un sensor perfecto, que es justo
 * el que nadie necesita probar.
 *
 * <h2>Deriva continua</h2>
 * La deriva se escala por <b>distancia recorrida</b>, no por tiempo: un robot parado no deriva, y
 * uno que cruza el campo si. Es como se comporta el VIO real. El yaw deriva por rotacion acumulada,
 * por la misma razon.
 *
 * <h2>Modos de fallo</h2>
 * Los cuatro que documenta QuestNav, cada uno inyectable desde el Sim GUI o desde un test:
 *
 * <ul>
 *   <li><b>Perdida de features</b> (poca luz, reflejos, superficies uniformes). La documentacion
 *       dice que el Quest puede re-adquirir el tracking <i>con un offset</i>, asi que al recuperar
 *       la pose vuelve desplazada. Ese salto es el que rompe a un pose estimator afinado.</li>
 *   <li><b>Doble-tap a passthrough por vibracion</b>: discontinuidad de pose <i>sin</i> perder
 *       tracking. Mas traicionero, porque {@code isTracking} sigue en true.</li>
 *   <li><b>Bateria critica</b>: el headset se apaga solo a media match.</li>
 *   <li><b>Caida de red</b>: se deja de publicar y {@code isConnected()} cae a los 120 ms.</li>
 * </ul>
 *
 * <p>Los dos ultimos no son error de pose sino ausencia de datos, asi que los gestiona
 * {@link SimulatedQuestNav} consultando {@link #isPoweredOn()} y {@link #isLinkUp(double)}.
 *
 * <h2>Sobre los valores por defecto</h2>
 * Son <b>plausibles, no medidos</b>. Estan elegidos para que el orden de magnitud sea razonable y
 * para que los fallos se noten en una prueba corta. Si tienes logs de tu propio headset, ajustalos.
 *
 * <h2>Simplificacion consciente</h2>
 * La deriva de posicion y la de yaw se modelan como aditivas e independientes. En un VIO real el
 * error de rumbo tambien curva la trayectoria estimada. Reproducir eso requeriria arrastrar un
 * marco de referencia a la deriva, y para el proposito de esta feature --- romper un pose estimator
 * de forma realista --- no cambia nada: lo que importa es que el error crezca con el recorrido, que
 * salte en la re-adquisicion y que un POSE_RESET lo limpie.
 */
public class VioErrorModel {

    private final QuestNavSimConfig config;
    private final Random random;

    private Translation3d driftTranslation = Translation3d.kZero;
    private double driftYawRadians = 0.0;

    private Pose3d previousTruePose;
    private Pose3d frozenPose;

    private boolean tracking = true;
    private int trackingLostCounter = 0;
    private double trackingLostUntilSeconds = Double.NEGATIVE_INFINITY;
    private double linkDownUntilSeconds = Double.NEGATIVE_INFINITY;
    private double batteryPercent;

    private double lastSpeedMetersPerSecond = 0.0;
    private double lastAngularSpeedDegreesPerSecond = 0.0;

    // Ventana de mediana para separar un choque de un salto del pose estimator. Ver detectImpact.
    private final double[] speedWindow = new double[3];
    private final double[] angularWindow = new double[3];
    private int windowIndex = 0;
    private int windowFill = 0;
    private double lastMedianSpeed = Double.NaN;
    private double lastMedianAngular = Double.NaN;

    /**
     * Crea el modelo.
     *
     * @param config configuracion de la camara simulada, de donde salen todos los parametros.
     */
    public VioErrorModel(QuestNavSimConfig config) {
        this.config = config;
        this.random = new Random(config.randomSeed());
        this.batteryPercent = config.initialBatteryPercent();
    }

    /**
     * Avanza el modelo un paso y devuelve la pose que el headset deberia reportar.
     *
     * @param truePose    pose real del headset (robot + mount), segun la simulacion.
     * @param nowSeconds  instante actual, en segundos del FPGA.
     * @param dtSeconds   tiempo transcurrido desde la llamada anterior.
     * @return la pose con el error aplicado. Si el tracking esta perdido, la pose congelada.
     */
    public Pose3d update(Pose3d truePose, double nowSeconds, double dtSeconds) {
        drainBattery(dtSeconds);

        if (previousTruePose == null) {
            previousTruePose = truePose;
            frozenPose = truePose;
        }

        double distance = truePose.getTranslation().getDistance(previousTruePose.getTranslation());
        double rotation =
                Math.abs(
                        truePose.getRotation().toRotation2d().minus(
                                previousTruePose.getRotation().toRotation2d()).getRadians());

        if (dtSeconds > 0) {
            double speed = distance / dtSeconds;
            double angularSpeed = Units.radiansToDegrees(rotation) / dtSeconds;
            detectImpact(speed, angularSpeed, dtSeconds, nowSeconds);
            lastSpeedMetersPerSecond = speed;
            lastAngularSpeedDegreesPerSecond = angularSpeed;
        }

        previousTruePose = truePose;

        if (nowSeconds < trackingLostUntilSeconds) {
            // Sin tracking el Quest no tiene nada nuevo que decir: repite la ultima pose buena.
            tracking = false;
            return frozenPose;
        }

        if (!tracking) {
            // Re-adquisicion. La documentacion de QuestNav avisa de que el tracking puede volver
            // "con un offset": ese salto es el fallo mas destructivo del sensor y hay que poder
            // reproducirlo, no suavizarlo.
            tracking = true;
            driftTranslation = driftTranslation.plus(randomOffset(config.reacquireOffsetMeters()));
        }

        accumulateDrift(distance, rotation);

        Pose3d reported = applyError(truePose);
        frozenPose = reported;
        return reported;
    }

    private void accumulateDrift(double distanceMeters, double rotationRadians) {
        if (distanceMeters > 0) {
            double sigma = config.driftPerMeter() * distanceMeters;
            driftTranslation =
                    driftTranslation.plus(
                            new Translation3d(
                                    random.nextGaussian() * sigma,
                                    random.nextGaussian() * sigma,
                                    random.nextGaussian() * sigma * config.verticalDriftFactor()));
        }

        if (rotationRadians > 0) {
            double turns = rotationRadians / (2 * Math.PI);
            driftYawRadians +=
                    random.nextGaussian() * Units.degreesToRadians(config.driftDegreesPerTurn()) * turns;
        }
    }

    private Pose3d applyError(Pose3d truePose) {
        Translation3d noise =
                new Translation3d(
                        random.nextGaussian() * config.frameNoiseMeters(),
                        random.nextGaussian() * config.frameNoiseMeters(),
                        random.nextGaussian() * config.frameNoiseMeters());
        double yawNoise = random.nextGaussian() * Units.degreesToRadians(config.frameNoiseDegrees());

        return new Pose3d(
                truePose.getTranslation().plus(driftTranslation).plus(noise),
                truePose.getRotation().plus(new Rotation3d(0.0, 0.0, driftYawRadians + yawNoise)));
    }

    /**
     * Un golpe fuerte le tira el tracking al Quest. Se detecta por aceleracion o aceleracion
     * angular, no por velocidad: cruzar el campo rapido no rompe nada, chocar contra la pared si.
     *
     * <h2>Por que la velocidad se filtra por mediana</h2>
     * La pose que entra aqui suele venir del pose estimator del robot, y un pose estimator
     * <b>salta</b> cada vez que acepta una medicion de vision. Un salto de 20 cm en un frame de
     * 10 ms son 2000 m/s2 de aceleracion aparente --- cincuenta veces el umbral --- asi que en crudo
     * el detector confunde cada correccion de vision con un choque.
     *
     * <p>Contar frames consecutivos no lo arregla, porque un choque contra la pared tambien es un
     * solo frame: la pose deja de avanzar de golpe. Lo que de verdad los distingue es la forma.
     *
     * <ul>
     *   <li>Una correccion de vision es un <b>impulso</b>: la velocidad aparente se dispara un frame
     *       y vuelve al valor que traia.</li>
     *   <li>Un choque es un <b>escalon</b>: la velocidad venia alta y se queda baja.</li>
     * </ul>
     *
     * <p>Una mediana de tres muestras es justo la herramienta para eso: se come los impulsos y
     * respeta los escalones. La aceleracion se calcula sobre la velocidad filtrada, no sobre la
     * cruda.
     *
     * <p>Esto salio corriendo la feature contra una simulacion viva: el Quest perdia el tracking en
     * el 68% de los ciclos, cada re-adquisicion sumaba su offset, y la pose acababa fuera del campo.
     */
    private void detectImpact(
            double speed, double angularSpeed, double dtSeconds, double nowSeconds) {
        if (!config.impactTrackingLoss()) {
            return;
        }

        speedWindow[windowIndex] = speed;
        angularWindow[windowIndex] = angularSpeed;
        windowIndex = (windowIndex + 1) % speedWindow.length;
        windowFill = Math.min(windowFill + 1, speedWindow.length);

        if (windowFill < speedWindow.length) {
            return;
        }

        double medianSpeed = median3(speedWindow);
        double medianAngular = median3(angularWindow);

        if (!Double.isNaN(lastMedianSpeed)) {
            double acceleration = Math.abs(medianSpeed - lastMedianSpeed) / dtSeconds;
            double angularAcceleration = Math.abs(medianAngular - lastMedianAngular) / dtSeconds;

            if (acceleration > config.impactAccelThreshold()
                    || angularAcceleration > config.impactAngularAccelThreshold()) {
                injectTrackingLoss(config.trackingLossSeconds(), nowSeconds);
            }
        }

        lastMedianSpeed = medianSpeed;
        lastMedianAngular = medianAngular;
    }

    /** Mediana de tres valores, sin ordenar el array de verdad. */
    private static double median3(double[] values) {
        double a = values[0];
        double b = values[1];
        double c = values[2];
        return Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
    }

    private void drainBattery(double dtSeconds) {
        batteryPercent =
                Math.max(0.0, batteryPercent - config.batteryDrainPercentPerMinute() * dtSeconds / 60.0);
    }

    private Translation3d randomOffset(double magnitudeMeters) {
        double heading = random.nextDouble() * 2 * Math.PI;
        return new Translation3d(
                Math.cos(heading) * magnitudeMeters, Math.sin(heading) * magnitudeMeters, 0.0);
    }

    /**
     * Fuerza una perdida de tracking. Al recuperar, la pose vuelve desplazada.
     *
     * @param durationSeconds cuanto dura la perdida.
     * @param nowSeconds      instante actual, en segundos del FPGA.
     */
    public void injectTrackingLoss(double durationSeconds, double nowSeconds) {
        if (!tracking && nowSeconds < trackingLostUntilSeconds) {
            return; // ya estaba perdido, no cuenta como perdida nueva
        }
        tracking = false;
        trackingLostCounter++;
        trackingLostUntilSeconds = nowSeconds + durationSeconds;
    }

    /**
     * Inyecta una discontinuidad de pose sin perder el tracking. Reproduce el doble-tap a
     * passthrough que dispara la vibracion del chasis: {@code isTracking} se queda en true, asi que
     * el robot code no tiene forma de saber que la pose acaba de saltar.
     *
     * @param jump desplazamiento a aplicar, en metros.
     */
    public void injectPoseDiscontinuity(Translation3d jump) {
        driftTranslation = driftTranslation.plus(jump);
    }

    /**
     * Simula un corte de red o un cable flojo: se deja de publicar y el robot ve caer
     * {@code isConnected()} a los 120 ms.
     *
     * @param durationSeconds cuanto dura el corte.
     * @param nowSeconds      instante actual, en segundos del FPGA.
     */
    public void injectLinkLoss(double durationSeconds, double nowSeconds) {
        linkDownUntilSeconds = nowSeconds + durationSeconds;
    }

    /**
     * Borra la deriva acumulada. Lo llama el manejador de POSE_RESET, porque un reset real
     * reancla el origen del headset y deja el error en cero.
     */
    public void resetDrift() {
        driftTranslation = Translation3d.kZero;
        driftYawRadians = 0.0;
    }

    /**
     * Fija la carga de bateria. Util para probar el apagado sin esperar a que se descargue.
     *
     * @param percent porcentaje de 0 a 100.
     */
    public void setBatteryPercent(double percent) {
        batteryPercent = Math.max(0.0, Math.min(100.0, percent));
    }

    /** @return true si el headset esta reportando tracking valido. */
    public boolean isTracking() {
        return tracking;
    }

    /** @return cuantas veces se ha perdido el tracking desde el arranque. */
    public int trackingLostCounter() {
        return trackingLostCounter;
    }

    /** @return la carga de bateria, redondeada como la reporta el headset. */
    public int batteryPercent() {
        return (int) Math.round(batteryPercent);
    }

    /** @return false cuando la bateria llego a cero y el headset se apago. */
    public boolean isPoweredOn() {
        return batteryPercent > 0.0;
    }

    /**
     * Indica si el enlace de red esta arriba.
     *
     * @param nowSeconds instante actual, en segundos del FPGA.
     * @return false mientras dure un corte inyectado.
     */
    public boolean isLinkUp(double nowSeconds) {
        return nowSeconds >= linkDownUntilSeconds;
    }

    /** @return la deriva de traslacion acumulada, para telemetria. */
    public Translation3d driftTranslation() {
        return driftTranslation;
    }

    /** @return la deriva de yaw acumulada en grados, para telemetria. */
    public double driftYawDegrees() {
        return Units.radiansToDegrees(driftYawRadians);
    }
}
