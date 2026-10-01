// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Optional;
import java.util.function.BiFunction;

/**
 * Un Meta Quest con QuestNav como fuente de pose.
 *
 * <p>Envuelve el {@code QuestNav} del vendordep: consume los frames sin leer, deshace la transformada
 * de montaje para pasar de la pose del headset a la del robot, y descarta lo que llegue sin
 * tracking valido.
 *
 * <h2>Un VIO no es una camara</h2>
 * No hay tags, ni distancia, ni ambiguedad: el error de un VIO no depende de lo lejos que este de
 * nada, sino de cuanto lleva sin que lo corrijan. Por eso la estrategia por defecto es
 * {@link StdDevStrategies#vioTimeDrift}, y por eso hay que llamar a {@link #notifyPoseReset()}
 * cuando el robot le manda un {@code setPose} al headset: ahi la deriva real vuelve a cero y la
 * estrategia tiene que enterarse.
 *
 * <h2>Se queda con el frame mas nuevo</h2>
 * El headset publica a 100 Hz y el robot corre a 50, asi que cada ciclo hay dos o mas frames
 * pendientes. Se usa el ultimo con tracking valido. Meterlos todos al estimator tampoco estaria mal,
 * pero el mas nuevo ya contiene la informacion de los anteriores y cuesta la mitad.
 *
 * <h2>El aviso del timestamp</h2>
 * En simulacion, si {@code SimulatedQuestNav} publica en la misma instancia de NetworkTables que
 * consume el robot, {@code PoseFrame.dataTimestamp()} vale 1 microsegundo constante. Esta fuente lo
 * detecta y avisa una sola vez en lugar de meterle al pose estimator mediciones fechadas en el ano
 * cero, que es justo el fallo silencioso que hay que evitar. La solucion es encender
 * {@code withNetworkClient(true)} en el simulador.
 */
public class QuestNavPoseSource extends PoseSource {

    /**
     * Por debajo de este timestamp se asume que el valor es el centinela de NetworkTables y no un
     * instante real. Un robot nunca tiene un frame legitimo fechado en el primer milisegundo.
     */
    private static final double SENTINEL_TIMESTAMP_SECONDS = 1e-3;

    private final QuestNav questNav;
    private final Transform3d robotToHeadset;
    private final StdDevStrategies.VioDriftStrategy driftStrategy;

    private double fieldMarginMeters = 0.5;
    private boolean requireTracking = true;
    private boolean sentinelWarningIssued = false;
    private String detail = "";

    /**
     * Crea la fuente con la configuracion recomendada para un VIO.
     *
     * @param name           nombre de la fuente.
     * @param robotToHeadset transformada del origen del robot al headset, el mismo
     *                       {@code ROBOT_TO_QUEST} que usa tu subsistema.
     * @return la fuente.
     */
    public static QuestNavPoseSource create(String name, Transform3d robotToHeadset) {
        return new QuestNavPoseSource(
                name, robotToHeadset, new QuestNav(), PoseSourceConfig.forVisualInertialOdometry());
    }

    /**
     * Crea la fuente.
     *
     * @param name           nombre de la fuente.
     * @param robotToHeadset transformada del origen del robot al headset.
     * @param questNav       instancia del vendordep; se inyecta para poder probarla.
     * @param config         umbrales de los filtros.
     */
    public QuestNavPoseSource(
            String name, Transform3d robotToHeadset, QuestNav questNav, PoseSourceConfig config) {
        super(name, VecBuilder.fill(0.02, 0.02, 0.035), config);
        this.questNav = questNav;
        this.robotToHeadset = robotToHeadset;

        // Un Quest recien corregido es extremadamente preciso y se degrada con el tiempo. Los
        // numeros son un punto de partida razonable, no una caracterizacion del hardware.
        this.driftStrategy = StdDevStrategies.vioTimeDrift(0.02, 0.035, 0.002, 0.0002);
        setStdDevsStrategy(driftStrategy);
    }

    /**
     * Margen alrededor del campo dentro del cual una pose sigue siendo valida.
     *
     * @param meters margen en metros.
     * @return esta misma fuente.
     */
    public QuestNavPoseSource withFieldMargin(double meters) {
        this.fieldMarginMeters = meters;
        return this;
    }

    /**
     * Controla si se descartan los frames sin tracking valido. Apagarlo casi nunca es buena idea:
     * durante una perdida de tracking el headset repite la ultima pose buena, que enseguida deja de
     * describir donde esta el robot.
     *
     * @param enabled true para exigir tracking.
     * @return esta misma fuente.
     */
    public QuestNavPoseSource withRequireTracking(boolean enabled) {
        this.requireTracking = enabled;
        return this;
    }

    /**
     * Pone una estrategia de desviaciones propia, sustituyendo a la de deriva.
     *
     * @param strategy estrategia de {@link StdDevStrategies}.
     * @return esta misma fuente.
     */
    public QuestNavPoseSource withStdDevsStrategy(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        setStdDevsStrategy(strategy);
        return this;
    }

    /**
     * Avisa de que el headset acaba de ser reanclado y su deriva volvio a cero.
     *
     * <p>Llamalo justo despues de {@code questNav.setPose(...)}. Sin esto la estrategia de deriva
     * sigue inflando la incertidumbre para siempre y el estimator termina ignorando un sensor que
     * acaba de quedar perfecto.
     */
    public void notifyPoseReset() {
        driftStrategy.reset();
    }

    /**
     * Reancla el headset y avisa a la estrategia de deriva, en un solo paso.
     *
     * @param robotPose pose del robot en el campo que el headset deberia reportar.
     */
    public void resetPose(Pose2d robotPose) {
        questNav.setPose(new Pose3d(robotPose).transformBy(robotToHeadset));
        notifyPoseReset();
    }

    /** @return la instancia de QuestNav que se esta usando. */
    public QuestNav getQuestNav() {
        return questNav;
    }

    /** @return segundos desde la ultima correccion del headset. */
    public double getSecondsSinceReset() {
        return driftStrategy.elapsedSeconds();
    }

    /**
     * Llamalo cada ciclo, junto al resto del robot code. Es lo que le permite a {@code QuestNav}
     * atender las respuestas de sus comandos.
     */
    public void periodic() {
        questNav.commandPeriodic();
    }

    @Override
    public Optional<PoseSample> getMeasurement() {
        if (!questNav.isConnected()) {
            // Se reporta la latencia medida y no el umbral: el criterio exacto lo decide
            // questnavlib, y afirmar un limite que no es el que aplica confunde mas que ayudar.
            // Si este numero sale alto de forma sostenida, los frames llegan en rafagas --- mira
            // las opciones de publicacion del simulador, no el headset.
            detail =
                    String.format(
                            "QuestNav lo da por desconectado; el ultimo frame llego hace %.0f ms",
                            questNav.getLatency());
            return Optional.empty();
        }

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) {
            detail = "conectado pero sin frames nuevos este ciclo";
            return Optional.empty();
        }

        PoseFrame newest = null;
        for (int i = frames.length - 1; i >= 0; i--) {
            if (!requireTracking || frames[i].isTracking()) {
                newest = frames[i];
                break;
            }
        }
        if (newest == null) {
            detail = frames.length + " frames, ninguno con tracking valido";
            return Optional.empty();
        }

        if (!isTimestampUsable(newest.dataTimestamp())) {
            detail = "dataTimestamp = " + newest.dataTimestamp() + " s, es el centinela de NT";
            return Optional.empty();
        }

        detail = String.format("frame %d, %.1f s desde el ultimo reset", newest.frameCount(),
                getSecondsSinceReset());

        Pose2d robotPose =
                newest.questPose3d().transformBy(robotToHeadset.inverse()).toPose2d();

        // Ni distancia ni tags: a la estrategia de deriva le dan igual y los ignora.
        return Optional.of(
                new PoseSample(robotPose, newest.dataTimestamp(), computeStdDevs(0.0, 1)));
    }

    /**
     * Detecta el timestamp centinela de NetworkTables.
     *
     * <p>Fechar mal una medicion es peor que no tenerla: el pose estimator la interpola hacia atras
     * contra su historial y corrompe el resultado sin que salte ningun error. Mejor descartar y
     * decir por que.
     */
    private boolean isTimestampUsable(double timestamp) {
        if (timestamp > SENTINEL_TIMESTAMP_SECONDS) {
            return true;
        }
        if (!sentinelWarningIssued) {
            sentinelWarningIssued = true;
            DriverStation.reportWarning(
                    "[QuestNavPoseSource] '"
                            + getName()
                            + "': dataTimestamp vale "
                            + timestamp
                            + " s, que es el centinela de NetworkTables para valores publicados en la"
                            + " misma instancia que los consume. Se descartan las mediciones. En"
                            + " simulacion, enciende QuestNavSimConfig.withNetworkClient(true).",
                    false);
        }
        return false;
    }

    @Override
    public boolean isPoseValid(Pose2d pose) {
        boolean valid = PoseFusion.isInField(pose, fieldMarginMeters);
        if (!valid) {
            // Casi siempre significa lo mismo: nadie ha llamado a resetPose(), asi que el headset
            // sigue reportando en su origen arbitrario y no en el del campo.
            detail =
                    String.format(
                            "pose (%.2f, %.2f) fuera del campo: falta un resetPose()?",
                            pose.getX(), pose.getY());
        }
        return valid;
    }

    @Override
    protected String telemetryDetail() {
        return detail;
    }
}
