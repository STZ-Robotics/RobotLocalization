// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import gg.questnav.questnav.protos.generated.Commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Un Meta Quest con QuestNav, simulado.
 *
 * <p>Publica en NetworkTables exactamente lo que publica un headset real, asi que el robot code de
 * produccion --- el que usa {@code gg.questnav.questnav.QuestNav} --- funciona sin cambios y sin
 * hardware. Es la contraparte de {@code SimulatedLimelight}: alli hacemos de camara, aqui hacemos
 * de headset.
 *
 * <h2>Uso</h2>
 * <pre>{@code
 * public class Robot extends TimedRobot {
 *     private final SimulatedQuestNav quest =
 *             new SimulatedQuestNav(
 *                     new Transform3d(0, -0.332, 0, new Rotation3d(0, 0, Math.toRadians(-90))),
 *                     new QuestNavSimConfig());
 *
 *     @Override
 *     public void simulationPeriodic() {
 *         quest.periodic(drivetrain.getSimulatedPose());
 *     }
 * }
 * }</pre>
 *
 * El {@code Transform3d} es el mismo {@code ROBOT_TO_QUEST} que le pasas a tu subsistema real.
 *
 * <h2>Lo que hace distinto a una camara simulada</h2>
 * <ul>
 *   <li><b>Siempre reporta algo.</b> Una Limelight ve el tag o no lo ve; un Quest siempre da una
 *       pose, solo que con error. Por eso el contenido de verdad esta en {@link VioErrorModel} y no
 *       en la geometria.</li>
 *   <li><b>Varios frames por ciclo.</b> El headset va a 100 Hz y el robot a 50, asi que cada
 *       {@code periodic()} publica ~2 frames con timestamps separados 10 ms, interpolando la pose
 *       entre medias. Eso es lo que hace que {@code getAllUnreadPoseFrames()} devuelva lo que
 *       devolveria de verdad.</li>
 *   <li><b>Origen propio.</b> El Quest no reporta poses de campo hasta que el robot code llama a
 *       {@code setPose()}. Ver {@link QuestOriginFrame}.</li>
 * </ul>
 *
 * <h2>Cuidado con {@code dataTimestamp()}</h2>
 * Por defecto se publica en la misma instancia de NetworkTables que consume el robot code, y en ese
 * caso {@code PoseFrame.dataTimestamp()} <b>no sirve</b>: vale 1 microsegundo constante. No es un
 * fallo de esta clase sino de la topologia --- ese valor sale del {@code serverTime} que asigna el
 * servidor NT al <i>recibir</i>, y ntcore marca con el centinela {@code serverTime = 1} todo lo que
 * se publica localmente.
 *
 * <p>Si tu robot code fusiona por timestamp, que es el caso normal de un pose estimator, enciende
 * {@link QuestNavSimConfig#withNetworkClient(boolean)}: la simulacion pasa a publicar como cliente
 * NT4 contra el servidor del robot, igual que el headset real, y los timestamps salen correctos.
 * Todo lo demas --- pose, tracking, bateria, comandos --- funciona igual en los dos modos.
 *
 * <h2>En robot real</h2>
 * El constructor detecta {@code RobotBase.isReal()} y no crea nada. {@link #periodic(Pose2d)} es un
 * retorno inmediato.
 */
public class SimulatedQuestNav implements AutoCloseable {

    /** Tope de frames por ciclo, para que un tiron del bucle no dispare una avalancha. */
    private static final int MAX_FRAMES_PER_LOOP = 20;

    private static final List<SimulatedQuestNav> INSTANCES = new ArrayList<>();
    private static final Map<String, Integer> NEXT_INDEX = new HashMap<>();

    private final String name;
    private final QuestNavSimConfig config;
    private final boolean enabled;

    /** Instancia de NetworkTables propia, solo si se publica como cliente de red. */
    private final NetworkTableInstance ownedInstance;

    private final QuestNavNTBridge bridge;
    private final VioErrorModel errorModel;
    private final QuestOriginFrame originFrame;
    private final QuestNavSimDevice simDevice;
    private final QuestNavViewfinder viewfinder;

    private Transform3d robotToHeadset;

    private double startTimeSeconds = Double.NaN;
    private double lastPeriodicSeconds = Double.NaN;
    private double nextFrameSeconds = Double.NaN;
    private double nextDeviceDataSeconds = Double.NaN;
    private Pose3d previousTrueRobotPose;

    private int frameCount = 0;
    private boolean trackingLossLatch = false;
    private QuestNavFrame lastFrame;

    /**
     * Crea un headset simulado con la configuracion por defecto.
     *
     * @param robotToHeadset transformada del origen del robot al headset, en convencion WPILib. Es
     *                       el mismo valor que tu {@code ROBOT_TO_QUEST}.
     */
    public SimulatedQuestNav(Transform3d robotToHeadset) {
        this(robotToHeadset, new QuestNavSimConfig());
    }

    /**
     * Crea un headset simulado.
     *
     * @param robotToHeadset transformada del origen del robot al headset, en convencion WPILib.
     * @param config         configuracion del modelo de error y de lo que se publica.
     */
    public SimulatedQuestNav(Transform3d robotToHeadset, QuestNavSimConfig config) {
        this.name = config.tableName();
        this.config = config;
        this.robotToHeadset = robotToHeadset;
        this.enabled = RobotBase.isSimulation();

        if (!enabled) {
            ownedInstance = null;
            bridge = null;
            errorModel = null;
            originFrame = null;
            simDevice = null;
            viewfinder = null;
            return;
        }

        ownedInstance = config.networkClient() ? createClientInstance(config) : null;
        bridge =
                ownedInstance != null
                        ? new QuestNavNTBridge(config, ownedInstance)
                        : new QuestNavNTBridge(config);
        errorModel = new VioErrorModel(config);
        originFrame =
                config.randomizedOrigin()
                        ? QuestOriginFrame.randomized(
                                config.randomOriginTranslationMeters(),
                                config.randomOriginRotationDegrees(),
                                new Random(config.randomSeed()))
                        : new QuestOriginFrame();
        simDevice =
                config.createSimDevice() ? new QuestNavSimDevice(name, allocateIndex(name)) : null;
        viewfinder =
                config.viewfinder() ? new QuestNavViewfinder(name, robotToHeadset, config) : null;

        synchronized (INSTANCES) {
            INSTANCES.add(this);
        }
    }

    /**
     * Crea la instancia cliente de NetworkTables que reproduce la topologia real: el headset es un
     * cliente NT4 y el robot es el servidor. Es lo que hace que {@code PoseFrame.dataTimestamp()}
     * traiga un {@code serverTime} de verdad. Ver {@link QuestNavSimConfig#withNetworkClient}.
     */
    private static NetworkTableInstance createClientInstance(QuestNavSimConfig config) {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startClient4(config.clientName());
        instance.setServer(config.serverAddress(), NetworkTableInstance.kDefaultPort4);
        return instance;
    }

    private static synchronized int allocateIndex(String name) {
        int index = NEXT_INDEX.getOrDefault(name, 0);
        NEXT_INDEX.put(name, index + 1);
        return index;
    }

    /**
     * Avanza todos los headsets simulados con la misma pose del robot.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public static void periodicAll(Pose2d simulatedRobotPose) {
        periodicAll(new Pose3d(simulatedRobotPose));
    }

    /**
     * Avanza todos los headsets simulados con la misma pose del robot.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        List<SimulatedQuestNav> snapshot;
        synchronized (INSTANCES) {
            snapshot = List.copyOf(INSTANCES);
        }
        for (SimulatedQuestNav quest : snapshot) {
            quest.periodic(simulatedRobotPose);
        }
    }

    /**
     * Devuelve los headsets simulados activos.
     *
     * @return una lista inmutable de instancias.
     */
    public static List<SimulatedQuestNav> instances() {
        synchronized (INSTANCES) {
            return List.copyOf(INSTANCES);
        }
    }

    /**
     * Ciclo del headset. Llamalo desde {@code simulationPeriodic()} con la pose que tu simulacion
     * considera verdadera.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public void periodic(Pose2d simulatedRobotPose) {
        periodic(new Pose3d(simulatedRobotPose));
    }

    /**
     * Ciclo del headset, con pose en 3D.
     *
     * <h2>Que pose pasarle, y cual no</h2>
     * Tiene que ser la <b>verdad</b>: la pose de tu simulacion de fisica, o la odometria pura de
     * ruedas. <b>No</b> la que sale de tu pose estimator.
     *
     * <p>No es una preferencia de estilo. La fisica alimenta al sensor y el sensor al estimador;
     * pasarle al sensor la salida del estimador cierra un lazo que no existe en un robot real. Y
     * tiene un sintoma concreto: un estimator corregido por vision <b>salta</b> en cada ciclo, y
     * esos saltos entran al modelo de error como aceleraciones de miles de m/s2. El detector de
     * impactos los confunde con choques, el Quest pierde el tracking sin parar, y cada
     * re-adquisicion suma su offset hasta que la pose se va del campo.
     *
     * <p>El modelo filtra los saltos aislados con una mediana, pero contra una realimentacion
     * periodica no hay filtro causal que valga. Si no puedes darle una pose limpia, apaga la
     * deteccion automatica con
     * {@link QuestNavSimConfig#withImpactTrackingLoss(boolean)} e inyecta las perdidas a mano.
     *
     * @param simulatedRobotPose pose simulada actual del robot, sin correcciones de vision.
     */
    public void periodic(Pose3d simulatedRobotPose) {
        if (!enabled) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (Double.isNaN(startTimeSeconds)) {
            initialize(now, simulatedRobotPose);
            return;
        }

        applySimDeviceRequests(now);
        handleRobotRequests(simulatedRobotPose, now);

        boolean publishing = errorModel.isPoweredOn() && errorModel.isLinkUp(now);
        emitFrames(simulatedRobotPose, now, publishing);

        if (publishing && now >= nextDeviceDataSeconds) {
            bridge.publishDeviceData(errorModel.batteryPercent(), errorModel.trackingLostCounter());
            nextDeviceDataSeconds = now + 1.0 / config.deviceDataRateHz();
        }

        if (lastFrame != null) {
            if (config.publishTelemetry()) {
                bridge.publishTelemetry(
                        lastFrame.truePose(), lastFrame.questPose(), reportedRobotPose(lastFrame));
            }
            if (simDevice != null) {
                simDevice.update(lastFrame, errorModel, publishing);
            }
        }

        if (viewfinder != null) {
            viewfinder.update(simulatedRobotPose);
        }

        // Publicando por red hay que empujar el buffer: si no, los frames de este ciclo se quedan
        // esperando al temporizador de NetworkTables y llegan en rafagas. El robot mide la conexion
        // por cuanto hace que llego el ultimo frame, asi que una rafaga cada 100 ms lo deja
        // "desconectado" la mayor parte del tiempo.
        if (ownedInstance != null && publishing) {
            bridge.flush();
        }

        previousTrueRobotPose = simulatedRobotPose;
        lastPeriodicSeconds = now;
    }

    private void initialize(double now, Pose3d robotPose) {
        startTimeSeconds = now;
        lastPeriodicSeconds = now;
        previousTrueRobotPose = robotPose;
        nextFrameSeconds = now;
        nextDeviceDataSeconds = now;
    }

    /** Traduce los interruptores del Sim GUI a eventos del modelo de error. */
    private void applySimDeviceRequests(double now) {
        if (simDevice == null) {
            return;
        }

        // Nivel, no flanco: mientras la casilla este marcada, el tracking se mantiene perdido.
        boolean requested = simDevice.forceTrackingLoss();
        if (requested) {
            errorModel.injectTrackingLoss(config.trackingLossSeconds(), now);
            trackingLossLatch = true;
        } else {
            trackingLossLatch = false;
        }

        // Pulso: un solo salto por marca, o la pose brincaria en cada ciclo.
        if (simDevice.forcePoseJump()) {
            errorModel.injectPoseDiscontinuity(
                    new Translation3d(config.reacquireOffsetMeters(), 0.0, 0.0));
            simDevice.clearPoseJumpRequest();
        }

        if (simDevice.forceLinkLoss()) {
            errorModel.injectLinkLoss(0.25, now);
        }
    }

    /** Atiende los comandos que el robot code publico en {@code request}. */
    private void handleRobotRequests(Pose3d robotPose, double now) {
        Pose3d headsetPose = robotPose.transformBy(robotToHeadset);

        for (QuestNavNTBridge.Request request : bridge.readRequests()) {
            if (request.type() != Commands.QuestNavCommandType.POSE_RESET) {
                bridge.publishResponse(
                        request.commandId(), false, "Comando no soportado por la simulacion");
                continue;
            }

            if (request.targetPose().isEmpty()) {
                bridge.publishResponse(request.commandId(), false, "POSE_RESET sin targetPose");
                continue;
            }

            originFrame.reset(headsetPose, request.targetPose().get());
            errorModel.resetDrift();
            bridge.publishResponse(request.commandId(), true, "");
        }
    }

    /**
     * Genera y publica los frames que tocan en este ciclo.
     *
     * <p>El headset corre a su propia frecuencia, no a la del bucle del robot, asi que hay que
     * ponerse al dia: se emite un frame por cada instante pendiente, interpolando la pose verdadera
     * entre la del ciclo anterior y la actual. Sin esa interpolacion los frames intermedios serian
     * copias del mismo valor y el robot code veria una pose escalonada que el hardware real no
     * produce.
     */
    private void emitFrames(Pose3d robotPose, double now, boolean publishing) {
        double period = 1.0 / config.frameRateHz();
        double span = now - lastPeriodicSeconds;
        int emitted = 0;

        while (nextFrameSeconds <= now && emitted < MAX_FRAMES_PER_LOOP) {
            double frameTime = nextFrameSeconds;
            double fraction = span > 0 ? (frameTime - lastPeriodicSeconds) / span : 1.0;
            fraction = Math.max(0.0, Math.min(1.0, fraction));

            Pose3d trueRobotAtFrame = previousTrueRobotPose.interpolate(robotPose, fraction);
            Pose3d trueHeadsetAtFrame = trueRobotAtFrame.transformBy(robotToHeadset);

            Pose3d errored = errorModel.update(trueHeadsetAtFrame, frameTime, period);
            Pose3d questPose = originFrame.toQuestFrame(errored);

            QuestNavFrame frame =
                    new QuestNavFrame(
                            questPose,
                            trueRobotAtFrame,
                            errorModel.isTracking() && !trackingLossLatch,
                            ++frameCount,
                            frameTime - startTimeSeconds,
                            (long) (frameTime * 1e6));

            if (publishing) {
                bridge.publishFrame(frame);
            }
            lastFrame = frame;

            nextFrameSeconds += period;
            emitted++;
        }

        if (emitted >= MAX_FRAMES_PER_LOOP) {
            // El bucle se atraso tanto que ponerse al dia no tiene sentido: mejor saltar adelante
            // que arrastrar una deuda que nunca se paga.
            nextFrameSeconds = now + period;
        }
    }

    private Pose3d reportedRobotPose(QuestNavFrame frame) {
        return frame.questPose().transformBy(robotToHeadset.inverse());
    }

    /**
     * Fuerza una perdida de tracking desde codigo, sin pasar por el Sim GUI.
     *
     * @param durationSeconds cuanto dura.
     */
    public void injectTrackingLoss(double durationSeconds) {
        if (enabled) {
            errorModel.injectTrackingLoss(durationSeconds, Timer.getFPGATimestamp());
        }
    }

    /**
     * Inyecta un salto de pose sin perder el tracking, como el que provoca el doble-tap a
     * passthrough.
     *
     * @param jump desplazamiento en metros.
     */
    public void injectPoseDiscontinuity(Translation3d jump) {
        if (enabled) {
            errorModel.injectPoseDiscontinuity(jump);
        }
    }

    /**
     * Simula un corte de red: se deja de publicar y el robot ve caer {@code isConnected()}.
     *
     * @param durationSeconds cuanto dura el corte.
     */
    public void injectLinkLoss(double durationSeconds) {
        if (enabled) {
            errorModel.injectLinkLoss(durationSeconds, Timer.getFPGATimestamp());
        }
    }

    /**
     * Cambia el montaje del headset en caliente.
     *
     * @param robotToHeadset nueva transformada robot-&gt;headset.
     */
    public void setRobotToHeadset(Transform3d robotToHeadset) {
        this.robotToHeadset = robotToHeadset;
        if (viewfinder != null) {
            viewfinder.setRobotToHeadset(robotToHeadset);
        }
    }

    /** @return el nombre del headset, que es tambien el de la tabla NT. */
    public String name() {
        return name;
    }

    /** @return true si la simulacion esta activa. */
    public boolean isEnabled() {
        return enabled;
    }

    /** @return la transformada robot-&gt;headset vigente. */
    public Transform3d robotToHeadset() {
        return robotToHeadset;
    }

    /** @return el ultimo frame generado, o {@code null} si todavia no hay ninguno. */
    public QuestNavFrame lastFrame() {
        return lastFrame;
    }

    /** @return el modelo de error, para inspeccionarlo o ajustarlo desde un test. */
    public VioErrorModel errorModel() {
        return errorModel;
    }

    /** @return el marco de origen del headset. */
    public QuestOriginFrame originFrame() {
        return originFrame;
    }

    /** @return la configuracion con la que se creo. */
    public QuestNavSimConfig config() {
        return config;
    }

    /** @return el visor, o {@code null} si esta apagado. */
    public QuestNavViewfinder viewfinder() {
        return viewfinder;
    }

    /** Libera los recursos. En robot real no hay nada que liberar. */
    @Override
    public void close() {
        synchronized (INSTANCES) {
            INSTANCES.remove(this);
        }
        if (!enabled) {
            return;
        }
        if (simDevice != null) {
            simDevice.close();
        }
        if (viewfinder != null) {
            viewfinder.close();
        }
        bridge.close();
        if (ownedInstance != null) {
            ownedInstance.stopClient();
            ownedInstance.close();
        }
    }

    /**
     * Indica si la simulacion esta entregando datos al robot.
     *
     * <p>En modo local siempre es true. Publicando como cliente de red, es false hasta que se
     * establece la conexion con el servidor NT4 del robot: si se queda en false, el robot no esta
     * levantando su servidor y no va a llegar ni un frame.
     *
     * @return true si el transporte esta listo.
     */
    public boolean isPublishing() {
        if (!enabled) {
            return false;
        }
        return ownedInstance == null || ownedInstance.isConnected();
    }

    /** Cierra todos los headsets simulados. Pensado para tests. */
    public static void closeAll() {
        List<SimulatedQuestNav> snapshot;
        synchronized (INSTANCES) {
            snapshot = new ArrayList<>(INSTANCES);
        }
        Collections.reverse(snapshot);
        for (SimulatedQuestNav quest : snapshot) {
            quest.close();
        }
        synchronized (SimulatedQuestNav.class) {
            NEXT_INDEX.clear();
        }
    }
}
