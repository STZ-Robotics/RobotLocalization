// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

import gg.questnav.questnav.protos.generated.Commands;
import gg.questnav.questnav.protos.generated.Data;
import gg.questnav.questnav.protos.wpilib.CommandProto;
import gg.questnav.questnav.protos.wpilib.CommandResponseProto;
import gg.questnav.questnav.protos.wpilib.DeviceDataProto;
import gg.questnav.questnav.protos.wpilib.FrameDataProto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * El puente de NetworkTables: publica los topics que publica un Meta Quest con QuestNav, y atiende
 * los comandos que le manda el robot code.
 *
 * <p>Se usan los <b>mismos adaptadores protobuf que usa el vendor</b>
 * ({@link FrameDataProto}, {@link DeviceDataProto}, {@link CommandProto},
 * {@link CommandResponseProto}) en vez de reimplementar la serializacion. Compartir el adaptador es
 * lo que garantiza compatibilidad de wire: si QuestNav cambia el esquema, esto deja de compilar en
 * vez de publicar bytes que el robot code no sabe leer.
 *
 * <h2>Topics</h2>
 * <table>
 *   <caption>Esquema de la tabla {@code QuestNav}</caption>
 *   <tr><th>Topic</th><th>Direccion</th><th>Tipo</th></tr>
 *   <tr><td>{@code frameData}</td><td>headset &rarr; robot</td><td>{@code ProtobufQuestNavFrameData}</td></tr>
 *   <tr><td>{@code deviceData}</td><td>headset &rarr; robot</td><td>{@code ProtobufQuestNavDeviceData}</td></tr>
 *   <tr><td>{@code version}</td><td>headset &rarr; robot</td><td>{@code String}</td></tr>
 *   <tr><td>{@code response}</td><td>headset &rarr; robot</td><td>{@code ProtobufQuestNavCommandResponse}</td></tr>
 *   <tr><td>{@code request}</td><td>robot &rarr; headset</td><td>{@code ProtobufQuestNavCommand}</td></tr>
 * </table>
 */
public class QuestNavNTBridge implements AutoCloseable {

    private final NetworkTable table;
    private final NetworkTableInstance instance;

    private final ProtobufPublisher<Data.ProtobufQuestNavFrameData> framePublisher;
    private final ProtobufPublisher<Data.ProtobufQuestNavDeviceData> devicePublisher;
    private final ProtobufPublisher<Commands.ProtobufQuestNavCommandResponse> responsePublisher;
    private final ProtobufSubscriber<Commands.ProtobufQuestNavCommand> requestSubscriber;
    private final StringPublisher versionPublisher;

    private final StructPublisher<Pose3d> truePosePublisher;
    private final StructPublisher<Pose3d> questPosePublisher;
    private final StructPublisher<Pose3d> reportedRobotPosePublisher;

    // Los mensajes de quickbuf son mutables y se reutilizan a proposito: publicar serializa en el
    // momento, asi que no hace falta una instancia nueva por frame.
    private final Data.ProtobufQuestNavFrameData cachedFrame = Data.ProtobufQuestNavFrameData.newInstance();
    private final Data.ProtobufQuestNavDeviceData cachedDevice = Data.ProtobufQuestNavDeviceData.newInstance();
    private final Commands.ProtobufQuestNavCommandResponse cachedResponse =
            Commands.ProtobufQuestNavCommandResponse.newInstance();

    /**
     * Una peticion del robot code, ya extraida a tipos de Java.
     *
     * <p>Se copia el contenido en cuanto se lee de la cola: los mensajes de quickbuf se reutilizan
     * internamente, y guardarse una referencia es una forma silenciosa de leer basura mas tarde.
     *
     * @param commandId identificador que hay que devolver en la respuesta.
     * @param type      tipo de comando.
     * @param targetPose pose objetivo, presente solo en un POSE_RESET.
     */
    public record Request(int commandId, Commands.QuestNavCommandType type, Optional<Pose3d> targetPose) {}

    /**
     * Crea el puente sobre la instancia global de NetworkTables.
     *
     * @param config configuracion de la camara simulada.
     */
    public QuestNavNTBridge(QuestNavSimConfig config) {
        this(config, NetworkTableInstance.getDefault());
    }

    /**
     * Crea el puente sobre una instancia concreta de NetworkTables.
     *
     * @param config   configuracion de la camara simulada.
     * @param instance instancia de NetworkTables a usar.
     */
    public QuestNavNTBridge(QuestNavSimConfig config, NetworkTableInstance instance) {
        this.instance = instance;
        this.table = instance.getTable(config.tableName());

        // Las opciones del publicador no son decoracion. Por defecto NetworkTables agrupa los
        // valores y los manda cada ~100 ms; un headset real escupe frames a 100 Hz por el cable.
        // Sin esto el robot recibe rafagas cada 100 ms, el 80% de sus ciclos no ve ningun frame
        // nuevo, y QuestNav.isConnected() --- que mide cuanto hace que llego el ultimo --- pasa mas
        // tiempo en false que en true.
        framePublisher =
                table
                        .getProtobufTopic("frameData", new FrameDataProto())
                        .publish(
                                PubSubOption.periodic(0.005),
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true));

        devicePublisher =
                table
                        .getProtobufTopic("deviceData", new DeviceDataProto())
                        .publish(PubSubOption.periodic(0.1));

        responsePublisher =
                table
                        .getProtobufTopic("response", new CommandResponseProto())
                        .publish(PubSubOption.periodic(0.005), PubSubOption.sendAll(true));

        requestSubscriber =
                table
                        .getProtobufTopic("request", new CommandProto())
                        .subscribe(
                                Commands.ProtobufQuestNavCommand.newInstance(),
                                PubSubOption.sendAll(true),
                                PubSubOption.pollStorage(20),
                                PubSubOption.keepDuplicates(true));

        versionPublisher = table.getStringTopic("version").publish();
        versionPublisher.set(config.version());

        NetworkTable telemetry = table.getSubTable("sim");
        truePosePublisher = telemetry.getStructTopic("truePose", Pose3d.struct).publish();
        questPosePublisher = telemetry.getStructTopic("questPose", Pose3d.struct).publish();
        reportedRobotPosePublisher =
                telemetry.getStructTopic("reportedRobotPose", Pose3d.struct).publish();
    }

    /** @return la tabla de NetworkTables sobre la que se publica. */
    public NetworkTable table() {
        return table;
    }

    /**
     * Publica un frame con un timestamp explicito.
     *
     * <p>El timestamp explicito es imprescindible: el headset va a 100 Hz y el ciclo del robot a 50,
     * asi que cada ciclo publica varios frames. Sin separarlos en el tiempo, el robot code los veria
     * como simultaneos y {@code PoseFrame.dataTimestamp()} --- que sale del reloj de NetworkTables,
     * no del protobuf --- quedaria inservible para fechar mediciones.
     *
     * @param frame frame a publicar.
     */
    public void publishFrame(QuestNavFrame frame) {
        cachedFrame.clear();
        cachedFrame
                .setFrameCount(frame.frameCount())
                .setTimestamp(frame.appTimestampSeconds())
                .setIsTracking(frame.tracking());
        Pose3d.proto.pack(cachedFrame.getMutablePose3D(), frame.questPose());

        framePublisher.set(cachedFrame, frame.ntTimestampMicros());
    }

    /**
     * Publica el estado del dispositivo.
     *
     * @param batteryPercent      carga de bateria.
     * @param trackingLostCounter cuantas veces se ha perdido el tracking.
     */
    public void publishDeviceData(int batteryPercent, int trackingLostCounter) {
        cachedDevice.clear();
        cachedDevice.setBatteryPercent(batteryPercent).setTrackingLostCounter(trackingLostCounter);
        devicePublisher.set(cachedDevice);
    }

    /**
     * Publica las poses de telemetria que dibuja AdvantageScope. No forman parte del protocolo de
     * QuestNav: van en la subtabla {@code sim} para que quede claro que son nuestras.
     *
     * @param truePose          pose real del robot.
     * @param questPose         pose que el headset esta reportando.
     * @param reportedRobotPose pose de robot que se deduce de lo que reporta el headset.
     */
    public void publishTelemetry(Pose3d truePose, Pose3d questPose, Pose3d reportedRobotPose) {
        truePosePublisher.set(truePose);
        questPosePublisher.set(questPose);
        reportedRobotPosePublisher.set(reportedRobotPose);
    }

    /**
     * Lee los comandos que el robot code publico desde la ultima llamada.
     *
     * @return las peticiones pendientes, en orden de llegada.
     */
    public List<Request> readRequests() {
        List<Request> requests = new ArrayList<>();
        for (var timestamped : requestSubscriber.readQueue()) {
            Commands.ProtobufQuestNavCommand command = timestamped.value;
            if (command == null || !command.hasCommandId()) {
                continue;
            }

            Optional<Pose3d> target = Optional.empty();
            if (command.hasPoseResetPayload() && command.getPoseResetPayload().hasTargetPose()) {
                target = Optional.of(Pose3d.proto.unpack(command.getPoseResetPayload().getTargetPose()));
            }

            requests.add(new Request(command.getCommandId(), command.getType(), target));
        }
        return requests;
    }

    /**
     * Empuja al cable lo publicado en este ciclo, sin esperar al temporizador de NetworkTables.
     *
     * <p>Solo hace falta cuando se publica como cliente de red: ahi los valores cruzan un socket, y
     * dejarlos en el buffer hasta el siguiente flush convierte 100 Hz de frames en rafagas de 10 Hz.
     */
    public void flush() {
        instance.flush();
    }

    /**
     * Contesta a un comando. El robot code empareja la respuesta con la peticion por el
     * {@code commandId}.
     *
     * @param commandId    identificador de la peticion.
     * @param success      si el comando se pudo ejecutar.
     * @param errorMessage motivo del fallo, vacio si tuvo exito.
     */
    public void publishResponse(int commandId, boolean success, String errorMessage) {
        cachedResponse.clear();
        cachedResponse.setCommandId(commandId).setSuccess(success).setErrorMessage(errorMessage);
        responsePublisher.set(cachedResponse);
    }

    /** Cierra los publicadores y el suscriptor. */
    @Override
    public void close() {
        framePublisher.close();
        devicePublisher.close();
        responsePublisher.close();
        requestSubscriber.close();
        versionPublisher.close();
        truePosePublisher.close();
        questPosePublisher.close();
        reportedRobotPosePublisher.close();
    }
}
