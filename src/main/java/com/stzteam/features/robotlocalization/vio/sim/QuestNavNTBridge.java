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
 * The NetworkTables bridge: it publishes the topics a Meta Quest running QuestNav publishes, and
 * services the commands the robot code sends it.
 *
 * <p>It uses the <b>same protobuf adapters the vendor uses</b> ({@link FrameDataProto},
 * {@link DeviceDataProto}, {@link CommandProto}, {@link CommandResponseProto}) instead of
 * reimplementing the serialization. Sharing the adapter is what guarantees wire compatibility: if
 * QuestNav changes the schema, this stops compiling instead of publishing bytes the robot code cannot
 * read.
 *
 * <h2>Topics</h2>
 * <table>
 *   <caption>Schema of the {@code QuestNav} table</caption>
 *   <tr><th>Topic</th><th>Direction</th><th>Type</th></tr>
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

    // The quickbuf messages are mutable and reused on purpose: publishing serializes on the spot, so
    // a new instance per frame is unnecessary.
    private final Data.ProtobufQuestNavFrameData cachedFrame = Data.ProtobufQuestNavFrameData.newInstance();
    private final Data.ProtobufQuestNavDeviceData cachedDevice = Data.ProtobufQuestNavDeviceData.newInstance();
    private final Commands.ProtobufQuestNavCommandResponse cachedResponse =
            Commands.ProtobufQuestNavCommandResponse.newInstance();

    /**
     * A request from the robot code, already extracted into Java types.
     *
     * <p>The content is copied as soon as it is read off the queue: the quickbuf messages are reused
     * internally, and holding on to a reference is a silent way to read garbage later.
     *
     * @param commandId the identifier that has to be echoed back in the response.
     * @param type      command type.
     * @param targetPose target pose, present only in a POSE_RESET.
     */
    public record Request(int commandId, Commands.QuestNavCommandType type, Optional<Pose3d> targetPose) {}

    /**
     * Creates the bridge on the global NetworkTables instance.
     *
     * @param config the simulated device's configuration.
     */
    public QuestNavNTBridge(QuestNavSimConfig config) {
        this(config, NetworkTableInstance.getDefault());
    }

    /**
     * Creates the bridge on a specific NetworkTables instance.
     *
     * @param config   the simulated device's configuration.
     * @param instance the NetworkTables instance to use.
     */
    public QuestNavNTBridge(QuestNavSimConfig config, NetworkTableInstance instance) {
        this.instance = instance;
        this.table = instance.getTable(config.tableName());

        // The publisher options are not decoration. By default NetworkTables batches values and
        // sends them every ~100 ms; a real headset spits frames down the wire at 100 Hz. Without
        // this the robot receives bursts every 100 ms, 80% of its cycles see no new frame, and
        // QuestNav.isConnected() --- which measures how long ago the last one arrived --- spends more
        // time false than true.
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

    /** @return the NetworkTables table being published to. */
    public NetworkTable table() {
        return table;
    }

    /**
     * Publishes a frame with an explicit timestamp.
     *
     * <p>The explicit timestamp is essential: the headset runs at 100 Hz and the robot cycle at 50, so
     * every cycle publishes several frames. Without separating them in time, the robot code would see
     * them as simultaneous and {@code PoseFrame.dataTimestamp()} --- which comes from the
     * NetworkTables clock, not from the protobuf --- would be useless for dating measurements.
     *
     * @param frame the frame to publish.
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
     * Publishes the device's state.
     *
     * @param batteryPercent      battery charge.
     * @param trackingLostCounter how many times tracking has been lost.
     */
    public void publishDeviceData(int batteryPercent, int trackingLostCounter) {
        cachedDevice.clear();
        cachedDevice.setBatteryPercent(batteryPercent).setTrackingLostCounter(trackingLostCounter);
        devicePublisher.set(cachedDevice);
    }

    /**
     * Publishes the telemetry poses AdvantageScope draws. They are not part of the QuestNav protocol:
     * they go in the {@code sim} subtable to make clear they are ours.
     *
     * @param truePose          the robot's real pose.
     * @param questPose         the pose the headset is reporting.
     * @param reportedRobotPose the robot pose deduced from what the headset reports.
     */
    public void publishTelemetry(Pose3d truePose, Pose3d questPose, Pose3d reportedRobotPose) {
        truePosePublisher.set(truePose);
        questPosePublisher.set(questPose);
        reportedRobotPosePublisher.set(reportedRobotPose);
    }

    /**
     * Reads the commands the robot code published since the last call.
     *
     * @return the pending requests, in arrival order.
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
     * Pushes what was published this cycle onto the wire, without waiting for the NetworkTables
     * timer.
     *
     * <p>It is only needed when publishing as a network client: there the values cross a socket, and
     * leaving them in the buffer until the next flush turns 100 Hz of frames into 10 Hz bursts.
     */
    public void flush() {
        instance.flush();
    }

    /**
     * Answers a command. The robot code matches the response to the request by {@code commandId}.
     *
     * @param commandId    the request's identifier.
     * @param success      whether the command could be executed.
     * @param errorMessage reason for the failure, empty on success.
     */
    public void publishResponse(int commandId, boolean success, String errorMessage) {
        cachedResponse.clear();
        cachedResponse.setCommandId(commandId).setSuccess(success).setErrorMessage(errorMessage);
        responsePublisher.set(cachedResponse);
    }

    /** Closes the publishers and the subscriber. */
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
