// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import com.stzteam.features.robotlocalization.vio.sim.SimulatedQuestNav;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeSet;

/**
 * Publishes the simulated world as a 3D field, to view in AdvantageScope or in MARS Desktop.
 *
 * <p>PhotonVision already maintains a debug {@code Field2d}, but a flat one: it loses the height of
 * the tags and the tilt of the cameras, which is exactly what you want to look at when you are
 * fighting a mount. This class publishes the same thing in 3D, as {@code struct:Pose3d[]} topics,
 * which is what three-dimensional field viewers consume.
 *
 * <h2>What it publishes</h2>
 * Under {@code VisionSim/<world>/}:
 *
 * <table>
 *   <caption>Published topics</caption>
 *   <tr><th>Topic</th><th>Type</th><th>What it is</th></tr>
 *   <tr><td>{@code robot}</td><td>{@code struct:Pose3d}</td><td>the robot's real pose</td></tr>
 *   <tr><td>{@code cameras}</td><td>{@code struct:Pose3d[]}</td><td>where each camera is</td></tr>
 *   <tr><td>{@code fieldTags}</td><td>{@code struct:Pose3d[]}</td><td>every AprilTag on the field</td></tr>
 *   <tr><td>{@code visionTargets}</td><td>{@code struct:Pose3d[]}</td><td>only the ones some camera is seeing</td></tr>
 * </table>
 *
 * <p>The names carry {@code tag}, {@code target} or {@code vision} on purpose: MARS Desktop's 3D
 * viewer detects from those words alone that a pose topic is a vision one and draws it with the right
 * icon, with nothing to configure.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * private final VisionSimTelemetry telemetry = VisionSimTelemetry.forWorld("main");
 *
 * @Override
 * public void simulationPeriodic() {
 *     SimulatedLimelight.periodicAll(pose);
 *     telemetry.publish();
 * }
 * }</pre>
 */
public class VisionSimTelemetry {

    private static final Map<String, VisionSimTelemetry> INSTANCES = new LinkedHashMap<>();
    private static final Pose3d[] EMPTY = new Pose3d[0];

    private final VisionSimWorld world;
    private final StructPublisher<Pose3d> robotPublisher;
    private final StructArrayPublisher<Pose3d> cameraPublisher;
    private final StructArrayPublisher<Pose3d> fieldTagPublisher;
    private final StructArrayPublisher<Pose3d> visionTargetPublisher;

    private boolean fieldTagsPublished = false;

    private VisionSimTelemetry(VisionSimWorld world, NetworkTableInstance instance) {
        this.world = world;
        NetworkTable table = instance.getTable("VisionSim").getSubTable(world.name());

        robotPublisher = table.getStructTopic("robot", Pose3d.struct).publish();
        cameraPublisher = table.getStructArrayTopic("cameras", Pose3d.struct).publish();
        fieldTagPublisher = table.getStructArrayTopic("fieldTags", Pose3d.struct).publish();
        visionTargetPublisher =
                table.getStructArrayTopic("visionTargets", Pose3d.struct).publish();

        // Photonlib's flat Field2d is still useful for a quick top-down view in Glass.
        SmartDashboard.putData("VisionSim/" + world.name() + "/field2d", world.debugField());
    }

    /**
     * Returns a world's telemetry, creating it the first time.
     *
     * @param worldName the simulated world's name, the same one the cameras use.
     * @return that world's telemetry.
     */
    public static synchronized VisionSimTelemetry forWorld(String worldName) {
        return forWorld(worldName, NetworkTableInstance.getDefault());
    }

    /**
     * Returns a world's telemetry on a specific NetworkTables instance.
     *
     * @param worldName the simulated world's name.
     * @param instance  the NetworkTables instance.
     * @return that world's telemetry.
     */
    public static synchronized VisionSimTelemetry forWorld(
            String worldName, NetworkTableInstance instance) {
        return INSTANCES.computeIfAbsent(
                worldName,
                name ->
                        new VisionSimTelemetry(
                                // If the world already exists because a camera created it, its layout
                                // wins and this one is ignored. Otherwise it is created empty with the
                                // default field and fills up when the first camera shows up.
                                VisionSimWorld.get(name, FieldLayouts.defaultField()),
                                instance));
    }

    /**
     * Publishes every world that has telemetry created.
     *
     * @return how many worlds were published.
     */
    public static synchronized int publishAll() {
        INSTANCES.values().forEach(VisionSimTelemetry::publish);
        return INSTANCES.size();
    }

    /** Forgets every telemetry created. Meant for tests. */
    public static synchronized void reset() {
        INSTANCES.values().forEach(VisionSimTelemetry::close);
        INSTANCES.clear();
    }

    /** Publishes the world's current state. Call it once per simulation cycle. */
    public void publish() {
        robotPublisher.set(world.sim().getRobotPose());
        cameraPublisher.set(cameraPoses());

        // The field tags do not move: publishing them once is enough and saves an array per cycle.
        if (!fieldTagsPublished) {
            fieldTagPublisher.set(fieldTagPoses());
            fieldTagsPublished = true;
        }

        visionTargetPublisher.set(visibleTagPoses());
    }

    private Pose3d[] cameraPoses() {
        List<Pose3d> poses = new ArrayList<>();
        for (PhotonCameraSim camera : world.sim().getCameraSims()) {
            world.sim().getCameraPose(camera).ifPresent(poses::add);
        }
        return poses.toArray(EMPTY);
    }

    private Pose3d[] fieldTagPoses() {
        List<Pose3d> poses = new ArrayList<>();
        for (VisionTargetSim target : world.sim().getVisionTargets()) {
            poses.add(target.getPose());
        }
        return poses.toArray(EMPTY);
    }

    /**
     * Poses of the tags some camera in this world is seeing right now.
     *
     * <p>They are reconstructed from the ids each simulated camera reports and not from photonlib's
     * {@code Field2d}, which only keeps flat poses. The union is taken over a sorted set so two
     * cameras looking at the same tag do not draw it twice and so the order does not jump between
     * cycles.
     */
    private Pose3d[] visibleTagPoses() {
        TreeSet<Integer> visibleIds = new TreeSet<>();

        for (SimulatedLimelight limelight : SimulatedLimelight.instances()) {
            if (limelight.world() != world) {
                continue;
            }
            LimelightFrame frame = limelight.lastFrame();
            if (frame == null || !frame.hasTarget()) {
                continue;
            }
            for (LimelightFrame.RawFiducial fiducial : frame.fiducials()) {
                visibleIds.add(fiducial.id());
            }
        }

        if (visibleIds.isEmpty()) {
            return EMPTY;
        }

        Map<Integer, Pose3d> byId = new LinkedHashMap<>();
        for (VisionTargetSim target : world.sim().getVisionTargets()) {
            if (target.fiducialID >= 0) {
                byId.put(target.fiducialID, target.getPose());
            }
        }

        List<Pose3d> poses = new ArrayList<>(visibleIds.size());
        for (int id : visibleIds) {
            Optional.ofNullable(byId.get(id)).ifPresent(poses::add);
        }
        return poses.toArray(EMPTY);
    }

    /** @return the world it is publishing. */
    public VisionSimWorld world() {
        return world;
    }

    /** Closes the publishers. */
    public void close() {
        robotPublisher.close();
        cameraPublisher.close();
        fieldTagPublisher.close();
        visionTargetPublisher.close();
    }

    /**
     * A shortcut to publish everything simulated in a cycle: advances cameras and headsets with the
     * given pose and publishes the 3D telemetry.
     *
     * @param simulatedRobotPose the robot's simulated pose.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        SimulatedLimelight.periodicAll(simulatedRobotPose);
        SimulatedQuestNav.periodicAll(simulatedRobotPose);
        publishAll();
    }
}
