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
 * Publica el mundo simulado como campo 3D, para verlo en AdvantageScope o en MARS Desktop.
 *
 * <p>PhotonVision ya mantiene un {@code Field2d} de depuracion, pero plano: pierde la altura de los
 * tags y la inclinacion de las camaras, que es justo lo que uno quiere mirar cuando esta peleando
 * con un montaje. Esta clase publica lo mismo en 3D, como topics {@code struct:Pose3d[]}, que es lo
 * que consumen los visores de campo tridimensional.
 *
 * <h2>Que publica</h2>
 * Bajo {@code VisionSim/<mundo>/}:
 *
 * <table>
 *   <caption>Topics publicados</caption>
 *   <tr><th>Topic</th><th>Tipo</th><th>Que es</th></tr>
 *   <tr><td>{@code robot}</td><td>{@code struct:Pose3d}</td><td>pose real del robot</td></tr>
 *   <tr><td>{@code cameras}</td><td>{@code struct:Pose3d[]}</td><td>donde esta cada camara</td></tr>
 *   <tr><td>{@code fieldTags}</td><td>{@code struct:Pose3d[]}</td><td>todos los AprilTags del campo</td></tr>
 *   <tr><td>{@code visionTargets}</td><td>{@code struct:Pose3d[]}</td><td>solo los que alguna camara esta viendo</td></tr>
 * </table>
 *
 * <p>Los nombres llevan {@code tag}, {@code target} o {@code vision} a proposito: el visor 3D de
 * MARS Desktop detecta solo por esas palabras que un topic de poses es de vision y lo dibuja con el
 * icono correcto, sin configurar nada.
 *
 * <h2>Uso</h2>
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

        // El Field2d plano de photonlib sigue siendo util para una vista cenital rapida en Glass.
        SmartDashboard.putData("VisionSim/" + world.name() + "/field2d", world.debugField());
    }

    /**
     * Devuelve la telemetria de un mundo, creandola la primera vez.
     *
     * @param worldName nombre del mundo simulado, el mismo que usan las camaras.
     * @return la telemetria de ese mundo.
     */
    public static synchronized VisionSimTelemetry forWorld(String worldName) {
        return forWorld(worldName, NetworkTableInstance.getDefault());
    }

    /**
     * Devuelve la telemetria de un mundo sobre una instancia concreta de NetworkTables.
     *
     * @param worldName nombre del mundo simulado.
     * @param instance  instancia de NetworkTables.
     * @return la telemetria de ese mundo.
     */
    public static synchronized VisionSimTelemetry forWorld(
            String worldName, NetworkTableInstance instance) {
        return INSTANCES.computeIfAbsent(
                worldName,
                name ->
                        new VisionSimTelemetry(
                                // Si el mundo ya existe porque lo creo una camara, manda su layout y
                                // este se ignora. Si no, se crea vacio con el campo por defecto y se
                                // llena cuando aparezca la primera camara.
                                VisionSimWorld.get(name, FieldLayouts.defaultField()),
                                instance));
    }

    /**
     * Publica todos los mundos que tengan telemetria creada.
     *
     * @return cuantos mundos se publicaron.
     */
    public static synchronized int publishAll() {
        INSTANCES.values().forEach(VisionSimTelemetry::publish);
        return INSTANCES.size();
    }

    /** Olvida todas las telemetrias creadas. Pensado para tests. */
    public static synchronized void reset() {
        INSTANCES.values().forEach(VisionSimTelemetry::close);
        INSTANCES.clear();
    }

    /** Publica el estado actual del mundo. Llamalo una vez por ciclo de simulacion. */
    public void publish() {
        robotPublisher.set(world.sim().getRobotPose());
        cameraPublisher.set(cameraPoses());

        // Los tags del campo no se mueven: publicarlos una vez basta y ahorra un array por ciclo.
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
     * Poses de los tags que alguna camara de este mundo esta viendo ahora mismo.
     *
     * <p>Se reconstruyen desde los ids que reporta cada camara simulada y no desde el
     * {@code Field2d} de photonlib, que solo guarda poses planas. La union se hace sobre un conjunto
     * ordenado para que dos camaras que miran el mismo tag no lo dibujen dos veces y para que el
     * orden no baile entre ciclos.
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

    /** @return el mundo que esta publicando. */
    public VisionSimWorld world() {
        return world;
    }

    /** Cierra los publicadores. */
    public void close() {
        robotPublisher.close();
        cameraPublisher.close();
        fieldTagPublisher.close();
        visionTargetPublisher.close();
    }

    /**
     * Atajo para publicar todo lo simulado de un ciclo: avanza camaras y headsets con la pose dada y
     * publica la telemetria 3D.
     *
     * @param simulatedRobotPose pose simulada del robot.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        SimulatedLimelight.periodicAll(simulatedRobotPose);
        SimulatedQuestNav.periodicAll(simulatedRobotPose);
        publishAll();
    }
}
