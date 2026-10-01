// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.simulation.VisionSystemSim;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Registro de mundos simulados compartidos.
 *
 * <p>Un {@code VisionSystemSim} representa un campo con sus AprilTags y todas las camaras
 * que lo miran. Varias {@link SimulatedLimelight} que declaren el mismo nombre de mundo
 * comparten una sola instancia, que es lo que hace falta para multi-camara: los tags se
 * cargan una vez y la pose del robot se propaga a todas las camaras de golpe.
 *
 * <p>La actualizacion se deduplica dentro de un mismo ciclo, asi que da igual si llamas a
 * {@link SimulatedLimelight#periodic(Pose2d)} en cada camara o a
 * {@link #updateAll(Pose2d)} una sola vez: el mundo avanza una vez por ciclo.
 */
public final class VisionSimWorld {

    /**
     * Ventana de deduplicacion. Dos llamadas dentro de este margen se consideran el mismo
     * ciclo del robot; el periodo tipico de 20 ms lo supera de sobra.
     */
    private static final double SAME_LOOP_WINDOW_SECONDS = 0.001;

    private static final Map<String, VisionSimWorld> WORLDS = new LinkedHashMap<>();

    private final String name;
    private final VisionSystemSim sim;
    private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;

    private VisionSimWorld(String name, AprilTagFieldLayout layout) {
        this.name = name;
        this.sim = new VisionSystemSim(name);
        this.sim.addAprilTags(layout);
    }

    /**
     * Devuelve el mundo con ese nombre, creandolo con el layout dado si es la primera vez.
     *
     * <p>Si el mundo ya existia se ignora el layout: el primero en pedirlo lo define. Eso
     * evita que dos camaras con configuraciones distintas se pisen los tags.
     *
     * @param name   identificador del mundo.
     * @param layout layout de AprilTags a cargar si hay que crearlo.
     * @return el mundo compartido.
     */
    public static synchronized VisionSimWorld get(String name, AprilTagFieldLayout layout) {
        return WORLDS.computeIfAbsent(name, key -> new VisionSimWorld(key, layout));
    }

    /**
     * Avanza todos los mundos registrados con la misma pose del robot. Atajo para el caso
     * normal de un solo robot con una o varias camaras.
     *
     * @param robotPose pose simulada actual del robot.
     */
    public static synchronized void updateAll(Pose2d robotPose) {
        updateAll(new Pose3d(robotPose));
    }

    /**
     * Avanza todos los mundos registrados con la misma pose del robot.
     *
     * @param robotPose pose simulada actual del robot.
     */
    public static synchronized void updateAll(Pose3d robotPose) {
        for (VisionSimWorld world : WORLDS.values()) {
            world.update(robotPose);
        }
    }

    /**
     * Devuelve los mundos registrados hasta ahora.
     *
     * @return la coleccion de mundos, en orden de creacion.
     */
    public static synchronized Collection<VisionSimWorld> all() {
        return WORLDS.values();
    }

    /**
     * Olvida todos los mundos registrados. Pensado para tests, donde cada caso quiere
     * partir de un campo limpio.
     */
    public static synchronized void reset() {
        WORLDS.clear();
    }

    /**
     * Avanza este mundo. Si ya se avanzo en este mismo ciclo, no hace nada.
     *
     * @param robotPose pose simulada actual del robot.
     */
    public void update(Pose3d robotPose) {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateSeconds < SAME_LOOP_WINDOW_SECONDS) {
            return;
        }
        lastUpdateSeconds = now;
        sim.update(robotPose);
    }

    /**
     * Reposiciona el robot en el mundo sin generar frames, util al reiniciar una rutina
     * de autonomo o al colocar el robot desde el Sim GUI.
     *
     * @param robotPose nueva pose del robot.
     */
    public void resetRobotPose(Pose3d robotPose) {
        sim.resetRobotPose(robotPose);
    }

    /** @return el nombre de este mundo. */
    public String name() {
        return name;
    }

    /** @return el {@code VisionSystemSim} subyacente, por si necesitas su API completa. */
    public VisionSystemSim sim() {
        return sim;
    }

    /**
     * Campo de depuracion que photonlib mantiene con la pose del robot, las camaras y los
     * targets visibles. Publicalo en un {@code SmartDashboard} o en Glass para ver que
     * esta mirando la camara.
     *
     * @return el {@code Field2d} de depuracion.
     */
    public Field2d debugField() {
        return sim.getDebugField();
    }
}
