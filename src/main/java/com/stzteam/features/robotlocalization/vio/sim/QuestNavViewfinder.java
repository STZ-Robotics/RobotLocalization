// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import com.stzteam.features.robotlocalization.vision.sim.VisionSimWorld;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/**
 * Un visor de "que esta mirando el headset", para depurar montaje y oclusiones.
 *
 * <p>Monta una {@code PhotonCameraSim} donde va el Quest y le pide a photonlib que renderice el
 * wireframe del campo y los AprilTags desde ese punto de vista. El resultado sale por
 * {@code CameraServer} como un stream MJPEG llamado <code>&lt;nombre&gt;-raw</code>, visible en
 * Glass, Shuffleboard o AdvantageScope.
 *
 * <h2>Es cosmetico, y conviene tenerlo claro</h2>
 * Un Quest <b>no localiza viendo AprilTags</b>: hace odometria visual-inercial sobre las features
 * naturales del entorno. Esta imagen no alimenta nada; la pose sale entera de
 * {@link VioErrorModel}. Sirve para ver si el headset queda mirando a la pared, si el bumper le
 * tapa media vista o si el angulo de montaje es el que creias, y para nada mas.
 *
 * <p>Va apagado por defecto porque renderiza con OpenCV en cada frame y eso cuesta CPU de verdad.
 *
 * <p>Nota: Gazebo tampoco puede dar esta imagen hoy. El protocolo 0.1.0 de SimulationStudio deja
 * camaras y vision explicitamente fuera de alcance, porque requieren {@code gz-sensors} con
 * rendering. Asi que por ahora este visor es la unica forma de ver algo.
 */
public class QuestNavViewfinder implements AutoCloseable {

    private final VisionSimWorld world;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;

    /**
     * Crea el visor y lo engancha al mundo simulado compartido.
     *
     * @param name           nombre del headset; el stream se llama <code>&lt;nombre&gt;-raw</code>.
     * @param robotToHeadset transformada del origen del robot al headset.
     * @param config         configuracion, de donde salen el mundo y el layout del campo.
     */
    public QuestNavViewfinder(String name, Transform3d robotToHeadset, QuestNavSimConfig config) {
        world = VisionSimWorld.get(config.visionWorldName(), config.fieldLayout());

        // Resolucion y FPS bajos a proposito: esto es un visor, no un sensor. Renderizar a 1280x800
        // a 90 FPS para una imagen que nadie mide seria tirar CPU.
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(640, 480, Rotation2d.fromDegrees(100.0));
        properties.setFPS(30);

        camera = new PhotonCamera(name + "-view");
        // El layout va explicito: el constructor de dos argumentos le mete el campo por defecto de
        // WPILib, que en 2026.1.1 sigue siendo el de 2025.
        cameraSim = new PhotonCameraSim(camera, properties, config.fieldLayout());
        cameraSim.enableRawStream(true);
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableProcessedStream(false);

        world.sim().addCamera(cameraSim, robotToHeadset);
    }

    /**
     * Avanza el mundo para que se renderice un frame nuevo.
     *
     * <p>Si ya hay camaras simuladas en el mismo mundo, {@link VisionSimWorld} deduplica la llamada
     * dentro del ciclo, asi que esto no hace trabajo doble.
     *
     * @param truePose pose real del robot.
     */
    public void update(Pose3d truePose) {
        world.update(truePose);
    }

    /**
     * Mueve el visor cuando cambia el montaje del headset.
     *
     * @param robotToHeadset nueva transformada robot-&gt;headset.
     */
    public void setRobotToHeadset(Transform3d robotToHeadset) {
        world.sim().adjustCamera(cameraSim, robotToHeadset);
    }

    /** @return el nombre del stream de CameraServer que publica la imagen cruda. */
    public String streamName() {
        return camera.getName() + "-raw";
    }

    /** Libera la camara de renderizado. */
    @Override
    public void close() {
        world.sim().removeCamera(cameraSim);
        cameraSim.close();
        camera.close();
    }
}
