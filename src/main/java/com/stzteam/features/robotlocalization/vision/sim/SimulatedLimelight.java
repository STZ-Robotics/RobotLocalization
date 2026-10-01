// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Una camara Limelight simulada: PhotonVision genera las detecciones y esta clase las
 * publica con el esquema de NetworkTables de un Limelight real.
 *
 * <p>La idea es que el robot code de produccion no cambie ni una linea. Si tu subsistema
 * lee {@code LimelightHelpers.getTX("limelight-front")} contra hardware real, leera lo mismo
 * contra esta simulacion.
 *
 * <h2>Uso</h2>
 * <pre>{@code
 * public class Robot extends TimedRobot {
 *     private final SimulatedLimelight frontCamera =
 *             new SimulatedLimelight(
 *                     "limelight-front",
 *                     new Transform3d(
 *                             new Translation3d(0.3, 0.0, 0.25),
 *                             new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0)),
 *                     LimelightSimConfig.limelight3G());
 *
 *     @Override
 *     public void simulationPeriodic() {
 *         frontCamera.periodic(drivetrain.getSimulatedPose());
 *     }
 * }
 * }</pre>
 *
 * <h2>En robot real</h2>
 * El constructor detecta {@code RobotBase.isReal()} y no crea nada: ni
 * {@code VisionSystemSim}, ni {@code SimDevice}, ni entries de NetworkTables.
 * {@link #periodic(Pose2d)} queda como un retorno inmediato, asi que puedes dejar la
 * instancia declarada sin coste en competencia.
 *
 * <h2>Fuera de alcance por ahora</h2>
 * <ul>
 *   <li>Key {@code json} / {@code LimelightHelpers.getLatestResults}.</li>
 *   <li>Pipelines de detector neuronal ({@code tclass}, {@code rawdetections}).</li>
 *   <li>Modelado de crosshair movible: {@code tx} y {@code txnc} salen iguales.</li>
 * </ul>
 */
public class SimulatedLimelight implements AutoCloseable {

    private static final List<SimulatedLimelight> INSTANCES = new ArrayList<>();
    private static final Map<String, Integer> NEXT_INDEX = new HashMap<>();

    private final String name;
    private final LimelightSimConfig config;
    private final boolean enabled;

    private final VisionSimWorld world;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final LimelightTargetTranslator translator;
    private final LimelightNTPublisher publisher;
    private final LimelightSimDevice simDevice;
    private final Matrix<N3, N3> cameraMatrix;
    private final Matrix<N8, N1> distCoeffs;

    private Transform3d robotToCamera;
    private LimelightFrame lastFrame = LimelightFrame.empty(0.0, 0.0, 0.0);
    private long frameCounter = 0;

    /**
     * Crea una camara simulada con la configuracion por defecto (perfil Limelight 3G,
     * campo por defecto de WPILib).
     *
     * @param name          nombre de la camara; se usa como tabla de NetworkTables y como
     *                      nombre del dispositivo en el Sim GUI.
     * @param robotToCamera transformada del origen del robot a la camara, en convencion
     *                      WPILib (X adelante, Y izquierda, Z arriba).
     */
    public SimulatedLimelight(String name, Transform3d robotToCamera) {
        this(name, robotToCamera, new LimelightSimConfig());
    }

    /**
     * Crea una camara simulada.
     *
     * <p>El {@code name} manda: sobreescribe el {@code tableName} de la configuracion, para
     * que el nombre que ves en el Sim GUI y el de la tabla NT no puedan divergir.
     *
     * @param name          nombre de la camara.
     * @param robotToCamera transformada del origen del robot a la camara, en convencion WPILib.
     * @param config        configuracion de optica, campo y keys a publicar.
     */
    public SimulatedLimelight(String name, Transform3d robotToCamera, LimelightSimConfig config) {
        this.name = name;
        this.config = config.withTableName(name);
        this.robotToCamera = robotToCamera;
        this.enabled = RobotBase.isSimulation();

        if (!enabled) {
            world = null;
            camera = null;
            cameraSim = null;
            translator = null;
            publisher = null;
            simDevice = null;
            cameraMatrix = null;
            distCoeffs = null;
            return;
        }

        world = VisionSimWorld.get(config.visionWorldName(), config.fieldLayout());
        camera = new PhotonCamera(name);
        // El layout va explicito. PhotonCameraSim guarda el suyo para resolver el PnP multi-tag, y
        // el constructor de dos argumentos le mete AprilTagFields.kDefaultField --- que en WPILib
        // 2026.1.1 es el campo de 2025. Con el mundo en 2026 eso significa detectar tags con una
        // geometria y resolverlos contra otra: la pose sale a doce metros de donde esta el robot,
        // sin ningun error por ningun lado.
        cameraSim = new PhotonCameraSim(camera, config.cameraProperties(), config.fieldLayout());
        world.sim().addCamera(cameraSim, robotToCamera);

        // Photonlib por defecto no limita el alcance, asi que la camara simulada ve cualquier tag
        // dentro del FOV por lejos que este y a traves de lo que sea --- VisionSystemSim no modela
        // oclusion. Medido contra el campo 2026: sin limite veia siete tags, cinco de ellos entre 8
        // y 13 m con ambiguedades de 0.6 a 0.99, y el PnP multi-tag los mezclaba con los cercanos y
        // aterrizaba a metro y medio de altura. Un Limelight de verdad no resuelve un tag de 16.5 cm
        // a esa distancia.
        cameraSim.setMaxSightRange(config.maxSightRangeMeters());
        cameraSim.setMinTargetAreaPixels(config.minTargetAreaPixels());

        // El visor. Va apagado por defecto: renderizar con OpenCV en cada frame cuesta CPU de
        // verdad, y la mayoria de las veces lo unico que hace falta son las keys de NT.
        cameraSim.enableRawStream(config.rawStream());
        cameraSim.enableDrawWireframe(config.wireframe());
        cameraSim.enableProcessedStream(config.processedStream());
        if (config.publishDebugField()) {
            SmartDashboard.putData("VisionSim/" + config.visionWorldName(), world.debugField());
        }

        cameraMatrix = config.cameraProperties().getIntrinsics();
        distCoeffs = config.cameraProperties().getDistCoeffs();

        translator =
                new LimelightTargetTranslator(
                        config.fieldLayout(), robotToCamera, config.megaTag2HeadingScaleFactor());
        publisher = new LimelightNTPublisher(this.config);
        simDevice = config.createSimDevice() ? new LimelightSimDevice(name, allocateIndex(name)) : null;

        synchronized (INSTANCES) {
            INSTANCES.add(this);
        }
    }

    private static synchronized int allocateIndex(String name) {
        int index = NEXT_INDEX.getOrDefault(name, 0);
        NEXT_INDEX.put(name, index + 1);
        return index;
    }

    /**
     * Avanza todas las camaras simuladas creadas hasta ahora con la misma pose del robot.
     * Atajo para el caso normal: una llamada en {@code simulationPeriodic()} y listo.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public static void periodicAll(Pose2d simulatedRobotPose) {
        periodicAll(new Pose3d(simulatedRobotPose));
    }

    /**
     * Avanza todas las camaras simuladas creadas hasta ahora con la misma pose del robot.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public static void periodicAll(Pose3d simulatedRobotPose) {
        List<SimulatedLimelight> snapshot;
        synchronized (INSTANCES) {
            snapshot = List.copyOf(INSTANCES);
        }
        for (SimulatedLimelight limelight : snapshot) {
            limelight.periodic(simulatedRobotPose);
        }
    }

    /**
     * Devuelve las camaras simuladas activas.
     *
     * @return una lista inmutable de instancias.
     */
    public static List<SimulatedLimelight> instances() {
        synchronized (INSTANCES) {
            return List.copyOf(INSTANCES);
        }
    }

    /**
     * Ciclo de la camara. Llamalo desde {@code simulationPeriodic()} con la pose que tu
     * odometria o tu simulacion de chasis considera verdadera.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public void periodic(Pose2d simulatedRobotPose) {
        periodic(new Pose3d(simulatedRobotPose));
    }

    /**
     * Ciclo de la camara, con pose en 3D.
     *
     * @param simulatedRobotPose pose simulada actual del robot.
     */
    public void periodic(Pose3d simulatedRobotPose) {
        if (!enabled) {
            return;
        }

        LimelightNTInputs inputs = publisher.readInputs();
        applyCameraPoseOverride(inputs);

        world.update(simulatedRobotPose);

        PhotonPipelineResult result = latestAllowedResult(inputs);
        if (result == null) {
            return;
        }

        LimelightFrame frame =
                translator.translate(result, inputs, cameraMatrix, distCoeffs, config.publishMegaTag2());

        if (simDevice != null && simDevice.forceNoTargets()) {
            frame =
                    LimelightFrame.empty(
                            frame.pipelineLatencyMs(),
                            frame.captureLatencyMs(),
                            frame.captureTimestampSeconds());
        }

        publisher.publish(frame, inputs);
        if (simDevice != null) {
            simDevice.update(frame, inputs);
        }
        lastFrame = frame;
    }

    /**
     * Devuelve el resultado mas reciente que sobrevive al throttle, o {@code null} si este
     * ciclo no trajo ningun frame nuevo.
     *
     * <p>Que no haya frame nuevo es lo normal: la camara corre a su propio FPS, no al del
     * bucle del robot. En ese caso no se toca NetworkTables y los consumidores siguen viendo
     * el ultimo frame publicado, igual que con hardware real.
     */
    private PhotonPipelineResult latestAllowedResult(LimelightNTInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return null;
        }

        int throttle = config.honorThrottle() ? Math.max(0, inputs.throttle()) : 0;
        if (throttle == 0) {
            return results.get(results.size() - 1);
        }

        PhotonPipelineResult kept = null;
        for (PhotonPipelineResult result : results) {
            if (frameCounter % (throttle + 1) == 0) {
                kept = result;
            }
            frameCounter++;
        }
        return kept;
    }

    /** Aplica en caliente el {@code camerapose_robotspace_set} que escribio el robot code. */
    private void applyCameraPoseOverride(LimelightNTInputs inputs) {
        inputs.cameraPoseOverride()
                .filter(override -> !override.equals(robotToCamera))
                .ifPresent(this::setRobotToCamera);
    }

    /**
     * Mueve la camara respecto al robot. Equivale a
     * {@code LimelightHelpers.setCameraPose_RobotSpace}, pero desde codigo de simulacion.
     *
     * @param robotToCamera nueva transformada robot-&gt;camara.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        if (!enabled) {
            return;
        }
        translator.setRobotToCamera(robotToCamera);
        world.sim().adjustCamera(cameraSim, robotToCamera);
    }

    /** @return el nombre de la camara, que es tambien el de la tabla NT. */
    public String name() {
        return name;
    }

    /** @return true si la simulacion esta activa (es decir, no estamos en robot real). */
    public boolean isEnabled() {
        return enabled;
    }

    /** @return la transformada robot-&gt;camara vigente. */
    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    /** @return el ultimo frame traducido, util para telemetria o tests. */
    public LimelightFrame lastFrame() {
        return lastFrame;
    }

    /** @return la configuracion con la que se creo esta camara. */
    public LimelightSimConfig config() {
        return config;
    }

    /** @return el mundo simulado compartido, o {@code null} en robot real. */
    public VisionSimWorld world() {
        return world;
    }

    /** @return la {@code PhotonCamera} subyacente, o {@code null} en robot real. */
    public PhotonCamera camera() {
        return camera;
    }

    /** @return la {@code PhotonCameraSim} subyacente, o {@code null} en robot real. */
    public PhotonCameraSim cameraSim() {
        return cameraSim;
    }

    /**
     * Libera la camara y su {@code SimDevice}. En robot real no hay nada que liberar.
     *
     * <p>El mundo simulado compartido sobrevive: puede haber otras camaras usandolo. Para
     * limpiarlo entero usa {@link VisionSimWorld#reset()}.
     */
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
        world.sim().removeCamera(cameraSim);
        cameraSim.close();
        camera.close();
    }

    /** Cierra todas las camaras simuladas. Pensado para tests. */
    public static void closeAll() {
        List<SimulatedLimelight> snapshot;
        synchronized (INSTANCES) {
            snapshot = new ArrayList<>(INSTANCES);
        }
        Collections.reverse(snapshot);
        for (SimulatedLimelight limelight : snapshot) {
            limelight.close();
        }
        synchronized (SimulatedLimelight.class) {
            NEXT_INDEX.clear();
        }
    }
}
