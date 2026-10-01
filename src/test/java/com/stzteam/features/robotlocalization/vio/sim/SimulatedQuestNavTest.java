// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * La prueba que importa: instancia el {@link QuestNav} <b>real</b> del vendordep contra nuestro
 * publicador y comprueba que ve lo que veria contra un headset fisico.
 *
 * <p>Si esto pasa, el subsistema de un equipo funciona sin tocar una linea.
 */
class SimulatedQuestNavTest {

    private static final double LOOP_PERIOD = 0.02;

    /** El mount real de STZ. Se usa a proposito en vez de la identidad: un mount girado 90 grados
     * destapa errores de composicion que la identidad esconde. */
    private static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                    new Translation3d(0.0, -0.332, 0.0),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

    private SimulatedQuestNav sim;
    private QuestNav robotSide;
    private Pose2d robotPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(35.0));

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "no se pudo inicializar el HAL");
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        // A proposito no se levanta un servidor NT4 aqui. Se intento para que `serverTime` fuera
        // real, pero ntcore marca con el centinela todo lo publicado en la misma instancia que lo
        // consume, con servidor o sin el --- que es justo lo que fija
        // inLocalModeDataTimestampIsTheNtSentinelAndMustNotBeUsedForFusion. Lo unico que aportaba
        // era chocar con el puerto 5810 cuando hay una simulacion corriendo.

        sim = new SimulatedQuestNav(ROBOT_TO_QUEST, QuestNavSimConfig.perfect());
        robotSide = new QuestNav();
    }

    @AfterEach
    void tearDown() {
        SimulatedQuestNav.closeAll();

        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    @Test
    void theRealVendorLibrarySeesAConnectedTrackingHeadset() {
        runLoops(10);

        assertTrue(robotSide.isConnected(), "el robot code deberia ver el headset conectado");
        assertTrue(robotSide.isTracking(), "y deberia ver tracking valido");
    }

    @Test
    void theConnectionDropsWhenWeStopPublishing() {
        runLoops(10);
        assertTrue(robotSide.isConnected());

        // isConnected() mira si el ultimo frameData llego hace menos de 120 ms.
        sim.injectLinkLoss(1.0);
        runLoops(10);

        assertFalse(robotSide.isConnected(), "sin frames nuevos deberia caerse a los 120 ms");

        runLoops(60);
        assertTrue(robotSide.isConnected(), "y deberia recuperarse cuando vuelve el enlace");
    }

    @Test
    void severalFramesArriveEachLoopWithSpacedTimestamps() {
        // El headset va a 100 Hz y el bucle a 50: cada ciclo tienen que llegar ~2 frames, con
        // timestamps separados 10 ms. Es justo la API que los equipos usan mal mas seguido.
        runLoops(5);
        robotSide.getAllUnreadPoseFrames();

        runLoops(1);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();

        assertEquals(2, frames.length, "un ciclo de 20 ms a 100 Hz son 2 frames");
        assertEquals(
                frames[0].frameCount() + 1,
                frames[1].frameCount(),
                "el contador de frames deberia ser consecutivo");
        assertEquals(
                0.010,
                frames[1].appTimestamp() - frames[0].appTimestamp(),
                1e-9,
                "el reloj del headset deberia avanzar 10 ms entre frames");

    }

    @Test
    void inLocalModeDataTimestampIsTheNtSentinelAndMustNotBeUsedForFusion() {
        // Fija una limitacion real, para que nadie se la encuentre de sorpresa: publicando en la
        // misma instancia de NetworkTables que consume el robot, ntcore marca los valores con
        // serverTime = 1, y PoseFrame.dataTimestamp() --- que sale de ahi --- queda clavado en
        // 1 microsegundo. Para fusionar por timestamp hace falta withNetworkClient(true).
        runLoops(10);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();

        assertEquals(1e-6, frames[0].dataTimestamp(), 1e-12);
        assertEquals(frames[0].dataTimestamp(), frames[frames.length - 1].dataTimestamp(), 1e-12);

        // El reloj de la app si avanza, y es el que queda utilizable en modo local.
        assertTrue(
                frames[frames.length - 1].appTimestamp() > frames[0].appTimestamp(),
                "el reloj del headset deberia avanzar aunque el de NT no");
    }

    @Test
    void withoutErrorTheReportedRobotPoseMatchesTheTruth() {
        runLoops(10);
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        assertTrue(frames.length > 0);

        // Lo que hace el robot code: deshacer el mount sobre lo que reporta el headset.
        Pose3d reportedRobot =
                frames[frames.length - 1].questPose3d().transformBy(ROBOT_TO_QUEST.inverse());

        assertEquals(robotPose.getX(), reportedRobot.getX(), 1e-6);
        assertEquals(robotPose.getY(), reportedRobot.getY(), 1e-6);
        assertEquals(
                robotPose.getRotation().getRadians(), reportedRobot.getRotation().getZ(), 1e-6);
    }

    @Test
    void setPoseFromRobotCodeReanchorsTheHeadsetOrigin() {
        // Sin ruido, para que el reset se pueda comprobar exacto: lo que se prueba aqui es el
        // reanclaje del origen, no el modelo de error.
        sim = replaceSimWith(QuestNavSimConfig.perfect().withRandomizedOrigin(true).withRandomSeed(3));
        runLoops(5);
        robotSide.getAllUnreadPoseFrames();

        // Con el origen del Quest desplazado, lo que reporta NO es una pose de campo...
        Pose3d target = new Pose3d(robotPose).transformBy(ROBOT_TO_QUEST);
        robotSide.setPose(target);
        runLoops(5);

        // ...hasta que el robot code lo reancla, que es exactamente el paso que mas equipos olvidan.
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        Pose3d reported = frames[frames.length - 1].questPose3d();

        assertEquals(target.getX(), reported.getX(), 1e-6);
        assertEquals(target.getY(), reported.getY(), 1e-6);
        assertEquals(target.getRotation().getZ(), reported.getRotation().getZ(), 1e-6);
    }

    @Test
    void forcingTrackingLossFromTheSimGuiIsVisibleToTheRobotCode() {
        runLoops(10);
        assertTrue(robotSide.isTracking());

        SimDeviceSim device = new SimDeviceSim(QuestNavSimDevice.DEVICE_PREFIX + "QuestNav", 0);
        device.getBoolean("Force Tracking Loss").set(true);

        // deviceData va a 3 Hz, asi que el contador de perdidas tarda hasta un tercio de segundo en
        // llegar. Es el comportamiento del hardware, no una lentitud de la simulacion.
        runLoops(25);

        assertFalse(robotSide.isTracking(), "el robot code deberia ver que se perdio el tracking");
        assertTrue(
                robotSide.getTrackingLostCounter().orElse(0) > 0,
                "y el contador de perdidas deberia haber subido");
    }

    @Test
    void aPoseDiscontinuityIsInvisibleToTheRobotCode() {
        // El fallo mas traicionero del Quest: el doble-tap a passthrough que dispara la vibracion
        // del chasis mueve la pose de golpe sin tocar isTracking. El robot code no tiene ninguna
        // senal que mirar, y un pose estimator que confia 2 cm se lo traga entero.
        runLoops(10);
        robotSide.getAllUnreadPoseFrames();
        runLoops(5);
        Pose3d before = latestQuestPose();

        sim.injectPoseDiscontinuity(new Translation3d(0.4, 0.0, 0.0));
        runLoops(5);
        Pose3d after = latestQuestPose();

        assertEquals(
                0.4,
                after.getTranslation().getDistance(before.getTranslation()),
                1e-6,
                "la pose deberia haber saltado");
        assertTrue(robotSide.isTracking(), "y nada en el protocolo lo delata");
    }

    @Test
    void theHeadsetAppearsInTheSimGuiAsAnOtherDevice() {
        SimDeviceSim device = new SimDeviceSim(QuestNavSimDevice.DEVICE_PREFIX + "QuestNav", 0);

        runLoops(10);

        assertTrue(device.getBoolean("isTracking").get());
        assertTrue(device.getDouble("Frame Count").get() > 0.0);
        assertEquals(100.0, device.getDouble("Battery (%)").get(), 1e-9);
    }

    @Test
    void telemetryPosesArePublishedForAdvantageScope() {
        runLoops(10);

        var telemetry = NetworkTableInstance.getDefault().getTable("QuestNav").getSubTable("sim");
        Pose3d truePose =
                telemetry.getStructTopic("truePose", Pose3d.struct).subscribe(Pose3d.kZero).get();

        assertEquals(robotPose.getX(), truePose.getX(), 1e-9);
        assertEquals(robotPose.getY(), truePose.getY(), 1e-9);
    }

    /** Recrea el simulador con otra configuracion, manteniendo el robot code del test. */
    private SimulatedQuestNav replaceSimWith(QuestNavSimConfig config) {
        SimulatedQuestNav.closeAll();
        SimDeviceSim.resetData();
        return new SimulatedQuestNav(ROBOT_TO_QUEST, config);
    }

    private Pose3d latestQuestPose() {
        PoseFrame[] frames = robotSide.getAllUnreadPoseFrames();
        assertTrue(frames.length > 0, "deberia haber frames");
        return frames[frames.length - 1].questPose3d();
    }

    /** Avanza el reloj simulado y corre el ciclo, como haria un TimedRobot. */
    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            sim.periodic(robotPose);
            robotSide.commandPeriodic();
        }
    }
}
