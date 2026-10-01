// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Prueba de extremo a extremo: coloca el robot delante de un AprilTag real del campo, corre
 * unos ciclos de simulacion y comprueba que la tabla de NetworkTables queda como la dejaria
 * un Limelight fisico.
 *
 * <p>Es la version automatizada del checklist de validacion: si esto pasa, un subsistema
 * escrito contra {@code LimelightHelpers} funciona sin tocarlo.
 */
class SimulatedLimelightTest {

    private static final String CAMERA = "limelight-under-test";
    private static final int TAG_ID = 1;
    private static final double LOOP_PERIOD = 0.02;
    private static final double DISTANCE_TO_TAG = 3.0;
    private static final double CAMERA_HEIGHT = 0.5;
    private static final double CAMERA_FORWARD = 0.2;

    private AprilTagFieldLayout layout;
    private Pose2d robotPose;
    private SimulatedLimelight limelight;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "no se pudo inicializar el HAL");
        // Los SimDevice se registran por nombre y HAL_CreateSimDevice devuelve null si ya
        // existe uno igual, asi que hay que limpiar el registro entre casos.
        SimDeviceSim.resetData();
        clearRobotInputs();
        SimHooks.pauseTiming();

        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        robotPose = poseFacingTag(TAG_ID, DISTANCE_TO_TAG);

        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        new Transform3d(
                                new Translation3d(CAMERA_FORWARD, 0.0, CAMERA_HEIGHT),
                                new Rotation3d(0.0, -pitchTowardsTag(TAG_ID), 0.0)),
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));
    }

    /**
     * Inclinacion que necesita la camara para apuntar al tag desde la distancia del test. Se
     * calcula en vez de hardcodearse para que el test no se rompa cuando cambie el campo y
     * los tags esten a otra altura.
     */
    private double pitchTowardsTag(int tagId) {
        double tagHeight = layout.getTagPose(tagId).orElseThrow().getZ();
        return Math.atan2(tagHeight - CAMERA_HEIGHT, DISTANCE_TO_TAG - CAMERA_FORWARD);
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    @Test
    void publishesACoherentLimelightTableWhileLookingAtATag() {
        runLoops(10);

        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);

        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9, "deberia ver el tag");
        assertEquals(TAG_ID, (int) table.getEntry("tid").getDouble(-1.0));
        assertTrue(table.getEntry("tl").getDouble(0.0) > 0.0, "la latencia deberia ser real");
        assertTrue(
                Math.abs(table.getEntry("tx").getDouble(99.0)) < 10.0,
                "el tag esta casi centrado, tx deberia ser pequeno");

        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        int tagCount = (int) botpose[7];
        assertEquals(11 + 7 * tagCount, botpose.length, "el largo lo exige LimelightHelpers");
        assertEquals(robotPose.getX(), botpose[0], 0.3, "botpose X deberia acercarse a la pose real");
        assertEquals(robotPose.getY(), botpose[1], 0.3, "botpose Y deberia acercarse a la pose real");
    }

    @Test
    void redOriginKeyMirrorsTheBlueOriginKey() {
        runLoops(10);

        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        double[] blue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        double[] red = table.getEntry("botpose_wpired").getDoubleArray(new double[0]);

        assertEquals(layout.getFieldLength() - blue[0], red[0], 1e-6);
        assertEquals(layout.getFieldWidth() - blue[1], red[1], 1e-6);
    }

    @Test
    void appearsInTheSimGuiAsAnOtherDevice() {
        SimDeviceSim device = new SimDeviceSim(LimelightSimDevice.DEVICE_PREFIX + CAMERA, 0);

        runLoops(10);

        assertEquals(true, device.getBoolean("tv").get(), "el SimDevice deberia reflejar tv");
        assertNotEquals(
                0.0, device.getDouble("Botpose X (m)").get(), "el SimDevice deberia reflejar botpose");
    }

    @Test
    void forcingNoTargetsFromTheSimGuiBlindsTheCamera() {
        runLoops(10);
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9);

        new SimDeviceSim(LimelightSimDevice.DEVICE_PREFIX + CAMERA, 0)
                .getBoolean("Force No Targets")
                .set(true);
        runLoops(5);

        assertEquals(0.0, table.getEntry("tv").getDouble(-1.0), 1e-9);
        assertEquals(-1.0, table.getEntry("tid").getDouble(0.0), 1e-9);
        assertEquals(0, table.getEntry("botpose_wpiblue").getDoubleArray(new double[] {1.0}).length);
    }

    @Test
    void megaTag2AppearsOnceTheRobotPublishesItsOrientation() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);

        runLoops(10);
        assertEquals(
                0,
                table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {1.0}).length,
                "sin heading no deberia haber MegaTag2, igual que en hardware real");

        // Equivalente a LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0).
        table.getEntry("robot_orientation_set")
                .setDoubleArray(
                        new double[] {robotPose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        runLoops(10);

        double[] orb = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        assertTrue(orb.length >= 11, "MegaTag2 deberia publicar una pose completa");
        assertEquals(robotPose.getX(), orb[0], 0.3);
        assertEquals(robotPose.getY(), orb[1], 0.3);
    }

    @Test
    void fiducialFiltersFromRobotCodeHideTheTag() {
        runLoops(10);
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), 1e-9);

        // Equivalente a LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[] {99}).
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[] {99.0});
        runLoops(5);

        assertEquals(0.0, table.getEntry("tv").getDouble(-1.0), 1e-9);
    }

    /**
     * Borra las keys que el robot code le escribe a la camara. Los tests comparten la
     * instancia global de NetworkTables, asi que sin esto un filtro puesto por un caso
     * dejaria ciego al siguiente.
     */
    private static void clearRobotInputs() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(CAMERA);
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[0]);
        table.getEntry("robot_orientation_set").setDoubleArray(new double[0]);
        table.getEntry("camerapose_robotspace_set").setDoubleArray(new double[0]);
        table.getEntry("throttle_set").setDouble(0.0);
        table.getEntry("pipeline").setDouble(0.0);
    }

    /** Avanza el reloj simulado y corre el ciclo de la camara, como haria un TimedRobot. */
    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(robotPose);
        }
    }

    /**
     * Construye una pose de robot a {@code distance} metros del frente del tag, mirandolo de
     * cara.
     */
    private Pose2d poseFacingTag(int tagId, double distance) {
        Pose3d tag = layout.getTagPose(tagId).orElseThrow();
        Rotation2d tagYaw = tag.getRotation().toRotation2d();
        Translation2d offset = new Translation2d(distance, 0.0).rotateBy(tagYaw);

        return new Pose2d(
                tag.getTranslation().toTranslation2d().plus(offset),
                tagYaw.plus(Rotation2d.k180deg));
    }
}
