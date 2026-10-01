// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.vision.LimelightBotpose;
import com.stzteam.features.robotlocalization.vision.sim.LimelightSimConfig;
import com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight;
import com.stzteam.features.robotlocalization.vision.sim.VisionSimTelemetry;
import com.stzteam.features.robotlocalization.vision.sim.VisionSimWorld;
import com.stzteam.features.robotlocalization.vio.sim.QuestNavSimConfig;
import com.stzteam.features.robotlocalization.vio.sim.SimulatedQuestNav;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Las dos mitades de la feature, juntas: los simuladores producen mediciones y la fusion las
 * consume, sin una sola rama de codigo para simulacion.
 *
 * <p>Es el test que justifica que las dos vivan en el mismo paquete. Si alguien cambia el formato de
 * {@code botpose} en el publicador y se olvida del lector, esto se cae.
 */
class SimulatedSourcesIntegrationTest {

    private static final String CAMERA = "limelight-fusion-test";
    private static final double LOOP_PERIOD = 0.02;
    private static final int TAG_ID = 1;
    private static final double DISTANCE_TO_TAG = 3.0;
    private static final double CAMERA_HEIGHT = 0.5;

    private AprilTagFieldLayout layout;
    private Pose2d robotPose;
    private SimulatedLimelight limelight;
    private PoseFusion fusion;
    private final List<PoseSample> accepted = new ArrayList<>();

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        PoseFusion.setDefaultFieldLayout(layout);
        robotPose = poseFacingTag();
        accepted.clear();
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        SimulatedQuestNav.closeAll();
        VisionSimTelemetry.reset();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    private Pose2d poseFacingTag() {
        return poseFacingTagAt(DISTANCE_TO_TAG);
    }

    private Pose2d poseFacingTagAt(double distanceMeters) {
        Pose3d tag = layout.getTagPose(TAG_ID).orElseThrow();
        Rotation2d tagYaw = tag.getRotation().toRotation2d();
        Translation2d offset = new Translation2d(distanceMeters, 0.0).rotateBy(tagYaw);
        return new Pose2d(
                tag.getTranslation().toTranslation2d().plus(offset), tagYaw.plus(Rotation2d.k180deg));
    }

    private Transform3d robotToCamera() {
        double tagHeight = layout.getTagPose(TAG_ID).orElseThrow().getZ();
        double pitch = Math.atan2(tagHeight - CAMERA_HEIGHT, DISTANCE_TO_TAG - 0.2);
        return new Transform3d(
                new Translation3d(0.2, 0.0, CAMERA_HEIGHT), new Rotation3d(0.0, -pitch, 0.0));
    }

    private void startLimelight() {
        startLimelight(LimelightPoseSource.megaTag1(CAMERA));
    }

    private void startLimelight(LimelightPoseSource source) {
        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        robotToCamera(),
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));

        fusion =
                new PoseFusion(
                                (pose, timestamp, stdDevs) ->
                                        accepted.add(new PoseSample(pose, timestamp, stdDevs)),
                                LocalizationInputs.of(() -> robotPose, ChassisSpeeds::new))
                        .setTelemetryTable("LocalizationIntegration")
                        .registerSource(source);
    }

    /** Una fuente que no filtra por ambiguedad, para los tests que van de otra cosa. */
    private LimelightPoseSource permissiveSource() {
        return new LimelightPoseSource(
                        CAMERA,
                        LimelightPoseSource.Mode.MEGATAG_1,
                        new PoseSourceConfig().withMaxAmbiguity(Double.POSITIVE_INFINITY))
                .withStdDevsStrategy(StdDevStrategies.quadraticContinuous(0.08, 0.16));
    }

    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(robotPose);
            fusion.update();
        }
    }

    /** Lleva al robot a otra pose interpolando, como si condujera. */
    private void driveTo(Pose2d target, int steps) {
        Pose2d start = robotPose;
        for (int i = 1; i <= steps; i++) {
            robotPose = start.interpolate(target, (double) i / steps);
            runLoops(1);
        }
        robotPose = target;
    }

    @Test
    void aSimulatedLimelightFeedsTheFusionAllTheWayToTheEstimator() {
        startLimelight();

        runLoops(15);

        assertFalse(accepted.isEmpty(), "deberia haber llegado alguna medicion");
        PoseSample last = accepted.get(accepted.size() - 1);
        assertEquals(robotPose.getX(), last.pose().getX(), 0.3);
        assertEquals(robotPose.getY(), last.pose().getY(), 0.3);
    }

    @Test
    void theCaptureTimestampIsInThePastAndNotTheReadTime() {
        startLimelight();

        runLoops(15);

        PoseSample last = accepted.get(accepted.size() - 1);
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        assertTrue(last.timestampSeconds() < now, "la captura ocurrio antes de la lectura");
        assertTrue(now - last.timestampSeconds() < 0.5, "pero no hace medio segundo");
    }

    @Test
    void theAmbiguityGateIsActuallyWiredIn() {
        // El gate existia pero no lo llamaba nadie: una medicion con ambiguedad 0.9 pasaba entera.
        // Con el umbral en cero, cualquier solucion de un solo tag --- que siempre tiene dos
        // interpretaciones posibles, y por tanto ambiguedad estrictamente positiva --- se descarta.
        startLimelight(
                new LimelightPoseSource(
                        CAMERA, LimelightPoseSource.Mode.MEGATAG_1,
                        new PoseSourceConfig().withMaxAmbiguity(0.0)));

        runLoops(15);

        assertEquals(
                1.0,
                NetworkTableInstance.getDefault().getTable(CAMERA).getEntry("tv").getDouble(-1),
                "la camara si ve el tag");
        assertEquals(
                RejectionReason.AMBIGUITY,
                fusion.getSource(CAMERA).orElseThrow().getLastReason(),
                "pero el filtro la descarta");
        assertTrue(accepted.isEmpty());
    }

    @Test
    void theStdDevsGrowWithDistanceThroughTheStrategy() {
        startLimelight(permissiveSource());
        runLoops(15);
        double closeStdDev = accepted.get(accepted.size() - 1).stdDevs().get(0, 0);

        // Retroceder conduciendo y no teletransportando: photonlib interpola la pose del robot para
        // fechar cada frame, asi que un salto instantaneo produce unos ciclos de mediciones que no
        // describen ni el sitio viejo ni el nuevo.
        Pose2d farPose = poseFacingTagAt(DISTANCE_TO_TAG + 2.0);
        driveTo(farPose, 25);
        accepted.clear();
        runLoops(15);

        assertFalse(
                accepted.isEmpty(),
                "a 5 m todavia deberia medir; motivo="
                        + fusion.getSource(CAMERA).orElseThrow().getLastReason());
        double farStdDev = accepted.get(accepted.size() - 1).stdDevs().get(0, 0);
        assertTrue(
                farStdDev > closeStdDev,
                "mas lejos deberia inspirar menos confianza: " + closeStdDev + " -> " + farStdDev);
    }

    @Test
    void blindingTheCameraFromTheSimGuiStopsTheMeasurements() {
        startLimelight();
        runLoops(15);
        assertFalse(accepted.isEmpty());

        accepted.clear();
        new SimDeviceSim("Limelight:" + CAMERA, 0).getBoolean("Force No Targets").set(true);
        runLoops(10);

        assertTrue(accepted.isEmpty(), "sin targets no deberia llegar nada");
        assertEquals(
                RejectionReason.NO_MEASUREMENT,
                fusion.getSource(CAMERA).orElseThrow().getLastReason());
    }

    @Test
    void theBotposeFormatRoundTripsBetweenThePublisherAndTheReader() {
        // El formato lo escribe el simulador y lo lee la fusion. Este es el amarre entre los dos.
        startLimelight();
        runLoops(15);

        double[] raw =
                NetworkTableInstance.getDefault()
                        .getTable(CAMERA)
                        .getEntry("botpose_wpiblue")
                        .getDoubleArray(new double[0]);

        var parsed = LimelightBotpose.parse(raw, 1_000_000L, false).orElseThrow();

        assertTrue(parsed.hasTargets());
        assertEquals(
                LimelightBotpose.HEADER_LENGTH
                        + LimelightBotpose.VALUES_PER_FIDUCIAL * parsed.tagCount(),
                raw.length,
                "el largo lo exige el parser de LimelightHelpers");
        assertEquals(parsed.tagCount(), parsed.fiducials().size());
        assertTrue(parsed.worstAmbiguity().isPresent());
        assertTrue(parsed.avgTagDistMeters() > 0.0);
    }

    @Test
    void theVisionSimPublishesThe3dFieldForAdvantageScope() {
        startLimelight();
        VisionSimTelemetry telemetry = VisionSimTelemetry.forWorld("main");

        runLoops(15);
        telemetry.publish();

        var table = NetworkTableInstance.getDefault().getTable("VisionSim").getSubTable("main");

        Pose3d[] fieldTags =
                table.getStructArrayTopic("fieldTags", Pose3d.struct)
                        .subscribe(new Pose3d[0])
                        .get();
        Pose3d[] cameras =
                table.getStructArrayTopic("cameras", Pose3d.struct).subscribe(new Pose3d[0]).get();
        Pose3d[] visible =
                table.getStructArrayTopic("visionTargets", Pose3d.struct)
                        .subscribe(new Pose3d[0])
                        .get();

        assertEquals(layout.getTags().size(), fieldTags.length, "el campo entero en 3D");
        assertEquals(1, cameras.length, "una camara montada");
        assertTrue(visible.length >= 1, "deberia estar viendo al menos un tag");
        assertTrue(fieldTags[0].getZ() > 0.0, "los tags tienen altura, que es el punto de ser 3D");
    }

    @Test
    void aQuestNavSourceRefusesToUseTheSentinelTimestamp() {
        // Publicando en la misma instancia de NetworkTables, dataTimestamp vale 1 microsegundo.
        // Fechar mal una medicion corrompe el estimator sin que salte ningun error, asi que la
        // fuente prefiere no aportar nada y decir por que.
        Transform3d robotToQuest =
                new Transform3d(
                        new Translation3d(0.0, -0.332, 0.0),
                        new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

        SimulatedQuestNav quest =
                new SimulatedQuestNav(robotToQuest, QuestNavSimConfig.perfect());
        QuestNavPoseSource source = QuestNavPoseSource.create("quest", robotToQuest);

        PoseFusion questFusion =
                new PoseFusion(
                                (pose, timestamp, stdDevs) ->
                                        accepted.add(new PoseSample(pose, timestamp, stdDevs)),
                                LocalizationInputs.of(() -> robotPose, ChassisSpeeds::new))
                        .setTelemetryTable("LocalizationQuest")
                        .registerSource(source);

        for (int i = 0; i < 15; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            quest.periodic(robotPose);
            source.periodic();
            questFusion.update();
        }

        assertTrue(accepted.isEmpty(), "mejor ninguna medicion que una mal fechada");
        assertEquals(RejectionReason.NO_MEASUREMENT, source.getLastReason());
    }
}
