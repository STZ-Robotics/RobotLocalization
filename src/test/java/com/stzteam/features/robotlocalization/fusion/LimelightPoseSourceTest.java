// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.vision.LimelightBotpose;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Optional;

/**
 * La fuente de Limelight leyendo arrays de {@code botpose} escritos a mano.
 *
 * <p>Sin simulador de por medio: aqui se fija exactamente que hace con cada forma del array,
 * incluido el caso que la simulacion viva destapo.
 */
class LimelightPoseSourceTest {

    private static final String CAMERA = "limelight-unit-test";

    private NetworkTableInstance instance;
    private NetworkTable table;
    private LimelightPoseSource source;

    @BeforeEach
    void setUp() {
        instance = NetworkTableInstance.create();
        table = instance.getTable(CAMERA);
        source =
                new LimelightPoseSource(
                        CAMERA,
                        LimelightPoseSource.Mode.MEGATAG_1,
                        new PoseSourceConfig().withMaxAmbiguity(0.3),
                        instance);
        source.bindInputs(LocalizationInputs.NONE);
    }

    @AfterEach
    void tearDown() {
        instance.close();
    }

    /** Construye un botpose con N tags, todos con la misma ambiguedad. */
    private static double[] botpose(int tagCount, double ambiguity) {
        double[] out =
                new double
                        [LimelightBotpose.HEADER_LENGTH
                                + LimelightBotpose.VALUES_PER_FIDUCIAL * tagCount];
        out[0] = 4.0; // x
        out[1] = 3.0; // y
        out[5] = 30.0; // yaw en grados
        out[6] = 20.0; // latencia ms
        out[7] = tagCount;
        out[9] = 2.5; // distancia promedio
        for (int i = 0; i < tagCount; i++) {
            int base = LimelightBotpose.HEADER_LENGTH + i * LimelightBotpose.VALUES_PER_FIDUCIAL;
            out[base] = i + 1;
            out[base + 4] = 2.5;
            out[base + 6] = ambiguity;
        }
        return out;
    }

    private void publish(double[] value) {
        table.getEntry("botpose_wpiblue").setDoubleArray(value);
    }

    @Test
    void aSingleAmbiguousTagIsReportedSoTheGateCanRejectIt() {
        publish(botpose(1, 0.9));

        Optional<PoseSample> sample = source.getMeasurement();

        assertTrue(sample.isPresent(), "la medicion existe...");
        assertTrue(source.getAmbiguity().isPresent(), "...y trae ambiguedad");
        assertEquals(0.9, source.getAmbiguity().getAsDouble(), 1e-9);
        assertEquals(
                RejectionReason.AMBIGUITY,
                source.checkMeasurement(sample.get()),
                "con un solo tag el filtro tiene que morder");
    }

    @Test
    void aMultiTagSolutionIgnoresThePerTagAmbiguity() {
        // Esto lo destapo la simulacion viva: tres tags a doce metros dan ambiguedades de 0.97 por
        // tag, pero la pose sale de un PnP multi-tag que no depende de desambiguar ninguno. Filtrar
        // por ese numero se comia el 100% de los frames buenos.
        publish(botpose(3, 0.97));

        Optional<PoseSample> sample = source.getMeasurement();

        assertTrue(sample.isPresent());
        assertFalse(
                source.getAmbiguity().isPresent(),
                "con varios tags la ambiguedad por tag no significa nada");
        assertEquals(
                RejectionReason.ACCEPTED,
                source.checkMeasurement(sample.get()),
                "y por tanto no deberia descartarse");
    }

    @Test
    void anEmptyBotposeProducesNoMeasurement() {
        publish(new double[0]);

        assertTrue(source.getMeasurement().isEmpty());
    }

    @Test
    void aBotposeWithZeroTagsProducesNoMeasurement() {
        double[] noTags = botpose(0, 0.0);

        publish(noTags);

        assertTrue(source.getMeasurement().isEmpty(), "tagCount 0 no es una medicion");
    }

    @Test
    void theCaptureTimestampSubtractsTheLatency() {
        publish(botpose(2, 0.1));

        PoseSample sample = source.getMeasurement().orElseThrow();

        double ntNowSeconds = edu.wpi.first.networktables.NetworkTablesJNI.now() / 1e6;
        // 20 ms de latencia: la captura ocurrio antes de que el valor llegara a NetworkTables.
        assertTrue(sample.timestampSeconds() < ntNowSeconds, "la captura esta en el pasado");
    }

    @Test
    void megaTag2PublishesTheRobotHeadingOnItsOwn() {
        // Es el paso que mas equipos olvidan, y sin el MegaTag2 no devuelve nada.
        LimelightPoseSource megaTag2 =
                new LimelightPoseSource(
                        CAMERA,
                        LimelightPoseSource.Mode.MEGATAG_2,
                        new PoseSourceConfig(),
                        instance);
        megaTag2.bindInputs(
                new LocalizationInputs(
                        () -> edu.wpi.first.math.geometry.Rotation2d.fromDegrees(42.0),
                        null,
                        null,
                        null,
                        null));

        megaTag2.getMeasurement();

        double[] written = table.getEntry("robot_orientation_set").getDoubleArray(new double[0]);
        assertEquals(6, written.length);
        assertEquals(42.0, written[0], 1e-9);
    }

    @Test
    void theDetailExplainsWhyThereIsNoMeasurement() {
        publish(new double[0]);
        source.getMeasurement();

        assertFalse(source.telemetryDetail().isEmpty(), "un NO_MEASUREMENT sin explicacion es inutil");
    }
}
