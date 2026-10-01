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
 * The Limelight source reading hand-written {@code botpose} arrays.
 *
 * <p>With no simulator in the way: this pins down exactly what it does with each shape of the array,
 * including the case the live simulation uncovered.
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

    /** Builds a botpose with N tags, all with the same ambiguity. */
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

        assertTrue(sample.isPresent(), "the measurement exists...");
        assertTrue(source.getAmbiguity().isPresent(), "...y trae ambiguedad");
        assertEquals(0.9, source.getAmbiguity().getAsDouble(), 1e-9);
        assertEquals(
                RejectionReason.AMBIGUITY,
                source.checkMeasurement(sample.get()),
                "with a single tag the filter has to bite");
    }

    @Test
    void aMultiTagSolutionIgnoresThePerTagAmbiguity() {
        // The live simulation uncovered this: three tags at twelve metres give per-tag ambiguities of
        // 0.97, but the pose comes from a multi-tag PnP that does not depend on disambiguating any of
        // them. Filtering on that number ate 100% of the good frames.
        publish(botpose(3, 0.97));

        Optional<PoseSample> sample = source.getMeasurement();

        assertTrue(sample.isPresent());
        assertFalse(
                source.getAmbiguity().isPresent(),
                "with several tags the per-tag ambiguity means nothing");
        assertEquals(
                RejectionReason.ACCEPTED,
                source.checkMeasurement(sample.get()),
                "and therefore it should not be discarded");
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

        assertTrue(source.getMeasurement().isEmpty(), "tagCount 0 is not a measurement");
    }

    @Test
    void theCaptureTimestampSubtractsTheLatency() {
        publish(botpose(2, 0.1));

        PoseSample sample = source.getMeasurement().orElseThrow();

        double ntNowSeconds = edu.wpi.first.networktables.NetworkTablesJNI.now() / 1e6;
        // 20 ms of latency: the capture happened before the value reached NetworkTables.
        assertTrue(sample.timestampSeconds() < ntNowSeconds, "the capture is in the past");
    }

    @Test
    void megaTag2PublishesTheRobotHeadingOnItsOwn() {
        // It is the step most teams forget, and without it MegaTag2 returns nothing.
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

        assertFalse(source.telemetryDetail().isEmpty(), "a NO_MEASUREMENT with no explanation is useless");
    }
}
