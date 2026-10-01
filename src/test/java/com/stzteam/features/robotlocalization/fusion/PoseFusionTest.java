// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.forgemini.io.NetworkIO;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

/** El portero completo: filtros, consenso, Mahalanobis y telemetria. */
class PoseFusionTest {

    private static final String TABLE = "LocalizationTest";

    /** Una fuente controlada a mano, para poder provocar cada caso. */
    private static final class FakeSource extends PoseSource {
        private Optional<PoseSample> next = Optional.empty();
        private OptionalDouble ambiguity = OptionalDouble.empty();
        private boolean poseValid = true;
        private RuntimeException toThrow;
        private int readCount = 0;

        FakeSource(String name, PoseSourceConfig config) {
            super(name, VecBuilder.fill(0.1, 0.1, 0.1), config);
        }

        @Override
        public Optional<PoseSample> getMeasurement() {
            readCount++;
            if (toThrow != null) {
                throw toThrow;
            }
            return next;
        }

        @Override
        public boolean isPoseValid(Pose2d pose) {
            return poseValid;
        }

        @Override
        public OptionalDouble getAmbiguity() {
            return ambiguity;
        }

        void report(double x, double timestamp) {
            next = Optional.of(sample(x, timestamp));
        }
    }

    private static PoseSample sample(double x, double timestamp) {
        return new PoseSample(
                new Pose2d(x, 1.0, Rotation2d.kZero), timestamp, VecBuilder.fill(0.1, 0.1, 0.1));
    }

    private final List<PoseSample> accepted = new ArrayList<>();
    private Pose2d estimatorPose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private double angularDegPerSec = 0.0;
    private double accelerationGs = 0.0;

    private PoseFusion fusion;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        accepted.clear();

        LocalizationInputs inputs =
                new LocalizationInputs(
                        () -> Rotation2d.fromDegrees(30.0),
                        () -> DegreesPerSecond.of(angularDegPerSec),
                        () -> speeds,
                        () -> VecBuilder.fill(accelerationGs, 0.0),
                        () -> estimatorPose);

        fusion =
                new PoseFusion(
                                (pose, timestamp, stdDevs) ->
                                        accepted.add(new PoseSample(pose, timestamp, stdDevs)),
                                inputs)
                        .setTelemetryTable(TABLE);
    }

    @AfterEach
    void tearDown() {
        HAL.shutdown();
    }

    private FakeSource register(String name, PoseSourceConfig config) {
        FakeSource source = new FakeSource(name, config);
        fusion.registerSource(source);
        // Deja el interruptor del dashboard en un estado conocido: los tests comparten la instancia
        // global de NetworkTables.
        NetworkIO.set(TABLE, name + "/enabled", true);
        source.setEnabled(true);
        return source;
    }

    @Test
    void aValidMeasurementReachesTheEstimator() {
        FakeSource source = register("a", new PoseSourceConfig());
        source.report(1.2, 10.0);

        fusion.update();

        assertEquals(1, accepted.size());
        assertEquals(1.2, accepted.get(0).pose().getX(), 1e-9);
        assertEquals(RejectionReason.ACCEPTED, source.getLastReason());
    }

    @Test
    void disablingFromCodeSurvivesTheCycle() {
        // El bug: pollDashboardControls() corria al principio de update() y sobrescribia el valor
        // con el de NetworkTables, asi que apagar una fuente desde codigo no duraba ni un ciclo.
        FakeSource source = register("a", new PoseSourceConfig());
        source.report(1.2, 10.0);
        fusion.update();
        assertEquals(1, accepted.size());

        source.setEnabled(false);
        source.report(1.2, 11.0);
        fusion.update();

        assertEquals(1, accepted.size(), "la fuente apagada desde codigo no deberia aportar nada");
        assertFalse(source.isEnabled(), "y deberia seguir apagada");
        assertEquals(RejectionReason.DISABLED, source.getLastReason());
    }

    @Test
    void disablingFromTheDashboardAlsoWins() {
        FakeSource source = register("a", new PoseSourceConfig());
        source.report(1.2, 10.0);
        fusion.update();

        NetworkIO.set(TABLE, "a/enabled", false);
        source.report(1.2, 11.0);
        fusion.update();

        assertEquals(1, accepted.size());
        assertFalse(source.isEnabled());
    }

    @Test
    void spinningTooFastBlocksTheSourceBeforeReadingHardware() {
        FakeSource source =
                register(
                        "a",
                        new PoseSourceConfig().withMaxAngularVelocity(DegreesPerSecond.of(360)));
        source.report(1.2, 10.0);
        angularDegPerSec = 900.0;

        fusion.update();

        assertTrue(accepted.isEmpty());
        assertEquals(RejectionReason.ANGULAR_VELOCITY, source.getLastReason());
        assertEquals(0, source.readCount, "ni siquiera deberia leer el hardware");
    }

    @Test
    void hardAccelerationBlocksTheSource() {
        FakeSource source =
                register("a", new PoseSourceConfig().withMaxAcceleration(Gs.of(2.0)));
        source.report(1.2, 10.0);
        accelerationGs = 5.0;

        fusion.update();

        assertEquals(RejectionReason.ACCELERATION, source.getLastReason());
    }

    @Test
    void drivingTooFastBlocksTheSource() {
        FakeSource source =
                register("a", new PoseSourceConfig().withMaxPlanarVelocity(MetersPerSecond.of(2.0)));
        source.report(1.2, 10.0);
        speeds = new ChassisSpeeds(4.0, 0.0, 0.0);

        fusion.update();

        assertEquals(RejectionReason.PLANAR_VELOCITY, source.getLastReason());
    }

    @Test
    void ambiguousSolutionsAreRejected() {
        // El gate existia pero no lo llamaba nadie: una medicion con ambiguedad 0.9 pasaba entera.
        FakeSource source = register("a", new PoseSourceConfig().withMaxAmbiguity(0.3));
        source.report(1.2, 10.0);
        source.ambiguity = OptionalDouble.of(0.9);

        fusion.update();

        assertTrue(accepted.isEmpty());
        assertEquals(RejectionReason.AMBIGUITY, source.getLastReason());
    }

    @Test
    void sourcesWithoutAmbiguitySkipThatGate() {
        FakeSource source = register("a", new PoseSourceConfig().withMaxAmbiguity(0.3));
        source.report(1.2, 10.0);
        source.ambiguity = OptionalDouble.empty();

        fusion.update();

        assertEquals(1, accepted.size());
    }

    @Test
    void staleMeasurementsAreRejected() {
        FakeSource source = register("a", new PoseSourceConfig());
        source.report(1.2, 10.0);
        fusion.update();

        source.report(1.3, 9.0);
        fusion.update();

        assertEquals(1, accepted.size());
        assertEquals(RejectionReason.STALE_TIMESTAMP, source.getLastReason());
    }

    @Test
    void anInvalidPoseIsRejected() {
        FakeSource source = register("a", new PoseSourceConfig());
        source.report(1.2, 10.0);
        source.poseValid = false;

        fusion.update();

        assertEquals(RejectionReason.POSE_INVALID, source.getLastReason());
    }

    @Test
    void aBrokenSourceDoesNotTakeDownTheOthers() {
        FakeSource broken = register("broken", new PoseSourceConfig());
        FakeSource healthy = register("healthy", new PoseSourceConfig());
        broken.toThrow = new IllegalStateException("camara desconectada");
        healthy.report(1.2, 10.0);

        fusion.update();

        assertEquals(1, accepted.size(), "la sana deberia seguir aportando");
        assertEquals(RejectionReason.SOURCE_ERROR, broken.getLastReason());
        assertEquals(RejectionReason.ACCEPTED, healthy.getLastReason());
    }

    @Test
    void theFirstMeasurementIsAlwaysAcceptedByTheMahalanobisGate() {
        // Antes esto funcionaba por accidente: con lastAcceptedTimestamp en -1, el dt valia decenas
        // de segundos y la varianza crecia tanto que aceptaba cualquier cosa.
        FakeSource source =
                register("a", new PoseSourceConfig().withMahalanobisGate(3.0, 0.05, 0.05, 0.0));
        source.report(50.0, 10.0);

        fusion.update();

        assertEquals(1, accepted.size(), "no hay contra que compararla todavia");
    }

    @Test
    void aWildJumpIsRejectedByTheMahalanobisGate() {
        FakeSource source =
                register("a", new PoseSourceConfig().withMahalanobisGate(3.0, 0.05, 0.05, 0.0));
        source.report(1.0, 10.0);
        fusion.update();

        source.report(50.0, 10.02);
        fusion.update();

        assertEquals(1, accepted.size());
        assertEquals(RejectionReason.MAHALANOBIS, source.getLastReason());
    }

    @Test
    void theMahalanobisGateOpensAfterALongSilence() {
        // Si la puerta no se abriera con el tiempo, un estimator que se fue de sitio no podria
        // volver nunca: todo le pareceria irrazonable para siempre.
        FakeSource source =
                register("a", new PoseSourceConfig().withMahalanobisGate(3.0, 0.05, 0.05, 5.0));
        source.report(1.0, 10.0);
        fusion.update();
        accepted.clear();

        source.report(4.0, 40.0);
        fusion.update();

        assertEquals(1, accepted.size(), "tras 30 s sin datos la incertidumbre justifica el salto");
    }

    @Test
    void twoDisagreeingSourcesAreBothDropped() {
        FakeSource a = register("a", new PoseSourceConfig());
        FakeSource b = register("b", new PoseSourceConfig());
        a.report(1.0, 10.0);
        b.report(9.0, 10.0);

        fusion.update();

        assertTrue(accepted.isEmpty());
        assertEquals(RejectionReason.CONSENSUS, a.getLastReason());
        assertEquals(RejectionReason.CONSENSUS, b.getLastReason());
    }

    @Test
    void consensusCanBeTurnedOff() {
        fusion.setConsensusEnabled(false);
        FakeSource a = register("a", new PoseSourceConfig());
        FakeSource b = register("b", new PoseSourceConfig());
        a.report(1.0, 10.0);
        b.report(9.0, 10.0);

        fusion.update();

        assertEquals(2, accepted.size());
    }

    @Test
    void telemetryNamesTheReasonSoTheProblemIsFindable() {
        FakeSource source =
                register(
                        "a",
                        new PoseSourceConfig().withMaxAngularVelocity(DegreesPerSecond.of(90)));
        source.report(1.2, 10.0);
        angularDegPerSec = 900.0;

        fusion.update();

        assertFalse(NetworkIO.get(TABLE, "a/accepted", true));
        assertEquals(0, fusion.getAcceptedCount());
    }

    @Test
    void updateIsSafeWithNoSourcesAtAll() {
        fusion.update();

        assertEquals(0, fusion.getAcceptedCount());
        assertTrue(accepted.isEmpty());
    }
}
