// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.junit.jupiter.api.Test;

/**
 * Comprueba el modelo de error del VIO, que es el contenido real de la feature. Son tests puros: no
 * hacen falta ni HAL ni NetworkTables.
 */
class VioErrorModelTest {

    private static final double DT = 0.01;

    private static Pose3d at(double x) {
        return new Pose3d(x, 0.0, 0.0, Rotation3d.kZero);
    }

    @Test
    void withoutDriftOrNoiseTheReportedPoseIsTheTruth() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());

        Pose3d reported = model.update(at(3.0), 1.0, DT);

        assertEquals(3.0, reported.getX(), 1e-12);
        assertTrue(model.isTracking());
    }

    @Test
    void driftGrowsWithDistanceTravelledNotWithTime() {
        VioErrorModel still = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(3));
        VioErrorModel moving = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(3));

        for (int i = 0; i < 500; i++) {
            still.update(at(0.0), i * DT, DT);
            moving.update(at(i * 0.02), i * DT, DT);
        }

        assertEquals(0.0, still.driftTranslation().getNorm(), 1e-12, "parado no deberia derivar");
        assertTrue(
                moving.driftTranslation().getNorm() > 0.0,
                "recorrer 10 m deberia acumular deriva");
    }

    @Test
    void driftMagnitudeGrowsWithDistanceOnAverage() {
        // La deriva es un random walk, asi que una corrida concreta puede encogerse: lo que crece
        // con el recorrido es su magnitud *esperada*. Afirmar monotonia sobre una sola semilla seria
        // un test intermitente que tarde o temprano falla sin que nada este roto.
        double shortRun = averageDriftAfterSteps(20);
        double longRun = averageDriftAfterSteps(600);

        // En un random walk la desviacion crece como la raiz del numero de pasos: 30 veces mas
        // pasos son ~5.5 veces mas deriva. Se exige 3 para dejar margen al muestreo.
        assertTrue(
                longRun > shortRun * 3.0,
                "recorrer 30 veces mas deberia derivar bastante mas: " + shortRun + " -> " + longRun);
    }

    /** Promedia la deriva acumulada sobre varias semillas, para no depender de una realizacion. */
    private static double averageDriftAfterSteps(int steps) {
        int seeds = 40;
        double total = 0.0;
        for (int seed = 0; seed < seeds; seed++) {
            VioErrorModel model = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(seed));
            for (int i = 0; i < steps; i++) {
                model.update(at(i * 0.05), i * DT, DT);
            }
            total += model.driftTranslation().getNorm();
        }
        return total / seeds;
    }

    @Test
    void whileTrackingIsLostThePoseFreezesAndTheCounterGoesUp() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());
        model.update(at(1.0), 0.0, DT);

        model.injectTrackingLoss(0.5, 1.0);
        Pose3d duringLoss = model.update(at(5.0), 1.1, DT);

        assertFalse(model.isTracking());
        assertEquals(1, model.trackingLostCounter());
        assertEquals(1.0, duringLoss.getX(), 1e-9, "sin tracking la pose se congela");
    }

    @Test
    void regainingTrackingReacquiresAtAnOffset() {
        // Es el fallo que documenta QuestNav y el que rompe a un pose estimator que confia 2 cm.
        QuestNavSimConfig config =
                QuestNavSimConfig.perfect().withReacquireOffsetMeters(0.25).withRandomSeed(1);
        VioErrorModel model = new VioErrorModel(config);
        model.update(at(1.0), 0.0, DT);

        model.injectTrackingLoss(0.5, 1.0);
        model.update(at(1.0), 1.1, DT);
        Pose3d afterRecovery = model.update(at(1.0), 2.0, DT);

        assertTrue(model.isTracking(), "deberia haber recuperado el tracking");
        double jump = afterRecovery.getTranslation().getDistance(new Translation3d(1.0, 0.0, 0.0));
        assertEquals(0.25, jump, 1e-6, "la re-adquisicion deberia venir desplazada");
    }

    @Test
    void aPoseDiscontinuityJumpsWithoutDroppingTracking() {
        // El doble-tap a passthrough por vibracion: isTracking sigue en true, que es lo traicionero.
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());
        model.update(at(2.0), 0.0, DT);

        model.injectPoseDiscontinuity(new Translation3d(0.4, 0.0, 0.0));
        Pose3d afterJump = model.update(at(2.0), 0.1, DT);

        assertTrue(model.isTracking(), "el doble-tap no tira el tracking");
        assertEquals(2.4, afterJump.getX(), 1e-9);
    }

    @Test
    void aPoseResetClearsTheAccumulatedDrift() {
        VioErrorModel model = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(9));
        for (int i = 0; i < 500; i++) {
            model.update(at(i * 0.05), i * DT, DT);
        }
        assertTrue(model.driftTranslation().getNorm() > 0.0);

        model.resetDrift();

        assertEquals(0.0, model.driftTranslation().getNorm(), 1e-12);
        assertEquals(0.0, model.driftYawDegrees(), 1e-12);
    }

    @Test
    void theHeadsetShutsDownWhenTheBatteryRunsOut() {
        VioErrorModel model =
                new VioErrorModel(
                        new QuestNavSimConfig()
                                .withInitialBatteryPercent(1.0)
                                .withBatteryDrainPercentPerMinute(120.0));

        assertTrue(model.isPoweredOn());
        for (int i = 0; i < 200; i++) {
            model.update(at(0.0), i * DT, DT);
        }

        assertFalse(model.isPoweredOn(), "la bateria deberia haberse agotado");
        assertEquals(0, model.batteryPercent());
    }

    @Test
    void anInjectedLinkLossBringsTheLinkDownForItsDuration() {
        VioErrorModel model = new VioErrorModel(QuestNavSimConfig.perfect());

        assertTrue(model.isLinkUp(1.0));
        model.injectLinkLoss(0.5, 1.0);

        assertFalse(model.isLinkUp(1.2));
        assertTrue(model.isLinkUp(1.6));
    }

    @Test
    void aHardImpactKnocksOutTrackingOnItsOwn() {
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(30.0, 2000.0));

        // Acelerar como acelera un robot de FRC --- unos 6 m/s2 --- no rompe nada...
        double x = 0.0;
        double v = 0.0;
        int steps = 50;
        for (int i = 0; i < steps; i++) {
            v = Math.min(3.0, v + 0.06);
            x += v * DT;
            model.update(at(x), i * DT, DT);
        }
        assertTrue(model.isTracking(), "acelerar normal no deberia tirar el tracking");

        // ...pero pararse en seco contra la pared si: de 3 m/s a 0 en un ciclo son 300 m/s2.
        model.update(at(x), steps * DT, DT);
        model.update(at(x), (steps + 1) * DT, DT);

        assertFalse(model.isTracking(), "un frenazo brusco deberia tirar el tracking");
    }

    @Test
    void aSinglePoseJumpIsNotMistakenForACrash() {
        // El caso que destapo la simulacion viva. La pose que alimenta al modelo viene del pose
        // estimator del robot, que SALTA cada vez que acepta una medicion de vision. Un salto de
        // 20 cm en un frame de 10 ms son 2000 m/s2 aparentes: mirando un solo frame, cada
        // correccion de vision parecia un choque y el Quest perdia el tracking en dos de cada tres
        // ciclos.
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(40.0, 2000.0));

        double x = 0.0;
        for (int i = 0; i < 20; i++) {
            model.update(at(x), i * DT, DT);
        }

        // Correccion de vision: la pose salta 20 cm de un frame al siguiente y sigue quieta.
        x += 0.20;
        for (int i = 20; i < 40; i++) {
            model.update(at(x), i * DT, DT);
        }

        assertTrue(model.isTracking(), "una correccion de vision no es un choque");
        assertEquals(0, model.trackingLostCounter());
    }

    @Test
    void feedingBackTheFusedEstimateDefeatsTheImpactDetector() {
        // Fija una limitacion, no un comportamiento deseable.
        //
        // Si le pasas al simulador la pose que sale del pose estimator, y ese estimator se corrige
        // con vision en cada ciclo del robot, la pose salta cada dos frames del VIO. Eso no es un
        // impulso aislado --- que la mediana si filtra --- sino una oscilacion a la mitad de la
        // frecuencia de muestreo, y ningun filtro causal la distingue de un movimiento real.
        //
        // La conclusion no es afinar el detector: es que alimentar un sensor simulado con la salida
        // del estimador es un lazo que no existe en la realidad. La fisica alimenta al sensor y el
        // sensor al estimador, nunca al reves. Ver SimulatedQuestNav#periodic.
        VioErrorModel model =
                new VioErrorModel(QuestNavSimConfig.perfect().withImpactTrackingLoss(true));

        double x = 0.0;
        for (int i = 0; i < 200; i++) {
            if (i % 2 == 0) {
                x += 0.15; // "correccion de vision" a la mitad de la tasa de frames
            }
            model.update(at(x), i * DT, DT);
        }

        assertTrue(
                model.trackingLostCounter() > 0,
                "realimentar el estimador confunde al detector, y esto lo deja documentado");
    }

    @Test
    void aSustainedDecelerationStillCountsAsACrash() {
        // Y lo contrario tiene que seguir funcionando: un frenazo real dura decenas de ms.
        VioErrorModel model =
                new VioErrorModel(
                        QuestNavSimConfig.perfect()
                                .withImpactTrackingLoss(true)
                                .withImpactThresholds(30.0, 2000.0));

        double x = 0.0;
        double v = 0.0;
        for (int i = 0; i < 50; i++) {
            v = Math.min(3.0, v + 0.06);
            x += v * DT;
            model.update(at(x), i * DT, DT);
        }
        assertTrue(model.isTracking());

        // Choque: la velocidad se derrumba y se queda en cero varios frames.
        for (int i = 50; i < 60; i++) {
            model.update(at(x), i * DT, DT);
        }

        assertFalse(model.isTracking(), "un frenazo sostenido si deberia tirar el tracking");
    }

    @Test
    void theSameSeedProducesTheSameRun() {
        VioErrorModel a = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(42));
        VioErrorModel b = new VioErrorModel(new QuestNavSimConfig().withRandomSeed(42));

        for (int i = 0; i < 100; i++) {
            Pose3d pa = a.update(at(i * 0.05), i * DT, DT);
            Pose3d pb = b.update(at(i * 0.05), i * DT, DT);
            assertEquals(pa.getX(), pb.getX(), 1e-12);
        }
    }
}
