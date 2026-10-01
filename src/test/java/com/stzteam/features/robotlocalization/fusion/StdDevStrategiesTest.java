// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.function.BiFunction;

/** Las estrategias de confianza. Matematica pura: ni HAL, ni NetworkTables, ni hardware. */
class StdDevStrategiesTest {

    private static final double EPS = 1e-9;

    @Test
    void quadraticGrowsWithTheSquareOfDistance() {
        var strategy = StdDevStrategies.quadraticContinuous(0.1, 0.2);

        double atOneMeter = strategy.apply(1.0, 1).get(0, 0);
        double atTwoMeters = strategy.apply(2.0, 1).get(0, 0);

        assertEquals(0.1, atOneMeter, EPS);
        assertEquals(4.0, atTwoMeters / atOneMeter, EPS, "el doble de lejos, cuatro veces peor");
    }

    @Test
    void quadraticShrinksWithMoreTags() {
        var strategy = StdDevStrategies.quadraticContinuous(0.1, 0.2);

        assertEquals(
                strategy.apply(2.0, 1).get(0, 0) / 2.0, strategy.apply(2.0, 2).get(0, 0), EPS);
    }

    @Test
    void quadraticSurvivesZeroTags() {
        // Una division entre cero aqui envenena el estimator con NaN sin decir nada.
        Matrix<N3, N1> result = StdDevStrategies.quadraticContinuous(0.1, 0.2).apply(2.0, 0);

        assertTrue(Double.isFinite(result.get(0, 0)));
    }

    @Test
    void clampedBoundsXandYIndependently() {
        // Acotar solo X y copiarlo en Y funciona mientras todas las estrategias devuelvan los dos
        // iguales, y deja de funcionar en silencio en cuanto alguna no lo haga.
        BiFunction<Double, Integer, Matrix<N3, N1>> asymmetric =
                (distance, tags) -> VecBuilder.fill(0.1, 9.0, 0.5);

        Matrix<N3, N1> result = StdDevStrategies.clamped(asymmetric, 0.2, 4.0, 0.1, 1.0).apply(1.0, 1);

        assertEquals(0.2, result.get(0, 0), EPS, "X deberia subir hasta el minimo");
        assertEquals(4.0, result.get(1, 0), EPS, "Y deberia bajar hasta el maximo, no copiar a X");
        assertEquals(0.5, result.get(2, 0), EPS);
    }

    @Test
    void clampedKeepsAnUnboundedStrategyFromReachingInfinity() {
        BiFunction<Double, Integer, Matrix<N3, N1>> explosive =
                (distance, tags) -> VecBuilder.fill(
                        Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

        Matrix<N3, N1> result = StdDevStrategies.clamped(explosive, 0.01, 5.0, 0.01, 5.0).apply(1.0, 1);

        assertEquals(5.0, result.get(0, 0), EPS);
        assertEquals(5.0, result.get(2, 0), EPS);
    }

    @Test
    void stepWisePicksTheFirstBracketAboveTheDistance() {
        var strategy =
                StdDevStrategies.dualStepWise(
                        VecBuilder.fill(2.0, 2.0, 4.0),
                        List.of(
                                new StdDevStrategies.DistanceStep(4.0, 0.5, 1.0),
                                new StdDevStrategies.DistanceStep(2.0, 0.1, 0.2)),
                        List.of(new StdDevStrategies.TagStep(1, 1.0, 1.0)));

        assertEquals(0.1, strategy.apply(1.0, 1).get(0, 0), EPS, "dentro del tramo cercano");
        assertEquals(0.5, strategy.apply(3.0, 1).get(0, 0), EPS, "dentro del tramo lejano");
        assertEquals(2.0, strategy.apply(9.0, 1).get(0, 0), EPS, "mas alla de todo, el fallback");
    }

    @Test
    void stepWiseAppliesTheHighestMatchingTagMultiplier() {
        var strategy =
                StdDevStrategies.dualStepWise(
                        VecBuilder.fill(1.0, 1.0, 1.0),
                        List.of(new StdDevStrategies.DistanceStep(10.0, 1.0, 1.0)),
                        List.of(
                                new StdDevStrategies.TagStep(1, 1.0, 1.0),
                                new StdDevStrategies.TagStep(2, 0.5, 0.5)));

        assertEquals(1.0, strategy.apply(1.0, 1).get(0, 0), EPS);
        assertEquals(0.5, strategy.apply(1.0, 3).get(0, 0), EPS, "3 tags cae en el tramo de 2+");
    }

    @Test
    void empiricalInterpolatesBetweenMeasuredPoints() {
        InterpolatingDoubleTreeMap xy = new InterpolatingDoubleTreeMap();
        xy.put(1.0, 0.1);
        xy.put(3.0, 0.5);
        InterpolatingDoubleTreeMap theta = new InterpolatingDoubleTreeMap();
        theta.put(1.0, 0.2);
        theta.put(3.0, 1.0);

        var strategy =
                StdDevStrategies.interpolatedEmpirical(
                        xy, theta, List.of(new StdDevStrategies.TagStep(1, 1.0, 1.0)));

        assertEquals(0.3, strategy.apply(2.0, 1).get(0, 0), 1e-9, "a mitad de camino");
        assertEquals(0.5, strategy.apply(9.0, 1).get(0, 0), 1e-9, "satura en el ultimo punto");
    }

    @Test
    void motionDegradationDoesNothingBelowTheThresholds() {
        var base = StdDevStrategies.constant(0.1, 0.2);
        ChassisSpeeds slow = new ChassisSpeeds(0.5, 0.0, 0.1);

        var strategy = StdDevStrategies.dynamicStateDegradation(base, () -> slow, 3.0, 2.0);

        assertEquals(0.1, strategy.apply(1.0, 1).get(0, 0), EPS);
    }

    @Test
    void motionDegradationPunishesTurningMoreThanDriving() {
        var base = StdDevStrategies.constant(0.1, 0.2);
        ChassisSpeeds spinning = new ChassisSpeeds(0.0, 0.0, 4.0);
        ChassisSpeeds driving = new ChassisSpeeds(5.0, 0.0, 0.0);

        var turn = StdDevStrategies.dynamicStateDegradation(base, () -> spinning, 3.0, 2.0);
        var drive = StdDevStrategies.dynamicStateDegradation(base, () -> driving, 3.0, 2.0);

        assertTrue(
                turn.apply(1.0, 1).get(0, 0) > drive.apply(1.0, 1).get(0, 0),
                "girar deforma la imagen mas que avanzar");
        assertTrue(
                turn.apply(1.0, 1).get(2, 0) > 0.2,
                "y el rumbo es lo primero que se pierde");
    }

    @Test
    void motionDegradationNeverReturnsInfinity() {
        var base = StdDevStrategies.constant(0.1, 0.2);
        ChassisSpeeds absurd = new ChassisSpeeds(500.0, 500.0, 500.0);

        Matrix<N3, N1> result =
                StdDevStrategies.dynamicStateDegradation(base, () -> absurd, 3.0, 2.0).apply(1.0, 1);

        assertTrue(Double.isFinite(result.get(0, 0)), "la red de seguridad deberia sostenerlo");
        assertTrue(Double.isFinite(result.get(2, 0)));
    }

    @Test
    void motionDegradationFallsBackWhenThereAreNoSpeeds() {
        var base = StdDevStrategies.constant(0.1, 0.2);

        Matrix<N3, N1> result =
                StdDevStrategies.dynamicStateDegradation(base, () -> null, 3.0, 2.0).apply(1.0, 1);

        assertEquals(0.1, result.get(0, 0), EPS);
    }

    @Test
    void vioDriftGrowsWithTimeAndResetsOnDemand() {
        double[] clock = {100.0};
        var strategy = StdDevStrategies.vioTimeDrift(0.02, 0.01, 0.1, 0.01, () -> clock[0]);

        assertEquals(0.02, strategy.apply(0.0, 1).get(0, 0), EPS, "recien creada no ha derivado");

        clock[0] = 110.0;
        assertEquals(1.02, strategy.apply(0.0, 1).get(0, 0), 1e-9, "10 s despues");

        // Un POSE_RESET devuelve la deriva real a cero, y la estrategia tiene que enterarse. Sin
        // esto sigue inflando para siempre y el estimator termina ignorando un sensor que ya esta
        // perfecto.
        strategy.reset();
        assertEquals(0.02, strategy.apply(0.0, 1).get(0, 0), EPS);
        assertEquals(0.0, strategy.elapsedSeconds(), EPS);
    }

    @Test
    void vioDriftLeavesHeadingAlmostAlone() {
        double[] clock = {0.0};
        var strategy = StdDevStrategies.vioTimeDrift(0.02, 0.01, 0.1, 0.001, () -> clock[0]);

        clock[0] = 100.0;
        Matrix<N3, N1> result = strategy.apply(0.0, 1);

        assertTrue(
                result.get(2, 0) < result.get(0, 0),
                "el giroscopio del Quest deriva mucho menos que su posicion");
    }

    @Test
    void scaledMultipliesWithoutTouchingTheShape() {
        var strategy = StdDevStrategies.scaled(StdDevStrategies.constant(0.1, 0.2), 2.0, 10.0);
        Matrix<N3, N1> result = strategy.apply(1.0, 1);

        assertEquals(0.2, result.get(0, 0), EPS);
        assertEquals(0.2, result.get(1, 0), EPS);
        assertEquals(2.0, result.get(2, 0), EPS);
    }
}
