// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Estrategias para calcular la confianza de una medicion de vision.
 *
 * <p>Cada estrategia es una {@code BiFunction<Double, Integer, Matrix<N3,N1>>} que va de
 * {@code (distancia en metros, numero de tags)} a {@code [sigmaX, sigmaY, sigmaTheta]}, listo para
 * {@code addVisionMeasurement}. Se le pasan a una fuente con
 * {@link PoseSource#setStdDevsStrategy(BiFunction)}.
 *
 * <p>Se componen: una estrategia base se envuelve con una degradacion dinamica y el resultado se
 * acota con {@link #clamped}:
 *
 * <pre>{@code
 * source.setStdDevsStrategy(
 *     StdDevStrategies.clamped(
 *         StdDevStrategies.dynamicStateDegradation(
 *             StdDevStrategies.quadraticContinuous(0.05, 0.10),
 *             drivetrain::getChassisSpeeds,
 *             3.0, 2.0),
 *         0.02, 4.0,    // sigma XY entre 2 cm y 4 m
 *         0.02, 10.0)); // sigma theta entre 0.02 y 10 rad
 * }</pre>
 *
 * <p><b>Acota siempre el resultado final.</b> Las estrategias multiplicativas no tienen techo
 * propio, y un valor desbocado --- o un {@code Infinity} --- rompe la inversion de matrices del
 * filtro de Kalman por dentro, sin un error que diga que paso.
 */
public final class StdDevStrategies {

    private StdDevStrategies() {}

    // ------------------------------------------------------------------
    // Composicion
    // ------------------------------------------------------------------

    /**
     * Acota la salida de una estrategia entre un minimo y un maximo.
     *
     * @param strategy estrategia a envolver.
     * @param minXY    sigma minima de X e Y, en metros.
     * @param maxXY    sigma maxima de X e Y, en metros.
     * @param minTheta sigma minima de rumbo, en radianes.
     * @param maxTheta sigma maxima de rumbo, en radianes.
     * @return una estrategia cuya salida siempre cae dentro de los limites.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> clamped(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy,
            double minXY,
            double maxXY,
            double minTheta,
            double maxTheta) {
        return (distanceMeters, tagCount) -> {
            Matrix<N3, N1> raw = strategy.apply(distanceMeters, tagCount);
            // X e Y se acotan por separado. Acotar solo X y copiarlo en Y funciona mientras todas
            // las estrategias devuelvan los dos iguales, y deja de funcionar en silencio en cuanto
            // alguna no lo haga.
            return VecBuilder.fill(
                    MathUtil.clamp(raw.get(0, 0), minXY, maxXY),
                    MathUtil.clamp(raw.get(1, 0), minXY, maxXY),
                    MathUtil.clamp(raw.get(2, 0), minTheta, maxTheta));
        };
    }

    /**
     * Multiplica la salida de una estrategia por un factor constante. Util para desconfiar de una
     * camara concreta sin reescribir su estrategia.
     *
     * @param strategy   estrategia a envolver.
     * @param factorXY   factor para X e Y.
     * @param factorTheta factor para el rumbo.
     * @return la estrategia escalada.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> scaled(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy,
            double factorXY,
            double factorTheta) {
        return (distanceMeters, tagCount) -> {
            Matrix<N3, N1> raw = strategy.apply(distanceMeters, tagCount);
            return VecBuilder.fill(
                    raw.get(0, 0) * factorXY, raw.get(1, 0) * factorXY, raw.get(2, 0) * factorTheta);
        };
    }

    /**
     * Estrategia constante: la misma confianza pase lo que pase.
     *
     * @param stdXY    sigma de X e Y.
     * @param stdTheta sigma de rumbo.
     * @return la estrategia.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> constant(
            double stdXY, double stdTheta) {
        Matrix<N3, N1> value = VecBuilder.fill(stdXY, stdXY, stdTheta);
        return (distanceMeters, tagCount) -> value;
    }

    // ------------------------------------------------------------------
    // Basadas en distancia y numero de tags
    // ------------------------------------------------------------------

    /**
     * La cuadratica continua, que es la recomendacion habitual en FRC.
     *
     * <p>El error crece con el cuadrado de la distancia --- un tag al doble de lejos ocupa la cuarta
     * parte de los pixeles, y el PnP se degrada igual de rapido --- y baja con el numero de tags
     * visibles.
     *
     * @param baseXY    coeficiente de X e Y.
     * @param baseTheta coeficiente del rumbo.
     * @return la estrategia {@code base * distancia^2 / tags}.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> quadraticContinuous(
            double baseXY, double baseTheta) {
        return (distanceMeters, tagCount) -> {
            int tags = Math.max(1, tagCount);
            double distanceSquared = distanceMeters * distanceMeters;
            double xy = baseXY * distanceSquared / tags;
            double theta = baseTheta * distanceSquared / tags;
            return VecBuilder.fill(xy, xy, theta);
        };
    }

    /**
     * Un tramo de distancia con su error base.
     *
     * @param limitDistanceMeters distancia por debajo de la cual aplica este tramo.
     * @param baseStdXY           sigma de X e Y en este tramo.
     * @param baseStdTheta        sigma de rumbo en este tramo.
     */
    public record DistanceStep(
            double limitDistanceMeters, double baseStdXY, double baseStdTheta) {}

    /**
     * Un tramo de numero de tags con su multiplicador de confianza.
     *
     * @param minTags         tags minimos para que aplique este tramo.
     * @param multiplierXY    multiplicador de X e Y.
     * @param multiplierTheta multiplicador del rumbo.
     */
    public record TagStep(int minTags, double multiplierXY, double multiplierTheta) {}

    /**
     * Combina una tabla por tramos de distancia con un multiplicador por numero de tags.
     *
     * @param fallback      error base cuando la distancia supera todos los tramos.
     * @param distanceSteps tramos de distancia, en cualquier orden.
     * @param tagSteps      tramos de numero de tags, en cualquier orden.
     * @return la estrategia combinada.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> dualStepWise(
            Matrix<N3, N1> fallback, List<DistanceStep> distanceSteps, List<TagStep> tagSteps) {

        List<DistanceStep> sortedDistances = new ArrayList<>(distanceSteps);
        sortedDistances.sort(Comparator.comparingDouble(DistanceStep::limitDistanceMeters));

        List<TagStep> sortedTags = sortedDescending(tagSteps);

        return (distanceMeters, tagCount) -> {
            double baseXY = fallback.get(0, 0);
            double baseTheta = fallback.get(2, 0);

            for (DistanceStep step : sortedDistances) {
                if (distanceMeters < step.limitDistanceMeters()) {
                    baseXY = step.baseStdXY();
                    baseTheta = step.baseStdTheta();
                    break;
                }
            }

            TagStep tagStep = selectTagStep(sortedTags, tagCount);
            double multXY = tagStep == null ? 1.0 : tagStep.multiplierXY();
            double multTheta = tagStep == null ? 1.0 : tagStep.multiplierTheta();

            return VecBuilder.fill(baseXY * multXY, baseXY * multXY, baseTheta * multTheta);
        };
    }

    /**
     * Interpola el error base de una curva medida en la practica, y le aplica el multiplicador por
     * numero de tags.
     *
     * <p>Es la mejor opcion si te tomaste el trabajo de medir: pon el robot a distancias conocidas,
     * anota la dispersion real, y mete esos puntos en los mapas. WPILib interpola entre ellos y
     * satura en el ultimo punto conocido.
     *
     * @param xyMap    mapa distancia (m) a sigma de X e Y (m).
     * @param thetaMap mapa distancia (m) a sigma de rumbo (rad).
     * @param tagSteps tramos de numero de tags.
     * @return la estrategia empirica.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> interpolatedEmpirical(
            InterpolatingDoubleTreeMap xyMap,
            InterpolatingDoubleTreeMap thetaMap,
            List<TagStep> tagSteps) {

        List<TagStep> sortedTags = sortedDescending(tagSteps);

        return (distanceMeters, tagCount) -> {
            double baseXY = xyMap.get(distanceMeters);
            double baseTheta = thetaMap.get(distanceMeters);

            TagStep tagStep = selectTagStep(sortedTags, tagCount);
            double multXY = tagStep == null ? 1.0 : tagStep.multiplierXY();
            double multTheta = tagStep == null ? 1.0 : tagStep.multiplierTheta();

            return VecBuilder.fill(baseXY * multXY, baseXY * multXY, baseTheta * multTheta);
        };
    }

    private static List<TagStep> sortedDescending(List<TagStep> tagSteps) {
        List<TagStep> sorted = new ArrayList<>(tagSteps);
        sorted.sort(Comparator.comparingInt(TagStep::minTags).reversed());
        return sorted;
    }

    private static TagStep selectTagStep(List<TagStep> sortedDescending, int tagCount) {
        for (TagStep step : sortedDescending) {
            if (tagCount >= step.minTags()) {
                return step;
            }
        }
        return null;
    }

    // ------------------------------------------------------------------
    // Degradacion por movimiento
    // ------------------------------------------------------------------

    /**
     * Infla el error base cuando el robot se mueve, para compensar el desenfoque y la deformacion de
     * obturador rodante.
     *
     * <p>La velocidad lineal castiga la confianza en X e Y. La angular castiga las dos, y mas fuerte:
     * girando rapido, los tags salen sesgados y el PnP pierde primero la rotacion, que es
     * precisamente el eje que menos se puede permitir equivocar.
     *
     * <p>Los multiplicadores se acotan internamente a 400 como red de seguridad, para que esta
     * estrategia no pueda devolver {@code Infinity} aunque la velocidad se dispare. Para controlar el
     * rango final de verdad, envuelvela con {@link #clamped}.
     *
     * @param baseStrategy     estrategia a inflar.
     * @param speedsSupplier   velocidades actuales del chasis.
     * @param maxSafeLinearMps velocidad lineal a partir de la cual la imagen se degrada.
     * @param maxSafeOmegaRads velocidad angular a partir de la cual los tags salen sesgados.
     * @param angularBias      cuanto pesa el exceso angular frente al lineal. 2.0 es un punto de
     *                         partida razonable: girar estropea la imagen mas que avanzar.
     * @return la estrategia inflada.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> dynamicStateDegradation(
            BiFunction<Double, Integer, Matrix<N3, N1>> baseStrategy,
            Supplier<ChassisSpeeds> speedsSupplier,
            double maxSafeLinearMps,
            double maxSafeOmegaRads,
            double angularBias) {

        return (distanceMeters, tagCount) -> {
            Matrix<N3, N1> base = baseStrategy.apply(distanceMeters, tagCount);

            ChassisSpeeds speeds = speedsSupplier == null ? null : speedsSupplier.get();
            if (speeds == null) {
                return base;
            }

            double linear = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            double angular = Math.abs(speeds.omegaRadiansPerSecond);

            double multXY = 1.0;
            double multTheta = 1.0;

            if (linear > maxSafeLinearMps) {
                multXY += square(linear - maxSafeLinearMps);
            }

            if (angular > maxSafeOmegaRads) {
                double excess = angular - maxSafeOmegaRads;
                multXY += square(excess * angularBias);
                multTheta += square(excess);
            }

            multXY = MathUtil.clamp(multXY, 1.0, 400.0);
            multTheta = MathUtil.clamp(multTheta, 1.0, 400.0);

            return VecBuilder.fill(
                    base.get(0, 0) * multXY, base.get(1, 0) * multXY, base.get(2, 0) * multTheta);
        };
    }

    /**
     * Version con el sesgo angular por defecto de 2.0.
     *
     * @param baseStrategy     estrategia a inflar.
     * @param speedsSupplier   velocidades actuales del chasis.
     * @param maxSafeLinearMps velocidad lineal segura.
     * @param maxSafeOmegaRads velocidad angular segura.
     * @return la estrategia inflada.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> dynamicStateDegradation(
            BiFunction<Double, Integer, Matrix<N3, N1>> baseStrategy,
            Supplier<ChassisSpeeds> speedsSupplier,
            double maxSafeLinearMps,
            double maxSafeOmegaRads) {
        return dynamicStateDegradation(
                baseStrategy, speedsSupplier, maxSafeLinearMps, maxSafeOmegaRads, 2.0);
    }

    /**
     * Desconfia de un VIO cuando el robot gira rapido, que es cuando su tracking sufre.
     *
     * @param baseXY             sigma de X e Y en reposo.
     * @param baseTheta          sigma de rumbo en reposo.
     * @param angularVelocity    velocidad angular actual.
     * @param toleranceDegPerSec giro a partir del cual empieza a desconfiar.
     * @param scaleDegPerSec     cada cuantos grados por segundo de exceso se duplica el error.
     * @return la estrategia.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> vioMotionBlur(
            double baseXY,
            double baseTheta,
            Supplier<AngularVelocity> angularVelocity,
            double toleranceDegPerSec,
            double scaleDegPerSec) {

        return (distanceMeters, tagCount) -> {
            AngularVelocity velocity = angularVelocity == null ? null : angularVelocity.get();
            if (velocity == null) {
                return VecBuilder.fill(baseXY, baseXY, baseTheta);
            }

            double current = Math.abs(velocity.in(DegreesPerSecond));
            double multiplier = 1.0;
            if (current > toleranceDegPerSec) {
                multiplier = 1.0 + square((current - toleranceDegPerSec) / scaleDegPerSec);
            }

            return VecBuilder.fill(baseXY * multiplier, baseXY * multiplier, baseTheta * multiplier);
        };
    }

    // ------------------------------------------------------------------
    // Deriva de VIO
    // ------------------------------------------------------------------

    /**
     * Estrategia de deriva para un VIO, que se puede resetear.
     *
     * <p>Un sensor de odometria visual-inercial no se equivoca por estar lejos de nada: se equivoca
     * por llevar rato sin que nadie lo corrija. Su error crece con el tiempo desde la ultima
     * correccion, no con la distancia ni con el numero de tags.
     *
     * <p>Por eso esta estrategia es una clase y no una lambda: cuando el robot le manda un
     * {@code POSE_RESET} al headset, la deriva real vuelve a cero y hay que decirselo con
     * {@link #reset()}. Una version que capture el instante de arranque y no se pueda resetear
     * sigue inflando el error para siempre, y acaba haciendo que el estimator ignore un sensor que
     * ya esta bien.
     */
    public static final class VioDriftStrategy
            implements BiFunction<Double, Integer, Matrix<N3, N1>> {

        private final double baseXY;
        private final double baseTheta;
        private final double driftPerSecondXY;
        private final double driftPerSecondTheta;
        private final DoubleSupplier clock;

        private double lastResetSeconds;

        private VioDriftStrategy(
                double baseXY,
                double baseTheta,
                double driftPerSecondXY,
                double driftPerSecondTheta,
                DoubleSupplier clock) {
            this.baseXY = baseXY;
            this.baseTheta = baseTheta;
            this.driftPerSecondXY = driftPerSecondXY;
            this.driftPerSecondTheta = driftPerSecondTheta;
            this.clock = clock;
            this.lastResetSeconds = clock.getAsDouble();
        }

        /** Vuelve a poner la deriva en cero. Llamalo justo despues de corregir el sensor. */
        public void reset() {
            this.lastResetSeconds = clock.getAsDouble();
        }

        /** @return segundos desde la ultima correccion. */
        public double elapsedSeconds() {
            return Math.max(0.0, clock.getAsDouble() - lastResetSeconds);
        }

        @Override
        public Matrix<N3, N1> apply(Double distanceMeters, Integer tagCount) {
            double elapsed = elapsedSeconds();
            double xy = baseXY + driftPerSecondXY * elapsed;
            double theta = baseTheta + driftPerSecondTheta * elapsed;
            return VecBuilder.fill(xy, xy, theta);
        }
    }

    /**
     * Crea una estrategia de deriva para un VIO, con el reloj del FPGA.
     *
     * @param baseXY              sigma de X e Y recien corregido.
     * @param baseTheta           sigma de rumbo recien corregido.
     * @param driftPerSecondXY    metros de incertidumbre que suma cada segundo.
     * @param driftPerSecondTheta radianes de incertidumbre que suma cada segundo. El rumbo de un
     *                            Quest deriva mucho menos que la posicion, asi que suele ser un
     *                            orden de magnitud menor.
     * @return la estrategia, que hay que resetear tras cada correccion.
     */
    public static VioDriftStrategy vioTimeDrift(
            double baseXY, double baseTheta, double driftPerSecondXY, double driftPerSecondTheta) {
        return vioTimeDrift(
                baseXY, baseTheta, driftPerSecondXY, driftPerSecondTheta, Timer::getFPGATimestamp);
    }

    /**
     * Crea una estrategia de deriva con un reloj propio, para tests.
     *
     * @param baseXY              sigma de X e Y recien corregido.
     * @param baseTheta           sigma de rumbo recien corregido.
     * @param driftPerSecondXY    metros por segundo.
     * @param driftPerSecondTheta radianes por segundo.
     * @param clock               fuente de tiempo en segundos.
     * @return la estrategia.
     */
    public static VioDriftStrategy vioTimeDrift(
            double baseXY,
            double baseTheta,
            double driftPerSecondXY,
            double driftPerSecondTheta,
            DoubleSupplier clock) {
        return new VioDriftStrategy(baseXY, baseTheta, driftPerSecondXY, driftPerSecondTheta, clock);
    }

    private static double square(double value) {
        return value * value;
    }
}
