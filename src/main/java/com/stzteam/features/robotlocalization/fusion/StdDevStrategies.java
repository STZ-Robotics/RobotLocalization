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
 * Strategies for computing the confidence of a vision measurement.
 *
 * <p>Each strategy is a {@code BiFunction<Double, Integer, Matrix<N3,N1>>} going from
 * {@code (distance in metres, tag count)} to {@code [sigmaX, sigmaY, sigmaTheta]}, ready for
 * {@code addVisionMeasurement}. You hand them to a source with
 * {@link PoseSource#setStdDevsStrategy(BiFunction)}.
 *
 * <p>They compose: a base strategy is wrapped in a dynamic degradation and the result is bounded
 * with {@link #clamped}:
 *
 * <pre>{@code
 * source.setStdDevsStrategy(
 *     StdDevStrategies.clamped(
 *         StdDevStrategies.dynamicStateDegradation(
 *             StdDevStrategies.quadraticContinuous(0.05, 0.10),
 *             drivetrain::getChassisSpeeds,
 *             3.0, 2.0),
 *         0.02, 4.0,    // sigma XY between 2 cm and 4 m
 *         0.02, 10.0)); // sigma theta between 0.02 and 10 rad
 * }</pre>
 *
 * <p><b>Always bound the final result.</b> Multiplicative strategies have no ceiling of their own,
 * and a runaway value --- or an {@code Infinity} --- breaks the Kalman filter's matrix inversion
 * from the inside, with no error to say what happened.
 */
public final class StdDevStrategies {

    private StdDevStrategies() {}

    // ------------------------------------------------------------------
    // Composition
    // ------------------------------------------------------------------

    /**
     * Bounds a strategy's output between a minimum and a maximum.
     *
     * @param strategy strategy to wrap.
     * @param minXY    minimum sigma for X and Y, in metres.
     * @param maxXY    maximum sigma for X and Y, in metres.
     * @param minTheta minimum heading sigma, in radians.
     * @param maxTheta maximum heading sigma, in radians.
     * @return a strategy whose output always falls within the bounds.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> clamped(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy,
            double minXY,
            double maxXY,
            double minTheta,
            double maxTheta) {
        return (distanceMeters, tagCount) -> {
            Matrix<N3, N1> raw = strategy.apply(distanceMeters, tagCount);
            // X and Y are bounded separately. Bounding only X and copying it into Y works as long
            // as every strategy returns the two equal, and silently stops working the moment one
            // does not.
            return VecBuilder.fill(
                    MathUtil.clamp(raw.get(0, 0), minXY, maxXY),
                    MathUtil.clamp(raw.get(1, 0), minXY, maxXY),
                    MathUtil.clamp(raw.get(2, 0), minTheta, maxTheta));
        };
    }

    /**
     * Multiplies a strategy's output by a constant factor. Useful to distrust one specific camera
     * without rewriting its strategy.
     *
     * @param strategy   strategy to wrap.
     * @param factorXY   factor for X and Y.
     * @param factorTheta factor for the heading.
     * @return the scaled strategy.
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
     * A constant strategy: the same confidence no matter what.
     *
     * @param stdXY    sigma for X and Y.
     * @param stdTheta heading sigma.
     * @return the strategy.
     */
    public static BiFunction<Double, Integer, Matrix<N3, N1>> constant(
            double stdXY, double stdTheta) {
        Matrix<N3, N1> value = VecBuilder.fill(stdXY, stdXY, stdTheta);
        return (distanceMeters, tagCount) -> value;
    }

    // ------------------------------------------------------------------
    // Based on distance and tag count
    // ------------------------------------------------------------------

    /**
     * The continuous quadratic, which is the usual recommendation in FRC.
     *
     * <p>The error grows with the square of the distance --- a tag twice as far away covers a quarter
     * of the pixels, and PnP degrades just as fast --- and drops with the number of visible tags.
     *
     * @param baseXY    coefficient for X and Y.
     * @param baseTheta coefficient for the heading.
     * @return the {@code base * distance^2 / tags} strategy.
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
     * A distance band with its base error.
     *
     * @param limitDistanceMeters distance below which this band applies.
     * @param baseStdXY           sigma for X and Y in this band.
     * @param baseStdTheta        heading sigma in this band.
     */
    public record DistanceStep(
            double limitDistanceMeters, double baseStdXY, double baseStdTheta) {}

    /**
     * A tag-count band with its confidence multiplier.
     *
     * @param minTags         minimum tags for this band to apply.
     * @param multiplierXY    multiplier for X and Y.
     * @param multiplierTheta multiplier for the heading.
     */
    public record TagStep(int minTags, double multiplierXY, double multiplierTheta) {}

    /**
     * Combines a distance-banded table with a tag-count multiplier.
     *
     * @param fallback      base error when the distance exceeds every band.
     * @param distanceSteps distance bands, in any order.
     * @param tagSteps      tag-count bands, in any order.
     * @return the combined strategy.
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
     * Interpolates the base error from a curve measured in practice, then applies the tag-count
     * multiplier.
     *
     * <p>This is the best option if you went to the trouble of measuring: put the robot at known
     * distances, note the real spread, and feed those points into the maps. WPILib interpolates
     * between them and saturates at the last known point.
     *
     * @param xyMap    map from distance (m) to sigma for X and Y (m).
     * @param thetaMap map from distance (m) to heading sigma (rad).
     * @param tagSteps tag-count bands.
     * @return the empirical strategy.
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
    // Motion-induced degradation
    // ------------------------------------------------------------------

    /**
     * Inflates the base error when the robot is moving, to account for blur and rolling-shutter
     * distortion.
     *
     * <p>Linear velocity penalises confidence in X and Y. Angular velocity penalises both, and
     * harder: spinning fast, the tags come out skewed and PnP loses the rotation first, which is
     * precisely the axis you can least afford to get wrong.
     *
     * <p>The multipliers are internally bounded to 400 as a safety net, so this strategy cannot
     * return {@code Infinity} even if the speed runs away. To really control the final range, wrap
     * it in {@link #clamped}.
     *
     * @param baseStrategy     strategy to inflate.
     * @param speedsSupplier   the chassis' current speeds.
     * @param maxSafeLinearMps linear velocity above which the image degrades.
     * @param maxSafeOmegaRads angular velocity above which the tags come out skewed.
     * @param angularBias      how much the angular excess weighs against the linear one. 2.0 is a
     *                         reasonable starting point: spinning ruins the image more than driving.
     * @return the inflated strategy.
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
     * The version with the default angular bias of 2.0.
     *
     * @param baseStrategy     strategy to inflate.
     * @param speedsSupplier   the chassis' current speeds.
     * @param maxSafeLinearMps safe linear velocity.
     * @param maxSafeOmegaRads safe angular velocity.
     * @return the inflated strategy.
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
     * Distrusts a VIO when the robot spins fast, which is when its tracking suffers.
     *
     * @param baseXY             sigma for X and Y at rest.
     * @param baseTheta          heading sigma at rest.
     * @param angularVelocity    the current angular velocity.
     * @param toleranceDegPerSec the turn rate at which it starts to distrust.
     * @param scaleDegPerSec     how many degrees per second of excess double the error.
     * @return the strategy.
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
    // VIO drift
    // ------------------------------------------------------------------

    /**
     * A drift strategy for a VIO, which can be reset.
     *
     * <p>A visual-inertial odometry sensor does not go wrong from being far from anything: it goes
     * wrong from having gone a while without anyone correcting it. Its error grows with the time
     * since the last correction, not with distance or tag count.
     *
     * <p>That is why this strategy is a class and not a lambda: when the robot sends a
     * {@code POSE_RESET} to the headset, the real drift goes back to zero and you have to say so
     * with {@link #reset()}. A version that captures the start instant and cannot be reset keeps
     * inflating the error forever, and ends up making the estimator ignore a sensor that is already
     * fine.
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

        /** Puts the drift back to zero. Call it right after correcting the sensor. */
        public void reset() {
            this.lastResetSeconds = clock.getAsDouble();
        }

        /** @return seconds since the last correction. */
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
     * Creates a drift strategy for a VIO, on the FPGA clock.
     *
     * @param baseXY              sigma for X and Y when freshly corrected.
     * @param baseTheta           heading sigma when freshly corrected.
     * @param driftPerSecondXY    metres of uncertainty added each second.
     * @param driftPerSecondTheta radians of uncertainty added each second. A Quest's heading drifts
     *                            far less than its position, so this is usually an order of
     *                            magnitude smaller.
     * @return the strategy, which must be reset after each correction.
     */
    public static VioDriftStrategy vioTimeDrift(
            double baseXY, double baseTheta, double driftPerSecondXY, double driftPerSecondTheta) {
        return vioTimeDrift(
                baseXY, baseTheta, driftPerSecondXY, driftPerSecondTheta, Timer::getFPGATimestamp);
    }

    /**
     * Creates a drift strategy with your own clock, for tests.
     *
     * @param baseXY              sigma for X and Y when freshly corrected.
     * @param baseTheta           heading sigma when freshly corrected.
     * @param driftPerSecondXY    metres per second.
     * @param driftPerSecondTheta radians per second.
     * @param clock               time source, in seconds.
     * @return the strategy.
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
