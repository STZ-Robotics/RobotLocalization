// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

/**
 * The robot state that every pose source shares.
 *
 * <p>It is built once and {@link PoseFusion} hands it to every source you register. Sources use it
 * to decide whether this is a good moment to trust a measurement: a robot spinning at 600 degrees
 * per second produces images you cannot do PnP with, and one that has just crashed produces IMU
 * readings that describe nothing.
 *
 * <h2>Every field is optional</h2>
 * A {@code null} means "I do not have that datum", and any filter that depends on it is skipped
 * instead of failing. That way you can start with the bare minimum --- a pose and some speeds ---
 * and add sensors without touching the sources.
 *
 * <h2>Acceleration units</h2>
 * {@code planarAcceleration} is in <b>G</b>, which is what {@code Pigeon2.getAccelerationX} and
 * {@code getAccelerationY} return. It is the only field with an implicit unit, so it is worth not
 * guessing it:
 *
 * <pre>{@code
 * () -> VecBuilder.fill(
 *         pigeon.getAccelerationX().getValueAsDouble(),
 *         pigeon.getAccelerationY().getValueAsDouble())
 * }</pre>
 *
 * @param gyroHeading        gyro heading. Limelight's MegaTag2 needs it.
 * @param angularVelocity    robot angular velocity.
 * @param chassisSpeeds      chassis speeds, in robot or field frame; only their magnitude is used.
 * @param planarAcceleration in-plane acceleration, in G.
 * @param currentPose        the pose the estimator holds right now. The Mahalanobis filter uses it
 *                           to measure how far a new measurement strays.
 */
public record LocalizationInputs(
        Supplier<Rotation2d> gyroHeading,
        Supplier<AngularVelocity> angularVelocity,
        Supplier<ChassisSpeeds> chassisSpeeds,
        Supplier<Vector<N2>> planarAcceleration,
        Supplier<Pose2d> currentPose) {

    /** Empty inputs: no filter that depends on the robot state is applied. */
    public static final LocalizationInputs NONE =
            new LocalizationInputs(null, null, null, null, null);

    /**
     * Builds inputs with the useful minimum: the current pose and the chassis speeds.
     *
     * @param currentPose   the pose the estimator holds.
     * @param chassisSpeeds chassis speeds.
     * @return the inputs.
     */
    public static LocalizationInputs of(
            Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> chassisSpeeds) {
        return new LocalizationInputs(null, null, chassisSpeeds, null, currentPose);
    }

    /**
     * Gyro heading, if there is one.
     *
     * @return the heading, or empty.
     */
    public Optional<Rotation2d> heading() {
        return read(gyroHeading);
    }

    /**
     * The pose the estimator holds right now, if there is one.
     *
     * @return the pose, or empty.
     */
    public Optional<Pose2d> pose() {
        return read(currentPose);
    }

    /**
     * Chassis speeds, if there are any.
     *
     * @return the speeds, or empty.
     */
    public Optional<ChassisSpeeds> speeds() {
        return read(chassisSpeeds);
    }

    /**
     * Magnitude of the angular velocity in degrees per second.
     *
     * @return the magnitude, or empty if there is no gyro.
     */
    public OptionalDouble angularSpeedDegreesPerSecond() {
        return read(angularVelocity)
                .map(v -> OptionalDouble.of(Math.abs(v.in(DegreesPerSecond))))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * Magnitude of the linear velocity in metres per second.
     *
     * @return the magnitude, or empty if there are no chassis speeds.
     */
    public OptionalDouble planarSpeedMetersPerSecond() {
        return speeds()
                .map(s -> OptionalDouble.of(Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond)))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * Magnitude of the in-plane acceleration, in G.
     *
     * @return the magnitude, or empty if there is no accelerometer.
     */
    public OptionalDouble planarAccelerationGs() {
        return read(planarAcceleration)
                .map(v -> OptionalDouble.of(v.norm()))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * In-plane velocity vector, or the zero vector if there are no speeds.
     *
     * @return the velocity vector.
     */
    public Vector<N2> planarVelocityVector() {
        return speeds()
                .map(s -> VecBuilder.fill(s.vxMetersPerSecond, s.vyMetersPerSecond))
                .orElseGet(() -> VecBuilder.fill(0.0, 0.0));
    }

    /**
     * Reads a supplier, tolerating both a null supplier and a supplier that returns null. The
     * second happens more than you would think: a hardware getter can return null before the device
     * finishes initialising.
     */
    private static <T> Optional<T> read(Supplier<T> supplier) {
        return supplier == null ? Optional.empty() : Optional.ofNullable(supplier.get());
    }
}
