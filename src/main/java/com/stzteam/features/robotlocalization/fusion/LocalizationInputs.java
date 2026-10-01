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
 * El estado del robot que todas las fuentes de pose comparten.
 *
 * <p>Se construye una vez y {@link PoseFusion} se la pasa a cada fuente que registres. Las fuentes
 * la usan para decidir si el momento es bueno para fiarse de una medicion: un robot girando a 600
 * grados por segundo produce imagenes con las que no se puede hacer PnP, y uno que acaba de chocar
 * produce lecturas de IMU que no describen nada.
 *
 * <h2>Todos los campos son opcionales</h2>
 * Un {@code null} significa "no tengo ese dato", y el filtro que dependa de el se salta en vez de
 * fallar. Asi se puede empezar con lo minimo --- una pose y unas velocidades --- e ir anadiendo
 * sensores sin tocar las fuentes.
 *
 * <h2>Unidades de la aceleracion</h2>
 * {@code planarAcceleration} va en <b>G</b>, que es lo que devuelven {@code Pigeon2.getAccelerationX}
 * y {@code getAccelerationY}. Es el unico campo con una unidad implicita, asi que conviene no
 * inventarla:
 *
 * <pre>{@code
 * () -> VecBuilder.fill(
 *         pigeon.getAccelerationX().getValueAsDouble(),
 *         pigeon.getAccelerationY().getValueAsDouble())
 * }</pre>
 *
 * @param gyroHeading        rumbo del giroscopio. Lo necesita MegaTag2 de Limelight.
 * @param angularVelocity    velocidad angular del robot.
 * @param chassisSpeeds      velocidades del chasis, en marco del robot o del campo; solo se usa su
 *                           magnitud.
 * @param planarAcceleration aceleracion en el plano, en G.
 * @param currentPose        pose que el estimator tiene ahora mismo. La usa el filtro de
 *                           Mahalanobis para medir cuanto se desvia una medicion nueva.
 */
public record LocalizationInputs(
        Supplier<Rotation2d> gyroHeading,
        Supplier<AngularVelocity> angularVelocity,
        Supplier<ChassisSpeeds> chassisSpeeds,
        Supplier<Vector<N2>> planarAcceleration,
        Supplier<Pose2d> currentPose) {

    /** Entradas vacias: ningun filtro que dependa del estado del robot se aplica. */
    public static final LocalizationInputs NONE =
            new LocalizationInputs(null, null, null, null, null);

    /**
     * Construye unas entradas con lo minimo util: la pose actual y las velocidades del chasis.
     *
     * @param currentPose   pose que tiene el estimator.
     * @param chassisSpeeds velocidades del chasis.
     * @return las entradas.
     */
    public static LocalizationInputs of(
            Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> chassisSpeeds) {
        return new LocalizationInputs(null, null, chassisSpeeds, null, currentPose);
    }

    /**
     * Rumbo del giroscopio, si hay.
     *
     * @return el rumbo, o vacio.
     */
    public Optional<Rotation2d> heading() {
        return read(gyroHeading);
    }

    /**
     * Pose que el estimator tiene ahora mismo, si hay.
     *
     * @return la pose, o vacio.
     */
    public Optional<Pose2d> pose() {
        return read(currentPose);
    }

    /**
     * Velocidades del chasis, si hay.
     *
     * @return las velocidades, o vacio.
     */
    public Optional<ChassisSpeeds> speeds() {
        return read(chassisSpeeds);
    }

    /**
     * Magnitud de la velocidad angular en grados por segundo.
     *
     * @return la magnitud, o vacio si no hay giroscopio.
     */
    public OptionalDouble angularSpeedDegreesPerSecond() {
        return read(angularVelocity)
                .map(v -> OptionalDouble.of(Math.abs(v.in(DegreesPerSecond))))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * Magnitud de la velocidad lineal en metros por segundo.
     *
     * @return la magnitud, o vacio si no hay velocidades de chasis.
     */
    public OptionalDouble planarSpeedMetersPerSecond() {
        return speeds()
                .map(s -> OptionalDouble.of(Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond)))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * Magnitud de la aceleracion en el plano, en G.
     *
     * @return la magnitud, o vacio si no hay acelerometro.
     */
    public OptionalDouble planarAccelerationGs() {
        return read(planarAcceleration)
                .map(v -> OptionalDouble.of(v.norm()))
                .orElseGet(OptionalDouble::empty);
    }

    /**
     * Vector de velocidad en el plano, o el vector cero si no hay velocidades.
     *
     * @return el vector de velocidad.
     */
    public Vector<N2> planarVelocityVector() {
        return speeds()
                .map(s -> VecBuilder.fill(s.vxMetersPerSecond, s.vyMetersPerSecond))
                .orElseGet(() -> VecBuilder.fill(0.0, 0.0));
    }

    /**
     * Lee un supplier tolerando tanto que el supplier sea nulo como que devuelva nulo. Lo segundo
     * pasa mas de lo que parece: un getter de hardware puede devolver nulo antes de que el
     * dispositivo termine de inicializarse.
     */
    private static <T> Optional<T> read(Supplier<T> supplier) {
        return supplier == null ? Optional.empty() : Optional.ofNullable(supplier.get());
    }
}
