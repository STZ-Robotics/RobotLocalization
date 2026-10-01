// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Optional;

/**
 * Lo que el robot code le escribe al Limelight por NetworkTables.
 *
 * <p>Un Limelight real no es solo una fuente de datos: tambien lee keys que el robot
 * escribe para configurarlo en caliente. La simulacion las lee y las respeta para que el
 * mismo codigo que llama a {@code LimelightHelpers.SetRobotOrientation} o
 * {@code SetFiducialIDFiltersOverride} tenga efecto aqui tambien.
 *
 * @param pipeline           indice de pipeline pedido (key {@code pipeline}).
 * @param ledMode            modo de LEDs pedido (key {@code ledMode}).
 * @param throttle           frames a saltarse entre capturas (key {@code throttle_set}).
 * @param robotHeading       yaw del robot para MegaTag2, de {@code robot_orientation_set}.
 * @param fiducialIdFilters  ids validos para localizacion, de {@code fiducial_id_filters_set};
 *                           vacio significa "sin filtro".
 * @param cameraPoseOverride transformada robot-&gt;camara pedida por
 *                           {@code camerapose_robotspace_set}, si el robot la escribio.
 */
public record LimelightNTInputs(
        int pipeline,
        int ledMode,
        int throttle,
        Optional<Rotation2d> robotHeading,
        int[] fiducialIdFilters,
        Optional<Transform3d> cameraPoseOverride) {

    /** Entradas neutras: sin filtros, sin heading y sin override de camara. */
    public static final LimelightNTInputs NONE =
            new LimelightNTInputs(0, 0, 0, Optional.empty(), new int[0], Optional.empty());

    /**
     * Indica si un id de AprilTag pasa el filtro que puso el robot code.
     *
     * @param id id del fiducial.
     * @return true si no hay filtro o si el id esta en la lista.
     */
    public boolean acceptsFiducial(int id) {
        if (fiducialIdFilters.length == 0) {
            return true;
        }
        for (int allowed : fiducialIdFilters) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }

    /** @return true si el robot code puso algun filtro de ids. */
    public boolean hasFiducialFilter() {
        return fiducialIdFilters.length > 0;
    }
}
