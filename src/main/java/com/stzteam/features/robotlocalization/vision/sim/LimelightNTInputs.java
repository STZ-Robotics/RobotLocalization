// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Optional;

/**
 * What the robot code writes to the Limelight over NetworkTables.
 *
 * <p>A real Limelight is not only a data source: it also reads keys the robot writes to
 * configure it on the fly. The simulation reads and honours them so that the same code
 * calling {@code LimelightHelpers.SetRobotOrientation} or
 * {@code SetFiducialIDFiltersOverride} takes effect here too.
 *
 * @param pipeline           requested pipeline index (key {@code pipeline}).
 * @param ledMode            requested LED mode (key {@code ledMode}).
 * @param throttle           frames to skip between captures (key {@code throttle_set}).
 * @param robotHeading       robot yaw for MegaTag2, from {@code robot_orientation_set}.
 * @param fiducialIdFilters  ids valid for localization, from {@code fiducial_id_filters_set};
 *                           empty means "no filter".
 * @param cameraPoseOverride robot-&gt;camera transform requested by
 *                           {@code camerapose_robotspace_set}, if the robot wrote one.
 */
public record LimelightNTInputs(
        int pipeline,
        int ledMode,
        int throttle,
        Optional<Rotation2d> robotHeading,
        int[] fiducialIdFilters,
        Optional<Transform3d> cameraPoseOverride) {

    /** Neutral inputs: no filters, no heading and no camera override. */
    public static final LimelightNTInputs NONE =
            new LimelightNTInputs(0, 0, 0, Optional.empty(), new int[0], Optional.empty());

    /**
     * Says whether an AprilTag id passes the filter the robot code installed.
     *
     * @param id fiducial id.
     * @return true if there is no filter or if the id is in the list.
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

    /** @return true if the robot code installed any id filter. */
    public boolean hasFiducialFilter() {
        return fiducialIdFilters.length > 0;
    }
}
