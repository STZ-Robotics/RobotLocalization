// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * One frame of the simulated QuestNav, ready to publish.
 *
 * <p>The Quest publishes at 100 Hz and the robot cycle runs at 50, so every cycle generates several
 * of these frames, with NetworkTables timestamps 10 ms apart. That is the biggest shape difference
 * from the simulated Limelight, which publishes at most one per cycle.
 *
 * @param questPose            headset pose, in the Quest's own origin frame and already with the VIO
 *                             model's error applied. This is literally what goes into the protobuf's
 *                             {@code pose3d} field.
 * @param truePose             the robot's real pose according to the simulation, for telemetry only.
 *                             It is never published to the {@code QuestNav} table.
 * @param tracking             value of the {@code isTracking} field.
 * @param frameCount           the Quest's frame counter, monotonic.
 * @param appTimestampSeconds  seconds since the Quest app started (the protobuf's {@code timestamp}
 *                             field). Note: the robot code does not use this value to date
 *                             measurements, it uses the NetworkTables timestamp.
 * @param ntTimestampMicros    the instant the frame is published, in microseconds of the
 *                             NetworkTables time base.
 */
public record QuestNavFrame(
        Pose3d questPose,
        Pose3d truePose,
        boolean tracking,
        int frameCount,
        double appTimestampSeconds,
        long ntTimestampMicros) {

    /**
     * Distance between the pose the Quest reports (converted to a robot pose) and the truth. It is
     * the metric that makes the drift visible in AdvantageScope.
     *
     * @param reportedRobotPose the robot pose deduced from the headset.
     * @return the error in metres.
     */
    public double translationErrorMeters(Pose3d reportedRobotPose) {
        return reportedRobotPose.getTranslation().getDistance(truePose.getTranslation());
    }
}
