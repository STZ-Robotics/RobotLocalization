// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The headset's origin frame, and what happens to it when a POSE_RESET arrives.
 *
 * <p>This is the point of this feature that is easy to get wrong, the equivalent of the sign of
 * {@code tx} in the simulated Limelight. It lives in its own class, with pure tests, for that
 * reason.
 *
 * <h2>The misunderstanding it prevents</h2>
 * A Quest <b>does not publish the robot's field pose</b>. It publishes the headset's pose in an
 * arbitrary origin of its own, the place where it was when the app started. Making that origin
 * coincide with the field origin is something the robot code has to <i>cause</i>, by calling
 * {@code setPose()}. If the simulation published the field pose directly, it would be hiding
 * exactly the step most teams skip.
 *
 * <h2>The arithmetic</h2>
 * If the Quest's origin is at field pose {@code O} and the headset is physically at {@code H}, what
 * gets published is {@code H} seen from {@code O}:
 *
 * <pre>{@code published = H.relativeTo(O)}</pre>
 *
 * A POSE_RESET to {@code T} asks that, without moving the headset, what is published becomes
 * {@code T}. Solving the equation above for {@code O}:
 *
 * <pre>{@code O = H ∘ T⁻¹}</pre>
 */
public class QuestOriginFrame {

    private Pose3d origin;

    /** Creates a frame whose origin coincides with the field's. */
    public QuestOriginFrame() {
        this(Pose3d.kZero);
    }

    /**
     * Creates a frame with a specific origin.
     *
     * @param origin pose of the Quest's origin, in field coordinates.
     */
    public QuestOriginFrame(Pose3d origin) {
        this.origin = origin;
    }

    /**
     * Creates a frame with a randomly displaced origin, to force the robot code to do its seeding
     * properly. With the origin at identity, a team that never calls {@code setPose()} sees correct
     * field poses by accident and the bug does not show up until competition.
     *
     * @param translationMeters maximum displacement in metres.
     * @param rotationDegrees   maximum yaw deviation in degrees.
     * @param random            the randomness source, so the test stays reproducible.
     * @return a frame with an arbitrary origin.
     */
    public static QuestOriginFrame randomized(
            double translationMeters, double rotationDegrees, java.util.Random random) {
        double x = (random.nextDouble() * 2 - 1) * translationMeters;
        double y = (random.nextDouble() * 2 - 1) * translationMeters;
        double yaw = Math.toRadians((random.nextDouble() * 2 - 1) * rotationDegrees);
        return new QuestOriginFrame(
                new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d(0.0, 0.0, yaw)));
    }

    /**
     * Converts a headset field pose into what the Quest would publish.
     *
     * @param headsetFieldPose the headset's real field pose.
     * @return the pose in the Quest's frame.
     */
    public Pose3d toQuestFrame(Pose3d headsetFieldPose) {
        return headsetFieldPose.relativeTo(origin);
    }

    /**
     * Re-anchors the origin so a headset physically at {@code headsetFieldPose} starts reporting
     * {@code targetPose}. This is what a real POSE_RESET does.
     *
     * @param headsetFieldPose the headset's real field pose.
     * @param targetPose       the pose the robot code wants the Quest to report.
     */
    public void reset(Pose3d headsetFieldPose, Pose3d targetPose) {
        origin =
                headsetFieldPose.transformBy(
                        new Transform3d(targetPose.getTranslation(), targetPose.getRotation()).inverse());
    }

    /** @return the pose of the Quest's origin in field coordinates. */
    public Pose3d origin() {
        return origin;
    }
}
