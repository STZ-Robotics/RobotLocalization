// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Optional;
import java.util.function.BiFunction;

/**
 * A Meta Quest running QuestNav as a pose source.
 *
 * <p>It wraps the vendordep's {@code QuestNav}: it consumes the unread frames, undoes the mounting
 * transform to go from the headset pose to the robot pose, and discards anything that arrives without
 * valid tracking.
 *
 * <h2>A VIO is not a camera</h2>
 * There are no tags, no distance and no ambiguity: a VIO's error does not depend on how far it is
 * from anything, but on how long it has gone without being corrected. That is why the default
 * strategy is {@link StdDevStrategies#vioTimeDrift}, and why you have to call
 * {@link #notifyPoseReset()} when the robot sends a {@code setPose} to the headset: that is where the
 * real drift goes back to zero, and the strategy has to hear about it.
 *
 * <h2>It keeps the newest frame</h2>
 * The headset publishes at 100 Hz and the robot runs at 50, so every cycle has two or more frames
 * pending. The last one with valid tracking is used. Feeding them all to the estimator would not be
 * wrong either, but the newest already contains the information of the earlier ones and costs half as
 * much.
 *
 * <h2>The timestamp warning</h2>
 * In simulation, if {@code SimulatedQuestNav} publishes on the same NetworkTables instance the robot
 * consumes, {@code PoseFrame.dataTimestamp()} is a constant 1 microsecond. This source detects that
 * and warns once instead of feeding the pose estimator measurements dated to year zero, which is
 * exactly the silent failure worth avoiding. The fix is to switch on
 * {@code withNetworkClient(true)} in the simulator.
 */
public class QuestNavPoseSource extends PoseSource {

    /**
     * Below this timestamp the value is assumed to be the NetworkTables sentinel rather than a real
     * instant. A robot never has a legitimate frame dated within the first millisecond.
     */
    private static final double SENTINEL_TIMESTAMP_SECONDS = 1e-3;

    private final QuestNav questNav;
    private final Transform3d robotToHeadset;
    private final StdDevStrategies.VioDriftStrategy driftStrategy;

    private double fieldMarginMeters = 0.5;
    private boolean requireTracking = true;
    private boolean sentinelWarningIssued = false;
    private String detail = "";

    /**
     * Creates the source with the recommended configuration for a VIO.
     *
     * @param name           source name.
     * @param robotToHeadset transform from the robot origin to the headset, the same
     *                       {@code ROBOT_TO_QUEST} your subsystem uses.
     * @return the source.
     */
    public static QuestNavPoseSource create(String name, Transform3d robotToHeadset) {
        return new QuestNavPoseSource(
                name, robotToHeadset, new QuestNav(), PoseSourceConfig.forVisualInertialOdometry());
    }

    /**
     * Creates the source.
     *
     * @param name           source name.
     * @param robotToHeadset transform from the robot origin to the headset.
     * @param questNav       the vendordep instance; injected so it can be tested.
     * @param config         filter thresholds.
     */
    public QuestNavPoseSource(
            String name, Transform3d robotToHeadset, QuestNav questNav, PoseSourceConfig config) {
        super(name, VecBuilder.fill(0.02, 0.02, 0.035), config);
        this.questNav = questNav;
        this.robotToHeadset = robotToHeadset;

        // A freshly corrected Quest is extremely precise and degrades over time. The numbers are a
        // reasonable starting point, not a characterisation of the hardware.
        this.driftStrategy = StdDevStrategies.vioTimeDrift(0.02, 0.035, 0.002, 0.0002);
        setStdDevsStrategy(driftStrategy);
    }

    /**
     * Margin around the field within which a pose is still valid.
     *
     * @param meters margin in metres.
     * @return this same source.
     */
    public QuestNavPoseSource withFieldMargin(double meters) {
        this.fieldMarginMeters = meters;
        return this;
    }

    /**
     * Controls whether frames without valid tracking are discarded. Switching it off is almost never
     * a good idea: during a tracking loss the headset repeats the last good pose, which very soon
     * stops describing where the robot is.
     *
     * @param enabled true to require tracking.
     * @return this same source.
     */
    public QuestNavPoseSource withRequireTracking(boolean enabled) {
        this.requireTracking = enabled;
        return this;
    }

    /**
     * Installs your own deviation strategy, replacing the drift one.
     *
     * @param strategy a strategy from {@link StdDevStrategies}.
     * @return this same source.
     */
    public QuestNavPoseSource withStdDevsStrategy(
            BiFunction<Double, Integer, Matrix<N3, N1>> strategy) {
        setStdDevsStrategy(strategy);
        return this;
    }

    /**
     * Signals that the headset has just been re-anchored and its drift went back to zero.
     *
     * <p>Call it right after {@code questNav.setPose(...)}. Without this the drift strategy keeps
     * inflating the uncertainty forever and the estimator ends up ignoring a sensor that has just
     * become perfect.
     */
    public void notifyPoseReset() {
        driftStrategy.reset();
    }

    /**
     * Re-anchors the headset and notifies the drift strategy, in a single step.
     *
     * @param robotPose the robot's field pose the headset should report.
     */
    public void resetPose(Pose2d robotPose) {
        questNav.setPose(new Pose3d(robotPose).transformBy(robotToHeadset));
        notifyPoseReset();
    }

    /** @return the QuestNav instance in use. */
    public QuestNav getQuestNav() {
        return questNav;
    }

    /** @return seconds since the headset was last corrected. */
    public double getSecondsSinceReset() {
        return driftStrategy.elapsedSeconds();
    }

    /**
     * Call this every cycle, alongside the rest of the robot code. It is what lets {@code QuestNav}
     * service its command responses.
     */
    public void periodic() {
        questNav.commandPeriodic();
    }

    @Override
    public Optional<PoseSample> getMeasurement() {
        if (!questNav.isConnected()) {
            // The measured latency is reported rather than the threshold: questnavlib decides the
            // exact criterion, and asserting a limit that is not the one in force confuses more than
            // it helps. If this number is sustainedly high, the frames are arriving in bursts --- look
            // at the simulator's publishing options, not at the headset.
            detail =
                    String.format(
                            "QuestNav considers it disconnected; the last frame arrived %.0f ms ago",
                            questNav.getLatency());
            return Optional.empty();
        }

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) {
            detail = "connected but no new frames this cycle";
            return Optional.empty();
        }

        PoseFrame newest = null;
        for (int i = frames.length - 1; i >= 0; i--) {
            if (!requireTracking || frames[i].isTracking()) {
                newest = frames[i];
                break;
            }
        }
        if (newest == null) {
            detail = frames.length + " frames, none with valid tracking";
            return Optional.empty();
        }

        if (!isTimestampUsable(newest.dataTimestamp())) {
            detail = "dataTimestamp = " + newest.dataTimestamp() + " s, that is the NT sentinel";
            return Optional.empty();
        }

        detail = String.format("frame %d, %.1f s since the last reset", newest.frameCount(),
                getSecondsSinceReset());

        Pose2d robotPose =
                newest.questPose3d().transformBy(robotToHeadset.inverse()).toPose2d();

        // Neither distance nor tags: the drift strategy does not care and ignores them.
        return Optional.of(
                new PoseSample(robotPose, newest.dataTimestamp(), computeStdDevs(0.0, 1)));
    }

    /**
     * Detects the NetworkTables sentinel timestamp.
     *
     * <p>Mis-dating a measurement is worse than not having it: the pose estimator interpolates it
     * backwards against its history and corrupts the result with no error raised. Better to discard
     * and say why.
     */
    private boolean isTimestampUsable(double timestamp) {
        if (timestamp > SENTINEL_TIMESTAMP_SECONDS) {
            return true;
        }
        if (!sentinelWarningIssued) {
            sentinelWarningIssued = true;
            DriverStation.reportWarning(
                    "[QuestNavPoseSource] '"
                            + getName()
                            + "': dataTimestamp is "
                            + timestamp
                            + " s, which is the NetworkTables sentinel for values published on the"
                            + " same instance that consumes them. Measurements are being discarded."
                            + " In simulation, switch on"
                            + " QuestNavSimConfig.withNetworkClient(true).",
                    false);
        }
        return false;
    }

    @Override
    public boolean isPoseValid(Pose2d pose) {
        boolean valid = PoseFusion.isInField(pose, fieldMarginMeters);
        if (!valid) {
            // It almost always means the same thing: nobody has called resetPose(), so the headset
            // is still reporting in its arbitrary origin rather than the field's.
            detail =
                    String.format(
                            "pose (%.2f, %.2f) outside the field: is a resetPose() missing?",
                            pose.getX(), pose.getY());
        }
        return valid;
    }

    @Override
    protected String telemetryDetail() {
        return detail;
    }
}
