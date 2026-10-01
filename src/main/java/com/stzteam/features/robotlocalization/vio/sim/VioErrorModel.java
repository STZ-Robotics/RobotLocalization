// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.Random;

/**
 * The visual-inertial odometry error model. It is the real content of this feature.
 *
 * <p>A Limelight either sees the tag or it does not. A Quest <b>always</b> reports a pose; what
 * changes is how much it lies. Simulating the protocol without simulating the lie would give a
 * perfect sensor, which is exactly the one nobody needs to test against.
 *
 * <h2>Continuous drift</h2>
 * The drift scales with <b>distance travelled</b>, not with time: a stationary robot does not drift,
 * and one crossing the field does. That is how real VIO behaves. The yaw drifts with accumulated
 * rotation, for the same reason.
 *
 * <h2>Failure modes</h2>
 * The four QuestNav documents, each injectable from the Sim GUI or from a test:
 *
 * <ul>
 *   <li><b>Feature loss</b> (low light, reflections, uniform surfaces). The documentation says the
 *       Quest may re-acquire tracking <i>with an offset</i>, so when it recovers the pose comes back
 *       displaced. That jump is what breaks a finely tuned pose estimator.</li>
 *   <li><b>Vibration-induced double-tap to passthrough</b>: a pose discontinuity <i>without</i>
 *       losing tracking. More treacherous, because {@code isTracking} stays true.</li>
 *   <li><b>Critical battery</b>: the headset shuts itself off mid-match.</li>
 *   <li><b>Network drop</b>: publishing stops and {@code isConnected()} falls after 120 ms.</li>
 * </ul>
 *
 * <p>The last two are not pose error but absence of data, so {@link SimulatedQuestNav} handles them
 * by consulting {@link #isPoweredOn()} and {@link #isLinkUp(double)}.
 *
 * <h2>About the default values</h2>
 * They are <b>plausible, not measured</b>. They are chosen so the order of magnitude is reasonable
 * and so the failures show up within a short test. If you have logs from your own headset, tune
 * them.
 *
 * <h2>A deliberate simplification</h2>
 * Position drift and yaw drift are modelled as additive and independent. In a real VIO the heading
 * error also curves the estimated trajectory. Reproducing that would require dragging a drifting
 * reference frame around, and for this feature's purpose --- breaking a pose estimator realistically
 * --- it changes nothing: what matters is that the error grows with distance travelled, that it jumps
 * on re-acquisition, and that a POSE_RESET clears it.
 */
public class VioErrorModel {

    private final QuestNavSimConfig config;
    private final Random random;

    private Translation3d driftTranslation = Translation3d.kZero;
    private double driftYawRadians = 0.0;

    private Pose3d previousTruePose;
    private Pose3d frozenPose;

    private boolean tracking = true;
    private int trackingLostCounter = 0;
    private double trackingLostUntilSeconds = Double.NEGATIVE_INFINITY;
    private double linkDownUntilSeconds = Double.NEGATIVE_INFINITY;
    private double batteryPercent;

    private double lastSpeedMetersPerSecond = 0.0;
    private double lastAngularSpeedDegreesPerSecond = 0.0;

    // Median window to separate a collision from a pose estimator jump. See detectImpact.
    private final double[] speedWindow = new double[3];
    private final double[] angularWindow = new double[3];
    private int windowIndex = 0;
    private int windowFill = 0;
    private double lastMedianSpeed = Double.NaN;
    private double lastMedianAngular = Double.NaN;

    /**
     * Creates the model.
     *
     * @param config the simulated camera's configuration, where every parameter comes from.
     */
    public VioErrorModel(QuestNavSimConfig config) {
        this.config = config;
        this.random = new Random(config.randomSeed());
        this.batteryPercent = config.initialBatteryPercent();
    }

    /**
     * Advances the model one step and returns the pose the headset should report.
     *
     * @param truePose    the headset's real pose (robot + mount), according to the simulation.
     * @param nowSeconds  the current instant, in FPGA seconds.
     * @param dtSeconds   time elapsed since the previous call.
     * @return the pose with the error applied. If tracking is lost, the frozen pose.
     */
    public Pose3d update(Pose3d truePose, double nowSeconds, double dtSeconds) {
        drainBattery(dtSeconds);

        if (previousTruePose == null) {
            previousTruePose = truePose;
            frozenPose = truePose;
        }

        double distance = truePose.getTranslation().getDistance(previousTruePose.getTranslation());
        double rotation =
                Math.abs(
                        truePose.getRotation().toRotation2d().minus(
                                previousTruePose.getRotation().toRotation2d()).getRadians());

        if (dtSeconds > 0) {
            double speed = distance / dtSeconds;
            double angularSpeed = Units.radiansToDegrees(rotation) / dtSeconds;
            detectImpact(speed, angularSpeed, dtSeconds, nowSeconds);
            lastSpeedMetersPerSecond = speed;
            lastAngularSpeedDegreesPerSecond = angularSpeed;
        }

        previousTruePose = truePose;

        if (nowSeconds < trackingLostUntilSeconds) {
            // Without tracking the Quest has nothing new to say: it repeats the last good pose.
            tracking = false;
            return frozenPose;
        }

        if (!tracking) {
            // Re-acquisition. QuestNav's documentation warns that tracking can come back "with an
            // offset": that jump is the sensor's most destructive failure and it has to be
            // reproducible, not smoothed away.
            tracking = true;
            driftTranslation = driftTranslation.plus(randomOffset(config.reacquireOffsetMeters()));
        }

        accumulateDrift(distance, rotation);

        Pose3d reported = applyError(truePose);
        frozenPose = reported;
        return reported;
    }

    private void accumulateDrift(double distanceMeters, double rotationRadians) {
        if (distanceMeters > 0) {
            double sigma = config.driftPerMeter() * distanceMeters;
            driftTranslation =
                    driftTranslation.plus(
                            new Translation3d(
                                    random.nextGaussian() * sigma,
                                    random.nextGaussian() * sigma,
                                    random.nextGaussian() * sigma * config.verticalDriftFactor()));
        }

        if (rotationRadians > 0) {
            double turns = rotationRadians / (2 * Math.PI);
            driftYawRadians +=
                    random.nextGaussian() * Units.degreesToRadians(config.driftDegreesPerTurn()) * turns;
        }
    }

    private Pose3d applyError(Pose3d truePose) {
        Translation3d noise =
                new Translation3d(
                        random.nextGaussian() * config.frameNoiseMeters(),
                        random.nextGaussian() * config.frameNoiseMeters(),
                        random.nextGaussian() * config.frameNoiseMeters());
        double yawNoise = random.nextGaussian() * Units.degreesToRadians(config.frameNoiseDegrees());

        return new Pose3d(
                truePose.getTranslation().plus(driftTranslation).plus(noise),
                truePose.getRotation().plus(new Rotation3d(0.0, 0.0, driftYawRadians + yawNoise)));
    }

    /**
     * A hard knock knocks the Quest's tracking out. It is detected from acceleration or angular
     * acceleration, not from velocity: crossing the field fast breaks nothing, hitting the wall does.
     *
     * <h2>Why the velocity is median-filtered</h2>
     * The pose that comes in here usually comes from the robot's pose estimator, and a pose estimator
     * <b>jumps</b> every time it accepts a vision measurement. A 20 cm jump in a 10 ms frame is
     * 2000 m/s2 of apparent acceleration --- fifty times the threshold --- so raw, the detector
     * mistakes every vision correction for a collision.
     *
     * <p>Counting consecutive frames does not fix it, because hitting the wall is also a single
     * frame: the pose just stops advancing. What really tells them apart is the shape.
     *
     * <ul>
     *   <li>A vision correction is an <b>impulse</b>: the apparent velocity spikes for one frame and
     *       returns to the value it had.</li>
     *   <li>A collision is a <b>step</b>: the velocity was high and stays low.</li>
     * </ul>
     *
     * <p>A median of three samples is exactly the tool for that: it eats impulses and respects steps.
     * The acceleration is computed on the filtered velocity, not on the raw one.
     *
     * <p>This came out of running the feature against a live simulation: the Quest lost tracking on
     * 68% of the cycles, each re-acquisition added its offset, and the pose ended up off the field.
     */
    private void detectImpact(
            double speed, double angularSpeed, double dtSeconds, double nowSeconds) {
        if (!config.impactTrackingLoss()) {
            return;
        }

        speedWindow[windowIndex] = speed;
        angularWindow[windowIndex] = angularSpeed;
        windowIndex = (windowIndex + 1) % speedWindow.length;
        windowFill = Math.min(windowFill + 1, speedWindow.length);

        if (windowFill < speedWindow.length) {
            return;
        }

        double medianSpeed = median3(speedWindow);
        double medianAngular = median3(angularWindow);

        if (!Double.isNaN(lastMedianSpeed)) {
            double acceleration = Math.abs(medianSpeed - lastMedianSpeed) / dtSeconds;
            double angularAcceleration = Math.abs(medianAngular - lastMedianAngular) / dtSeconds;

            if (acceleration > config.impactAccelThreshold()
                    || angularAcceleration > config.impactAngularAccelThreshold()) {
                injectTrackingLoss(config.trackingLossSeconds(), nowSeconds);
            }
        }

        lastMedianSpeed = medianSpeed;
        lastMedianAngular = medianAngular;
    }

    /** Median of three values, without actually sorting the array. */
    private static double median3(double[] values) {
        double a = values[0];
        double b = values[1];
        double c = values[2];
        return Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
    }

    private void drainBattery(double dtSeconds) {
        batteryPercent =
                Math.max(0.0, batteryPercent - config.batteryDrainPercentPerMinute() * dtSeconds / 60.0);
    }

    private Translation3d randomOffset(double magnitudeMeters) {
        double heading = random.nextDouble() * 2 * Math.PI;
        return new Translation3d(
                Math.cos(heading) * magnitudeMeters, Math.sin(heading) * magnitudeMeters, 0.0);
    }

    /**
     * Forces a tracking loss. On recovery, the pose comes back displaced.
     *
     * @param durationSeconds how long the loss lasts.
     * @param nowSeconds      the current instant, in FPGA seconds.
     */
    public void injectTrackingLoss(double durationSeconds, double nowSeconds) {
        if (!tracking && nowSeconds < trackingLostUntilSeconds) {
            return; // it was already lost; this does not count as a new loss
        }
        tracking = false;
        trackingLostCounter++;
        trackingLostUntilSeconds = nowSeconds + durationSeconds;
    }

    /**
     * Injects a pose discontinuity without losing tracking. It reproduces the double-tap to
     * passthrough that chassis vibration triggers: {@code isTracking} stays true, so the robot code
     * has no way of knowing the pose has just jumped.
     *
     * @param jump the displacement to apply, in metres.
     */
    public void injectPoseDiscontinuity(Translation3d jump) {
        driftTranslation = driftTranslation.plus(jump);
    }

    /**
     * Simulates a network drop or a loose cable: publishing stops and the robot sees
     * {@code isConnected()} fall after 120 ms.
     *
     * @param durationSeconds how long the drop lasts.
     * @param nowSeconds      the current instant, in FPGA seconds.
     */
    public void injectLinkLoss(double durationSeconds, double nowSeconds) {
        linkDownUntilSeconds = nowSeconds + durationSeconds;
    }

    /**
     * Clears the accumulated drift. The POSE_RESET handler calls it, because a real reset
     * re-anchors the headset's origin and leaves the error at zero.
     */
    public void resetDrift() {
        driftTranslation = Translation3d.kZero;
        driftYawRadians = 0.0;
    }

    /**
     * Sets the battery charge. Useful to test the shutdown without waiting for it to drain.
     *
     * @param percent percentage from 0 to 100.
     */
    public void setBatteryPercent(double percent) {
        batteryPercent = Math.max(0.0, Math.min(100.0, percent));
    }

    /** @return true if the headset is reporting valid tracking. */
    public boolean isTracking() {
        return tracking;
    }

    /** @return how many times tracking has been lost since startup. */
    public int trackingLostCounter() {
        return trackingLostCounter;
    }

    /** @return the battery charge, rounded the way the headset reports it. */
    public int batteryPercent() {
        return (int) Math.round(batteryPercent);
    }

    /** @return false once the battery hit zero and the headset shut off. */
    public boolean isPoweredOn() {
        return batteryPercent > 0.0;
    }

    /**
     * Says whether the network link is up.
     *
     * @param nowSeconds the current instant, in FPGA seconds.
     * @return false for as long as an injected drop lasts.
     */
    public boolean isLinkUp(double nowSeconds) {
        return nowSeconds >= linkDownUntilSeconds;
    }

    /** @return the accumulated translation drift, for telemetry. */
    public Translation3d driftTranslation() {
        return driftTranslation;
    }

    /** @return the accumulated yaw drift in degrees, for telemetry. */
    public double driftYawDegrees() {
        return Units.radiansToDegrees(driftYawRadians);
    }
}
