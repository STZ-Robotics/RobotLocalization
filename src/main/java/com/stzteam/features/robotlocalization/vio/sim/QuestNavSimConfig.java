// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import gg.questnav.questnav.BuildConfig;

/**
 * Configuration for a {@link SimulatedQuestNav}.
 *
 * <p>The setters are fluent, just as in {@code LimelightSimConfig}:
 *
 * <pre>{@code
 * var config = new QuestNavSimConfig()
 *         .withDriftPerMeter(0.005)
 *         .withRandomizedOrigin(true);
 * }</pre>
 *
 * <h2>About the error model's default values</h2>
 * They are <b>plausible, not measured</b>. They are chosen so the order of magnitude is reasonable
 * (a well-mounted Quest drifts little) and so the failures show up within a short test. If you have
 * logs from your own headset, tune them; do not treat them as a characterisation of the hardware.
 */
public class QuestNavSimConfig {

    private String tableName = "QuestNav";
    private String version = BuildConfig.APP_VERSION;
    private long randomSeed = 0L;

    private boolean createSimDevice = true;
    private boolean publishTelemetry = true;
    private boolean viewfinder = false;
    private String visionWorldName = "main";
    private AprilTagFieldLayout fieldLayout = FieldLayouts.defaultField();

    private double frameRateHz = 100.0;
    private double deviceDataRateHz = 3.0;

    private boolean networkClient = false;
    private String clientName = "questnav-sim";
    private String serverAddress = "127.0.0.1";

    private boolean randomizedOrigin = false;
    private double randomOriginTranslationMeters = 1.0;
    private double randomOriginRotationDegrees = 30.0;

    private double driftPerMeter = 0.003;
    private double driftDegreesPerTurn = 0.5;
    private double verticalDriftFactor = 0.3;
    private double frameNoiseMeters = 0.001;
    private double frameNoiseDegrees = 0.02;

    private double reacquireOffsetMeters = 0.10;
    private double trackingLossSeconds = 0.5;
    private boolean impactTrackingLoss = true;
    private double impactAccelThreshold = 40.0;
    private double impactAngularAccelThreshold = 2000.0;

    private double initialBatteryPercent = 100.0;
    private double batteryDrainPercentPerMinute = 0.5;

    /** Creates a configuration with the default values. */
    public QuestNavSimConfig() {}

    /**
     * A preset with no error at all: the published pose is the exact truth. Useful to isolate a bug
     * in the robot code without noise getting in the way, and as a starting point for a test.
     *
     * @return a configuration with no drift, no noise and no automatic failures.
     */
    public static QuestNavSimConfig perfect() {
        return new QuestNavSimConfig()
                .withDriftPerMeter(0.0)
                .withDriftDegreesPerTurn(0.0)
                .withFrameNoise(0.0, 0.0)
                .withImpactTrackingLoss(false)
                .withBatteryDrainPercentPerMinute(0.0);
    }

    /**
     * NetworkTables table name. Only changed for tests: production robot code talks to
     * {@code "QuestNav"}, which is what {@code questnavlib} has hardcoded.
     *
     * @param name table name.
     * @return this same configuration.
     */
    public QuestNavSimConfig withTableName(String name) {
        this.tableName = name;
        return this;
    }

    /**
     * The version published to the {@code version} topic. By default it is that of the
     * {@code questnavlib} on the classpath, so the robot code's version check always passes. Change
     * it to test that your code warns about a mismatch.
     *
     * @param version version string.
     * @return this same configuration.
     */
    public QuestNavSimConfig withVersion(String version) {
        this.version = version;
        return this;
    }

    /**
     * Seed of the noise generator. Pinning it makes a run reproducible, which is what you want in a
     * test.
     *
     * @param seed the seed.
     * @return this same configuration.
     */
    public QuestNavSimConfig withRandomSeed(long seed) {
        this.randomSeed = seed;
        return this;
    }

    /**
     * Controls whether the HAL {@code SimDevice} that shows up under "Other Devices" is registered.
     *
     * @param enabled true to register it.
     * @return this same configuration.
     */
    public QuestNavSimConfig withSimDevice(boolean enabled) {
        this.createSimDevice = enabled;
        return this;
    }

    /**
     * Controls publication of the telemetry poses for AdvantageScope and of the {@code Field2d}.
     *
     * @param enabled true to publish them.
     * @return this same configuration.
     */
    public QuestNavSimConfig withTelemetry(boolean enabled) {
        this.publishTelemetry = enabled;
        return this;
    }

    /**
     * Switches on the MJPEG viewfinder: a {@code PhotonCameraSim} mounted where the headset goes,
     * which renders the field wireframe from its point of view and publishes it over
     * {@code CameraServer}.
     *
     * <p><b>It is cosmetic.</b> It does not feed the pose: the Quest localises by visual-inertial
     * odometry, not by seeing AprilTags. It is off by default because it renders with OpenCV on every
     * frame and that costs real CPU.
     *
     * @param enabled true to switch it on.
     * @return this same configuration.
     */
    public QuestNavSimConfig withViewfinder(boolean enabled) {
        this.viewfinder = enabled;
        return this;
    }

    /**
     * The shared simulated world the viewfinder uses. Set it to the same one as your cameras so
     * everything shares a single {@code VisionSystemSim}.
     *
     * @param name world identifier.
     * @return this same configuration.
     */
    public QuestNavSimConfig withVisionWorld(String name) {
        this.visionWorldName = name;
        return this;
    }

    /**
     * The AprilTag layout the viewfinder draws. It is only used for rendering: the Quest does not
     * localise with tags. If the simulated world already exists because another camera created it,
     * that camera's layout wins.
     *
     * @param layout the field layout.
     * @return this same configuration.
     */
    public QuestNavSimConfig withFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        return this;
    }

    /**
     * Rate at which the headset publishes frames. The real hardware runs at 100 Hz.
     *
     * @param hz rate in hertz.
     * @return this same configuration.
     */
    public QuestNavSimConfig withFrameRateHz(double hz) {
        this.frameRateHz = hz;
        return this;
    }

    /**
     * Rate of {@code deviceData} (battery and loss counter). The real hardware runs at 3 Hz.
     *
     * @param hz rate in hertz.
     * @return this same configuration.
     */
    public QuestNavSimConfig withDeviceDataRateHz(double hz) {
        this.deviceDataRateHz = hz;
        return this;
    }

    /**
     * Publishes from a separate NetworkTables instance, connected as a client to the robot's own NT
     * server, instead of writing directly to the global instance.
     *
     * <h2>Why this option exists</h2>
     * {@code PoseFrame.dataTimestamp()} --- the value you have to feed {@code addVisionMeasurement}
     * --- does not come from the protobuf: it comes from the {@code serverTime} NetworkTables stamps
     * on each value. And that {@code serverTime} is assigned by the <b>server on receipt</b>. When the
     * publisher lives in the same instance as the consumer, ntcore marks the values with the sentinel
     * {@code serverTime = 1}, so {@code dataTimestamp()} comes out as a constant 1 microsecond and any
     * timestamp-based fusion falls apart.
     *
     * <p>With this option on, the simulation reproduces the real topology --- the headset is an NT4
     * client and the robot is the server --- and the timestamps come out correct, network latency
     * included.
     *
     * <h2>Why it is off by default</h2>
     * If the program has no NT4 server listening, the client does not connect and <b>nothing</b> gets
     * published. Publishing with poor timestamps is a degraded failure; not publishing is a total one.
     * In local mode everything else --- pose, tracking, battery, commands --- works the same.
     *
     * <p>Switch it on if your robot code fuses on {@code dataTimestamp()}, which is the normal case
     * for a pose estimator.
     *
     * @param enabled true to publish as a network client.
     * @return this same configuration.
     */
    public QuestNavSimConfig withNetworkClient(boolean enabled) {
        this.networkClient = enabled;
        return this;
    }

    /**
     * The name the NT4 client identifies itself with, if {@link #withNetworkClient(boolean)} is on.
     *
     * @param name client name.
     * @return this same configuration.
     */
    public QuestNavSimConfig withClientName(String name) {
        this.clientName = name;
        return this;
    }

    /**
     * Address of the NT4 server to connect to. Loopback by default, which is where the robot listens
     * when you run {@code simulateJava}.
     *
     * @param address server address.
     * @return this same configuration.
     */
    public QuestNavSimConfig withServerAddress(String address) {
        this.serverAddress = address;
        return this;
    }

    /**
     * Starts with the Quest's origin randomly displaced instead of coinciding with the field's.
     *
     * <p>It is worth switching on at least once: with the origin at identity, a team that never calls
     * {@code setPose()} sees correct field poses by accident, and the bug does not show up until they
     * switch the real headset on.
     *
     * @param enabled true to randomise it.
     * @return this same configuration.
     */
    public QuestNavSimConfig withRandomizedOrigin(boolean enabled) {
        this.randomizedOrigin = enabled;
        return this;
    }

    /**
     * Magnitude of the random origin.
     *
     * @param translationMeters maximum displacement in metres.
     * @param rotationDegrees   maximum yaw deviation in degrees.
     * @return this same configuration.
     */
    public QuestNavSimConfig withRandomOriginRange(
            double translationMeters, double rotationDegrees) {
        this.randomOriginTranslationMeters = translationMeters;
        this.randomOriginRotationDegrees = rotationDegrees;
        return this;
    }

    /**
     * Position drift per metre travelled, as a fraction. 0.003 is 0.3 %.
     *
     * @param fraction drift fraction per metre.
     * @return this same configuration.
     */
    public QuestNavSimConfig withDriftPerMeter(double fraction) {
        this.driftPerMeter = fraction;
        return this;
    }

    /**
     * Heading drift per full turn of the robot, in degrees.
     *
     * @param degrees degrees of drift per turn.
     * @return this same configuration.
     */
    public QuestNavSimConfig withDriftDegreesPerTurn(double degrees) {
        this.driftDegreesPerTurn = degrees;
        return this;
    }

    /**
     * How much the Z axis drifts relative to the horizontal ones. A robot on the floor barely
     * drifts in height, hence the default factor below 1.
     *
     * @param factor multiplicative factor.
     * @return this same configuration.
     */
    public QuestNavSimConfig withVerticalDriftFactor(double factor) {
        this.verticalDriftFactor = factor;
        return this;
    }

    /**
     * Per-frame white noise, on top of the accumulated drift.
     *
     * @param meters  standard deviation in metres.
     * @param degrees yaw standard deviation in degrees.
     * @return this same configuration.
     */
    public QuestNavSimConfig withFrameNoise(double meters, double degrees) {
        this.frameNoiseMeters = meters;
        this.frameNoiseDegrees = degrees;
        return this;
    }

    /**
     * How far the pose shifts when tracking recovers. QuestNav's documentation warns that the
     * headset may re-acquire "with an offset", and that jump is the most damaging failure there is.
     *
     * @param meters magnitude of the jump in metres.
     * @return this same configuration.
     */
    public QuestNavSimConfig withReacquireOffsetMeters(double meters) {
        this.reacquireOffsetMeters = meters;
        return this;
    }

    /**
     * How long a tracking loss lasts.
     *
     * @param seconds duration in seconds.
     * @return this same configuration.
     */
    public QuestNavSimConfig withTrackingLossSeconds(double seconds) {
        this.trackingLossSeconds = seconds;
        return this;
    }

    /**
     * Controls whether a hard knock drops tracking on its own.
     *
     * @param enabled true to enable it.
     * @return this same configuration.
     */
    public QuestNavSimConfig withImpactTrackingLoss(boolean enabled) {
        this.impactTrackingLoss = enabled;
        return this;
    }

    /**
     * Thresholds above which a knock drops tracking.
     *
     * @param metersPerSecondSquared  linear acceleration.
     * @param degreesPerSecondSquared angular acceleration.
     * @return this same configuration.
     */
    public QuestNavSimConfig withImpactThresholds(
            double metersPerSecondSquared, double degreesPerSecondSquared) {
        this.impactAccelThreshold = metersPerSecondSquared;
        this.impactAngularAccelThreshold = degreesPerSecondSquared;
        return this;
    }


    /**
     * Battery charge at startup.
     *
     * @param percent percentage from 0 to 100.
     * @return this same configuration.
     */
    public QuestNavSimConfig withInitialBatteryPercent(double percent) {
        this.initialBatteryPercent = percent;
        return this;
    }

    /**
     * Battery drain per minute. On reaching zero the headset shuts off and stops publishing, just
     * like the real one.
     *
     * @param percentPerMinute percentage points per minute.
     * @return this same configuration.
     */
    public QuestNavSimConfig withBatteryDrainPercentPerMinute(double percentPerMinute) {
        this.batteryDrainPercentPerMinute = percentPerMinute;
        return this;
    }

    /** @return the NT table name. */
    public String tableName() {
        return tableName;
    }

    /** @return the published version. */
    public String version() {
        return version;
    }

    /** @return the noise generator's seed. */
    public long randomSeed() {
        return randomSeed;
    }

    /** @return true if the SimDevice should be registered. */
    public boolean createSimDevice() {
        return createSimDevice;
    }

    /** @return true if telemetry should be published. */
    public boolean publishTelemetry() {
        return publishTelemetry;
    }

    /** @return true if the MJPEG viewfinder is on. */
    public boolean viewfinder() {
        return viewfinder;
    }

    /** @return the name of the viewfinder's simulated world. */
    public String visionWorldName() {
        return visionWorldName;
    }

    /** @return the AprilTag layout the viewfinder draws. */
    public AprilTagFieldLayout fieldLayout() {
        return fieldLayout;
    }

    /** @return the frame publication rate. */
    public double frameRateHz() {
        return frameRateHz;
    }

    /** @return the deviceData publication rate. */
    public double deviceDataRateHz() {
        return deviceDataRateHz;
    }

    /** @return true if publication should happen from a NetworkTables client instance. */
    public boolean networkClient() {
        return networkClient;
    }

    /** @return the NT4 client name. */
    public String clientName() {
        return clientName;
    }

    /** @return the NT4 server address. */
    public String serverAddress() {
        return serverAddress;
    }

    /** @return true if the Quest's origin starts randomly displaced. */
    public boolean randomizedOrigin() {
        return randomizedOrigin;
    }

    /** @return maximum displacement of the random origin, in metres. */
    public double randomOriginTranslationMeters() {
        return randomOriginTranslationMeters;
    }

    /** @return maximum yaw deviation of the random origin, in degrees. */
    public double randomOriginRotationDegrees() {
        return randomOriginRotationDegrees;
    }

    /** @return the drift per metre, as a fraction. */
    public double driftPerMeter() {
        return driftPerMeter;
    }

    /** @return the heading drift per turn, in degrees. */
    public double driftDegreesPerTurn() {
        return driftDegreesPerTurn;
    }

    /** @return the vertical drift factor. */
    public double verticalDriftFactor() {
        return verticalDriftFactor;
    }

    /** @return the per-frame noise in metres. */
    public double frameNoiseMeters() {
        return frameNoiseMeters;
    }

    /** @return the per-frame yaw noise in degrees. */
    public double frameNoiseDegrees() {
        return frameNoiseDegrees;
    }

    /** @return the pose jump on re-acquisition, in metres. */
    public double reacquireOffsetMeters() {
        return reacquireOffsetMeters;
    }

    /** @return the duration of a tracking loss, in seconds. */
    public double trackingLossSeconds() {
        return trackingLossSeconds;
    }

    /** @return true if knocks drop tracking. */
    public boolean impactTrackingLoss() {
        return impactTrackingLoss;
    }

    /** @return the linear acceleration threshold for detecting a knock. */
    public double impactAccelThreshold() {
        return impactAccelThreshold;
    }

    /** @return the angular acceleration threshold for detecting a knock. */
    public double impactAngularAccelThreshold() {
        return impactAngularAccelThreshold;
    }


    /** @return the initial battery charge. */
    public double initialBatteryPercent() {
        return initialBatteryPercent;
    }

    /** @return the battery drain per minute. */
    public double batteryDrainPercentPerMinute() {
        return batteryDrainPercentPerMinute;
    }
}
