// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.simulation.SimCameraProperties;

/**
 * Configuration for a {@link SimulatedLimelight}: everything a prototype would hardcode
 * (NT table name, camera optics, which keys to publish) lives here.
 *
 * <p>The setters are fluent and return {@code this}, so they chain:
 *
 * <pre>{@code
 * var config = LimelightSimConfig.limelight3G()
 *         .withTableName("limelight-front")
 *         .withMegaTag2(true);
 * }</pre>
 *
 * <p>The camera presets start from Limelight's published specifications and are
 * reasonable approximations, not real calibrations. If you have your camera's
 * calibration, pass it with {@link #withCameraProperties(SimCameraProperties)}.
 */
public class LimelightSimConfig {

    /**
     * Each of the origins a Limelight publishes the robot pose in.
     *
     * <p>A real Limelight publishes all three, and with MegaTag2 that is six arrays in total. Each
     * array is {@code 11 + 7 * tags} doubles: with three tags visible, measured against a live
     * simulation, the six add up to <b>0.19 Mbps</b> of NetworkTables at a 10 Hz subscription --- and
     * almost everybody reads just one.
     */
    public enum BotposeVariant {
        /** {@code botpose}: legacy origin at the centre of the field. */
        LEGACY_CENTER,
        /** {@code botpose_wpiblue}: origin at the blue corner. This is the one almost everybody uses. */
        WPI_BLUE,
        /** {@code botpose_wpired}: origin at the red corner. */
        WPI_RED
    }

    private String tableName = "limelight";
    private String visionWorldName = "main";
    private AprilTagFieldLayout fieldLayout = FieldLayouts.defaultField();
    private SimCameraProperties cameraProperties = limelight3GProperties();

    private boolean createSimDevice = true;
    private boolean publishBotpose = true;
    private boolean publishMegaTag2 = true;
    private boolean publishRawFiducials = true;
    private boolean publishCorners = false;
    private boolean publishTargetSpacePoses = false;

    private java.util.EnumSet<BotposeVariant> botposeVariants =
            java.util.EnumSet.allOf(BotposeVariant.class);

    private double maxSightRangeMeters = 7.0;
    private double minTargetAreaPixels = 200.0;

    private boolean rawStream = false;
    private boolean wireframe = false;
    private boolean processedStream = false;
    private boolean publishDebugField = false;

    private boolean honorCameraPoseOverride = true;
    private boolean honorFiducialFilters = true;
    private boolean honorThrottle = true;

    private double megaTag2HeadingScaleFactor = 0.02;

    /** Creates a configuration with the default values (Limelight 3G profile). */
    public LimelightSimConfig() {}

    /**
     * Approximate preset for a Limelight 2/2+ at 1280x720, taken straight from the
     * presets photonlib ships.
     *
     * @return a new configuration with that camera profile.
     */
    public static LimelightSimConfig limelight2() {
        return new LimelightSimConfig().withCameraProperties(SimCameraProperties.LL2_1280_720());
    }

    /**
     * Approximate preset for a Limelight 3: 1280x800, ~75 degree diagonal FOV, 90 FPS.
     *
     * @return a new configuration with that camera profile.
     */
    public static LimelightSimConfig limelight3() {
        return new LimelightSimConfig().withCameraProperties(limelight3Properties());
    }

    /**
     * Approximate preset for a Limelight 3G: same resolution as the 3 but wider optics,
     * ~91 degree diagonal FOV.
     *
     * @return a new configuration with that camera profile.
     */
    public static LimelightSimConfig limelight3G() {
        return new LimelightSimConfig().withCameraProperties(limelight3GProperties());
    }

    /**
     * Approximate preset for a Limelight 4: the 3G's optics with more FPS and less latency.
     *
     * @return a new configuration with that camera profile.
     */
    public static LimelightSimConfig limelight4() {
        SimCameraProperties props = limelight3GProperties();
        props.setFPS(120);
        props.setAvgLatencyMs(8);
        props.setLatencyStdDevMs(1.5);
        return new LimelightSimConfig().withCameraProperties(props);
    }

    private static SimCameraProperties limelight3Properties() {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(75.2));
        props.setCalibError(0.25, 0.08);
        props.setFPS(90);
        props.setAvgLatencyMs(11);
        props.setLatencyStdDevMs(2);
        return props;
    }

    private static SimCameraProperties limelight3GProperties() {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(91.2));
        props.setCalibError(0.25, 0.08);
        props.setFPS(90);
        props.setAvgLatencyMs(11);
        props.setLatencyStdDevMs(2);
        return props;
    }

    /**
     * Name of the NetworkTables table the keys are published to. It has to be the same
     * string the robot code passes to {@code LimelightHelpers}.
     *
     * @param name table name, for example {@code "limelight-front"}.
     * @return this same configuration.
     */
    public LimelightSimConfig withTableName(String name) {
        this.tableName = (name == null || name.isBlank()) ? "limelight" : name;
        return this;
    }

    /**
     * Name of the shared simulated world. Several cameras with the same name share a
     * single {@code VisionSystemSim}, which is what you want for multi-camera setups.
     *
     * @param name world identifier.
     * @return this same configuration.
     */
    public LimelightSimConfig withVisionWorld(String name) {
        this.visionWorldName = name;
        return this;
    }

    /**
     * The AprilTag layout detections are simulated and poses estimated against.
     *
     * @param layout the field layout.
     * @return this same configuration.
     */
    public LimelightSimConfig withFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        return this;
    }

    /**
     * Optical and timing properties of the simulated camera.
     *
     * @param properties photonlib properties.
     * @return this same configuration.
     */
    public LimelightSimConfig withCameraProperties(SimCameraProperties properties) {
        this.cameraProperties = properties;
        return this;
    }

    /**
     * Controls whether a HAL {@code SimDevice} is registered so the camera shows up in the
     * Sim GUI's "Other Devices" panel.
     *
     * @param enabled true to register it.
     * @return this same configuration.
     */
    public LimelightSimConfig withSimDevice(boolean enabled) {
        this.createSimDevice = enabled;
        return this;
    }

    /**
     * Controls publication of {@code botpose}, {@code botpose_wpiblue} and
     * {@code botpose_wpired}.
     *
     * @param enabled true to publish them.
     * @return this same configuration.
     */
    public LimelightSimConfig withBotpose(boolean enabled) {
        this.publishBotpose = enabled;
        return this;
    }

    /**
     * Controls publication of the {@code botpose_orb*} keys (the MegaTag2 equivalent). It
     * requires the robot code to call {@code LimelightHelpers.SetRobotOrientation} every
     * cycle; otherwise those keys stay empty, exactly as with real hardware.
     *
     * @param enabled true to publish them.
     * @return this same configuration.
     */
    public LimelightSimConfig withMegaTag2(boolean enabled) {
        this.publishMegaTag2 = enabled;
        return this;
    }

    /**
     * Controls publication of the {@code rawfiducials} key.
     *
     * @param enabled true to publish it.
     * @return this same configuration.
     */
    public LimelightSimConfig withRawFiducials(boolean enabled) {
        this.publishRawFiducials = enabled;
        return this;
    }

    /**
     * Controls publication of {@code tcornxy}. Off by default because it is the heaviest
     * array and almost nobody uses it.
     *
     * @param enabled true to publish it.
     * @return this same configuration.
     */
    public LimelightSimConfig withCorners(boolean enabled) {
        this.publishCorners = enabled;
        return this;
    }

    /**
     * Controls publication of {@code targetpose_cameraspace},
     * {@code targetpose_robotspace}, {@code camerapose_targetspace} and
     * {@code botpose_targetspace}.
     *
     * <p>Off by default on purpose: Limelight's target-space axis convention (X right, Y
     * down, Z outward) is the part of the schema easiest to misread, and these keys staying
     * obviously empty is preferable to them publishing plausible but wrong numbers. If you
     * switch them on, validate the signs against your real hardware before trusting them.
     *
     * @param enabled true to publish them.
     * @return this same configuration.
     */
    public LimelightSimConfig withTargetSpacePoses(boolean enabled) {
        this.publishTargetSpacePoses = enabled;
        return this;
    }

    /**
     * Distance beyond which the simulated camera stops seeing a tag.
     *
     * <p><b>This is not fine tuning, it is what separates a camera from an X-ray.</b> Photonlib does
     * not limit the range by default, and {@code VisionSystemSim} does not model occlusion: without a
     * limit, the camera sees every tag inside its FOV however far away it is, straight through the
     * field walls.
     *
     * <p>Measured against the 2026 field with the robot at (2.7, 2.6): with no limit, seven tags came
     * in, five of them between 8 and 13 m with ambiguities from 0.6 to 0.99, and the multi-tag PnP
     * mixed them with the two good nearby ones and returned a pose twelve metres from the robot and a
     * metre and a half off the ground.
     *
     * <p>The 7 m default is what a Limelight 3G reliably resolves on a 16.5 cm tag. If you measure
     * yours, use the number you measured.
     *
     * @param meters maximum range in metres.
     * @return this same configuration.
     */
    public LimelightSimConfig withMaxSightRange(double meters) {
        this.maxSightRangeMeters = meters;
        return this;
    }

    /**
     * Minimum area in pixels for a tag to count as detected.
     *
     * <p>It is the other brake against impossible tags: one far away covers a handful of pixels and no
     * real detector picks it out. Photonlib uses 100 by default; here it is 200, which at 1280x800
     * corresponds roughly to the useful limit of a 16.5 cm tag.
     *
     * @param pixels minimum area in pixels.
     * @return this same configuration.
     */
    public LimelightSimConfig withMinTargetAreaPixels(double pixels) {
        this.minTargetAreaPixels = pixels;
        return this;
    }

    /**
     * Chooses which origins the robot pose is published in.
     *
     * <p>All three are published by default, which is what a real Limelight does. If your robot code
     * only reads one --- the normal case --- keeping just that one cuts two thirds of the NetworkTables
     * bandwidth this camera spends.
     *
     * <pre>{@code config.withBotposeVariants(BotposeVariant.WPI_BLUE);}</pre>
     *
     * @param variants the origins to publish; with no arguments, none are published.
     * @return this same configuration.
     */
    public LimelightSimConfig withBotposeVariants(BotposeVariant... variants) {
        this.botposeVariants =
                variants.length == 0
                        ? java.util.EnumSet.noneOf(BotposeVariant.class)
                        : java.util.EnumSet.of(variants[0], variants);
        return this;
    }

    /**
     * Publishes, over {@code CameraServer}, the raw MJPEG stream photonlib renders: the AprilTags as
     * seen from the camera, at <code>&lt;name&gt;-raw</code>. It shows up in Glass, Shuffleboard or
     * AdvantageScope.
     *
     * <p>Off by default because it renders with OpenCV on every frame and costs real CPU.
     *
     * @param enabled true to publish it.
     * @return this same configuration.
     */
    public LimelightSimConfig withRawStream(boolean enabled) {
        this.rawStream = enabled;
        return this;
    }

    /**
     * Draws the field wireframe on the raw stream. It helps a lot to understand where the camera is
     * looking when there is no tag in sight.
     *
     * <p>Requires {@link #withRawStream(boolean)}.
     *
     * @param enabled true to draw it.
     * @return this same configuration.
     */
    public LimelightSimConfig withWireframe(boolean enabled) {
        this.wireframe = enabled;
        return this;
    }

    /**
     * Also publishes the annotated stream, with the crosshair and the outlines of the detected
     * targets, at <code>&lt;name&gt;-processed</code>.
     *
     * @param enabled true to publish it.
     * @return this same configuration.
     */
    public LimelightSimConfig withProcessedStream(boolean enabled) {
        this.processedStream = enabled;
        return this;
    }

    /**
     * Publishes to {@code SmartDashboard} the debug {@code Field2d} photonlib maintains, with the
     * robot pose, the camera poses and the poses of the visible targets.
     *
     * @param enabled true to publish it.
     * @return this same configuration.
     */
    public LimelightSimConfig withDebugField(boolean enabled) {
        this.publishDebugField = enabled;
        return this;
    }

    /**
     * Controls whether the simulation honours {@code camerapose_robotspace_set}, that is,
     * whether the robot code can move the camera on the fly as on a real Limelight.
     *
     * @param enabled true to honour it.
     * @return this same configuration.
     */
    public LimelightSimConfig withCameraPoseOverride(boolean enabled) {
        this.honorCameraPoseOverride = enabled;
        return this;
    }

    /**
     * Controls whether the simulation honours {@code fiducial_id_filters_set} when estimating poses.
     *
     * @param enabled true to honour it.
     * @return this same configuration.
     */
    public LimelightSimConfig withFiducialFilters(boolean enabled) {
        this.honorFiducialFilters = enabled;
        return this;
    }

    /**
     * Controls whether the simulation honours {@code throttle_set} by skipping frames.
     *
     * @param enabled true to honour it.
     * @return this same configuration.
     */
    public LimelightSimConfig withThrottle(boolean enabled) {
        this.honorThrottle = enabled;
        return this;
    }

    /**
     * Weight of the heading in the MegaTag2 solver. Higher values penalise straying from
     * the yaw the robot code reported more heavily.
     *
     * @param factor scale factor, 0.02 by default.
     * @return this same configuration.
     */
    public LimelightSimConfig withMegaTag2HeadingScaleFactor(double factor) {
        this.megaTag2HeadingScaleFactor = factor;
        return this;
    }

    /** @return the configured NT table name. */
    public String tableName() {
        return tableName;
    }

    /** @return the name of the shared simulated world. */
    public String visionWorldName() {
        return visionWorldName;
    }

    /** @return the configured AprilTag layout. */
    public AprilTagFieldLayout fieldLayout() {
        return fieldLayout;
    }

    /** @return the simulated camera's properties. */
    public SimCameraProperties cameraProperties() {
        return cameraProperties;
    }

    /** @return true if the HAL SimDevice should be registered. */
    public boolean createSimDevice() {
        return createSimDevice;
    }

    /** @return true if the botpose keys should be published. */
    public boolean publishBotpose() {
        return publishBotpose;
    }

    /** @return true if the MegaTag2 keys should be published. */
    public boolean publishMegaTag2() {
        return publishMegaTag2;
    }

    /** @return true if rawfiducials should be published. */
    public boolean publishRawFiducials() {
        return publishRawFiducials;
    }

    /** @return true if tcornxy should be published. */
    public boolean publishCorners() {
        return publishCorners;
    }

    /** @return true if the target-space poses should be published. */
    public boolean publishTargetSpacePoses() {
        return publishTargetSpacePoses;
    }

    /** @return the camera's maximum range in metres. */
    public double maxSightRangeMeters() {
        return maxSightRangeMeters;
    }

    /** @return minimum area in pixels to detect a tag. */
    public double minTargetAreaPixels() {
        return minTargetAreaPixels;
    }

    /**
     * Says whether a specific origin should be published.
     *
     * @param variant the origin to check.
     * @return true if it is in the configured list.
     */
    public boolean publishesBotposeVariant(BotposeVariant variant) {
        return botposeVariants.contains(variant);
    }

    /** @return true if the raw MJPEG stream should be published. */
    public boolean rawStream() {
        return rawStream;
    }

    /** @return true if the field wireframe should be drawn. */
    public boolean wireframe() {
        return wireframe;
    }

    /** @return true if the annotated stream should be published. */
    public boolean processedStream() {
        return processedStream;
    }

    /** @return true if the debug Field2d should be published. */
    public boolean publishDebugField() {
        return publishDebugField;
    }

    /** @return true if camerapose_robotspace_set is honoured. */
    public boolean honorCameraPoseOverride() {
        return honorCameraPoseOverride;
    }

    /** @return true if fiducial_id_filters_set is honoured. */
    public boolean honorFiducialFilters() {
        return honorFiducialFilters;
    }

    /** @return true if throttle_set is honoured. */
    public boolean honorThrottle() {
        return honorThrottle;
    }

    /** @return the heading scale factor for MegaTag2. */
    public double megaTag2HeadingScaleFactor() {
        return megaTag2HeadingScaleFactor;
    }
}
