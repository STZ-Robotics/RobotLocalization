package com.stzteam.features.robotlocalization;

import com.stzteam.features.robotlocalization.generated.FeatureConstants;

/**
 * RobotLocalization
 *
 * <p>Localization utilities for MARS.</p>
 *
 * <p>What the feature ships today:</p>
 * <ul>
 *   <li>{@link com.stzteam.features.robotlocalization.vision.sim} &mdash; a simulated Limelight
 *       built on PhotonVision. It generates physically coherent detections against the official
 *       {@code AprilTagFieldLayout} and publishes them using the NetworkTables schema of a real
 *       Limelight, and also registers itself as a HAL {@code SimDevice} so it shows up under
 *       "Other Devices" in the Sim GUI. Entry point:
 *       {@link com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight}.</li>
 *   <li>{@link com.stzteam.features.robotlocalization.vio.sim} &mdash; a simulated QuestNav.
 *       It stands in for the headset: it publishes the NetworkTables protobuf schema a Meta Quest
 *       publishes, with a model of drift and of the real VIO failure modes (tracking loss with
 *       offset re-acquisition, vibration-induced discontinuities, battery and network). Entry
 *       point: {@link com.stzteam.features.robotlocalization.vio.sim.SimulatedQuestNav}.</li>
 * </ul>
 *
 * <p>Feature ID: RobotLocalization &middot; author: STZ-Robotics</p>
 */
public final class RobotLocalization {

    private RobotLocalization() {}

    /**
     * Feature version, exactly as {@code MarsFeature.json} declares it.
     *
     * @return the published version.
     */
    public static String version() {
        return FeatureConstants.FEATURE_VERSION;
    }

    /**
     * Feature name.
     *
     * @return the feature identifier.
     */
    public static String featureName() {
        return FeatureConstants.FEATURE_NAME;
    }
}
