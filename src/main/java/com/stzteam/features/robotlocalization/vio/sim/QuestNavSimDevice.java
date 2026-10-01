// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.util.Units;

/**
 * Registers the simulated headset as a HAL {@code SimDevice}, so it shows up on its own in the Sim
 * GUI's "Other Devices" panel under the {@code QuestNav} node, just as the simulated Limelight shows
 * up under {@code Limelight}.
 *
 * <p>Outside simulation {@code SimDevice.create} returns {@code null} and the whole class becomes a
 * no-op.
 *
 * <h2>The three switches</h2>
 * The {@code kInput} fields are the genuinely useful part: you tick them from the Sim GUI, on the
 * fly, without recompiling or moving the robot.
 *
 * <ul>
 *   <li><b>Force Tracking Loss</b> &mdash; the Quest loses features. On recovery, the pose comes back
 *       displaced.</li>
 *   <li><b>Force Pose Jump</b> &mdash; the double-tap to passthrough. The pose jumps but
 *       {@code isTracking} stays true, which is the treacherous part.</li>
 *   <li><b>Force Link Loss</b> &mdash; loose cable. Publishing stops and the robot sees
 *       {@code isConnected()} fall.</li>
 * </ul>
 *
 * <pre>{@code
 * var sim = new SimDeviceSim("QuestNav:questnav", 0);
 * sim.getBoolean("Force Tracking Loss").set(true);
 * double drift = sim.getDouble("Drift (m)").get();
 * }</pre>
 */
public class QuestNavSimDevice implements AutoCloseable {

    /** The prefix that groups every headset of this feature in the Sim GUI. */
    public static final String DEVICE_PREFIX = "QuestNav:";

    private final SimDevice device;

    private final SimBoolean tracking;
    private final SimBoolean connected;
    private final SimDouble frameCount;
    private final SimDouble trackingLostCounter;
    private final SimDouble battery;

    private final SimDouble questX;
    private final SimDouble questY;
    private final SimDouble questYaw;
    private final SimDouble driftMeters;
    private final SimDouble driftYaw;

    private final SimBoolean forceTrackingLoss;
    private final SimBoolean forcePoseJump;
    private final SimBoolean forceLinkLoss;

    /**
     * Creates the simulated device.
     *
     * @param name  the headset's logical name.
     * @param index an index to tell apart several headsets with the same name.
     */
    public QuestNavSimDevice(String name, int index) {
        device = SimDevice.create(DEVICE_PREFIX + name, index);

        if (device == null) {
            tracking = null;
            connected = null;
            frameCount = null;
            trackingLostCounter = null;
            battery = null;
            questX = null;
            questY = null;
            questYaw = null;
            driftMeters = null;
            driftYaw = null;
            forceTrackingLoss = null;
            forcePoseJump = null;
            forceLinkLoss = null;
            return;
        }

        tracking = device.createBoolean("isTracking", SimDevice.Direction.kOutput, false);
        connected = device.createBoolean("Publishing", SimDevice.Direction.kOutput, false);
        frameCount = device.createDouble("Frame Count", SimDevice.Direction.kOutput, 0.0);
        trackingLostCounter =
                device.createDouble("Tracking Lost Count", SimDevice.Direction.kOutput, 0.0);
        battery = device.createDouble("Battery (%)", SimDevice.Direction.kOutput, 100.0);

        questX = device.createDouble("Quest X (m)", SimDevice.Direction.kOutput, 0.0);
        questY = device.createDouble("Quest Y (m)", SimDevice.Direction.kOutput, 0.0);
        questYaw = device.createDouble("Quest Yaw (deg)", SimDevice.Direction.kOutput, 0.0);
        driftMeters = device.createDouble("Drift (m)", SimDevice.Direction.kOutput, 0.0);
        driftYaw = device.createDouble("Drift Yaw (deg)", SimDevice.Direction.kOutput, 0.0);

        forceTrackingLoss =
                device.createBoolean("Force Tracking Loss", SimDevice.Direction.kInput, false);
        forcePoseJump = device.createBoolean("Force Pose Jump", SimDevice.Direction.kInput, false);
        forceLinkLoss = device.createBoolean("Force Link Loss", SimDevice.Direction.kInput, false);
    }

    /** @return true if we are in simulation and the device exists. */
    public boolean isActive() {
        return device != null;
    }

    /** @return true if somebody asked for a tracking loss. */
    public boolean forceTrackingLoss() {
        return forceTrackingLoss != null && forceTrackingLoss.get();
    }

    /** @return true if somebody asked for a pose jump. */
    public boolean forcePoseJump() {
        return forcePoseJump != null && forcePoseJump.get();
    }

    /** @return true if somebody asked to cut the link. */
    public boolean forceLinkLoss() {
        return forceLinkLoss != null && forceLinkLoss.get();
    }

    /**
     * Clears the pose-jump switch. It is a pulse, not a state: if it stayed ticked, the pose would
     * jump every cycle and the Sim GUI would have no way to send a single jump.
     */
    public void clearPoseJumpRequest() {
        if (forcePoseJump != null) {
            forcePoseJump.set(false);
        }
    }

    /**
     * Dumps the state into the device's fields.
     *
     * @param frame     the last generated frame.
     * @param model     the error model, where the counters and the drift come from.
     * @param publishing whether anything is actually being published this cycle.
     */
    public void update(QuestNavFrame frame, VioErrorModel model, boolean publishing) {
        if (device == null) {
            return;
        }

        tracking.set(frame.tracking());
        connected.set(publishing);
        frameCount.set(frame.frameCount());
        trackingLostCounter.set(model.trackingLostCounter());
        battery.set(model.batteryPercent());

        questX.set(frame.questPose().getX());
        questY.set(frame.questPose().getY());
        questYaw.set(Units.radiansToDegrees(frame.questPose().getRotation().getZ()));

        driftMeters.set(model.driftTranslation().getNorm());
        driftYaw.set(model.driftYawDegrees());
    }

    /** Releases the HAL device. */
    @Override
    public void close() {
        if (device != null) {
            device.close();
        }
    }
}
