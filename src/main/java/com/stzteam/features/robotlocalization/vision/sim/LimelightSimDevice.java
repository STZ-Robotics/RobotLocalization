// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.util.Units;

/**
 * Registers the simulated camera as a HAL {@code SimDevice} so it shows up on its own in the
 * "Other Devices" panel of WPILib's Sim GUI.
 *
 * <p>No GUI code is needed: the Sim GUI groups devices by name prefix, so every camera this
 * feature creates lands under the {@code Limelight} node, just like CTRE's
 * {@code CANMotor:TalonFX[n]}.
 *
 * <p>Outside simulation {@code SimDevice.create} returns {@code null}, so the whole class
 * becomes a no-op without having to scatter conditionals everywhere.
 *
 * <h2>Inspecting and forcing from tests</h2>
 * <pre>{@code
 * var sim = new SimDeviceSim("Limelight:limelight-front", 0);
 * double tx = sim.getDouble("tx").get();
 * sim.getBoolean("Force No Targets").set(true); // simulates losing sight of the tag
 * }</pre>
 */
public class LimelightSimDevice implements AutoCloseable {

    /** The prefix that groups every camera of this feature in the Sim GUI. */
    public static final String DEVICE_PREFIX = "Limelight:";

    private final SimDevice device;

    private final SimBoolean tv;
    private final SimDouble tx;
    private final SimDouble ty;
    private final SimDouble ta;
    private final SimDouble ts;
    private final SimDouble tid;
    private final SimDouble tl;
    private final SimDouble cl;
    private final SimDouble tagCount;
    private final SimDouble avgTagDist;

    private final SimDouble botposeX;
    private final SimDouble botposeY;
    private final SimDouble botposeYaw;
    private final SimBoolean megaTag2Valid;

    private final SimDouble pipeline;
    private final SimDouble ledMode;
    private final SimBoolean forceNoTargets;

    /**
     * Creates the simulated device.
     *
     * @param cameraName the camera's logical name, usually the same as the NT table.
     * @param index      an index to tell apart several cameras with the same name.
     */
    public LimelightSimDevice(String cameraName, int index) {
        device = SimDevice.create(DEVICE_PREFIX + cameraName, index);

        if (device == null) {
            tv = null;
            tx = null;
            ty = null;
            ta = null;
            ts = null;
            tid = null;
            tl = null;
            cl = null;
            tagCount = null;
            avgTagDist = null;
            botposeX = null;
            botposeY = null;
            botposeYaw = null;
            megaTag2Valid = null;
            pipeline = null;
            ledMode = null;
            forceNoTargets = null;
            return;
        }

        tv = device.createBoolean("tv", SimDevice.Direction.kOutput, false);
        tx = device.createDouble("tx", SimDevice.Direction.kOutput, 0.0);
        ty = device.createDouble("ty", SimDevice.Direction.kOutput, 0.0);
        ta = device.createDouble("ta", SimDevice.Direction.kOutput, 0.0);
        ts = device.createDouble("ts", SimDevice.Direction.kOutput, 0.0);
        tid = device.createDouble("tid", SimDevice.Direction.kOutput, -1.0);
        tl = device.createDouble("tl (ms)", SimDevice.Direction.kOutput, 0.0);
        cl = device.createDouble("cl (ms)", SimDevice.Direction.kOutput, 0.0);
        tagCount = device.createDouble("Tag Count", SimDevice.Direction.kOutput, 0.0);
        avgTagDist = device.createDouble("Avg Tag Dist (m)", SimDevice.Direction.kOutput, 0.0);

        botposeX = device.createDouble("Botpose X (m)", SimDevice.Direction.kOutput, 0.0);
        botposeY = device.createDouble("Botpose Y (m)", SimDevice.Direction.kOutput, 0.0);
        botposeYaw = device.createDouble("Botpose Yaw (deg)", SimDevice.Direction.kOutput, 0.0);
        megaTag2Valid = device.createBoolean("MegaTag2 Valid", SimDevice.Direction.kOutput, false);

        pipeline = device.createDouble("Pipeline", SimDevice.Direction.kOutput, 0.0);
        ledMode = device.createDouble("LED Mode", SimDevice.Direction.kOutput, 0.0);

        // kInput: the Sim GUI (or a test) writes it and the simulation reads it. It lets you try
        // "what happens if I suddenly lose the tag" without moving the robot.
        forceNoTargets = device.createBoolean("Force No Targets", SimDevice.Direction.kInput, false);
    }

    /** @return true if we are in simulation and the device really exists. */
    public boolean isActive() {
        return device != null;
    }

    /**
     * Says whether somebody asked, from the Sim GUI, for the camera to pretend it sees nothing.
     *
     * @return true if this cycle's detections should be discarded.
     */
    public boolean forceNoTargets() {
        return forceNoTargets != null && forceNoTargets.get();
    }

    /**
     * Dumps a frame into the device's fields. It is called in the same place and at the same
     * rate as {@link LimelightNTPublisher#publish}, so the Sim GUI and NetworkTables never show
     * different cycles.
     *
     * @param frame  the already translated frame.
     * @param inputs the inputs read this cycle.
     */
    public void update(LimelightFrame frame, LimelightNTInputs inputs) {
        if (device == null) {
            return;
        }

        tv.set(frame.hasTarget());
        tx.set(frame.tx());
        ty.set(frame.ty());
        ta.set(frame.ta());
        ts.set(frame.ts());
        tid.set(frame.tid());
        tl.set(frame.pipelineLatencyMs());
        cl.set(frame.captureLatencyMs());
        tagCount.set(frame.targetCount());
        avgTagDist.set(frame.avgTagDistMeters());

        frame.megaTag1Pose()
                .ifPresentOrElse(
                        pose -> {
                            botposeX.set(pose.getX());
                            botposeY.set(pose.getY());
                            botposeYaw.set(Units.radiansToDegrees(pose.getRotation().getZ()));
                        },
                        () -> {
                            botposeX.set(0.0);
                            botposeY.set(0.0);
                            botposeYaw.set(0.0);
                        });

        megaTag2Valid.set(frame.megaTag2Pose().isPresent());
        pipeline.set(inputs.pipeline());
        ledMode.set(inputs.ledMode());
    }

    /** Releases the HAL device. On a real robot there is nothing to release. */
    @Override
    public void close() {
        if (device != null) {
            device.close();
        }
    }
}
