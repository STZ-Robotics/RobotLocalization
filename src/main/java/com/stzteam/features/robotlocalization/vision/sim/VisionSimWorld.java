// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.simulation.VisionSystemSim;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A registry of shared simulated worlds.
 *
 * <p>A {@code VisionSystemSim} represents a field with its AprilTags and every camera looking
 * at it. Several {@link SimulatedLimelight} instances declaring the same world name share a
 * single instance, which is what multi-camera setups need: the tags load once and the robot
 * pose propagates to every camera at once.
 *
 * <p>Updates are deduplicated within one cycle, so it makes no difference whether you call
 * {@link SimulatedLimelight#periodic(Pose2d)} on each camera or {@link #updateAll(Pose2d)}
 * once: the world advances once per cycle.
 */
public final class VisionSimWorld {

    /**
     * The deduplication window. Two calls within this margin count as the same robot cycle;
     * the typical 20 ms period clears it comfortably.
     */
    private static final double SAME_LOOP_WINDOW_SECONDS = 0.001;

    private static final Map<String, VisionSimWorld> WORLDS = new LinkedHashMap<>();

    private final String name;
    private final VisionSystemSim sim;
    private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;

    private VisionSimWorld(String name, AprilTagFieldLayout layout) {
        this.name = name;
        this.sim = new VisionSystemSim(name);
        this.sim.addAprilTags(layout);
    }

    /**
     * Returns the world with that name, creating it with the given layout the first time.
     *
     * <p>If the world already existed the layout is ignored: whoever asked first defines it. That
     * keeps two cameras with different configurations from trampling each other's tags.
     *
     * @param name   world identifier.
     * @param layout the AprilTag layout to load if it has to be created.
     * @return the shared world.
     */
    public static synchronized VisionSimWorld get(String name, AprilTagFieldLayout layout) {
        return WORLDS.computeIfAbsent(name, key -> new VisionSimWorld(key, layout));
    }

    /**
     * Advances every registered world with the same robot pose. A shortcut for the normal case
     * of a single robot with one or several cameras.
     *
     * @param robotPose the robot's current simulated pose.
     */
    public static synchronized void updateAll(Pose2d robotPose) {
        updateAll(new Pose3d(robotPose));
    }

    /**
     * Advances every registered world with the same robot pose.
     *
     * @param robotPose the robot's current simulated pose.
     */
    public static synchronized void updateAll(Pose3d robotPose) {
        for (VisionSimWorld world : WORLDS.values()) {
            world.update(robotPose);
        }
    }

    /**
     * Returns the worlds registered so far.
     *
     * @return the collection of worlds, in creation order.
     */
    public static synchronized Collection<VisionSimWorld> all() {
        return WORLDS.values();
    }

    /**
     * Forgets every registered world. Meant for tests, where each case wants to start from a
     * clean field.
     */
    public static synchronized void reset() {
        WORLDS.clear();
    }

    /**
     * Advances this world. If it already advanced in this same cycle, it does nothing.
     *
     * @param robotPose the robot's current simulated pose.
     */
    public void update(Pose3d robotPose) {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateSeconds < SAME_LOOP_WINDOW_SECONDS) {
            return;
        }
        lastUpdateSeconds = now;
        sim.update(robotPose);
    }

    /**
     * Repositions the robot in the world without generating frames, useful when restarting an
     * autonomous routine or when placing the robot from the Sim GUI.
     *
     * @param robotPose the robot's new pose.
     */
    public void resetRobotPose(Pose3d robotPose) {
        sim.resetRobotPose(robotPose);
    }

    /** @return this world's name. */
    public String name() {
        return name;
    }

    /** @return the underlying {@code VisionSystemSim}, in case you need its full API. */
    public VisionSystemSim sim() {
        return sim;
    }

    /**
     * The debug field photonlib maintains with the robot pose, the cameras and the visible
     * targets. Publish it to a {@code SmartDashboard} or to Glass to see what the camera is
     * looking at.
     *
     * @return the debug {@code Field2d}.
     */
    public Field2d debugField() {
        return sim.getDebugField();
    }
}
