// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import com.stzteam.features.robotlocalization.vision.sim.VisionSimWorld;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/**
 * A "what is the headset looking at" viewfinder, for debugging mounting and occlusions.
 *
 * <p>It mounts a {@code PhotonCameraSim} where the Quest goes and asks photonlib to render the field
 * wireframe and the AprilTags from that point of view. The result comes out over
 * {@code CameraServer} as an MJPEG stream named <code>&lt;name&gt;-raw</code>, visible in Glass,
 * Shuffleboard or AdvantageScope.
 *
 * <h2>It is cosmetic, and that is worth being clear about</h2>
 * A Quest <b>does not localise by seeing AprilTags</b>: it does visual-inertial odometry on the
 * environment's natural features. This image feeds nothing; the pose comes entirely from
 * {@link VioErrorModel}. It is there to show whether the headset ends up facing the wall, whether the
 * bumper blocks half its view, or whether the mounting angle is the one you thought, and nothing
 * else.
 *
 * <p>It is off by default because it renders with OpenCV on every frame and that costs real CPU.
 *
 * <p>Note: Gazebo cannot give this image today either. SimulationStudio's 0.1.0 protocol leaves
 * cameras and vision explicitly out of scope, because they require {@code gz-sensors} with
 * rendering. So for now this viewfinder is the only way to see anything.
 */
public class QuestNavViewfinder implements AutoCloseable {

    private final VisionSimWorld world;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates the viewfinder and hooks it into the shared simulated world.
     *
     * @param name           the headset's name; the stream is called <code>&lt;name&gt;-raw</code>.
     * @param robotToHeadset transform from the robot origin to the headset.
     * @param config         the configuration, where the world and the field layout come from.
     */
    public QuestNavViewfinder(String name, Transform3d robotToHeadset, QuestNavSimConfig config) {
        world = VisionSimWorld.get(config.visionWorldName(), config.fieldLayout());

        // Low resolution and FPS on purpose: this is a viewfinder, not a sensor. Rendering at
        // 1280x800 and 90 FPS for an image nobody measures would be throwing CPU away.
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(640, 480, Rotation2d.fromDegrees(100.0));
        properties.setFPS(30);

        camera = new PhotonCamera(name + "-view");
        // The layout is passed explicitly: the two-argument constructor hands it WPILib's default
        // field, which on 2026.1.1 is still the 2025 one.
        cameraSim = new PhotonCameraSim(camera, properties, config.fieldLayout());
        cameraSim.enableRawStream(true);
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableProcessedStream(false);

        world.sim().addCamera(cameraSim, robotToHeadset);
    }

    /**
     * Advances the world so a new frame gets rendered.
     *
     * <p>If there are already simulated cameras in the same world, {@link VisionSimWorld}
     * deduplicates the call within the cycle, so this does no double work.
     *
     * @param truePose the robot's real pose.
     */
    public void update(Pose3d truePose) {
        world.update(truePose);
    }

    /**
     * Moves the viewfinder when the headset's mounting changes.
     *
     * @param robotToHeadset the new robot-&gt;headset transform.
     */
    public void setRobotToHeadset(Transform3d robotToHeadset) {
        world.sim().adjustCamera(cameraSim, robotToHeadset);
    }

    /** @return the name of the CameraServer stream that publishes the raw image. */
    public String streamName() {
        return camera.getName() + "-raw";
    }

    /** Releases the rendering camera. */
    @Override
    public void close() {
        world.sim().removeCamera(cameraSim);
        cameraSim.close();
        camera.close();
    }
}
