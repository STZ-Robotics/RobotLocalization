# RobotLocalization

Localization for MARS: pose fusion with filtering and consensus, plus simulated Limelight and QuestNav with a NetworkTables bridge and SimDevice.

| | |
|---|---|
| Feature ID | `RobotLocalization` |
| Group ID | `com.stzteam.features.robotlocalization` |
| Version | `1.0.0` |
| Author | STZ-Robotics |
| MARS core | `>=1.6.0` |
| ForgeMini | `>=1.1.0` |

---

## Simulated Limelight

Package `com.stzteam.features.robotlocalization.vision.sim`.

PhotonVision generates the detections (`VisionSystemSim` + `PhotonCameraSim` against the official `AprilTagFieldLayout`) and this feature translates them into the NetworkTables schema a real Limelight publishes. Production robot code does not change a single line: if your subsystem reads `LimelightHelpers.getTX("limelight-front")` against hardware, it reads the same thing against the simulation.

Everything runs inside the robot process. There is no external bridge and no HALSim WebSocket.

### Usage

```java
public class Robot extends TimedRobot {

    private final SimulatedLimelight frontCamera =
            new SimulatedLimelight(
                    "limelight-front",
                    new Transform3d(
                            new Translation3d(0.30, 0.0, 0.50),
                            new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0)),
                    LimelightSimConfig.limelight3G());

    @Override
    public void simulationPeriodic() {
        frontCamera.periodic(drivetrain.getSimulatedPose());
    }
}
```

With several cameras, `SimulatedLimelight.periodicAll(pose)` advances them all at once; they share a single `VisionSystemSim` as long as they declare the same world (`withVisionWorld`).

On a real robot the constructor detects `RobotBase.isReal()` and creates nothing — no `VisionSystemSim`, no `SimDevice`, no NT entries. `periodic()` returns immediately, so you can leave the instance declared at no cost during competition.

### What it publishes

| Key | Source |
|---|---|
| `tv` | `1.0` if there are targets after applying the id filters |
| `tx` / `ty` | `-getYaw()` / `getPitch()` of the best target — Photon uses positive-left, Limelight positive-right |
| `txnc` / `tync` | equal to `tx`/`ty`: a movable crosshair is not modelled |
| `ta` | `getArea()`, a 0-100 percentage in both conventions |
| `ts` | `getSkew()` |
| `tid` | `getFiducialId()`, or `-1` with no target |
| `tl` / `cl` | the frame's real latency in `tl`; `cl` is `0` (photonlib does not separate capture from pipeline, and what matters for dating the measurement is the total) |
| `hb` | heartbeat, incremented per frame |
| `t2d` | the 17 targeting values, with the pixel metrics derived from the min-area rect |
| `rawfiducials` | 7 values per visible tag |
| `botpose` / `botpose_wpiblue` / `botpose_wpired` | MegaTag1: multi-tag PnP if there are two or more tags, otherwise the least ambiguous one. Full `11 + 7·tagCount` format |
| `botpose_orb*` | MegaTag2: constrained PnP pinning the yaw from `robot_orientation_set` |
| `getpipe` / `getpipetype` | echo of the requested pipeline |
| `tcornxy` | optional, `withCorners(true)` |
| `targetpose_*` / `camerapose_*` | optional, `withTargetSpacePoses(true)` — see below |

And it **reads** what the robot code writes: `pipeline`, `ledMode`, `throttle_set`, `robot_orientation_set`, `fiducial_id_filters_set` and `camerapose_robotspace_set`. Which means `SetRobotOrientation`, `SetFiducialIDFiltersOverride` and `setCameraPose_RobotSpace` have a real effect in simulation.

### Out of scope in this iteration

- The `json` key (`LimelightHelpers.getLatestResults`). Use the direct getters.
- Neural detector pipelines (`tclass`, `rawdetections`).
- `targetpose_*` / `camerapose_*` are **off by default**. Limelight's target-space convention (X right, Y down, Z outward) is the part of the schema easiest to misread, and those keys staying obviously empty is preferable to them publishing plausible but wrong numbers. If you switch them on, validate the signs against your hardware before trusting them.

### Sim GUI

Each camera registers as `Limelight:<name>[n]`, so it shows up on its own under the **Limelight** node in *Other Devices*, just like CTRE's `CANMotor:TalonFX[n]`. There is no GUI code.

The `Force No Targets` field is an input (`kInput`): tick it from the Sim GUI and the camera pretends to see nothing, without moving the robot. Useful to test what your code does when it loses the tag.

From tests or from another class:

```java
var sim = new SimDeviceSim("Limelight:limelight-front", 0);
double tx = sim.getDouble("tx").get();
sim.getBoolean("Force No Targets").set(true);
```

### Hardware validation checklist

What the tests already cover (`./gradlew test`): keys and array formats, the sign conventions of `tx`/`ty`, the blue/red/centre origins, the full Photon → NT pipeline against a real field tag, the `SimDevice` and MegaTag2.

What is worth verifying against a physical Limelight before trusting it blindly:

- [ ] The signs of `tx`/`ty` against Limelight's official documentation, not only against Photon's.
- [ ] The convention of `camerapose_robotspace_set`: here it is interpreted as `[forward, left, up, roll, pitch, yaw]` in the WPILib convention.
- [ ] The target-space keys, if you switch them on.
- [ ] The camera profiles (`limelight2/3/3G/4`): they are approximations of the published specs, not calibrations. With the real calibration, pass it with `withCameraProperties(...)`.

### TODO

User-configurable Gaussian noise, occlusion between tags, and modelling a movable crosshair to separate `tx` from `txnc`.

---

## Simulated QuestNav

Package `com.stzteam.features.robotlocalization.vio.sim`.

It stands in for the headset: it publishes to NetworkTables exactly what a Meta Quest running QuestNav publishes, so robot code using `gg.questnav.questnav.QuestNav` works unchanged and without hardware. It is the counterpart of the Limelight — there we stand in for the camera.

Until now there was no way to simulate it: QuestNav's official "Robot Sim Example" is robot code running in the WPILib simulator **against a real headset**. With no headset, `isConnected()` returns false and the whole subsystem is dead.

### Usage

```java
public class Robot extends TimedRobot {

    private final SimulatedQuestNav quest =
            new SimulatedQuestNav(VisionConstants.ROBOT_TO_QUEST, new QuestNavSimConfig());

    @Override
    public void simulationPeriodic() {
        quest.periodic(drivetrain.getSimulatedPose());
    }
}
```

The `Transform3d` is the same `ROBOT_TO_QUEST` you pass to your real subsystem.

### What makes it different from a simulated camera

A Limelight either sees the tag or it does not. A Quest **always** reports a pose; what changes is how much it lies. That is why the feature's real content lives in `VioErrorModel`, not in the geometry:

- **Drift** scaled by distance travelled, not by time — a stationary robot does not drift. The yaw drifts with accumulated rotation.
- **Feature loss** (low light, reflections, uniform surfaces). QuestNav's documentation warns that the headset may re-acquire *with an offset*: on recovery, **the pose comes back displaced**. That jump is what breaks a finely tuned pose estimator.
- **Vibration-induced double-tap to passthrough**: a pose jump **without** losing tracking. More treacherous, because `isTracking` stays true.
- **Critical battery**: the headset shuts itself off mid-match.
- **Network drop**: publishing stops and `isConnected()` falls after 120 ms.

The first three are triggered from the Sim GUI (`Force Tracking Loss`, `Force Pose Jump`, `Force Link Loss`) or from code with `injectTrackingLoss(...)`, `injectPoseDiscontinuity(...)`, `injectLinkLoss(...)`. On top of that, a hard stop drops tracking on its own.

The default values are **plausible, not measured**. If you have logs from your headset, tune them.

### The Quest's origin

A Quest **does not publish field poses**. It publishes its pose in an arbitrary origin of its own, the place where it was when the app started; making that coincide with the field's is something the robot code has to cause by calling `setPose()`. The simulation honours that, and `withRandomizedOrigin(true)` forces you to get it right: with the origin at identity, a team that never calls `setPose()` sees correct poses by accident and the bug does not show up until they switch the real headset on.

### Mind `dataTimestamp()`

By default publication happens on the same NetworkTables instance the robot code consumes, and in that case `PoseFrame.dataTimestamp()` is **useless**: it is a constant 1 µs. That is not a bug in this feature but in the topology — that value comes from the `serverTime` the NT server assigns **on receipt**, and ntcore marks everything published locally with the sentinel `serverTime = 1`.

If your robot code fuses on timestamps, which is the normal case for a pose estimator, switch on `withNetworkClient(true)`: the simulation then publishes as an NT4 client against the robot's server, just like the real headset, and the timestamps come out correct. It is off by default because if there is no NT4 server listening, the client does not connect and nothing gets published — publishing with poor timestamps is a degraded failure, publishing nothing is a total one.

### Viewfinder

`withViewfinder(true)` mounts a `PhotonCameraSim` where the headset goes and publishes an MJPEG stream with the field wireframe from its point of view.

**It is cosmetic.** The Quest does not localise by seeing AprilTags: it does visual-inertial odometry on the environment's natural features. The image feeds nothing; it is there to show whether the headset ends up facing the wall or whether the mounting angle is the one you thought. Off by default: it renders with OpenCV on every frame.

The same is available on the Limelight with `withRawStream(true)`, `withWireframe(true)`, `withProcessedStream(true)` and `withDebugField(true)`.

### Out of scope

- Integration with **SimulationStudio / Gazebo**. The bridge publishes the physics' ground-truth pose to `/MARS/Sim/truthPose3d`, which is exactly what `periodic(Pose3d)` needs, but reading it is left for later. Note: SimulationStudio's 0.1.0 protocol leaves cameras and vision explicitly out of scope, so photonlib's viewfinder is the only thing that gives an image today.
- The `POSE_RESET` command is the only one in the protocol; any other is answered with an error.

---

## Pose fusion

Package `com.stzteam.features.robotlocalization.fusion`.

The two sections above simulate sensors. This is the other half: **what the robot does with those measurements**. `PoseFusion` is not an estimator — WPILib already ships one — it is the **gatekeeper** that decides which measurements reach `addVisionMeasurement`.

### Usage

```java
private final PoseFusion fusion = new PoseFusion(
        drivetrain::addVisionMeasurement,
        new LocalizationInputs(
                () -> drivetrain.getState().Pose.getRotation(),
                () -> DegreesPerSecond.of(...),
                () -> drivetrain.getState().Speeds,
                null,                                   // no accelerometer
                () -> drivetrain.getState().Pose))
    .registerSource(LimelightPoseSource.megaTag2("limelight-front"))
    .registerSource(QuestNavPoseSource.create("quest", ROBOT_TO_QUEST));

@Override
public void periodic() {
    fusion.update();
}
```

### The filters

They are applied in a cascade, from cheap to expensive and from local to global. A source discarded on environment does not even spend the hardware read.

| Filter | What it discards |
|---|---|
| Enabled | Sources disabled from code or from the dashboard |
| Environment | A robot turning, accelerating or moving faster than the measurement can take |
| Timestamp | Measurements that are not newer than the last accepted one |
| Valid pose | Whatever the subclass considers absurd, typically outside the field |
| Ambiguity | PnP solutions with two almost equally good interpretations |
| Consensus | Whatever another source contradicts |
| Mahalanobis | Whatever is too far from what the estimator already believes |

Each source publishes to NetworkTables **why** its last measurement was discarded (`RejectionReason`). Without that, a badly calibrated filter and a broken camera look exactly the same from the field.

### Consensus

A single camera reporting an absurd pose passes every individual filter: it is geometrically valid, the ambiguity is low and the robot is standing still. The only thing that gives it away is that **another source says something different**.

Two modes: `REJECT_BOTH` (the prudent choice with two sources — with no third one to break the tie there is no way to tell which is lying) and `MAJORITY_VOTE`, which counts how many each one agrees with and keeps the best-supported ones.

The tolerance **widens with speed**: two measurements taken 0.1 s apart describe different positions if the robot moved in between, and that is not a disagreement. At 4 m/s that is 0.4 m, comparable to the whole tolerance.

### Mahalanobis

It discards whatever is too far from what the estimator already believes, measured in sigmas. The key is that the uncertainty it compares against **grows with time** since the last accepted measurement: if a source has been quiet for ten seconds, the estimator may have wandered off, and the gate opens up to let it be corrected. Without that the filter stays shut forever.

The first measurement is always accepted: there is nothing to compare it against.

### Standard deviations

`StdDevStrategies` is the library of `(distance, tagCount) → [σx, σy, σθ]` functions, composable:

```java
source.setStdDevsStrategy(
    StdDevStrategies.clamped(
        StdDevStrategies.dynamicStateDegradation(
            StdDevStrategies.quadraticContinuous(0.05, 0.10),
            drivetrain::getChassisSpeeds, 3.0, 2.0),
        0.02, 4.0, 0.02, 10.0));
```

**Always bound the final result.** Multiplicative strategies have no ceiling of their own, and a runaway value breaks the Kalman filter's matrix inversion from the inside, with no error to say what happened.

For a VIO the strategy is different: its error does not depend on the distance to anything, but on how long it has gone without being corrected. `vioTimeDrift` returns an object with `reset()`, which has to be called after each `POSE_RESET` — otherwise it keeps inflating the uncertainty forever and the estimator ends up ignoring a sensor that has just become perfect.

### The two concrete sources

`LimelightPoseSource` reads NetworkTables directly, without `LimelightHelpers` — that class is a file you copy, not a Maven artifact, and it is not needed because we know the schema: this feature's own simulator writes it. It supports MegaTag1 and MegaTag2, and **writes the heading to the Limelight on its own**, which is the step most often forgotten and the one that makes MegaTag2 return zeros without explaining why.

`QuestNavPoseSource` wraps the vendordep's `QuestNav`, undoes `ROBOT_TO_QUEST` and discards whatever arrives without tracking. It also detects the 1 µs sentinel `dataTimestamp` and warns once instead of feeding the estimator mis-dated measurements.

### The 3D field

`VisionSimTelemetry.forWorld("main").publish()` publishes the simulated world as `struct:Pose3d[]` topics: the robot's pose, where each camera is, every AprilTag on the field and which ones are being seen right now. It shows up in AdvantageScope or in MARS Desktop's 3D viewer, which detects from the topic name alone that these are vision poses.

Photonlib's flat `Field2d` keeps being published too, for a quick top-down view in Glass.

---

## Install this feature in a robot project

1. Open MARS Desktop and load your robot project.
2. Go to **Project > Packages > Manual Install** and paste:

   ```
   https://stz-robotics.github.io/RobotLocalization/MarsFeature.json
   ```

3. Press **INSTALL FROM URL**. MARS writes the descriptor into `workspace-mars/features/` and runs a Gradle build.

### Vendor requirements

This feature compiles against the vendor libraries below. The robot project that installs it must already have the matching vendordeps, otherwise Gradle will not resolve the transitive dependencies recorded in the published POM:

- **PhotonLib** — `org.photonvision:photonlib-java:v2026.1.1`, `org.photonvision:photontargeting-java:v2026.1.1`
- **QuestNavLib** — `gg.questnav:questnavlib-java:2026-2.1.0`

The robot project needs PhotonVision's vendordep (`vendordeps/photonlib.json`), not just the Maven coordinates: the `photontargeting` natives are required at runtime for the simulation. QuestNav is pure Java, with no natives.

## Publish / update

Publishing is a `git push`: `.github/workflows/publish.yml` builds the jar into `maven/`, generates the Javadoc and deploys both to GitHub Pages.

1. Create the repository `STZ-Robotics/RobotLocalization` on GitHub and push this folder to `main`:

   ```bash
   git add -A
   git commit -m "Initial feature"
   git push -u origin main
   ```

2. On GitHub: **Settings > Pages > Build and deployment**, source **Deploy from a branch**, branch **`gh-pages`**, folder **`/(root)`**.
3. Wait for the *Publish MARS Feature* action to finish (Actions tab).
4. To cut a new release, bump `version` in `MarsFeature.json` and push again. The Javadoc and the Maven artifact are regenerated on every push to `main`.

- Docs: https://stz-robotics.github.io/RobotLocalization/
- Maven: https://stz-robotics.github.io/RobotLocalization/maven/
- Descriptor: https://stz-robotics.github.io/RobotLocalization/MarsFeature.json

## Code layout

- `vision/sim/SimulatedLimelight.java` — public API of the simulated camera.
- `vision/sim/LimelightSimConfig.java` — configuration (NT table, optics, which keys to publish).
- `vision/sim/LimelightTargetTranslator.java` — `PhotonPipelineResult` → `LimelightFrame`.
- `vision/sim/LimelightNTPublisher.java` — writes the NT schema and reads the input keys.
- `vision/sim/LimelightSimDevice.java` — registers the HAL `SimDevice`.
- `vision/sim/LimelightCoordinates.java` — every convention conversion, with tests.
- `vision/sim/VisionSimWorld.java` — registry of shared `VisionSystemSim` worlds.
- `vio/sim/SimulatedQuestNav.java` — public API of the simulated headset.
- `vio/sim/VioErrorModel.java` — the VIO's drift and failure modes. The real content of that feature.
- `vio/sim/QuestOriginFrame.java` — the headset's origin and `POSE_RESET`, with tests.
- `vio/sim/QuestNavNTBridge.java` — publishes the protobuf topics and services the commands.
- `vio/sim/QuestNavSimDevice.java` — `SimDevice` with the failure switches.
- `fusion/PoseFusion.java` — the gatekeeper that filters the vision measurements.
- `fusion/PoseSource.java` — a measurement source; the filters live here.
- `fusion/ConsensusGate.java` — arbitration between sources that contradict each other, with tests.
- `fusion/StdDevStrategies.java` — how to compute the confidence of each measurement.
- `fusion/LimelightPoseSource.java` / `fusion/QuestNavPoseSource.java` — the two concrete sources.
- `vision/LimelightBotpose.java` — the `botpose` format, shared between the simulator and the reader.
- `RobotLocalization.java` — entry point of the feature.
- `generated/FeatureConstants.java` — rewritten by Gradle on every `compileJava`; don't edit it.
- `MarsFeature.json` — the descriptor MARS reads. `version` here drives both the Maven artifact and the installer.

### Tests

`./gradlew test`. On Windows the build points the test JVM at the WPILib JDK: the WPILib natives are built against its MSVC runtime and on any other JDK the JVM crashes while loading `ntcore`.

---

Generated with the MARS Desktop feature wizard from the template in [Mars](https://github.com/STZ-Robotics/Mars/tree/main/templates/feature).
