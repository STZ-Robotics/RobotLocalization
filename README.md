# RobotLocalization

Odometry utils for mars

| | |
|---|---|
| Feature ID | `RobotLocalization` |
| Group ID | `com.stzteam.features.robotlocalization` |
| Version | `1.0.0` |
| Author | STZ-Robotics |
| MARS core | `>=1.6.0` |
| ForgeMini | `>=1.1.0` |

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

- `src/main/java/com/stzteam/features/robotlocalization/RobotLocalization.java` — your entry point.
- `generated/FeatureConstants.java` — rewritten by Gradle on every `compileJava`; don't edit it.
- `MarsFeature.json` — the descriptor MARS reads. `version` here drives both the Maven artifact and the installer.

---

Generated with the MARS Desktop feature wizard from the template in [Mars](https://github.com/STZ-Robotics/Mars/tree/main/templates/feature).
