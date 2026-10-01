# RobotLocalization

Localización para MARS: fusión de pose con filtros y consenso, más Limelight y QuestNav simuladas con puente a NetworkTables y SimDevice.

| | |
|---|---|
| Feature ID | `RobotLocalization` |
| Group ID | `com.stzteam.features.robotlocalization` |
| Version | `1.0.0` |
| Author | STZ-Robotics |
| MARS core | `>=1.6.0` |
| ForgeMini | `>=1.1.0` |

---

## Limelight simulada

Paquete `com.stzteam.features.robotlocalization.vision.sim`.

PhotonVision genera las detecciones (`VisionSystemSim` + `PhotonCameraSim` contra el `AprilTagFieldLayout` oficial) y esta feature las traduce al esquema de NetworkTables que publica un Limelight real. El robot code de producción no cambia ni una línea: si tu subsistema lee `LimelightHelpers.getTX("limelight-front")` contra hardware, lee lo mismo contra la simulación.

Todo corre dentro del proceso del robot. No hay bridge externo ni HALSim WebSocket.

### Uso

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

Con varias cámaras, `SimulatedLimelight.periodicAll(pose)` las avanza todas de golpe; comparten un solo `VisionSystemSim` mientras declaren el mismo mundo (`withVisionWorld`).

En robot real el constructor detecta `RobotBase.isReal()` y no crea nada — ni `VisionSystemSim`, ni `SimDevice`, ni entries de NT. `periodic()` retorna de inmediato, así que puedes dejar la instancia declarada sin coste en competencia.

### Qué publica

| Key | Origen |
|---|---|
| `tv` | `1.0` si hay targets tras aplicar filtros de id |
| `tx` / `ty` | `-getYaw()` / `getPitch()` del mejor target — Photon usa positivo-izquierda, Limelight positivo-derecha |
| `txnc` / `tync` | iguales a `tx`/`ty`: no se modela un crosshair movible |
| `ta` | `getArea()`, porcentaje 0-100 en ambas convenciones |
| `ts` | `getSkew()` |
| `tid` | `getFiducialId()`, o `-1` sin target |
| `tl` / `cl` | latencia real del frame en `tl`; `cl` es `0` (photonlib no separa captura de pipeline, y lo que importa para fechar la medición es el total) |
| `hb` | heartbeat, incrementa por frame |
| `t2d` | los 17 valores de targeting, con las métricas en píxeles derivadas del min-area rect |
| `rawfiducials` | 7 valores por tag visible |
| `botpose` / `botpose_wpiblue` / `botpose_wpired` | MegaTag1: PnP multi-tag si hay dos o más tags, si no el de menor ambigüedad. Formato completo `11 + 7·tagCount` |
| `botpose_orb*` | MegaTag2: PnP restringido fijando el yaw de `robot_orientation_set` |
| `getpipe` / `getpipetype` | eco del pipeline pedido |
| `tcornxy` | opcional, `withCorners(true)` |
| `targetpose_*` / `camerapose_*` | opcional, `withTargetSpacePoses(true)` — ver más abajo |

Y **lee** lo que el robot code escribe: `pipeline`, `ledMode`, `throttle_set`, `robot_orientation_set`, `fiducial_id_filters_set` y `camerapose_robotspace_set`. O sea que `SetRobotOrientation`, `SetFiducialIDFiltersOverride` y `setCameraPose_RobotSpace` tienen efecto real en la simulación.

### Fuera de alcance en esta iteración

- La key `json` (`LimelightHelpers.getLatestResults`). Usa los getters directos.
- Pipelines de detector neuronal (`tclass`, `rawdetections`).
- `targetpose_*` / `camerapose_*` vienen **apagadas por defecto**. La convención de target space de Limelight (X derecha, Y abajo, Z saliendo) es la parte del esquema más fácil de interpretar mal, y es preferible que esas keys se queden vacías de forma obvia a que publiquen números plausibles pero equivocados. Si las enciendes, valida los signos contra tu hardware antes de confiar en ellas.

### Sim GUI

Cada cámara se registra como `Limelight:<nombre>[n]`, así que aparece sola bajo el nodo **Limelight** en *Other Devices*, igual que los `CANMotor:TalonFX[n]` de CTRE. No hay código de GUI.

El campo `Force No Targets` es de entrada (`kInput`): márcalo desde el Sim GUI y la cámara finge no ver nada, sin mover el robot. Útil para probar qué hace tu código cuando pierde el tag.

Desde tests o desde otra clase:

```java
var sim = new SimDeviceSim("Limelight:limelight-front", 0);
double tx = sim.getDouble("tx").get();
sim.getBoolean("Force No Targets").set(true);
```

### Checklist de validación con hardware

Lo que los tests ya cubren (`./gradlew test`): keys y formatos de array, convenciones de signo de `tx`/`ty`, orígenes azul/rojo/centro, el pipeline completo Photon → NT contra un tag real del campo, el `SimDevice` y MegaTag2.

Lo que conviene verificar contra un Limelight físico antes de confiar a ciegas:

- [ ] Signos de `tx`/`ty` contra la documentación oficial de Limelight, no solo contra la de Photon.
- [ ] Convención de `camerapose_robotspace_set`: aquí se interpreta como `[adelante, izquierda, arriba, roll, pitch, yaw]` en convención WPILib.
- [ ] Las keys de target space, si las enciendes.
- [ ] Perfiles de cámara (`limelight2/3/3G/4`): son aproximaciones de las specs publicadas, no calibraciones. Con la calibración real, pásala con `withCameraProperties(...)`.

### TODO

Ruido gaussiano configurable por el usuario, oclusión entre tags, y modelado de crosshair movible para separar `tx` de `txnc`.

---

## QuestNav simulada

Paquete `com.stzteam.features.robotlocalization.vio.sim`.

Hace de headset: publica en NetworkTables exactamente lo que publica un Meta Quest con QuestNav, así que el robot code que usa `gg.questnav.questnav.QuestNav` funciona sin cambios y sin hardware. Es la contraparte de la Limelight — allí hacemos de cámara.

Hoy no existía forma de simularlo: el "Robot Sim Example" oficial de QuestNav es robot code que corre en el simulador de WPILib **contra un headset real**. Sin headset, `isConnected()` devuelve false y el subsistema entero queda muerto.

### Uso

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

El `Transform3d` es el mismo `ROBOT_TO_QUEST` que le pasas a tu subsistema real.

### Qué hace distinto a una cámara simulada

Una Limelight ve el tag o no lo ve. Un Quest **siempre** reporta una pose; lo que cambia es cuánto miente. Por eso el contenido real de la feature está en `VioErrorModel`, no en la geometría:

- **Deriva** escalada por distancia recorrida, no por tiempo — un robot parado no deriva. El yaw deriva por rotación acumulada.
- **Pérdida de features** (poca luz, reflejos, superficies uniformes). La documentación de QuestNav avisa de que el headset puede re-adquirir *con un offset*: al recuperar, **la pose vuelve desplazada**. Ese salto es el que rompe a un pose estimator afinado.
- **Doble-tap a passthrough por vibración**: salto de pose **sin** perder tracking. Más traicionero, porque `isTracking` sigue en true.
- **Batería crítica**: el headset se apaga solo a media match.
- **Caída de red**: se deja de publicar y `isConnected()` cae a los 120 ms.

Los tres primeros se disparan desde el Sim GUI (`Force Tracking Loss`, `Force Pose Jump`, `Force Link Loss`) o desde código con `injectTrackingLoss(...)`, `injectPoseDiscontinuity(...)`, `injectLinkLoss(...)`. Además, un frenazo brusco tira el tracking solo.

Los valores por defecto son **plausibles, no medidos**. Si tienes logs de tu headset, ajústalos.

### El origen del Quest

Un Quest **no publica poses de campo**. Publica su pose en un origen propio y arbitrario, el sitio donde estaba cuando arrancó la app; que coincida con el del campo es algo que el robot code tiene que provocar llamando a `setPose()`. La simulación lo respeta, y `withRandomizedOrigin(true)` te obliga a hacerlo bien: con el origen en identidad, un equipo que nunca llama a `setPose()` ve poses correctas por accidente y el bug no aparece hasta encender el headset de verdad.

### Cuidado con `dataTimestamp()`

Por defecto se publica en la misma instancia de NetworkTables que consume el robot code, y en ese caso `PoseFrame.dataTimestamp()` **no sirve**: vale 1 µs constante. No es un fallo de esta feature sino de la topología — ese valor sale del `serverTime` que asigna el servidor NT **al recibir**, y ntcore marca con el centinela `serverTime = 1` todo lo que se publica localmente.

Si tu robot code fusiona por timestamp, que es el caso normal de un pose estimator, enciende `withNetworkClient(true)`: la simulación pasa a publicar como cliente NT4 contra el servidor del robot, igual que el headset real, y los timestamps salen correctos. Va apagado por defecto porque si no hay un servidor NT4 escuchando, el cliente no conecta y no se publica nada — publicar con timestamps pobres es un fallo degradado, no publicar es un fallo total.

### Visor

`withViewfinder(true)` monta una `PhotonCameraSim` donde va el headset y publica un stream MJPEG con el wireframe del campo desde su punto de vista.

**Es cosmético.** El Quest no localiza viendo AprilTags: hace odometría visual-inercial sobre las features naturales del entorno. La imagen no alimenta nada; sirve para ver si el headset queda mirando a la pared o si el ángulo de montaje es el que creías. Apagado por defecto: renderiza con OpenCV en cada frame.

Lo mismo está disponible en la Limelight con `withRawStream(true)`, `withWireframe(true)`, `withProcessedStream(true)` y `withDebugField(true)`.

### Fuera de alcance

- Integración con **SimulationStudio / Gazebo**. El bridge publica la pose de verdad de la física en `/MARS/Sim/truthPose3d`, que es justo lo que `periodic(Pose3d)` necesita, pero leerla se deja para después. Nota: el protocolo 0.1.0 de SimulationStudio deja cámaras y visión explícitamente fuera de alcance, así que el visor de photonlib es lo único que da imagen hoy.
- El comando `POSE_RESET` es el único del protocolo; cualquier otro se contesta con error.

---

## Fusión de pose

Paquete `com.stzteam.features.robotlocalization.fusion`.

Las dos secciones de arriba simulan sensores. Esta es la otra mitad: **qué hace el robot con esas mediciones**. `PoseFusion` no es un estimador — ese ya lo trae WPILib — es el **portero** que decide cuáles llegan a `addVisionMeasurement`.

### Uso

```java
private final PoseFusion fusion = new PoseFusion(
        drivetrain::addVisionMeasurement,
        new LocalizationInputs(
                () -> drivetrain.getState().Pose.getRotation(),
                () -> DegreesPerSecond.of(...),
                () -> drivetrain.getState().Speeds,
                null,                                   // sin acelerómetro
                () -> drivetrain.getState().Pose))
    .registerSource(LimelightPoseSource.megaTag2("limelight-front"))
    .registerSource(QuestNavPoseSource.create("quest", ROBOT_TO_QUEST));

@Override
public void periodic() {
    fusion.update();
}
```

### Los filtros

Se aplican en cascada, de barato a caro y de local a global. Una fuente descartada por entorno ni siquiera gasta la lectura del hardware.

| Filtro | Qué descarta |
|---|---|
| Encendida | Fuentes apagadas desde código o desde el dashboard |
| Entorno | Robot girando, acelerando o moviéndose más rápido de lo que la medición aguanta |
| Timestamp | Mediciones que no son más nuevas que la última aceptada |
| Pose válida | Lo que la subclase considere absurdo, típicamente fuera del campo |
| Ambigüedad | Soluciones de PnP con dos interpretaciones casi igual de buenas |
| Consenso | Lo que otra fuente contradice |
| Mahalanobis | Lo que está demasiado lejos de lo que el estimator ya cree |

Cada fuente publica en NetworkTables **por qué** se descartó su última medición (`RejectionReason`). Sin eso, un filtro mal calibrado y una cámara rota se ven exactamente igual desde la pista.

### Consenso

Una sola cámara que reporte una pose absurda pasa cualquier filtro individual: es geométricamente válida, la ambigüedad es baja y el robot está quieto. Lo único que la delata es que **otra fuente dice algo distinto**.

Dos modos: `REJECT_BOTH` (lo prudente con dos fuentes — sin una tercera que desempate no hay forma de saber cuál miente) y `MAJORITY_VOTE`, que cuenta con cuántas concuerda cada una y se queda con las más respaldadas.

La tolerancia **se ensancha con la velocidad**: dos mediciones tomadas con 0.1 s de diferencia describen posiciones distintas si el robot se movió en medio, y eso no es un desacuerdo. A 4 m/s son 0.4 m, comparable a la tolerancia entera.

### Mahalanobis

Descarta lo que esté demasiado lejos de lo que el estimator ya cree, medido en sigmas. La clave es que la incertidumbre con la que se compara **crece con el tiempo** desde la última medición aceptada: si una fuente lleva diez segundos callada, el estimator pudo haberse ido de sitio, y la puerta se abre para dejar que lo corrija. Sin eso el filtro se queda cerrado para siempre.

La primera medición se acepta siempre: no hay contra qué compararla.

### Desviaciones estándar

`StdDevStrategies` es la librería de funciones `(distancia, tagCount) → [σx, σy, σθ]`, componibles:

```java
source.setStdDevsStrategy(
    StdDevStrategies.clamped(
        StdDevStrategies.dynamicStateDegradation(
            StdDevStrategies.quadraticContinuous(0.05, 0.10),
            drivetrain::getChassisSpeeds, 3.0, 2.0),
        0.02, 4.0, 0.02, 10.0));
```

**Acota siempre el resultado final.** Las estrategias multiplicativas no tienen techo propio, y un valor desbocado rompe la inversión de matrices del filtro de Kalman por dentro, sin un error que diga qué pasó.

Para un VIO la estrategia es distinta: su error no depende de la distancia a nada, sino de cuánto lleva sin que lo corrijan. `vioTimeDrift` devuelve un objeto con `reset()`, que hay que llamar tras cada `POSE_RESET` — si no, sigue inflando la incertidumbre para siempre y el estimator termina ignorando un sensor que acaba de quedar perfecto.

### Las dos fuentes concretas

`LimelightPoseSource` lee NetworkTables directamente, sin `LimelightHelpers` — esa clase es un archivo que se copia, no un artefacto Maven, y no hace falta porque el esquema lo conocemos: lo escribe el simulador de esta misma feature. Soporta MegaTag1 y MegaTag2, y **le escribe el rumbo al Limelight sola**, que es el paso que más se olvida y el que hace que MegaTag2 devuelva ceros sin explicar por qué.

`QuestNavPoseSource` envuelve el `QuestNav` del vendordep, deshace `ROBOT_TO_QUEST` y descarta lo que llegue sin tracking. También detecta el `dataTimestamp` centinela de 1 µs y avisa una vez en vez de meterle al estimator mediciones mal fechadas.

### El campo 3D

`VisionSimTelemetry.forWorld("main").publish()` publica el mundo simulado como topics `struct:Pose3d[]`: la pose del robot, dónde está cada cámara, todos los AprilTags del campo y cuáles se están viendo ahora. Se ve en AdvantageScope o en el visor 3D de MARS Desktop, que detecta solo por el nombre del topic que son poses de visión.

El `Field2d` plano de photonlib sigue publicándose también, para una vista cenital rápida en Glass.

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

El proyecto de robot necesita el vendordep de PhotonVision (`vendordeps/photonlib.json`), no solo las coordenadas Maven: las nativas de `photontargeting` hacen falta en runtime para la simulación. QuestNav es Java puro, sin nativas.

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

- `vision/sim/SimulatedLimelight.java` — API pública de la cámara simulada.
- `vision/sim/LimelightSimConfig.java` — configuración (tabla NT, óptica, qué keys publicar).
- `vision/sim/LimelightTargetTranslator.java` — `PhotonPipelineResult` → `LimelightFrame`.
- `vision/sim/LimelightNTPublisher.java` — escribe el esquema NT y lee las keys de entrada.
- `vision/sim/LimelightSimDevice.java` — registra el `SimDevice` de HAL.
- `vision/sim/LimelightCoordinates.java` — todas las conversiones de convención, con tests.
- `vision/sim/VisionSimWorld.java` — registro de mundos `VisionSystemSim` compartidos.
- `vio/sim/SimulatedQuestNav.java` — API pública del headset simulado.
- `vio/sim/VioErrorModel.java` — deriva y modos de fallo del VIO. El contenido real de esa feature.
- `vio/sim/QuestOriginFrame.java` — origen del headset y `POSE_RESET`, con tests.
- `vio/sim/QuestNavNTBridge.java` — publica los topics protobuf y atiende los comandos.
- `vio/sim/QuestNavSimDevice.java` — `SimDevice` con los interruptores de fallo.
- `fusion/PoseFusion.java` — el portero que filtra las mediciones de visión.
- `fusion/PoseSource.java` — una fuente de mediciones; los filtros viven aquí.
- `fusion/ConsensusGate.java` — arbitraje entre fuentes que se contradicen, con tests.
- `fusion/StdDevStrategies.java` — cómo calcular la confianza de cada medición.
- `fusion/LimelightPoseSource.java` / `fusion/QuestNavPoseSource.java` — las dos fuentes concretas.
- `vision/LimelightBotpose.java` — el formato de `botpose`, compartido entre el simulador y el lector.
- `RobotLocalization.java` — entry point de la feature.
- `generated/FeatureConstants.java` — rewritten by Gradle on every `compileJava`; don't edit it.
- `MarsFeature.json` — the descriptor MARS reads. `version` here drives both the Maven artifact and the installer.

### Tests

`./gradlew test`. En Windows el build apunta la JVM de test al JDK de WPILib: las nativas de WPILib están compiladas contra su runtime MSVC y con otro JDK la JVM revienta al cargar `ntcore`.

---

Generated with the MARS Desktop feature wizard from the template in [Mars](https://github.com/STZ-Robotics/Mars/tree/main/templates/feature).
