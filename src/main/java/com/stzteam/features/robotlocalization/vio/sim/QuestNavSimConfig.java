// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import gg.questnav.questnav.BuildConfig;

/**
 * Configuracion de una {@link SimulatedQuestNav}.
 *
 * <p>Los setters son fluidos, igual que en {@code LimelightSimConfig}:
 *
 * <pre>{@code
 * var config = new QuestNavSimConfig()
 *         .withDriftPerMeter(0.005)
 *         .withRandomizedOrigin(true);
 * }</pre>
 *
 * <h2>Sobre los valores por defecto del modelo de error</h2>
 * Son <b>plausibles, no medidos</b>. Estan elegidos para que el orden de magnitud sea razonable
 * (un Quest bien montado deriva poco) y para que los fallos se noten en una prueba corta. Si tienes
 * logs de tu propio headset, ajustalos; no los trates como una caracterizacion del hardware.
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

    /** Crea una configuracion con los valores por defecto. */
    public QuestNavSimConfig() {}

    /**
     * Preset sin ningun error: la pose publicada es la verdad exacta. Util para aislar un bug del
     * robot code sin que el ruido estorbe, y como punto de partida de un test.
     *
     * @return una configuracion sin deriva, ruido ni fallos automaticos.
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
     * Nombre de la tabla de NetworkTables. Solo se cambia para tests: el robot code de produccion
     * habla contra {@code "QuestNav"}, que es lo que tiene fijado {@code questnavlib}.
     *
     * @param name nombre de la tabla.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withTableName(String name) {
        this.tableName = name;
        return this;
    }

    /**
     * Version que se publica en el topic {@code version}. Por defecto es la del propio
     * {@code questnavlib} del classpath, asi que el chequeo de version del robot code pasa siempre.
     * Cambiala para probar que tu codigo avisa de un desajuste.
     *
     * @param version cadena de version.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withVersion(String version) {
        this.version = version;
        return this;
    }

    /**
     * Semilla del generador de ruido. Fijarla hace que una corrida sea reproducible, que es lo que
     * quieres en un test.
     *
     * @param seed semilla.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withRandomSeed(long seed) {
        this.randomSeed = seed;
        return this;
    }

    /**
     * Controla si se registra el {@code SimDevice} de HAL que aparece en "Other Devices".
     *
     * @param enabled true para registrarlo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withSimDevice(boolean enabled) {
        this.createSimDevice = enabled;
        return this;
    }

    /**
     * Controla la publicacion de poses de telemetria para AdvantageScope y del {@code Field2d}.
     *
     * @param enabled true para publicarlas.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withTelemetry(boolean enabled) {
        this.publishTelemetry = enabled;
        return this;
    }

    /**
     * Enciende el visor MJPEG: un {@code PhotonCameraSim} montado donde va el headset, que renderiza
     * el wireframe del campo desde su punto de vista y lo publica por {@code CameraServer}.
     *
     * <p><b>Es cosmetico.</b> No alimenta la pose: el Quest localiza por odometria visual-inercial,
     * no viendo AprilTags. Va apagado por defecto porque renderiza con OpenCV en cada frame y eso
     * cuesta CPU de verdad.
     *
     * @param enabled true para encenderlo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withViewfinder(boolean enabled) {
        this.viewfinder = enabled;
        return this;
    }

    /**
     * Mundo simulado compartido que usa el visor. Ponlo igual que el de tus camaras para que todo
     * comparta un solo {@code VisionSystemSim}.
     *
     * @param name identificador del mundo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withVisionWorld(String name) {
        this.visionWorldName = name;
        return this;
    }

    /**
     * Layout de AprilTags que dibuja el visor. Solo se usa para renderizar: el Quest no localiza con
     * tags. Si el mundo simulado ya existe porque lo creo otra camara, manda el layout de aquella.
     *
     * @param layout layout del campo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        return this;
    }

    /**
     * Frecuencia a la que el headset publica frames. El hardware real va a 100 Hz.
     *
     * @param hz frecuencia en hercios.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withFrameRateHz(double hz) {
        this.frameRateHz = hz;
        return this;
    }

    /**
     * Frecuencia de {@code deviceData} (bateria y contador de perdidas). El hardware real va a 3 Hz.
     *
     * @param hz frecuencia en hercios.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withDeviceDataRateHz(double hz) {
        this.deviceDataRateHz = hz;
        return this;
    }

    /**
     * Publica desde una instancia de NetworkTables aparte, conectada como cliente al servidor NT
     * del propio robot, en vez de escribir directamente en la instancia global.
     *
     * <h2>Por que existe esta opcion</h2>
     * {@code PoseFrame.dataTimestamp()} --- el valor que hay que meterle a
     * {@code addVisionMeasurement} --- no sale del protobuf: sale del {@code serverTime} que
     * NetworkTables le pone a cada valor. Y ese {@code serverTime} lo asigna el <b>servidor al
     * recibir</b>. Cuando el publicador vive en la misma instancia que el consumidor, ntcore marca
     * los valores con el centinela {@code serverTime = 1}, asi que {@code dataTimestamp()} sale
     * 1 microsegundo constante y cualquier fusion por timestamp se va al suelo.
     *
     * <p>Con esta opcion encendida la simulacion reproduce la topologia real --- el headset es un
     * cliente NT4 y el robot es el servidor --- y los timestamps salen correctos, con latencia de
     * red incluida.
     *
     * <h2>Por que esta apagada por defecto</h2>
     * Si el programa no tiene un servidor NT4 escuchando, el cliente no conecta y no se publica
     * <b>nada</b>. Publicar con timestamps pobres es un fallo degradado; no publicar es un fallo
     * total. En modo local todo lo demas --- pose, tracking, bateria, comandos --- funciona igual.
     *
     * <p>Enciendela si tu robot code fusiona por {@code dataTimestamp()}, que es el caso normal de
     * un pose estimator.
     *
     * @param enabled true para publicar como cliente de red.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withNetworkClient(boolean enabled) {
        this.networkClient = enabled;
        return this;
    }

    /**
     * Nombre con el que se identifica el cliente NT4, si {@link #withNetworkClient(boolean)} esta
     * encendido.
     *
     * @param name nombre del cliente.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withClientName(String name) {
        this.clientName = name;
        return this;
    }

    /**
     * Direccion del servidor NT4 al que conectarse. Por defecto la loopback, que es donde escucha el
     * robot cuando corres {@code simulateJava}.
     *
     * @param address direccion del servidor.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withServerAddress(String address) {
        this.serverAddress = address;
        return this;
    }

    /**
     * Arranca con el origen del Quest desplazado al azar en vez de coincidiendo con el del campo.
     *
     * <p>Merece la pena encenderlo al menos una vez: con el origen en identidad, un equipo que nunca
     * llama a {@code setPose()} ve poses de campo correctas por accidente, y el bug no aparece hasta
     * que enciende el headset de verdad.
     *
     * @param enabled true para aleatorizarlo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withRandomizedOrigin(boolean enabled) {
        this.randomizedOrigin = enabled;
        return this;
    }

    /**
     * Magnitud del origen aleatorio.
     *
     * @param translationMeters desplazamiento maximo en metros.
     * @param rotationDegrees   desviacion maxima de yaw en grados.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withRandomOriginRange(
            double translationMeters, double rotationDegrees) {
        this.randomOriginTranslationMeters = translationMeters;
        this.randomOriginRotationDegrees = rotationDegrees;
        return this;
    }

    /**
     * Deriva de posicion por metro recorrido, como fraccion. 0.003 es 0.3 %.
     *
     * @param fraction fraccion de deriva por metro.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withDriftPerMeter(double fraction) {
        this.driftPerMeter = fraction;
        return this;
    }

    /**
     * Deriva de rumbo por vuelta completa del robot, en grados.
     *
     * @param degrees grados de deriva por vuelta.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withDriftDegreesPerTurn(double degrees) {
        this.driftDegreesPerTurn = degrees;
        return this;
    }

    /**
     * Cuanto deriva el eje Z respecto a los horizontales. Un robot en el suelo casi no deriva en
     * altura, de ahi que el factor por defecto sea menor que 1.
     *
     * @param factor factor multiplicativo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withVerticalDriftFactor(double factor) {
        this.verticalDriftFactor = factor;
        return this;
    }

    /**
     * Ruido blanco por frame, encima de la deriva acumulada.
     *
     * @param meters  desviacion tipica en metros.
     * @param degrees desviacion tipica de yaw en grados.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withFrameNoise(double meters, double degrees) {
        this.frameNoiseMeters = meters;
        this.frameNoiseDegrees = degrees;
        return this;
    }

    /**
     * Cuanto se desplaza la pose al recuperar el tracking. La documentacion de QuestNav avisa de que
     * el headset puede re-adquirir "con un offset", y ese salto es el fallo que mas dano hace.
     *
     * @param meters magnitud del salto en metros.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withReacquireOffsetMeters(double meters) {
        this.reacquireOffsetMeters = meters;
        return this;
    }

    /**
     * Cuanto dura una perdida de tracking.
     *
     * @param seconds duracion en segundos.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withTrackingLossSeconds(double seconds) {
        this.trackingLossSeconds = seconds;
        return this;
    }

    /**
     * Controla si un golpe fuerte tira el tracking solo.
     *
     * @param enabled true para activarlo.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withImpactTrackingLoss(boolean enabled) {
        this.impactTrackingLoss = enabled;
        return this;
    }

    /**
     * Umbrales a partir de los cuales un golpe tira el tracking.
     *
     * @param metersPerSecondSquared  aceleracion lineal.
     * @param degreesPerSecondSquared aceleracion angular.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withImpactThresholds(
            double metersPerSecondSquared, double degreesPerSecondSquared) {
        this.impactAccelThreshold = metersPerSecondSquared;
        this.impactAngularAccelThreshold = degreesPerSecondSquared;
        return this;
    }


    /**
     * Carga de bateria al arrancar.
     *
     * @param percent porcentaje de 0 a 100.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withInitialBatteryPercent(double percent) {
        this.initialBatteryPercent = percent;
        return this;
    }

    /**
     * Descarga de bateria por minuto. Al llegar a cero el headset se apaga y deja de publicar,
     * igual que el real.
     *
     * @param percentPerMinute puntos porcentuales por minuto.
     * @return esta misma configuracion.
     */
    public QuestNavSimConfig withBatteryDrainPercentPerMinute(double percentPerMinute) {
        this.batteryDrainPercentPerMinute = percentPerMinute;
        return this;
    }

    /** @return el nombre de la tabla NT. */
    public String tableName() {
        return tableName;
    }

    /** @return la version que se publica. */
    public String version() {
        return version;
    }

    /** @return la semilla del generador de ruido. */
    public long randomSeed() {
        return randomSeed;
    }

    /** @return true si hay que registrar el SimDevice. */
    public boolean createSimDevice() {
        return createSimDevice;
    }

    /** @return true si hay que publicar telemetria. */
    public boolean publishTelemetry() {
        return publishTelemetry;
    }

    /** @return true si el visor MJPEG esta encendido. */
    public boolean viewfinder() {
        return viewfinder;
    }

    /** @return el nombre del mundo simulado del visor. */
    public String visionWorldName() {
        return visionWorldName;
    }

    /** @return el layout de AprilTags que dibuja el visor. */
    public AprilTagFieldLayout fieldLayout() {
        return fieldLayout;
    }

    /** @return la frecuencia de publicacion de frames. */
    public double frameRateHz() {
        return frameRateHz;
    }

    /** @return la frecuencia de publicacion de deviceData. */
    public double deviceDataRateHz() {
        return deviceDataRateHz;
    }

    /** @return true si hay que publicar desde una instancia cliente de NetworkTables. */
    public boolean networkClient() {
        return networkClient;
    }

    /** @return el nombre del cliente NT4. */
    public String clientName() {
        return clientName;
    }

    /** @return la direccion del servidor NT4. */
    public String serverAddress() {
        return serverAddress;
    }

    /** @return true si el origen del Quest arranca desplazado al azar. */
    public boolean randomizedOrigin() {
        return randomizedOrigin;
    }

    /** @return desplazamiento maximo del origen aleatorio, en metros. */
    public double randomOriginTranslationMeters() {
        return randomOriginTranslationMeters;
    }

    /** @return desviacion maxima de yaw del origen aleatorio, en grados. */
    public double randomOriginRotationDegrees() {
        return randomOriginRotationDegrees;
    }

    /** @return la deriva por metro, como fraccion. */
    public double driftPerMeter() {
        return driftPerMeter;
    }

    /** @return la deriva de rumbo por vuelta, en grados. */
    public double driftDegreesPerTurn() {
        return driftDegreesPerTurn;
    }

    /** @return el factor de deriva vertical. */
    public double verticalDriftFactor() {
        return verticalDriftFactor;
    }

    /** @return el ruido por frame en metros. */
    public double frameNoiseMeters() {
        return frameNoiseMeters;
    }

    /** @return el ruido de yaw por frame en grados. */
    public double frameNoiseDegrees() {
        return frameNoiseDegrees;
    }

    /** @return el salto de pose al re-adquirir, en metros. */
    public double reacquireOffsetMeters() {
        return reacquireOffsetMeters;
    }

    /** @return la duracion de una perdida de tracking, en segundos. */
    public double trackingLossSeconds() {
        return trackingLossSeconds;
    }

    /** @return true si los golpes tiran el tracking. */
    public boolean impactTrackingLoss() {
        return impactTrackingLoss;
    }

    /** @return el umbral de aceleracion lineal para detectar un golpe. */
    public double impactAccelThreshold() {
        return impactAccelThreshold;
    }

    /** @return el umbral de aceleracion angular para detectar un golpe. */
    public double impactAngularAccelThreshold() {
        return impactAngularAccelThreshold;
    }


    /** @return la carga de bateria inicial. */
    public double initialBatteryPercent() {
        return initialBatteryPercent;
    }

    /** @return la descarga de bateria por minuto. */
    public double batteryDrainPercentPerMinute() {
        return batteryDrainPercentPerMinute;
    }
}
