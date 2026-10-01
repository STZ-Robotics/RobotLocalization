// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.simulation.SimCameraProperties;

/**
 * Configuracion de una {@link SimulatedLimelight}: todo lo que en un prototipo iria
 * hardcodeado (nombre de tabla NT, optica de la camara, que keys publicar) vive aqui.
 *
 * <p>Los setters son fluidos y devuelven {@code this}, asi que se encadenan:
 *
 * <pre>{@code
 * var config = LimelightSimConfig.limelight3G()
 *         .withTableName("limelight-front")
 *         .withMegaTag2(true);
 * }</pre>
 *
 * <p>Los presets de camara parten de las especificaciones publicadas por Limelight y
 * son aproximaciones razonables, no calibraciones reales. Si tienes la calibracion de
 * tu camara, pasala con {@link #withCameraProperties(SimCameraProperties)}.
 */
public class LimelightSimConfig {

    /**
     * Cada uno de los origenes en los que un Limelight publica la pose del robot.
     *
     * <p>Un Limelight real publica los tres, y con MegaTag2 son seis arrays en total. Cada array son
     * {@code 11 + 7 * tags} doubles: con tres tags visibles, medidos contra una simulacion viva, los
     * seis suman <b>0.19 Mbps</b> de NetworkTables a 10 Hz de suscripcion --- y casi todo el mundo
     * lee uno solo.
     */
    public enum BotposeVariant {
        /** {@code botpose}: origen legacy en el centro del campo. */
        LEGACY_CENTER,
        /** {@code botpose_wpiblue}: origen en la esquina azul. Es el que usa casi todo el mundo. */
        WPI_BLUE,
        /** {@code botpose_wpired}: origen en la esquina roja. */
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

    /** Crea una configuracion con los valores por defecto (perfil Limelight 3G). */
    public LimelightSimConfig() {}

    /**
     * Preset aproximado de un Limelight 2/2+ a 1280x720, tomado directamente de los
     * presets que trae photonlib.
     *
     * @return una configuracion nueva con ese perfil de camara.
     */
    public static LimelightSimConfig limelight2() {
        return new LimelightSimConfig().withCameraProperties(SimCameraProperties.LL2_1280_720());
    }

    /**
     * Preset aproximado de un Limelight 3: 1280x800, FOV diagonal ~75 grados, 90 FPS.
     *
     * @return una configuracion nueva con ese perfil de camara.
     */
    public static LimelightSimConfig limelight3() {
        return new LimelightSimConfig().withCameraProperties(limelight3Properties());
    }

    /**
     * Preset aproximado de un Limelight 3G: misma resolucion que el 3 pero optica mas
     * ancha, FOV diagonal ~91 grados.
     *
     * @return una configuracion nueva con ese perfil de camara.
     */
    public static LimelightSimConfig limelight3G() {
        return new LimelightSimConfig().withCameraProperties(limelight3GProperties());
    }

    /**
     * Preset aproximado de un Limelight 4: optica del 3G con mas FPS y menos latencia.
     *
     * @return una configuracion nueva con ese perfil de camara.
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
     * Nombre de la tabla de NetworkTables donde se publican las keys. Tiene que ser el
     * mismo string que el robot code le pasa a {@code LimelightHelpers}.
     *
     * @param name nombre de la tabla, por ejemplo {@code "limelight-front"}.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withTableName(String name) {
        this.tableName = (name == null || name.isBlank()) ? "limelight" : name;
        return this;
    }

    /**
     * Nombre del mundo simulado compartido. Varias camaras con el mismo nombre comparten
     * un unico {@code VisionSystemSim}, que es lo que quieres para multi-camara.
     *
     * @param name identificador del mundo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withVisionWorld(String name) {
        this.visionWorldName = name;
        return this;
    }

    /**
     * Layout de AprilTags contra el que se simulan las detecciones y se estiman poses.
     *
     * @param layout layout del campo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        return this;
    }

    /**
     * Propiedades opticas y de temporizado de la camara simulada.
     *
     * @param properties propiedades de photonlib.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withCameraProperties(SimCameraProperties properties) {
        this.cameraProperties = properties;
        return this;
    }

    /**
     * Controla si se registra un {@code SimDevice} de HAL para que la camara aparezca en
     * el panel "Other Devices" del Sim GUI.
     *
     * @param enabled true para registrarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withSimDevice(boolean enabled) {
        this.createSimDevice = enabled;
        return this;
    }

    /**
     * Controla la publicacion de {@code botpose}, {@code botpose_wpiblue} y
     * {@code botpose_wpired}.
     *
     * @param enabled true para publicarlas.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withBotpose(boolean enabled) {
        this.publishBotpose = enabled;
        return this;
    }

    /**
     * Controla la publicacion de las keys {@code botpose_orb*} (equivalente a MegaTag2).
     * Requiere que el robot code llame a {@code LimelightHelpers.SetRobotOrientation}
     * cada ciclo; si no, esas keys se quedan vacias, igual que con hardware real.
     *
     * @param enabled true para publicarlas.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withMegaTag2(boolean enabled) {
        this.publishMegaTag2 = enabled;
        return this;
    }

    /**
     * Controla la publicacion de la key {@code rawfiducials}.
     *
     * @param enabled true para publicarla.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withRawFiducials(boolean enabled) {
        this.publishRawFiducials = enabled;
        return this;
    }

    /**
     * Controla la publicacion de {@code tcornxy}. Apagado por defecto porque es el array
     * mas pesado y casi nadie lo usa.
     *
     * @param enabled true para publicarla.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withCorners(boolean enabled) {
        this.publishCorners = enabled;
        return this;
    }

    /**
     * Controla la publicacion de {@code targetpose_cameraspace},
     * {@code targetpose_robotspace}, {@code camerapose_targetspace} y
     * {@code botpose_targetspace}.
     *
     * <p>Apagado por defecto a proposito: la convencion de ejes de target space de
     * Limelight (X derecha, Y abajo, Z saliendo) es la parte del esquema mas facil de
     * interpretar mal, y es preferible que estas keys se queden vacias de forma obvia a
     * que publiquen numeros plausibles pero equivocados. Si las enciendes, valida los
     * signos contra tu hardware real antes de confiar en ellas.
     *
     * @param enabled true para publicarlas.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withTargetSpacePoses(boolean enabled) {
        this.publishTargetSpacePoses = enabled;
        return this;
    }

    /**
     * Distancia mas alla de la cual la camara simulada deja de ver un tag.
     *
     * <p><b>Esto no es afinado fino, es lo que separa una camara de un rayo X.</b> Photonlib no
     * limita el alcance por defecto, y {@code VisionSystemSim} no modela oclusion: sin un limite, la
     * camara ve todos los tags dentro de su FOV por lejos que esten y a traves de las paredes del
     * campo.
     *
     * <p>Medido contra el campo 2026 con el robot a (2.7, 2.6): sin limite entraban siete tags,
     * cinco de ellos entre 8 y 13 m con ambiguedades de 0.6 a 0.99, y el PnP multi-tag los mezclaba
     * con los dos cercanos buenos y devolvia una pose a doce metros del robot y a metro y medio de
     * altura.
     *
     * <p>El default de 7 m es lo que un Limelight 3G resuelve de forma fiable sobre un tag de
     * 16.5 cm. Si mides el tuyo, pon el numero medido.
     *
     * @param meters alcance maximo en metros.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withMaxSightRange(double meters) {
        this.maxSightRangeMeters = meters;
        return this;
    }

    /**
     * Area minima en pixeles para que un tag cuente como detectado.
     *
     * <p>Es el otro freno contra los tags imposibles: uno a mucha distancia ocupa unos pocos pixeles
     * y ningun detector real lo saca. Photonlib usa 100 por defecto; aqui son 200, que a 1280x800
     * corresponde aproximadamente al limite util de un tag de 16.5 cm.
     *
     * @param pixels area minima en pixeles.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withMinTargetAreaPixels(double pixels) {
        this.minTargetAreaPixels = pixels;
        return this;
    }

    /**
     * Elige en que origenes publicar la pose del robot.
     *
     * <p>Por defecto se publican los tres, que es lo que hace un Limelight real. Si tu robot code
     * solo lee uno --- lo normal --- dejar solo ese recorta dos tercios del ancho de banda que gasta
     * esta camara en NetworkTables.
     *
     * <pre>{@code config.withBotposeVariants(BotposeVariant.WPI_BLUE);}</pre>
     *
     * @param variants origenes a publicar; sin argumentos no se publica ninguno.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withBotposeVariants(BotposeVariant... variants) {
        this.botposeVariants =
                variants.length == 0
                        ? java.util.EnumSet.noneOf(BotposeVariant.class)
                        : java.util.EnumSet.of(variants[0], variants);
        return this;
    }

    /**
     * Publica por {@code CameraServer} el stream MJPEG crudo que renderiza photonlib: los AprilTags
     * vistos desde la camara, en <code>&lt;nombre&gt;-raw</code>. Se ve en Glass, Shuffleboard o
     * AdvantageScope.
     *
     * <p>Apagado por defecto porque renderiza con OpenCV en cada frame y cuesta CPU de verdad.
     *
     * @param enabled true para publicarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withRawStream(boolean enabled) {
        this.rawStream = enabled;
        return this;
    }

    /**
     * Dibuja el wireframe del campo en el stream crudo. Ayuda mucho a entender hacia donde esta
     * mirando la camara cuando no hay ningun tag a la vista.
     *
     * <p>Requiere {@link #withRawStream(boolean)}.
     *
     * @param enabled true para dibujarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withWireframe(boolean enabled) {
        this.wireframe = enabled;
        return this;
    }

    /**
     * Publica tambien el stream anotado, con el crosshair y los contornos de los targets detectados,
     * en <code>&lt;nombre&gt;-processed</code>.
     *
     * @param enabled true para publicarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withProcessedStream(boolean enabled) {
        this.processedStream = enabled;
        return this;
    }

    /**
     * Publica en {@code SmartDashboard} el {@code Field2d} de depuracion que mantiene photonlib, con
     * la pose del robot, la de las camaras y la de los targets visibles.
     *
     * @param enabled true para publicarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withDebugField(boolean enabled) {
        this.publishDebugField = enabled;
        return this;
    }

    /**
     * Controla si la simulacion respeta {@code camerapose_robotspace_set}, es decir, si
     * el robot code puede mover la camara en caliente como en un Limelight real.
     *
     * @param enabled true para respetarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withCameraPoseOverride(boolean enabled) {
        this.honorCameraPoseOverride = enabled;
        return this;
    }

    /**
     * Controla si la simulacion respeta {@code fiducial_id_filters_set} al estimar poses.
     *
     * @param enabled true para respetarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withFiducialFilters(boolean enabled) {
        this.honorFiducialFilters = enabled;
        return this;
    }

    /**
     * Controla si la simulacion respeta {@code throttle_set} saltandose frames.
     *
     * @param enabled true para respetarlo.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withThrottle(boolean enabled) {
        this.honorThrottle = enabled;
        return this;
    }

    /**
     * Peso del heading en el solver de MegaTag2. Valores mas altos penalizan mas
     * desviarse del yaw que reporto el robot code.
     *
     * @param factor factor de escala, por defecto 0.02.
     * @return esta misma configuracion.
     */
    public LimelightSimConfig withMegaTag2HeadingScaleFactor(double factor) {
        this.megaTag2HeadingScaleFactor = factor;
        return this;
    }

    /** @return el nombre de la tabla NT configurada. */
    public String tableName() {
        return tableName;
    }

    /** @return el nombre del mundo simulado compartido. */
    public String visionWorldName() {
        return visionWorldName;
    }

    /** @return el layout de AprilTags configurado. */
    public AprilTagFieldLayout fieldLayout() {
        return fieldLayout;
    }

    /** @return las propiedades de la camara simulada. */
    public SimCameraProperties cameraProperties() {
        return cameraProperties;
    }

    /** @return true si hay que registrar el SimDevice de HAL. */
    public boolean createSimDevice() {
        return createSimDevice;
    }

    /** @return true si hay que publicar las keys de botpose. */
    public boolean publishBotpose() {
        return publishBotpose;
    }

    /** @return true si hay que publicar las keys de MegaTag2. */
    public boolean publishMegaTag2() {
        return publishMegaTag2;
    }

    /** @return true si hay que publicar rawfiducials. */
    public boolean publishRawFiducials() {
        return publishRawFiducials;
    }

    /** @return true si hay que publicar tcornxy. */
    public boolean publishCorners() {
        return publishCorners;
    }

    /** @return true si hay que publicar las poses en target space. */
    public boolean publishTargetSpacePoses() {
        return publishTargetSpacePoses;
    }

    /** @return alcance maximo de la camara en metros. */
    public double maxSightRangeMeters() {
        return maxSightRangeMeters;
    }

    /** @return area minima en pixeles para detectar un tag. */
    public double minTargetAreaPixels() {
        return minTargetAreaPixels;
    }

    /**
     * Indica si hay que publicar un origen concreto.
     *
     * @param variant origen a comprobar.
     * @return true si esta en la lista configurada.
     */
    public boolean publishesBotposeVariant(BotposeVariant variant) {
        return botposeVariants.contains(variant);
    }

    /** @return true si hay que publicar el stream MJPEG crudo. */
    public boolean rawStream() {
        return rawStream;
    }

    /** @return true si hay que dibujar el wireframe del campo. */
    public boolean wireframe() {
        return wireframe;
    }

    /** @return true si hay que publicar el stream anotado. */
    public boolean processedStream() {
        return processedStream;
    }

    /** @return true si hay que publicar el Field2d de depuracion. */
    public boolean publishDebugField() {
        return publishDebugField;
    }

    /** @return true si se respeta camerapose_robotspace_set. */
    public boolean honorCameraPoseOverride() {
        return honorCameraPoseOverride;
    }

    /** @return true si se respeta fiducial_id_filters_set. */
    public boolean honorFiducialFilters() {
        return honorFiducialFilters;
    }

    /** @return true si se respeta throttle_set. */
    public boolean honorThrottle() {
        return honorThrottle;
    }

    /** @return el factor de escala del heading para MegaTag2. */
    public double megaTag2HeadingScaleFactor() {
        return megaTag2HeadingScaleFactor;
    }
}
