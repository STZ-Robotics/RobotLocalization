package com.stzteam.features.robotlocalization;

import com.stzteam.features.robotlocalization.generated.FeatureConstants;

/**
 * RobotLocalization
 *
 * <p>Utilidades de localizacion para MARS.</p>
 *
 * <p>Lo que hay hoy en la feature:</p>
 * <ul>
 *   <li>{@link com.stzteam.features.robotlocalization.vision.sim} &mdash; una Limelight
 *       simulada sobre PhotonVision. Genera detecciones fisicamente coherentes contra el
 *       {@code AprilTagFieldLayout} oficial y las publica con el esquema de NetworkTables de
 *       un Limelight real, ademas de registrarse como {@code SimDevice} de HAL para aparecer
 *       en "Other Devices" del Sim GUI. Punto de entrada:
 *       {@link com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight}.</li>
 *   <li>{@link com.stzteam.features.robotlocalization.vio.sim} &mdash; una QuestNav simulada.
 *       Hace de headset: publica el esquema protobuf de NetworkTables que publica un Meta Quest,
 *       con un modelo de deriva y de los modos de fallo reales del VIO (perdida de tracking con
 *       re-adquisicion desplazada, discontinuidades por vibracion, bateria y red). Punto de
 *       entrada: {@link com.stzteam.features.robotlocalization.vio.sim.SimulatedQuestNav}.</li>
 * </ul>
 *
 * <p>Feature ID: RobotLocalization &middot; author: STZ-Robotics</p>
 */
public final class RobotLocalization {

    private RobotLocalization() {}

    /**
     * Version de la feature, tal y como la declara {@code MarsFeature.json}.
     *
     * @return la version publicada.
     */
    public static String version() {
        return FeatureConstants.FEATURE_VERSION;
    }

    /**
     * Nombre de la feature.
     *
     * @return el identificador de la feature.
     */
    public static String featureName() {
        return FeatureConstants.FEATURE_NAME;
    }
}
