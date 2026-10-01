// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Un frame de la QuestNav simulada, listo para publicarse.
 *
 * <p>El Quest publica a 100 Hz y el ciclo del robot va a 50, asi que cada ciclo genera varios
 * frames de estos, con timestamps de NetworkTables separados 10 ms. Esa es la diferencia de forma
 * mas grande respecto a la Limelight simulada, que publica como mucho uno por ciclo.
 *
 * @param questPose            pose del headset, en el marco de origen del propio Quest y ya con el
 *                             error del modelo VIO aplicado. Esto es literalmente lo que va al
 *                             campo {@code pose3d} del protobuf.
 * @param truePose             pose real del robot segun la simulacion, solo para telemetria. Nunca
 *                             se publica en la tabla {@code QuestNav}.
 * @param tracking             valor del campo {@code isTracking}.
 * @param frameCount           contador de frames del Quest, monotono.
 * @param appTimestampSeconds  segundos desde que arranco la app del Quest (campo {@code timestamp}
 *                             del protobuf). Ojo: el robot code no usa este valor para fechar
 *                             mediciones, usa el timestamp de NetworkTables.
 * @param ntTimestampMicros    instante en el que se publica el frame, en microsegundos de la base
 *                             de tiempo de NetworkTables.
 */
public record QuestNavFrame(
        Pose3d questPose,
        Pose3d truePose,
        boolean tracking,
        int frameCount,
        double appTimestampSeconds,
        long ntTimestampMicros) {

    /**
     * Distancia entre la pose que el Quest reporta (convertida a pose de robot) y la verdad.
     * Es la metrica que hace visible la deriva en AdvantageScope.
     *
     * @param reportedRobotPose pose del robot deducida del headset.
     * @return el error en metros.
     */
    public double translationErrorMeters(Pose3d reportedRobotPose) {
        return reportedRobotPose.getTranslation().getDistance(truePose.getTranslation());
    }
}
