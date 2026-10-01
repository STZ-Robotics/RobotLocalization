// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import org.junit.jupiter.api.Test;

import java.util.Random;

/**
 * Fija la semantica del marco de origen del headset. Es el equivalente en esta feature a los tests
 * de signo de {@code tx} en la Limelight simulada: si esto se invierte, el robot code se localiza al
 * reves y no hay ningun error visible en ninguna parte.
 */
class QuestOriginFrameTest {

    private static final double EPS = 1e-9;

    /** El mount real de STZ: 33.2 cm a la derecha del centro y girado -90 grados. */
    private static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                    new Translation3d(0.0, -0.332, 0.0),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));

    @Test
    void withTheOriginAtTheFieldOriginTheQuestReportsFieldPoses() {
        QuestOriginFrame frame = new QuestOriginFrame();
        Pose3d headset = new Pose3d(3.0, 4.0, 0.5, new Rotation3d(0.0, 0.0, 1.0));

        Pose3d reported = frame.toQuestFrame(headset);

        assertEquals(headset.getX(), reported.getX(), EPS);
        assertEquals(headset.getY(), reported.getY(), EPS);
        assertEquals(headset.getRotation().getZ(), reported.getRotation().getZ(), EPS);
    }

    @Test
    void anArbitraryOriginMakesTheReportedPoseDifferFromTheFieldPose() {
        // Es el caso real: el Quest arranca donde arranca, no en la esquina azul del campo.
        QuestOriginFrame frame =
                new QuestOriginFrame(new Pose3d(1.0, 2.0, 0.0, new Rotation3d(0.0, 0.0, 0.7)));
        Pose3d headset = new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero);

        Pose3d reported = frame.toQuestFrame(headset);

        assertNotEquals(headset.getX(), reported.getX(), 1e-3);
    }

    @Test
    void aPoseResetMakesTheHeadsetReportExactlyTheTargetWithoutMoving() {
        QuestOriginFrame frame =
                QuestOriginFrame.randomized(2.0, 45.0, new Random(7));
        Pose3d headsetInField = new Pose3d(5.0, 2.5, 0.6, new Rotation3d(0.0, 0.0, 1.2));
        Pose3d target = new Pose3d(8.0, 4.0, 0.6, new Rotation3d(0.0, 0.0, -0.4));

        frame.reset(headsetInField, target);
        Pose3d reported = frame.toQuestFrame(headsetInField);

        assertEquals(target.getX(), reported.getX(), 1e-9);
        assertEquals(target.getY(), reported.getY(), 1e-9);
        assertEquals(target.getZ(), reported.getZ(), 1e-9);
        assertEquals(target.getRotation().getZ(), reported.getRotation().getZ(), 1e-9);
    }

    @Test
    void afterAResetTheRobotPoseRoundTripsThroughTheMountTransform() {
        // Esto es literalmente lo que hace el robot code: pide un reset a robotPose ⊕ ROBOT_TO_QUEST
        // y luego deshace el mount sobre lo que recibe. Si la composicion esta mal, el error aparece
        // aqui y no en competencia.
        QuestOriginFrame frame = QuestOriginFrame.randomized(3.0, 90.0, new Random(11));
        Pose3d robotInField = new Pose3d(6.0, 3.0, 0.0, new Rotation3d(0.0, 0.0, 0.9));
        Pose3d headsetInField = robotInField.transformBy(ROBOT_TO_QUEST);

        frame.reset(headsetInField, headsetInField);
        Pose3d reportedRobot = frame.toQuestFrame(headsetInField).transformBy(ROBOT_TO_QUEST.inverse());

        assertEquals(robotInField.getX(), reportedRobot.getX(), 1e-9);
        assertEquals(robotInField.getY(), reportedRobot.getY(), 1e-9);
        assertEquals(
                robotInField.getRotation().getZ(), reportedRobot.getRotation().getZ(), 1e-9);
    }

    @Test
    void resettingDoesNotMoveTheHeadsetItMovesTheOrigin() {
        QuestOriginFrame frame = new QuestOriginFrame();
        Pose3d headsetInField = new Pose3d(4.0, 1.0, 0.0, Rotation3d.kZero);
        Pose3d originBefore = frame.origin();

        frame.reset(headsetInField, new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero));

        assertNotEquals(originBefore.getX(), frame.origin().getX(), 1e-6);
        assertEquals(4.0, frame.origin().getX(), 1e-9);
    }
}
