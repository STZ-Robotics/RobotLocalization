// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.FieldLayouts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * La pose que resuelve la camara contra el campo <b>2026</b>, en la escena exacta de una simulacion
 * viva que la daba mal.
 *
 * <p>Los otros tests de integracion fijan el campo 2025 a proposito, para no depender de que cambie
 * el layout por defecto. Eso les impidio ver esto.
 */
class Field2026SolveTest {

    private static final String CAMERA = "limelight-2026";
    private static final double LOOP_PERIOD = 0.02;

    /** Donde estaba el robot en la simulacion cuando la solucion salia a 12 m de distancia. */
    private static final Pose2d ROBOT_POSE = new Pose2d(2.71, 2.56, Rotation2d.fromDegrees(1.4));

    /** El mismo montaje del proyecto de pruebas. */
    private static final Transform3d ROBOT_TO_CAMERA =
            new Transform3d(
                    new Translation3d(0.30, 0.0, 0.50),
                    new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));

    private AprilTagFieldLayout layout;
    private SimulatedLimelight limelight;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        SimDeviceSim.resetData();
        SimHooks.pauseTiming();

        layout = FieldLayouts.rebuilt2026Welded();
        limelight =
                new SimulatedLimelight(
                        CAMERA,
                        ROBOT_TO_CAMERA,
                        LimelightSimConfig.limelight3G().withFieldLayout(layout));
    }

    @AfterEach
    void tearDown() {
        SimulatedLimelight.closeAll();
        VisionSimWorld.reset();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    private void runLoops(int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(LOOP_PERIOD);
            limelight.periodic(ROBOT_POSE);
        }
    }

    @Test
    void theFieldLayoutIsThe2026One() {
        assertEquals(32, layout.getTags().size());
        assertEquals(16.541, layout.getFieldLength(), 1e-3);
    }

    @Test
    void theMegaTag1SolutionLandsOnTheRobot() {
        runLoops(20);

        LimelightFrame frame = limelight.lastFrame();
        assertTrue(frame.hasTarget(), "la camara deberia ver tags");
        assertTrue(frame.megaTag1Pose().isPresent(), "deberia haber solucion de MegaTag1");

        Pose3d solved = frame.megaTag1Pose().get();
        assertEquals(
                ROBOT_POSE.getX(),
                solved.getX(),
                0.30,
                "la solucion deberia caer sobre el robot, y cayo en x=" + solved.getX());
        assertEquals(ROBOT_POSE.getY(), solved.getY(), 0.30);
    }

    @Test
    void theCameraCannotSeeTagsAcrossTheWholeField() {
        // VisionSystemSim no modela oclusion, asi que sin limite de alcance la camara ve cualquier
        // tag dentro del FOV a traves de las paredes del campo. Con el default de 7 m entraban siete
        // tags --- cinco de ellos entre 8 y 13 m con ambiguedades de hasta 0.99 --- y el PnP
        // multi-tag los mezclaba con los cercanos y resolvia a doce metros del robot.
        runLoops(20);

        LimelightFrame frame = limelight.lastFrame();
        for (LimelightFrame.RawFiducial fiducial : frame.fiducials()) {
            assertTrue(
                    fiducial.distToCamera() <= 7.5,
                    "el tag " + fiducial.id() + " esta a " + fiducial.distToCamera()
                            + " m y no deberia ser visible");
        }
    }

    @Test
    void theSolutionStaysPutAsTagsComeAndGo() {
        // Una solucion que salta de un lado del campo al otro entre frames es peor que ninguna: el
        // pose estimator se la come y arrastra al robot con ella.
        runLoops(20);
        Pose3d first = limelight.lastFrame().megaTag1Pose().orElseThrow();

        runLoops(40);
        Pose3d later = limelight.lastFrame().megaTag1Pose().orElseThrow();

        assertEquals(
                0.0,
                first.getTranslation().getDistance(later.getTranslation()),
                0.20,
                "el robot no se movio, la solucion tampoco deberia");
    }
}
