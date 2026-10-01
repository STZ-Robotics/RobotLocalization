// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Optional;

/**
 * Comprueba que la tabla publicada es la que un consumidor de Limelight espera encontrar:
 * mismos nombres de key, mismas unidades y mismos largos de array.
 */
class LimelightNTPublisherTest {

    private static final double EPS = 1e-9;

    private NetworkTableInstance instance;
    private LimelightSimConfig config;
    private LimelightNTPublisher publisher;
    private NetworkTable table;

    @BeforeEach
    void setUp() {
        instance = NetworkTableInstance.create();
        config =
                new LimelightSimConfig()
                        .withTableName("limelight-test")
                        .withFieldLayout(
                                new AprilTagFieldLayout(List.of(new AprilTag(1, Pose3d.kZero)), 17.5, 8.0));
        publisher = new LimelightNTPublisher(config, instance);
        table = instance.getTable("limelight-test");
    }

    @AfterEach
    void tearDown() {
        instance.close();
    }

    @Test
    void publishesTheCoreTargetingKeys() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        assertEquals(1.0, table.getEntry("tv").getDouble(-1.0), EPS);
        assertEquals(-12.0, table.getEntry("tx").getDouble(0.0), EPS);
        assertEquals(4.0, table.getEntry("ty").getDouble(0.0), EPS);
        assertEquals(3.2, table.getEntry("ta").getDouble(0.0), EPS);
        assertEquals(7.0, table.getEntry("tid").getDouble(0.0), EPS);
        assertEquals(22.0, table.getEntry("tl").getDouble(0.0), EPS);
        assertEquals(0.0, table.getEntry("cl").getDouble(-1.0), EPS);
    }

    @Test
    void heartbeatAdvancesOnEveryPublish() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);
        double first = table.getEntry("hb").getDouble(-1.0);

        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);
        double second = table.getEntry("hb").getDouble(-1.0);

        assertEquals(first + 1.0, second, EPS);
    }

    @Test
    void t2dCarriesSeventeenValuesInTheDocumentedOrder() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        double[] t2d = table.getEntry("t2d").getDoubleArray(new double[0]);

        assertEquals(17, t2d.length);
        assertEquals(1.0, t2d[0], EPS); // targetValid
        assertEquals(1.0, t2d[1], EPS); // targetCount
        assertEquals(22.0, t2d[2], EPS); // tl
        assertEquals(-12.0, t2d[4], EPS); // tx
        assertEquals(7.0, t2d[9], EPS); // tid
    }

    @Test
    void botposeFamilyHasTheLengthLimelightHelpersRequires() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        double[] blue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        double[] red = table.getEntry("botpose_wpired").getDoubleArray(new double[0]);
        double[] center = table.getEntry("botpose").getDoubleArray(new double[0]);

        assertEquals(11 + 7, blue.length);
        assertEquals(blue.length, red.length);
        assertEquals(blue.length, center.length);

        assertEquals(3.0, blue[0], EPS);
        assertEquals(4.0, blue[1], EPS);
        assertEquals(17.5 - 3.0, red[0], 1e-6);
        assertEquals(8.0 - 4.0, red[1], 1e-6);
        assertEquals(3.0 - 17.5 / 2.0, center[0], 1e-6);
    }

    @Test
    void botposeIsEmptyWhenThereIsNoEstimate() {
        publisher.publish(LimelightFrame.empty(20.0, 0.0, 1.0), LimelightNTInputs.NONE);

        // Un array vacio hace que LimelightHelpers devuelva un PoseEstimate con tagCount 0,
        // que es la forma canonica de decir "no confies en esto".
        assertEquals(0, table.getEntry("botpose_wpiblue").getDoubleArray(new double[] {1.0}).length);
        assertEquals(0.0, table.getEntry("tv").getDouble(-1.0), EPS);
        assertEquals(-1.0, table.getEntry("tid").getDouble(0.0), EPS);
    }

    @Test
    void megaTag2KeysStayEmptyUntilTheRobotPublishesItsHeading() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        assertEquals(
                0, table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {1.0}).length);
    }

    @Test
    void theDefaultFieldIs2026AndNot2025() {
        // WPILib 2026.1.1 solo llega hasta k2025ReefscapeWelded, asi que kDefaultField devuelve el
        // campo del ano pasado sin avisar: 22 tags donde deberia haber 32, y todo lo demas
        // aparentemente funcionando. La feature trae su propia copia del oficial.
        var field = com.stzteam.features.robotlocalization.FieldLayouts.defaultField();

        assertEquals(32, field.getTags().size(), "el campo 2026 Rebuilt tiene 32 tags");
        assertEquals(16.541, field.getFieldLength(), 1e-3);
        assertEquals(8.069, field.getFieldWidth(), 1e-3);
    }

    @Test
    void unusedBotposeOriginsCanBeTurnedOffToSaveBandwidth() {
        // Seis arrays de 32 doubles a la tasa de la camara son 0.19 Mbps medidos, y casi todo el
        // mundo lee uno solo.
        config.withBotposeVariants(LimelightSimConfig.BotposeVariant.WPI_BLUE);

        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        assertTrue(
                table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]).length > 0,
                "el azul si se publica");
        assertEquals(
                0,
                table.getEntry("botpose_wpired").getDoubleArray(new double[0]).length,
                "el rojo no deberia haberse tocado");
        assertEquals(
                0,
                table.getEntry("botpose").getDoubleArray(new double[0]).length,
                "el legacy tampoco");
    }

    @Test
    void allBotposeOriginsArePublishedByDefault() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        for (String key : new String[] {"botpose", "botpose_wpiblue", "botpose_wpired"}) {
            assertTrue(
                    table.getEntry(key).getDoubleArray(new double[0]).length > 0,
                    key + " deberia publicarse por defecto, como en el hardware real");
        }
    }

    @Test
    void rawFiducialsArePackedSevenValuesPerTag() {
        publisher.publish(frameWithTarget(), LimelightNTInputs.NONE);

        double[] raw = table.getEntry("rawfiducials").getDoubleArray(new double[0]);

        assertEquals(7, raw.length);
        assertEquals(7.0, raw[0], EPS);
    }

    @Test
    void readsBackWhatTheRobotCodeWrites() {
        // Lo mismo que hacen LimelightHelpers.SetRobotOrientation,
        // SetFiducialIDFiltersOverride y setPipelineIndex.
        table.getEntry("robot_orientation_set")
                .setDoubleArray(new double[] {45.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[] {3.0, 5.0});
        table.getEntry("pipeline").setDouble(2.0);

        LimelightNTInputs inputs = publisher.readInputs();

        assertTrue(inputs.robotHeading().isPresent());
        assertEquals(45.0, inputs.robotHeading().get().getDegrees(), 1e-6);
        assertArrayEquals(new int[] {3, 5}, inputs.fiducialIdFilters());
        assertEquals(2, inputs.pipeline());
        assertTrue(inputs.acceptsFiducial(3));
        assertTrue(!inputs.acceptsFiducial(4));
    }

    @Test
    void cameraPoseOverrideIsIgnoredWhenDisabled() {
        config.withCameraPoseOverride(false);
        table.getEntry("camerapose_robotspace_set")
                .setDoubleArray(new double[] {0.3, 0.0, 0.25, 0.0, -20.0, 0.0});

        assertTrue(publisher.readInputs().cameraPoseOverride().isEmpty());

        config.withCameraPoseOverride(true);
        assertTrue(publisher.readInputs().cameraPoseOverride().isPresent());
    }

    @Test
    void getpipeEchoesThePipelineTheRobotAskedFor() {
        LimelightNTInputs inputs =
                new LimelightNTInputs(4, 1, 0, Optional.empty(), new int[0], Optional.empty());

        publisher.publish(frameWithTarget(), inputs);

        assertEquals(4.0, table.getEntry("getpipe").getDouble(-1.0), EPS);
        assertEquals("pipe_fiducial", table.getEntry("getpipetype").getString(""));
    }

    private static LimelightFrame frameWithTarget() {
        LimelightFrame.RawFiducial[] fiducials = {
            new LimelightFrame.RawFiducial(7, -12.0, 4.0, 3.2, 2.5, 2.7, 0.05)
        };
        return new LimelightFrame(
                true,
                -12.0,
                4.0,
                -12.0,
                4.0,
                3.2,
                0.0,
                7,
                22.0,
                0.0,
                123.0,
                1,
                fiducials,
                Optional.of(new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero)),
                Optional.empty(),
                0.0,
                2.5,
                3.2,
                Optional.empty(),
                Optional.empty(),
                new LimelightFrame.PixelMetrics(40.0, 30.0, 45.0, 35.0),
                new double[0]);
    }
}
