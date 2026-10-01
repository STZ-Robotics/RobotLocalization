// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.util.Units;

/**
 * Registra la camara simulada como un {@code SimDevice} de HAL para que aparezca sola en el
 * panel "Other Devices" del Sim GUI de WPILib.
 *
 * <p>No hace falta escribir nada de GUI: el Sim GUI agrupa los dispositivos por el prefijo
 * del nombre, asi que todas las camaras creadas por esta feature caen bajo el nodo
 * {@code Limelight}, igual que los {@code CANMotor:TalonFX[n]} de CTRE.
 *
 * <p>Fuera de simulacion {@code SimDevice.create} devuelve {@code null}, asi que la clase
 * entera se convierte en un no-op sin necesidad de sembrar condicionales por todos lados.
 *
 * <h2>Inspeccion y forzado desde tests</h2>
 * <pre>{@code
 * var sim = new SimDeviceSim("Limelight:limelight-front", 0);
 * double tx = sim.getDouble("tx").get();
 * sim.getBoolean("Force No Targets").set(true); // simula perder la vista del tag
 * }</pre>
 */
public class LimelightSimDevice implements AutoCloseable {

    /** Prefijo que agrupa a todas las camaras de esta feature en el Sim GUI. */
    public static final String DEVICE_PREFIX = "Limelight:";

    private final SimDevice device;

    private final SimBoolean tv;
    private final SimDouble tx;
    private final SimDouble ty;
    private final SimDouble ta;
    private final SimDouble ts;
    private final SimDouble tid;
    private final SimDouble tl;
    private final SimDouble cl;
    private final SimDouble tagCount;
    private final SimDouble avgTagDist;

    private final SimDouble botposeX;
    private final SimDouble botposeY;
    private final SimDouble botposeYaw;
    private final SimBoolean megaTag2Valid;

    private final SimDouble pipeline;
    private final SimDouble ledMode;
    private final SimBoolean forceNoTargets;

    /**
     * Crea el dispositivo simulado.
     *
     * @param cameraName nombre logico de la camara, normalmente el mismo que la tabla NT.
     * @param index      indice para distinguir varias camaras con el mismo nombre.
     */
    public LimelightSimDevice(String cameraName, int index) {
        device = SimDevice.create(DEVICE_PREFIX + cameraName, index);

        if (device == null) {
            tv = null;
            tx = null;
            ty = null;
            ta = null;
            ts = null;
            tid = null;
            tl = null;
            cl = null;
            tagCount = null;
            avgTagDist = null;
            botposeX = null;
            botposeY = null;
            botposeYaw = null;
            megaTag2Valid = null;
            pipeline = null;
            ledMode = null;
            forceNoTargets = null;
            return;
        }

        tv = device.createBoolean("tv", SimDevice.Direction.kOutput, false);
        tx = device.createDouble("tx", SimDevice.Direction.kOutput, 0.0);
        ty = device.createDouble("ty", SimDevice.Direction.kOutput, 0.0);
        ta = device.createDouble("ta", SimDevice.Direction.kOutput, 0.0);
        ts = device.createDouble("ts", SimDevice.Direction.kOutput, 0.0);
        tid = device.createDouble("tid", SimDevice.Direction.kOutput, -1.0);
        tl = device.createDouble("tl (ms)", SimDevice.Direction.kOutput, 0.0);
        cl = device.createDouble("cl (ms)", SimDevice.Direction.kOutput, 0.0);
        tagCount = device.createDouble("Tag Count", SimDevice.Direction.kOutput, 0.0);
        avgTagDist = device.createDouble("Avg Tag Dist (m)", SimDevice.Direction.kOutput, 0.0);

        botposeX = device.createDouble("Botpose X (m)", SimDevice.Direction.kOutput, 0.0);
        botposeY = device.createDouble("Botpose Y (m)", SimDevice.Direction.kOutput, 0.0);
        botposeYaw = device.createDouble("Botpose Yaw (deg)", SimDevice.Direction.kOutput, 0.0);
        megaTag2Valid = device.createBoolean("MegaTag2 Valid", SimDevice.Direction.kOutput, false);

        pipeline = device.createDouble("Pipeline", SimDevice.Direction.kOutput, 0.0);
        ledMode = device.createDouble("LED Mode", SimDevice.Direction.kOutput, 0.0);

        // kInput: lo escribe el Sim GUI (o un test) y lo lee la simulacion. Permite probar
        // "que pasa si de repente pierdo el tag" sin mover el robot.
        forceNoTargets = device.createBoolean("Force No Targets", SimDevice.Direction.kInput, false);
    }

    /** @return true si estamos en simulacion y el dispositivo existe de verdad. */
    public boolean isActive() {
        return device != null;
    }

    /**
     * Indica si alguien pidio desde el Sim GUI que la camara finja no ver nada.
     *
     * @return true si hay que descartar las detecciones de este ciclo.
     */
    public boolean forceNoTargets() {
        return forceNoTargets != null && forceNoTargets.get();
    }

    /**
     * Vuelca un frame en los campos del dispositivo. Se llama en el mismo sitio y con la
     * misma frecuencia que {@link LimelightNTPublisher#publish}, para que el Sim GUI y
     * NetworkTables nunca muestren ciclos distintos.
     *
     * @param frame  frame ya traducido.
     * @param inputs entradas leidas este ciclo.
     */
    public void update(LimelightFrame frame, LimelightNTInputs inputs) {
        if (device == null) {
            return;
        }

        tv.set(frame.hasTarget());
        tx.set(frame.tx());
        ty.set(frame.ty());
        ta.set(frame.ta());
        ts.set(frame.ts());
        tid.set(frame.tid());
        tl.set(frame.pipelineLatencyMs());
        cl.set(frame.captureLatencyMs());
        tagCount.set(frame.targetCount());
        avgTagDist.set(frame.avgTagDistMeters());

        frame.megaTag1Pose()
                .ifPresentOrElse(
                        pose -> {
                            botposeX.set(pose.getX());
                            botposeY.set(pose.getY());
                            botposeYaw.set(Units.radiansToDegrees(pose.getRotation().getZ()));
                        },
                        () -> {
                            botposeX.set(0.0);
                            botposeY.set(0.0);
                            botposeYaw.set(0.0);
                        });

        megaTag2Valid.set(frame.megaTag2Pose().isPresent());
        pipeline.set(inputs.pipeline());
        ledMode.set(inputs.ledMode());
    }

    /** Libera el dispositivo de HAL. En robot real no hay nada que liberar. */
    @Override
    public void close() {
        if (device != null) {
            device.close();
        }
    }
}
