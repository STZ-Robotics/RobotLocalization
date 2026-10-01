// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vio.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.util.Units;

/**
 * Registra el headset simulado como un {@code SimDevice} de HAL, para que aparezca solo en el panel
 * "Other Devices" del Sim GUI bajo el nodo {@code QuestNav}, igual que la Limelight simulada
 * aparece bajo {@code Limelight}.
 *
 * <p>Fuera de simulacion {@code SimDevice.create} devuelve {@code null} y la clase entera queda como
 * un no-op.
 *
 * <h2>Los tres interruptores</h2>
 * Los campos {@code kInput} son la parte util de verdad: se marcan desde el Sim GUI, en caliente,
 * sin recompilar ni mover el robot.
 *
 * <ul>
 *   <li><b>Force Tracking Loss</b> &mdash; el Quest pierde features. Al recuperar, la pose vuelve
 *       desplazada.</li>
 *   <li><b>Force Pose Jump</b> &mdash; el doble-tap a passthrough. La pose salta pero
 *       {@code isTracking} se queda en true, que es lo traicionero.</li>
 *   <li><b>Force Link Loss</b> &mdash; cable flojo. Se deja de publicar y el robot ve caer
 *       {@code isConnected()}.</li>
 * </ul>
 *
 * <pre>{@code
 * var sim = new SimDeviceSim("QuestNav:questnav", 0);
 * sim.getBoolean("Force Tracking Loss").set(true);
 * double drift = sim.getDouble("Drift (m)").get();
 * }</pre>
 */
public class QuestNavSimDevice implements AutoCloseable {

    /** Prefijo que agrupa a todos los headsets de esta feature en el Sim GUI. */
    public static final String DEVICE_PREFIX = "QuestNav:";

    private final SimDevice device;

    private final SimBoolean tracking;
    private final SimBoolean connected;
    private final SimDouble frameCount;
    private final SimDouble trackingLostCounter;
    private final SimDouble battery;

    private final SimDouble questX;
    private final SimDouble questY;
    private final SimDouble questYaw;
    private final SimDouble driftMeters;
    private final SimDouble driftYaw;

    private final SimBoolean forceTrackingLoss;
    private final SimBoolean forcePoseJump;
    private final SimBoolean forceLinkLoss;

    /**
     * Crea el dispositivo simulado.
     *
     * @param name  nombre logico del headset.
     * @param index indice para distinguir varios headsets con el mismo nombre.
     */
    public QuestNavSimDevice(String name, int index) {
        device = SimDevice.create(DEVICE_PREFIX + name, index);

        if (device == null) {
            tracking = null;
            connected = null;
            frameCount = null;
            trackingLostCounter = null;
            battery = null;
            questX = null;
            questY = null;
            questYaw = null;
            driftMeters = null;
            driftYaw = null;
            forceTrackingLoss = null;
            forcePoseJump = null;
            forceLinkLoss = null;
            return;
        }

        tracking = device.createBoolean("isTracking", SimDevice.Direction.kOutput, false);
        connected = device.createBoolean("Publishing", SimDevice.Direction.kOutput, false);
        frameCount = device.createDouble("Frame Count", SimDevice.Direction.kOutput, 0.0);
        trackingLostCounter =
                device.createDouble("Tracking Lost Count", SimDevice.Direction.kOutput, 0.0);
        battery = device.createDouble("Battery (%)", SimDevice.Direction.kOutput, 100.0);

        questX = device.createDouble("Quest X (m)", SimDevice.Direction.kOutput, 0.0);
        questY = device.createDouble("Quest Y (m)", SimDevice.Direction.kOutput, 0.0);
        questYaw = device.createDouble("Quest Yaw (deg)", SimDevice.Direction.kOutput, 0.0);
        driftMeters = device.createDouble("Drift (m)", SimDevice.Direction.kOutput, 0.0);
        driftYaw = device.createDouble("Drift Yaw (deg)", SimDevice.Direction.kOutput, 0.0);

        forceTrackingLoss =
                device.createBoolean("Force Tracking Loss", SimDevice.Direction.kInput, false);
        forcePoseJump = device.createBoolean("Force Pose Jump", SimDevice.Direction.kInput, false);
        forceLinkLoss = device.createBoolean("Force Link Loss", SimDevice.Direction.kInput, false);
    }

    /** @return true si estamos en simulacion y el dispositivo existe. */
    public boolean isActive() {
        return device != null;
    }

    /** @return true si alguien pidio perder el tracking. */
    public boolean forceTrackingLoss() {
        return forceTrackingLoss != null && forceTrackingLoss.get();
    }

    /** @return true si alguien pidio un salto de pose. */
    public boolean forcePoseJump() {
        return forcePoseJump != null && forcePoseJump.get();
    }

    /** @return true si alguien pidio cortar el enlace. */
    public boolean forceLinkLoss() {
        return forceLinkLoss != null && forceLinkLoss.get();
    }

    /**
     * Baja el interruptor de salto de pose. Es un pulso, no un estado: si se quedara marcado, la
     * pose saltaria en cada ciclo y el Sim GUI no tendria forma de mandar un solo salto.
     */
    public void clearPoseJumpRequest() {
        if (forcePoseJump != null) {
            forcePoseJump.set(false);
        }
    }

    /**
     * Vuelca el estado en los campos del dispositivo.
     *
     * @param frame     ultimo frame generado.
     * @param model     modelo de error, de donde salen contadores y deriva.
     * @param publishing si este ciclo se esta publicando de verdad.
     */
    public void update(QuestNavFrame frame, VioErrorModel model, boolean publishing) {
        if (device == null) {
            return;
        }

        tracking.set(frame.tracking());
        connected.set(publishing);
        frameCount.set(frame.frameCount());
        trackingLostCounter.set(model.trackingLostCounter());
        battery.set(model.batteryPercent());

        questX.set(frame.questPose().getX());
        questY.set(frame.questPose().getY());
        questYaw.set(Units.radiansToDegrees(frame.questPose().getRotation().getZ()));

        driftMeters.set(model.driftTranslation().getNorm());
        driftYaw.set(model.driftYawDegrees());
    }

    /** Libera el dispositivo de HAL. */
    @Override
    public void close() {
        if (device != null) {
            device.close();
        }
    }
}
