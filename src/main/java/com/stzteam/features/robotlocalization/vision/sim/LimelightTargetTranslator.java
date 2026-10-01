// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Traduce un {@code PhotonPipelineResult} al {@link LimelightFrame} equivalente.
 *
 * <p>Aqui vive toda la semantica: los cambios de signo entre convenciones, el calculo de
 * los metadatos que Limelight anexa a {@code botpose} (tag count, span, distancia y area
 * promedio) y las dos estimaciones de pose:
 *
 * <ul>
 *   <li><b>MegaTag1</b> ({@code botpose*}): PnP multi-tag si hay dos o mas tags visibles,
 *       y si no el tag de menor ambiguedad. No usa ninguna informacion externa.</li>
 *   <li><b>MegaTag2</b> ({@code botpose_orb*}): PnP restringido fijando el yaw que el robot
 *       code escribio en {@code robot_orientation_set}. Si el robot no escribe esa key, esta
 *       estimacion no existe, exactamente igual que con hardware real.</li>
 * </ul>
 */
public class LimelightTargetTranslator {

    private final AprilTagFieldLayout layout;
    private final PhotonPoseEstimator estimator;
    private final double headingScaleFactor;

    private Transform3d robotToCamera;
    private Pose3d lastEstimatedPose = Pose3d.kZero;
    private boolean constrainedSolverAvailable = true;

    /**
     * Crea un traductor atado a un layout de campo y a una posicion de camara.
     *
     * @param layout             layout de AprilTags contra el que se estiman poses.
     * @param robotToCamera      transformada del origen del robot a la camara, en convencion WPILib.
     * @param headingScaleFactor peso del heading en el solver de MegaTag2.
     */
    public LimelightTargetTranslator(
            AprilTagFieldLayout layout, Transform3d robotToCamera, double headingScaleFactor) {
        this.layout = layout;
        this.robotToCamera = robotToCamera;
        this.headingScaleFactor = headingScaleFactor;
        this.estimator = new PhotonPoseEstimator(layout, robotToCamera);

        // El PnP restringido de MegaTag2 baja a OpenCV. El constructor de 2 argumentos no
        // fuerza la carga de la nativa, asi que la pedimos aqui en vez de descubrir que falta
        // a mitad de un ciclo.
        try {
            OpenCvLoader.forceStaticLoad();
        } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
            constrainedSolverAvailable = false;
        }
    }

    /**
     * Actualiza la posicion de la camara respecto al robot. Se llama cuando el robot code
     * escribe {@code camerapose_robotspace_set}.
     *
     * @param robotToCamera nueva transformada robot-&gt;camara.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        estimator.setRobotToCameraTransform(robotToCamera);
    }

    /** @return la transformada robot-&gt;camara vigente. */
    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    /**
     * Traduce un resultado de PhotonVision al frame equivalente de Limelight.
     *
     * @param result       resultado recien leido de la camara simulada.
     * @param inputs       lo que el robot code escribio en la tabla este ciclo.
     * @param cameraMatrix intrinsecos de la camara, necesarios para MegaTag2.
     * @param distCoeffs   coeficientes de distorsion, necesarios para MegaTag2.
     * @param megaTag2     si hay que intentar la estimacion de MegaTag2.
     * @return el frame traducido.
     */
    public LimelightFrame translate(
            PhotonPipelineResult result,
            LimelightNTInputs inputs,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            boolean megaTag2) {

        double pipelineLatencyMs = result.metadata.getLatencyMillis();
        double captureTimestamp = result.getTimestampSeconds();

        List<PhotonTrackedTarget> targets = filterTargets(result, inputs);
        if (targets.isEmpty()) {
            return LimelightFrame.empty(pipelineLatencyMs, 0.0, captureTimestamp);
        }

        PhotonTrackedTarget best = targets.get(0);
        PhotonPipelineResult filtered = withTargets(result, targets, inputs);

        LimelightFrame.RawFiducial[] fiducials = buildFiducials(targets);
        Optional<Pose3d> megaTag1Pose = estimateMegaTag1(filtered);
        Optional<Pose3d> megaTag2Pose =
                megaTag2
                        ? estimateMegaTag2(filtered, inputs, cameraMatrix, distCoeffs, megaTag1Pose)
                        : Optional.empty();

        megaTag2Pose.or(() -> megaTag1Pose).ifPresent(pose -> lastEstimatedPose = pose);

        Transform3d cameraToTarget = best.getBestCameraToTarget();

        return new LimelightFrame(
                true,
                LimelightCoordinates.yawToTx(best.getYaw()),
                LimelightCoordinates.pitchToTy(best.getPitch()),
                LimelightCoordinates.yawToTx(best.getYaw()),
                LimelightCoordinates.pitchToTy(best.getPitch()),
                best.getArea(),
                best.getSkew(),
                best.getFiducialId(),
                pipelineLatencyMs,
                0.0,
                captureTimestamp,
                targets.size(),
                fiducials,
                megaTag1Pose,
                megaTag2Pose,
                tagSpanMeters(targets),
                averageTagDistance(targets),
                averageTagArea(targets),
                Optional.of(cameraToTarget),
                Optional.of(robotToCamera.plus(cameraToTarget)),
                pixelMetrics(best),
                flattenCorners(targets));
    }

    /**
     * Aplica el filtro de ids que puso el robot code. Sin filtro, devuelve los targets tal
     * cual, preservando el orden de photonlib (el mejor primero).
     */
    private List<PhotonTrackedTarget> filterTargets(
            PhotonPipelineResult result, LimelightNTInputs inputs) {
        if (!inputs.hasFiducialFilter()) {
            return result.getTargets();
        }
        List<PhotonTrackedTarget> kept = new ArrayList<>(result.getTargets().size());
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (inputs.acceptsFiducial(target.getFiducialId())) {
                kept.add(target);
            }
        }
        return kept;
    }

    /**
     * Reconstruye el resultado con la lista filtrada. El resultado multi-tag original solo
     * se conserva si todos los tags que uso sobrevivieron al filtro; si no, se descarta
     * para que el estimador no use una solucion contaminada.
     */
    private PhotonPipelineResult withTargets(
            PhotonPipelineResult original,
            List<PhotonTrackedTarget> targets,
            LimelightNTInputs inputs) {
        if (!inputs.hasFiducialFilter()) {
            return original;
        }

        var multitag = original.getMultiTagResult();
        boolean multitagStillValid =
                multitag.isPresent()
                        && multitag.get().fiducialIDsUsed.stream()
                                .allMatch(id -> inputs.acceptsFiducial(id.intValue()));

        return new PhotonPipelineResult(
                original.metadata, targets, multitagStillValid ? multitag : Optional.empty());
    }

    /** MegaTag1: multi-tag si lo hay, si no el tag de menor ambiguedad. */
    private Optional<Pose3d> estimateMegaTag1(PhotonPipelineResult result) {
        Optional<Pose3d> multiTag =
                estimator.estimateCoprocMultiTagPose(result).map(e -> e.estimatedPose);
        if (multiTag.isPresent()) {
            return multiTag;
        }
        return estimator.estimateLowestAmbiguityPose(result).map(e -> e.estimatedPose);
    }

    /**
     * MegaTag2: PnP restringido con el yaw del robot fijado. Devuelve vacio si el robot no
     * publico su orientacion, que es el mismo comportamiento que tiene un Limelight real
     * cuando nadie llama a {@code SetRobotOrientation}.
     */
    private Optional<Pose3d> estimateMegaTag2(
            PhotonPipelineResult result,
            LimelightNTInputs inputs,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            Optional<Pose3d> seed) {

        if (!constrainedSolverAvailable || inputs.robotHeading().isEmpty()) {
            return Optional.empty();
        }
        if (cameraMatrix == null || distCoeffs == null) {
            return Optional.empty();
        }

        Rotation2d heading = inputs.robotHeading().get();
        estimator.addHeadingData(result.getTimestampSeconds(), heading);

        try {
            return estimator
                    .estimateConstrainedSolvepnpPose(
                            result,
                            cameraMatrix,
                            distCoeffs,
                            seed.orElse(lastEstimatedPose),
                            false,
                            headingScaleFactor)
                    .map(e -> e.estimatedPose);
        } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
            // El solver restringido vive en JNI. Si la libreria nativa no esta disponible
            // (tests headless, por ejemplo) se desactiva MegaTag2 en vez de reventar cada ciclo.
            constrainedSolverAvailable = false;
            System.err.println(
                    "[SimulatedLimelight] Constrained SolvePnP no disponible; "
                            + "botpose_orb queda desactivado: "
                            + e.getMessage());
            return Optional.empty();
        }
    }

    private LimelightFrame.RawFiducial[] buildFiducials(List<PhotonTrackedTarget> targets) {
        LimelightFrame.RawFiducial[] out = new LimelightFrame.RawFiducial[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            double distToCamera = cameraToTarget.getTranslation().getNorm();
            double distToRobot = robotToCamera.plus(cameraToTarget).getTranslation().getNorm();

            out[i] =
                    new LimelightFrame.RawFiducial(
                            target.getFiducialId(),
                            LimelightCoordinates.yawToTx(target.getYaw()),
                            LimelightCoordinates.pitchToTy(target.getPitch()),
                            target.getArea(),
                            distToCamera,
                            distToRobot,
                            target.getPoseAmbiguity());
        }
        return out;
    }

    /** Distancia maxima entre los tags visibles, que es lo que Limelight llama "tag span". */
    private double tagSpanMeters(List<PhotonTrackedTarget> targets) {
        List<Translation3d> positions = new ArrayList<>(targets.size());
        for (PhotonTrackedTarget target : targets) {
            layout.getTagPose(target.getFiducialId())
                    .ifPresent(pose -> positions.add(pose.getTranslation()));
        }

        double span = 0.0;
        for (int i = 0; i < positions.size(); i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                span = Math.max(span, positions.get(i).getDistance(positions.get(j)));
            }
        }
        return span;
    }

    private double averageTagDistance(List<PhotonTrackedTarget> targets) {
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return sum / targets.size();
    }

    private double averageTagArea(List<PhotonTrackedTarget> targets) {
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getArea();
        }
        return sum / targets.size();
    }

    /**
     * Metricas en pixeles del target primario, derivadas del rectangulo de area minima que
     * ya calcula photonlib.
     */
    private LimelightFrame.PixelMetrics pixelMetrics(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        if (corners == null || corners.size() < 4) {
            return LimelightFrame.PixelMetrics.ZERO;
        }

        double sideA = distance(corners.get(0), corners.get(1));
        double sideB = distance(corners.get(1), corners.get(2));

        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (TargetCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        return new LimelightFrame.PixelMetrics(
                Math.max(sideA, sideB), Math.min(sideA, sideB), maxX - minX, maxY - minY);
    }

    private static double distance(TargetCorner a, TargetCorner b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private static double[] flattenCorners(List<PhotonTrackedTarget> targets) {
        List<TargetCorner> all = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            List<TargetCorner> corners = target.getDetectedCorners();
            if (corners != null) {
                all.addAll(corners);
            }
        }

        double[] out = new double[all.size() * 2];
        for (int i = 0; i < all.size(); i++) {
            out[i * 2] = all.get(i).x;
            out[i * 2 + 1] = all.get(i).y;
        }
        return out;
    }
}
