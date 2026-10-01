// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Una medicion de pose lista para entregarle al pose estimator.
 *
 * <p>Es exactamente la terna que espera {@code addVisionMeasurement}: donde cree la fuente que esta
 * el robot, cuando lo vio, y cuanto confia en ello.
 *
 * @param pose             pose del robot en el campo, con origen azul.
 * @param timestampSeconds instante de la <b>captura</b>, no el de la lectura, en la base de tiempo
 *                         del FPGA. Fecharlo mal es la forma mas comun de meterle ruido a un pose
 *                         estimator sin que salte ningun error.
 * @param stdDevs          desviaciones estandar {@code [x, y, theta]}, en metros y radianes. Ver
 *                         {@link StdDevStrategies} para calcularlas.
 */
public record PoseSample(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
