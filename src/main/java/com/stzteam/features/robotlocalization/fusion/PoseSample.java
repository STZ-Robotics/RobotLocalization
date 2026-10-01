// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A pose measurement ready to hand to the pose estimator.
 *
 * <p>It is exactly the triple {@code addVisionMeasurement} expects: where the source thinks the
 * robot is, when it saw it, and how much it trusts that.
 *
 * @param pose             robot pose on the field, blue-origin.
 * @param timestampSeconds the instant of <b>capture</b>, not of the read, in the FPGA time base.
 *                         Stamping this wrong is the most common way to feed noise into a pose
 *                         estimator without any error showing up.
 * @param stdDevs          standard deviations {@code [x, y, theta]}, in metres and radians. See
 *                         {@link StdDevStrategies} to compute them.
 */
public record PoseSample(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
