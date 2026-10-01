// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

/**
 * Why a source's last measurement was discarded.
 *
 * <p>This exists because a badly calibrated filter and a broken camera have the same symptom: no
 * measurements arrive. Without it, finding out which of the seven filters is eating the data means
 * commenting out code on the field. It is published to NetworkTables for each source.
 */
public enum RejectionReason {

    /** The measurement was accepted and handed to the pose estimator. */
    ACCEPTED,

    /** The source is off, either from code or from the dashboard. */
    DISABLED,

    /** The source had nothing to report this cycle. Normal when it sees no tags. */
    NO_MEASUREMENT,

    /** The robot was turning too fast. */
    ANGULAR_VELOCITY,

    /** The robot was accelerating too hard, typically a collision. */
    ACCELERATION,

    /** The robot was moving too fast. */
    PLANAR_VELOCITY,

    /** The measurement was not newer than the last accepted one. */
    STALE_TIMESTAMP,

    /** The source rejected its own pose, usually for falling outside the field. */
    POSE_INVALID,

    /** The PnP solution was too ambiguous. */
    AMBIGUITY,

    /** Another source contradicted this measurement. */
    CONSENSUS,

    /** The measurement was too far from what the estimator already believed. */
    MAHALANOBIS,

    /** The source threw an exception. */
    SOURCE_ERROR
}
