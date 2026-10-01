// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

/**
 * Por que se descarto la ultima medicion de una fuente.
 *
 * <p>Existe porque el sintoma de un filtro mal calibrado y el de una camara rota son el mismo: no
 * llegan mediciones. Sin esto, averiguar cual de los siete filtros esta comiendose los datos
 * significa comentar codigo en la pista. Se publica en NetworkTables por cada fuente.
 */
public enum RejectionReason {

    /** La medicion se acepto y se entrego al pose estimator. */
    ACCEPTED,

    /** La fuente esta apagada, desde codigo o desde el dashboard. */
    DISABLED,

    /** La fuente no tenia nada que reportar este ciclo. Lo normal cuando no ve tags. */
    NO_MEASUREMENT,

    /** El robot giraba demasiado rapido. */
    ANGULAR_VELOCITY,

    /** El robot aceleraba demasiado fuerte, tipicamente un choque. */
    ACCELERATION,

    /** El robot se movia demasiado rapido. */
    PLANAR_VELOCITY,

    /** La medicion no era mas nueva que la ultima aceptada. */
    STALE_TIMESTAMP,

    /** La fuente rechazo su propia pose, normalmente por caer fuera del campo. */
    POSE_INVALID,

    /** La solucion de PnP era demasiado ambigua. */
    AMBIGUITY,

    /** Otra fuente contradijo esta medicion. */
    CONSENSUS,

    /** La medicion estaba demasiado lejos de lo que el estimator ya creia. */
    MAHALANOBIS,

    /** La fuente lanzo una excepcion. */
    SOURCE_ERROR
}
