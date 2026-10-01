// Copyright (c) 2026 STZ Robotics
package com.stzteam.features.robotlocalization.fusion;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Arbitra entre fuentes que se contradicen.
 *
 * <p>Una sola camara que reporte una pose absurda pasa cualquier filtro individual: es
 * geometricamente valida, la ambiguedad es baja y el robot esta quieto. Lo unico que la delata es
 * que <b>otra fuente dice algo distinto</b>. Eso es lo que mira esta clase.
 *
 * <h2>Compensacion de movimiento</h2>
 * Dos mediciones tomadas con {@code dt} de diferencia describen posiciones distintas si el robot se
 * movio en medio, y eso no es un desacuerdo. En vez de propagar una pose hacia la otra --- que
 * obligaria a saber si las {@code ChassisSpeeds} vienen en marco de robot o de campo --- se ensancha
 * la tolerancia por la distancia que el robot pudo recorrer en ese tiempo:
 *
 * <pre>{@code tolerancia_efectiva = tolerancia + velocidad * dt}</pre>
 *
 * Es conservador y no depende del marco de referencia. Sin esto, con la ventana por defecto de 0.1 s
 * y un robot a 4 m/s, dos fuentes perfectas discrepan 0.4 m --- comparable a la tolerancia entera ---
 * y el filtro empieza a descartar mediciones buenas justo cuando mas falta hacen.
 *
 * <h2>Voto por mayoria</h2>
 * Se cuenta, por cada candidato, con cuantos otros concuerda. Sobreviven los del conteo mas alto. Si
 * empatan, se descartan todos los que esten en disputa: un empate significa que la evidencia no
 * alcanza para elegir, y meterle al estimator una pose que otra fuente contradice es peor que no
 * meterle ninguna.
 */
public class ConsensusGate {

    /** Que hacer cuando dos fuentes se contradicen. */
    public enum Mode {
        /**
         * Descartar a las dos. Es lo prudente con dos fuentes, que es el caso normal: si no hay una
         * tercera que desempate, no hay forma de saber cual miente.
         */
        REJECT_BOTH,

        /**
         * Quedarse con las mas respaldadas. Solo tiene sentido con tres fuentes o mas; con dos
         * siempre hay empate y se comporta como {@link #REJECT_BOTH}.
         */
        MAJORITY_VOTE
    }

    /**
     * Un candidato a arbitrar.
     *
     * @param name             nombre de la fuente, para los mensajes.
     * @param pose             pose propuesta.
     * @param timestampSeconds instante de la captura.
     */
    public record Candidate(String name, Pose2d pose, double timestampSeconds) {}

    /**
     * Resultado del arbitraje.
     *
     * @param rejected  indices de los candidatos descartados.
     * @param conflicts descripcion de cada desacuerdo, para telemetria o avisos.
     */
    public record Result(Set<Integer> rejected, List<String> conflicts) {

        /** Resultado sin descartes ni conflictos. */
        static final Result CLEAN = new Result(Set.of(), List.of());

        /** @return true si todas las fuentes concordaron. */
        public boolean isUnanimous() {
            return conflicts.isEmpty();
        }
    }

    private Mode mode = Mode.REJECT_BOTH;
    private double toleranceMeters = 0.5;
    private double timeWindowSeconds = 0.1;
    private boolean motionCompensation = true;

    /**
     * Que hacer ante un desacuerdo.
     *
     * @param newMode modo de arbitraje.
     * @return esta misma puerta.
     */
    public ConsensusGate withMode(Mode newMode) {
        this.mode = newMode;
        return this;
    }

    /**
     * Cuanto pueden separarse dos poses antes de considerarse contradictorias.
     *
     * @param meters tolerancia en metros.
     * @return esta misma puerta.
     */
    public ConsensusGate withToleranceMeters(double meters) {
        this.toleranceMeters = meters;
        return this;
    }

    /**
     * Separacion temporal maxima para que dos mediciones se comparen. Mas alla de esta ventana
     * describen momentos distintos y no se contrastan.
     *
     * @param seconds ventana en segundos.
     * @return esta misma puerta.
     */
    public ConsensusGate withTimeWindowSeconds(double seconds) {
        this.timeWindowSeconds = seconds;
        return this;
    }

    /**
     * Controla si la tolerancia se ensancha con la velocidad del robot.
     *
     * @param enabled true para compensar el movimiento.
     * @return esta misma puerta.
     */
    public ConsensusGate withMotionCompensation(boolean enabled) {
        this.motionCompensation = enabled;
        return this;
    }

    /** @return el modo de arbitraje vigente. */
    public Mode mode() {
        return mode;
    }

    /** @return la tolerancia en metros. */
    public double toleranceMeters() {
        return toleranceMeters;
    }

    /** @return la ventana temporal en segundos. */
    public double timeWindowSeconds() {
        return timeWindowSeconds;
    }

    /**
     * Arbitra entre los candidatos.
     *
     * @param candidates            candidatos del ciclo, en cualquier orden.
     * @param robotSpeedMetersPerSec velocidad del robot, para compensar el movimiento. Pasa 0 si no
     *                               la tienes.
     * @return que candidatos hay que descartar y por que.
     */
    public Result evaluate(List<Candidate> candidates, double robotSpeedMetersPerSec) {
        int n = candidates.size();
        if (n < 2) {
            return Result.CLEAN;
        }

        boolean[][] agrees = new boolean[n][n];
        Set<Integer> disputed = new HashSet<>();
        List<String> conflicts = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Candidate a = candidates.get(i);
                Candidate b = candidates.get(j);

                double dt = Math.abs(a.timestampSeconds() - b.timestampSeconds());
                if (dt > timeWindowSeconds) {
                    // Momentos distintos: no son comparables, ni para concordar ni para discrepar.
                    continue;
                }

                double distance = a.pose().getTranslation().getDistance(b.pose().getTranslation());
                double allowed = toleranceMeters;
                if (motionCompensation) {
                    allowed += Math.abs(robotSpeedMetersPerSec) * dt;
                }

                if (distance <= allowed) {
                    agrees[i][j] = true;
                    agrees[j][i] = true;
                } else {
                    disputed.add(i);
                    disputed.add(j);
                    conflicts.add(
                            String.format(
                                    "%s vs %s: %.2f m (limite %.2f m)",
                                    a.name(), b.name(), distance, allowed));
                }
            }
        }

        if (disputed.isEmpty()) {
            return Result.CLEAN;
        }

        if (mode == Mode.REJECT_BOTH) {
            return new Result(disputed, conflicts);
        }

        return new Result(majorityLosers(agrees, disputed, n), conflicts);
    }

    /**
     * Devuelve los candidatos en disputa que menos respaldo tienen.
     *
     * <p>El respaldo se cuenta sobre <b>todos</b> los candidatos, no solo sobre los que estan en
     * disputa: a una fuente la corrobora cualquiera que concuerde con ella, este o no discutiendo
     * con alguien mas.
     */
    private static Set<Integer> majorityLosers(boolean[][] agrees, Set<Integer> disputed, int n) {
        int[] support = new int[n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && agrees[i][j]) {
                    support[i]++;
                }
            }
        }

        int best = Integer.MIN_VALUE;
        int worst = Integer.MAX_VALUE;
        for (int i : disputed) {
            best = Math.max(best, support[i]);
            worst = Math.min(worst, support[i]);
        }

        if (best == worst) {
            // Todos los que discuten tienen el mismo respaldo: no hay mayoria, se van todos.
            return new HashSet<>(disputed);
        }

        Set<Integer> losers = new HashSet<>();
        for (int i : disputed) {
            if (support[i] < best) {
                losers.add(i);
            }
        }
        return losers;
    }
}
