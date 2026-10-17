import os
import sys
import casadi as ca
import numpy as np
import yaml
import pandas as pd

params = {
    'track_name': 'LVMS_3d_smoothed.csv',
    'raceline_name': 'LVMS_3d_dallaraAV21_timeoptimal.csv',
    # 'track_name': 'mount_panorama_3d_smoothed.csv',
    # 'raceline_name': 'mount_panorama_3d_dallaraAV21_timeoptimal.csv',
    'vehicle_name': 'dallaraAV21',
    'safety_distance': 0.5,  # safety distance to track bounds in m
    'gg_mode': 'diamond',  # polar, diamond
    'gg_margin': 0.0,
    'neglect_w_omega_y': True,
    'neglect_w_omega_x': True,
    'neglect_euler': True,
    'neglect_centrifugal': True,
    'neglect_w_dot': False,
    'neglect_V_omega': False,
    'V_guess': 6.0,  # initial velocity guess
    'w_jx': 1e-2,  # cost weight for jerk x-direction
    'w_jy': 1e-2,  # cost weight for jerk y-direction
    'w_dOmega_z': 0.0,  # cost weight for curvature in road plane (e.g. 1e3)
    'w_T': 1e0,  # cost weight for time (should be 1)
    'RK4_steps': 1,
    'sol_opts': {
        "ipopt.max_iter": 5000,
        "ipopt.hessian_approximation": 'limited-memory',  # 'exact' or 'limited-memory'
        "ipopt.line_search_method": 'cg-penalty',  # 'filter', 'cg-penalty'
        "ipopt.acceptable_tol": 1e-4,
        "ipopt.acceptable_dual_inf_tol": 1e-4,
        "ipopt.constr_viol_tol": 1e-4,
    }
}

# paths
dir_path = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(dir_path, '..', 'data')
vehicle_params_path = os.path.join(data_path, 'vehicle_params', 'params_' + params['vehicle_name'] + '.yml')
gg_diagram_path = os.path.join(data_path, 'gg_diagrams', params['vehicle_name'], 'velocity_frame')
track_path = os.path.join(data_path, 'smoothed_track_data')
raceline_out_path = os.path.join(data_path, 'global_racing_lines')
sys.path.append(os.path.join(dir_path, '..', 'src'))

# load vehicle and tire parameters
with open(vehicle_params_path, 'r') as stream:
    params.update(yaml.safe_load(stream))

from track3D import Track3D
from ggManager import GGManager

# Largest bound or constraint violation an unconverged solve may end with and
# still be kept: in the units of the problem, metres of track position and
# metres per second squared of grip.
ACCEPT_UNCONVERGED_VIOLATION = 0.25


def calc_global_raceline(
        track_name: str,
        vehicle_params: dict,
        gg_mode: str,
        gg_margin: float,
        safety_distance: float,
        w_T: float,
        w_jx: float,
        w_jy: float,
        w_dOmega_z: float,
        RK4_steps: int,
        V_guess: float,
        neglect_w_omega_x: bool,
        neglect_w_omega_y: bool,
        neglect_euler: bool,
        neglect_centrifugal: bool,
        neglect_w_dot: bool,
        neglect_V_omega: bool,
        sol_opt: dict,
        out_path: str,
        init_line=None,
        smooth_ellipse: bool = False,
):
    track_handler = Track3D(
        path=os.path.join(track_path, track_name)
    )

    gg_handler = GGManager(
        gg_path=gg_diagram_path,
        gg_margin=gg_margin
    )

    # Define state variables.
    # Velocity.
    V = ca.MX.sym('V')
    # Lateral displacement.
    n = ca.MX.sym('n')

    # Relative orientation w.r.t. reference line.
    chi = ca.MX.sym('chi')

    # Longitudinal acceleration.
    ax = ca.MX.sym('ax')

    # Lateral acceleration.
    ay = ca.MX.sym('ay')

    # Create state vector.
    x = ca.vertcat(V, n, chi, ax, ay)
    nx = x.shape[0]

    # Define control variables.
    # Longitudinal jerk.
    jx = ca.MX.sym('jx')

    # Lateral acceleration.
    jy = ca.MX.sym('jy')

    # Create control vector.
    u = ca.vertcat(jx, jy)
    nu = u.shape[0]

    s = ca.MX.sym('s')

    # Time-distance scaling factor (dt/ds).
    s_dot = (V * ca.cos(chi)) / (1.0 - n * track_handler.Omega_z_interpolator(s))
    # vertical velocity
    w = n * track_handler.Omega_x_interpolator(s) * s_dot

    # Differential equations for scaled point mass model.
    dV = 1.0 / s_dot * ax
    if not neglect_w_omega_y:
        dV += w * (track_handler.Omega_x_interpolator(s) * ca.sin(chi) - track_handler.Omega_y_interpolator(s) * ca.cos(chi))

    dn = 1.0 / s_dot * V * ca.sin(chi)

    dchi = 1.0 / s_dot * ay / V - track_handler.Omega_z_interpolator(s)
    if not neglect_w_omega_x:
        dchi += w * (track_handler.Omega_x_interpolator(s) * ca.cos(chi) + track_handler.Omega_y_interpolator(s) * ca.sin(chi)) / V

    dax = jx / s_dot
    day = jy / s_dot

    dOmega_z = track_handler.Omega_z_interpolator(s) + dchi  # curvature in road plane

    # Create ODE vector.
    dx = ca.vertcat(dV, dn, dchi, dax, day)

    # Objective function.
    L_t = w_T * 1.0 / s_dot
    L_reg = w_jx * (jx / s_dot) ** 2 + w_jy * (jy / s_dot) ** 2 + w_dOmega_z * dOmega_z ** 2

    # Discrete time dynamics using fixed step Runge-Kutta 4 integrator.
    M = RK4_steps  # RK4 steps per interval
    ds_rk = track_handler.ds / M  # Step size used for RK integration
    f = ca.Function('f', [x, u, s], [dx, L_t, L_reg])  # Function returning derivative and cost function
    X0 = ca.MX.sym('X0', nx)  # Input state.
    U = ca.MX.sym('U', nu)  # Input control.
    S0 = ca.MX.sym('S0')  # Input longitudinal position.
    X = X0  # Integrated derivative.
    S = S0  # Integrated longitudinal position.
    Q_t = 0  # Integrated cost function (time).
    Q_reg = 0  # Integrated cost function (regularization).
    for j in range(M):
        k1, k1_q_t, k1_q_reg = f(X, U, S)
        k2, k2_q_t, k2_q_reg = f(X + ds_rk / 2 * k1, U, S + ds_rk / 2)
        k3, k3_q_t, k3_q_reg = f(X + ds_rk / 2 * k2, U, S + ds_rk / 2)
        k4, k4_q_t, k4_q_reg = f(X + ds_rk * k3, U, S + ds_rk)
        X = X + ds_rk / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q_t = Q_t + ds_rk / 6 * (k1_q_t + 2 * k2_q_t + 2 * k3_q_t + k4_q_t)
        Q_reg = Q_reg + ds_rk / 6 * (k1_q_reg + 2 * k2_q_reg + 2 * k3_q_reg + k4_q_reg)
        S = S + ds_rk
    # Function to integrate derivative and costs.
    F = ca.Function('F', [X0, U, S0], [X, Q_t, Q_reg], ['x0', 'u', 's0'], ['xf', 'q_t', 'q_reg'])

    # Stintegy: start from a line already solved on the same stations when
    # one is given -- another lane's, which is feasible here or nearly so,
    # its offset clipped into this lane's road. From 30 m/s down the middle
    # the long fast circuits sat at a point that broke a constraint by a
    # metre for the whole time budget; from a line that already holds,
    # the solve only has to make it faster.
    def guess_state(k):
        lo = track_handler.w_tr_right[k] + vehicle_params['total_width'] / 2.0 + safety_distance
        hi = track_handler.w_tr_left[k] - vehicle_params['total_width'] / 2.0 - safety_distance
        if init_line is None:
            return [V_guess, (track_handler.w_tr_left[k] + track_handler.w_tr_right[k]) / 2.0, 0.0, 1e-6, 1e-6]
        row = init_line.iloc[min(k, len(init_line) - 1)]
        margin = min(0.05, max(0.0, (hi - lo) / 4.0))
        return [float(row['v_opt']), float(np.clip(row['n_opt'], lo + margin, hi - margin)),
                float(row['chi_opt']), float(row['ax_opt']), float(row['ay_opt'])]

    def guess_control(k):
        if init_line is None:
            return [0.0] * nu
        row = init_line.iloc[min(k, len(init_line) - 1)]
        return [float(row['jx_opt']), float(row['jy_opt'])]

    # Empty NLP
    w = []
    w0 = []
    lbw = []
    ubw = []
    J_t = 0.0
    J_reg = 0.0
    g = []
    lbg = []
    ubg = []
    # Stintegy: what each constraint is and where, so a refused solve can say.
    g_tag = []

    # Initial conditions
    Xk = ca.MX.sym('X0', nx)
    w += [Xk]
    lbw += [0.0, track_handler.w_tr_right[0] + vehicle_params['total_width'] / 2.0 + safety_distance, -np.pi / 2.0, -np.inf, -np.inf]
    ubw += [gg_handler.V_max, track_handler.w_tr_left[0] - vehicle_params['total_width'] / 2.0 - safety_distance, np.pi / 2.0, np.inf, np.inf]
    w0 += guess_state(0)

    # Formulate the NLP
    for k in range(0, track_handler.s.size):
        # Apparent accelerations
        ax_tilde, ay_tilde, g_tilde = track_handler.calc_apparent_accelerations(
            V=Xk[0],
            n=Xk[1],
            chi=Xk[2],
            ax=Xk[3],
            ay=Xk[4],
            s=k * track_handler.ds,
            h=vehicle_params['h'],
            neglect_w_omega_y=neglect_w_omega_y,
            neglect_w_omega_x=neglect_w_omega_x,
            neglect_euler=neglect_euler,
            neglect_centrifugal=neglect_centrifugal,
            neglect_w_dot=neglect_w_dot,
            neglect_V_omega=neglect_V_omega
        )

        # Constraints of gggv-Diagram.
        if gg_mode == 'polar':
            alpha = ca.arctan2(ax_tilde, ay_tilde)
            rho = ca.sqrt(ax_tilde ** 2 + ay_tilde ** 2)
            adherence_radius = gg_handler.gggv_interpolator(ca.vertcat(Xk[0], g_tilde, alpha))
            g += [adherence_radius - rho]
            lbg += [0.0]
            ubg += [np.inf]
        elif gg_mode == 'diamond':
            gg_exponent, ax_min, ax_max, ay_max = ca.vertsplit(gg_handler.acc_interpolator(ca.vertcat(Xk[0], g_tilde)))  # positive

            if smooth_ellipse:
                # Stintegy: with the exponent at two the diamond's combined
                # limit is the ellipse (ax / ax_min)^2 + (ay / ay_max)^2 <= 1,
                # and written that way it is smooth everywhere. TUM's form,
                # |ax| <= |ax_min| (1 - (|ay| / ay_max)^e)^(1/e) under fabs,
                # fmin and fmax, has a kink at every edge and a slope that
                # runs to infinity as the corner fills the circle; at speed,
                # braking into a fast sweeper, IPOPT stepped back and forth
                # across it for the whole time budget (Silverstone at 1266 m,
                # 66 m/s). Scaled by |ax_min| so the violation is still in
                # metres per second squared.
                #
                # It stands alone: |ay| <= ay_max is already inside it, and
                # kept as a second constraint the two are active together
                # wherever the line rides the lateral limit (every fast
                # sweeper), with gradients exactly parallel there. That
                # breaks IPOPT's constraint qualification, the multipliers
                # have no unique value, and the steps swing the iterate in
                # and out of feasibility round an unchanging lap time.
                g += [ca.fabs(ax_min) * (1.0 - (ax_tilde / ax_min) ** 2 - (ay_tilde / ay_max) ** 2)]
                lbg += [0.0]
                ubg += [np.inf]
                g_tag += [(k, 'brake')]
            else:
                g += [ay_max - ca.fabs(ay_tilde)]
                lbg += [0.0]
                ubg += [np.inf]
                g_tag += [(k, 'ay')]

                g += [ca.fabs(ax_min) * ca.power(
                        ca.fmax(
                            (1.0 - ca.power(
                                ca.fmin(ca.fabs(ay_tilde) / ay_max, 1.0),
                                gg_exponent
                            ))
                            , 1e-3
                        ),
                        1.0 / gg_exponent
                    ) - ca.fabs(ax_tilde)
                ]
                lbg += [0.0]
                ubg += [np.inf]
                g_tag += [(k, 'brake')]

            g += [ax_max - ax_tilde]
            lbg += [0.0]
            ubg += [np.inf]
            g_tag += [(k, 'drive')]
        else:
            raise RuntimeError('Unknown gg_mode.')

        # If last iteration, don't add new control or states.
        if k == track_handler.s.size - 1:
            break

        # New NLP variable for the control.
        Uk = ca.MX.sym('U_' + str(k), nu)
        w += [Uk]
        lbw += [-np.inf] * nu
        ubw += [np.inf] * nu
        w0 += guess_control(k)

        # Integrate till the end of the interval.
        Fk = F(x0=Xk, u=Uk, s0=k * track_handler.ds)
        Xk_end = Fk['xf']
        J_t = J_t + Fk['q_t']
        J_reg = J_reg + Fk['q_reg']

        # New NLP variable for state at end of interval.
        Xk = ca.MX.sym('X_' + str(k + 1), nx)
        w += [Xk]
        lbw += [0.0, track_handler.w_tr_right[k+1] + vehicle_params['total_width'] / 2.0 + safety_distance, -np.pi / 2.0, -np.inf, -np.inf]
        ubw += [gg_handler.V_max, track_handler.w_tr_left[k+1] - vehicle_params['total_width'] / 2.0 - safety_distance, np.pi / 2.0, np.inf, np.inf]
        w0 += guess_state(k + 1)

        # Add equality constraint for continuity.
        g += [Xk_end - Xk]
        lbg += [0.0] * nx
        ubg += [0.0] * nx
        g_tag += [(k, c) for c in ('cont V', 'cont n', 'cont chi', 'cont ax', 'cont ay')]

    # Boundary constraint: start states = final states.
    g += [w[0] - Xk]
    lbg += [0.0] * nx
    ubg += [0.0] * nx

    # Concatenate NLP vectors.
    w = ca.vertcat(*w)
    g = ca.vertcat(*g)
    w0 = ca.vertcat(w0)
    lbw = ca.vertcat(lbw)
    ubw = ca.vertcat(ubw)
    lbg = ca.vertcat(lbg)
    ubg = ca.vertcat(ubg)

    # Create an NLP solver.
    nlp = {'f': J_t + J_reg, 'x': w, 'g': g}
    solver = ca.nlpsol('solver', 'ipopt', nlp, sol_opt)
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    stats = solver.stats()
    if not stats.get('success', False):
        # Stopped by its iteration or time budget rather than converged. A
        # line whose every bound and constraint holds to within a hair is a
        # line, even if the last digits of optimality were never reached:
        # diagnosed on Daytona's right lane and Monaco, where the lap time had
        # stopped moving and the solver was circling a nearly feasible point.
        # Keep it when the worst violation is small; refuse it otherwise.
        x = np.array(sol['x']).flatten()
        g_val = np.array(sol['g']).flatten()
        lbx_a, ubx_a = np.array(lbw).flatten(), np.array(ubw).flatten()
        lbg_a, ubg_a = np.array(lbg).flatten(), np.array(ubg).flatten()
        worst = max(
            float(np.max(np.maximum(lbx_a - x, 0.0) + np.maximum(x - ubx_a, 0.0), initial=0.0)),
            float(np.max(np.maximum(lbg_a - g_val, 0.0) + np.maximum(g_val - ubg_a, 0.0), initial=0.0)),
        )
        status = stats.get('return_status')
        viol = np.maximum(lbg_a - g_val, 0.0) + np.maximum(g_val - ubg_a, 0.0)
        tags = g_tag + [('end', f'periodic {i}') for i in range(len(viol) - len(g_tag))]
        worst_rows = np.argsort(viol)[::-1][:6]
        print("Worst constraints: " + "; ".join(
            f"{tags[i][1]} at s={tags[i][0] * track_handler.ds if isinstance(tags[i][0], int) else 'end'} "
            f"by {viol[i]:.3g} (V {x[(nx + nu) * tags[i][0]] if isinstance(tags[i][0], int) else 0:.1f}, "
            f"n {x[(nx + nu) * tags[i][0] + 1] if isinstance(tags[i][0], int) else 0:.2f})" for i in worst_rows))
        xviol = np.maximum(lbx_a - x, 0.0) + np.maximum(x - ubx_a, 0.0)
        if xviol.size and xviol.max() > 0:
            i = int(np.argmax(xviol))
            print(f"Worst bound: variable {i % (nx + nu)} at s={(i // (nx + nu)) * track_handler.ds} by {xviol[i]:.3g}")
        if worst > ACCEPT_UNCONVERGED_VIOLATION:
            raise RuntimeError(
                f"IPOPT failed to bake a racing line: {status} (worst violation {worst:.3g})"
            )
        print(f"WARNING: {status}; keeping the last iterate, worst violation {worst:.3g}")

    # Extract lap time.
    laptime = ca.Function('f_laptime', [w], [J_t])
    print(f'Laptime: {float(laptime(sol["x"]))}')
    # Extract regularization costs.
    regularization = ca.Function('f_regularization', [w], [J_reg])

    # Extract solution.
    w_opt = sol['x'].full().flatten()
    s_opt = track_handler.s
    v_opt = np.array(w_opt[0::nx + nu])
    n_opt = np.array(w_opt[1::nx + nu])
    chi_opt = np.array(w_opt[2::nx + nu])
    ax_opt = np.array(w_opt[3::nx + nu])
    ay_opt = np.array(w_opt[4::nx + nu])
    jx_opt = np.array(w_opt[5::nx + nu])
    jy_opt = np.array(w_opt[6::nx + nu])

    trajectory_data_frame = pd.DataFrame()
    trajectory_data_frame['s_opt'] = s_opt
    trajectory_data_frame['v_opt'] = v_opt
    trajectory_data_frame['n_opt'] = n_opt
    trajectory_data_frame['chi_opt'] = chi_opt
    trajectory_data_frame['ax_opt'] = ax_opt
    trajectory_data_frame['ay_opt'] = ay_opt
    trajectory_data_frame['jx_opt'] = np.concatenate([jx_opt, [0]])
    trajectory_data_frame['jy_opt'] = np.concatenate([jy_opt, [0]])
    trajectory_data_frame['laptime'] = float(laptime(sol["x"]))

    # Save solution.
    if out_path:
        trajectory_data_frame.to_csv(path_or_buf=out_path, sep=',', index=True, float_format='%.6f')

    return trajectory_data_frame

if __name__ == '__main__':
    os.makedirs(os.path.join(raceline_out_path), exist_ok=True)
    raceline = calc_global_raceline(
            track_name=params['track_name'],
            vehicle_params=params['vehicle_params'],
            gg_mode=params['gg_mode'],
            gg_margin=params['gg_margin'],
            safety_distance=params['safety_distance'],
            w_T=params['w_T'],
            w_jx=params['w_jx'],
            w_jy=params['w_jy'],
            w_dOmega_z=params['w_dOmega_z'],
            RK4_steps=params['RK4_steps'],
            V_guess=params['V_guess'],
            neglect_w_omega_x=params['neglect_w_omega_x'],
            neglect_w_omega_y=params['neglect_w_omega_y'],
            neglect_euler=params['neglect_euler'],
            neglect_centrifugal=params['neglect_centrifugal'],
            neglect_w_dot=params['neglect_w_dot'],
            neglect_V_omega=params['neglect_V_omega'],
            sol_opt=params['sol_opts'],
            out_path=os.path.join(raceline_out_path, params['raceline_name']),
    )
