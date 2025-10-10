# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "control==0.10.2",
#     "imageio==2.37.0",
#     "ipython==9.4.0",
#     "kaleido==1.0.0",
#     "marimo",
#     "numba==0.61.2",
#     "numpy==2.2.6",
#     "plotly==6.2.0",
#     "scipy==1.16.0",
#     "sympy==1.14.0",
# ]
# ///

import marimo

__generated_with = "0.16.3"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo
    return (mo,)


@app.cell
def _():
    import numpy as np
    return (np,)


@app.cell
def _():
    import math
    import sympy as sp
    from sympy.physics.vector import dynamicsymbols, kinematic_equations
    from sympy.physics.vector.printing import vlatex

    from sympy import sin, cos, diff, Matrix, symbols, simplify, init_printing, Eq

    from IPython.display import Math, display

    init_printing()

    def dotprint(expr):
        display(Math(vlatex(expr)))
    return Matrix, cos, dotprint, dynamicsymbols, simplify, sin, sp, symbols


@app.cell
def _(symbols):
    t, mass, radius, friction, rot_friction, K_thrust, d_thrust = symbols('t, m, r, F_f, F_fr, K_trust, d_thrust')
    moment_of_inertia = 1/2 * mass * radius ** 2

    symbol_values = {
        mass: 0.5, # 200 grams
        radius: 0.1, # 10 cm
        friction: 0.01, # TODO
        rot_friction: 0.001, # TODO
        K_thrust: 0.1, # TODO
        d_thrust: 0.05, # TODO distance of thrusters from center
    }
    return (
        K_thrust,
        d_thrust,
        friction,
        mass,
        moment_of_inertia,
        radius,
        rot_friction,
        symbol_values,
    )


@app.cell
def _(mo):
    mo.md(r"""## EOMS""")
    return


@app.cell
def _(Matrix, dynamicsymbols):
    p_x, p_y, theta = dynamicsymbols('p_x, p_y, theta')

    p_x_dot = p_x.diff('t')
    p_x_ddot = p_x_dot.diff('t')

    p_y_dot = p_y.diff('t')
    p_y_ddot = p_y_dot.diff('t')

    position = Matrix([[p_x], [p_y]])
    velocity = position.diff('t')
    acceleration = velocity.diff('t')

    theta_dot = theta.diff('t')
    theta_ddot = theta_dot.diff('t')
    return (
        acceleration,
        p_x,
        p_x_ddot,
        p_x_dot,
        p_y,
        p_y_ddot,
        p_y_dot,
        theta,
        theta_ddot,
        theta_dot,
        velocity,
    )


@app.cell
def _(K_thrust, Matrix, cos, d_thrust, sin, symbols, theta):
    u_l, u_r = symbols('mu_L, mu_R')
    force_from_thrusters = (u_l + u_r) * K_thrust * Matrix([[cos(theta)],[sin(theta)]])
    torque_from_thrusters = (u_r - u_l) * K_thrust * d_thrust
    return force_from_thrusters, torque_from_thrusters, u_l, u_r


@app.cell
def _(acceleration, dotprint, force_from_thrusters, friction, mass, velocity):
    eom_position = (mass * acceleration) - (force_from_thrusters -friction * velocity)
    dotprint(eom_position)
    return (eom_position,)


@app.cell
def _(
    dotprint,
    moment_of_inertia,
    rot_friction,
    theta_ddot,
    theta_dot,
    torque_from_thrusters,
):
    eom_rotation = (moment_of_inertia * theta_ddot) - (torque_from_thrusters - rot_friction * theta_dot)
    dotprint(eom_rotation)
    return (eom_rotation,)


@app.cell
def _(Matrix, dotprint, eom_position, eom_rotation, simplify):
    eoms = simplify(Matrix([eom_position, eom_rotation]))
    dotprint(eoms)
    return (eoms,)


@app.cell
def _(dotprint, eoms, p_x_ddot, p_y_ddot, sp, symbol_values, theta_ddot):
    full_eoms = sp.solve(eoms.subs(symbol_values), (p_x_ddot, p_y_ddot, theta_ddot), simplify=True)
    dotprint(full_eoms)
    return (full_eoms,)


@app.cell
def _(mo):
    mo.md(r"""## Optimizer""")
    return


@app.cell
def _(
    full_eoms,
    np,
    p_x,
    p_x_ddot,
    p_x_dot,
    p_y,
    p_y_ddot,
    p_y_dot,
    sp,
    theta,
    theta_ddot,
    theta_dot,
    u_l,
    u_r,
):
    # Suppose you already defined these symbols:
    x_syms = (p_x, p_y, theta, p_x_dot, p_y_dot, theta_dot)
    u_syms = (u_l, u_r)

    # Replace these with your actual equations from full_eoms
    px_dd_expr = full_eoms[p_x_ddot]
    py_dd_expr = full_eoms[p_y_ddot]
    t_dd_expr = full_eoms[theta_ddot]

    # Create lambdified functions
    f_px_dd = sp.lambdify((x_syms, u_syms), px_dd_expr, 'numpy')
    f_py_dd = sp.lambdify((x_syms, u_syms), py_dd_expr, 'numpy')
    f_t_dd = sp.lambdify((x_syms, u_syms), t_dd_expr, 'numpy')

    def f_xdot(x, u):
        px_dd = f_px_dd(x, u)
        py_dd = f_py_dd(x, u)
        t_dd = f_t_dd(x, u)

        return np.array([x[3], x[4], x[5], px_dd, py_dd, t_dd])

    # f_xdot((0,0,0,0,0,0),(0,0))
    return (f_xdot,)


@app.function
# RK4
def rk4_step(f, x, u, dt):
    """Performs a single Runge-Kutta 4th order step."""
    k1 = f(x, u)
    k2 = f(x + dt / 2 * k1, u)
    k3 = f(x + dt / 2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


@app.cell
def _(f_xdot, np):
    from scipy.integrate import solve_ivp

    def simulate_system(t1, t2, t3, t4):
        dt = 0.01
        steps = 1200
        """Simulate the system with timed thruster input."""
        x = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        trajectory = np.zeros((steps + 1, 6), dtype=np.float64)
        us = np.zeros((steps + 1, 2), dtype=np.float64)

        c1 = t1
        c2 = c1 + t1
        c3 = c2 + t2
        c4 = c3 + t3
        c5 = c4 + t3
        c6 = c5 + t4

        bp = 0.5

        # Precompute control input schedule
        for i in range(steps):
            t = i * dt
            if t < c1:
                us[i] = [1, 0]
            elif t < c2:
                us[i] = [0, 1]
            elif t < c3:
                us[i] = [bp, bp]
            elif t < c4:
                us[i] = [0, 1]
            elif t < c5:
                us[i] = [1, 0]
            elif t < c6:
                us[i] = [bp, bp]

        # Initial state
        trajectory[0] = x

        # Integrate using RK4
        for i in range(steps):
            x = rk4_step(f_xdot, x, us[i], dt)
            trajectory[i + 1] = x

        # Return trajectory and control inputs
        return trajectory, us

    def simulate_system(t1, t2, t3, t4, x0 = np.zeros(6, dtype=np.float64), time_offset = 0):
        dt = 0.01
        steps = 1200 - int(time_offset / dt)
        t_span = (0, steps * dt)
        times = np.linspace(*t_span, steps + 1)

        # Time control intervals
        c1 = t1
        c2 = c1 + t1
        c3 = c2 + t2
        c4 = c3 + t3
        c5 = c4 + t3
        c6 = c5 + t4
        bp = 0.5

        v1 = np.array([1, 0])
        v2 = np.array([0, 1])
        v3 = np.array([bp, bp])
        v4 = np.array([0, 0])

        # Define time-dependent control input function
        def u_func(t):
            if t < c1:
                return v1
            elif t < c2:
                return v2
            elif t < c3:
                return v3
            elif t < c4:
                return v2
            elif t < c5:
                return v1
            elif t < c6:
                return v3
            else:
                return v4

        # Wrapper for solve_ivp to include control logic
        def dynamics(t, x):
            return f_xdot(x, u_func(t))


        # Integrate using scipy's solve_ivp with RK45
        sol = solve_ivp(dynamics, t_span, x0, t_eval=times, method='RK45', vectorized=False)

        # Collect control inputs at sampled times
        us = np.array([u_func(t) for t in sol.t])

        return sol.y.T, us

    # Example usage
    # trajectory, us = simulate_system(1.25, 2.0, 1.40, 2.1)
    # _ = simulate_system(1.25, 0, 0, 0)
    # trajectory
    return simulate_system, solve_ivp


@app.cell
def _(score_system, x0):
    score_system(x0)
    return


@app.cell
def _(simulate_system):
    _ = simulate_system(1.25, 1, 1, 1)
    return


@app.cell
def _(minimize, np, simulate_system):
    class Controller:
        def __init__(self, target, reference_ts):
            self.target = target

            self.t1 = reference_ts[0]
            self.t2 = reference_ts[1]
            self.t3 = reference_ts[2]
            self.t4 = reference_ts[3]

            self.max_t = 0.1

        def reoptimize(self, x, t):
            print(f"time is {t}")
            target = np.array([self.target[0], self.target[1], 0, 0, 0, 0], dtype=np.float64)
            weight = np.array([1,1,0,10,10,10], dtype=np.float64)
            def score_system(ts):
                t1, t2, t3, t4 = ts
                trajectory, us = simulate_system(t1, t2, t3, t4, x0 = x, time_offset = t)
                final_pos = trajectory[-1]
                final_offset = (final_pos - target)
                final_offset *= weight
                return np.inner(final_offset, final_offset)
            # Constraints
            constraints = [
                # {'type': 'ineq', 'fun': lambda t: 10 - sum(t)},      # t1 + t2 + t3 + t4 <= 10
                # {'type': 'ineq', 'fun': lambda t: t[0]},             # t1 >= 0
                # {'type': 'ineq', 'fun': lambda t: t[1]},             # t2 >= 0
                # {'type': 'ineq', 'fun': lambda t: t[2]},             # t3 >= 0
                # {'type': 'ineq', 'fun': lambda t: t[3]}              # t4 >= 0
            ]

            # x0 = [1.25, 2.0, 1.4, 2.1]
            # x0 = [1.5,1,1,2]
            t0 = np.array([self.t1 - t, self.t2, self.t3, self.t4])

            print(f"x {x}")
            print(f"orignal t {t0}")
            print(f"orignal score {score_system(t0)}")

            result = minimize(score_system, t0, method='COBYLA', constraints=constraints)
            if (result.success):
                print(f"post t {result.x}")
                print(f"post score {score_system(result.x)}")
                # self.t1 = result.x[0] + t
                # self.t2 = result.x[1]
                # self.t3 = result.x[2]
                # self.t4 = result.x[3]

        def __call__(self, x, t):
            if (t > self.max_t):
                if (int(t) > int(self.max_t)) and t < 12:
                # if t < 12:
                # if t < 1.5:
                    self.reoptimize(x, t)
                self.max_t = t

            c1 = self.t1
            c2 = c1 + self.t1
            c3 = c2 + self.t2
            c4 = c3 + self.t3
            c5 = c4 + self.t3
            c6 = c5 + self.t4
            bp = 0.5

            v1 = np.array([1, 0])
            v2 = np.array([0, 1])
            v3 = np.array([bp, bp])
            v4 = np.array([0, 0])

            if t < c1:
                return v1
            elif t < c2:
                return v2
            elif t < c3:
                return v3
            elif t < c4:
                return v2
            elif t < c5:
                return v1
            elif t < c6:
                return v3
            else:
                return v4
    return (Controller,)


@app.cell
def _(f_xdot, np, solve_ivp):
    def simulate_real_system(controller):
        dt = 0.01
        steps = 1200
        t_span = (0, steps * dt)
        times = np.linspace(*t_span, steps + 1)

        # Wrapper for solve_ivp to include control logic
        def dynamics(t, x):
            # return f_xdot(x, controller(x, t)) + np.array([0.05, 0, 0, 0, 0, 0])
            return f_xdot(x, controller(x, t))

        # Initial state
        x0 = np.zeros(6, dtype=np.float64)

        # Integrate using scipy's solve_ivp with RK45
        sol = solve_ivp(dynamics, t_span, x0, t_eval=times, method='RK45', vectorized=False)

        # Collect control inputs at sampled times
        # us = np.array([controller(x, t) for x, t in zip(sol.y.T, sol.t)])
        us = np.array([[0,0] for x, t in zip(sol.y.T, sol.t)])

        return sol.y.T, us
    return (simulate_real_system,)


@app.cell
def _():
    from scipy.optimize import minimize
    return (minimize,)


@app.cell(hide_code=True)
def _(np, simulate_system):
    target = np.array([0, -2, 0, 0, 0, 0], dtype=np.float64)
    weight = np.array([1,1,0,10,10,10], dtype=np.float64)

    def score_system(ts):
        t1, t2, t3, t4 = ts
        trajectory, us = simulate_system(t1, t2, t3, t4)
        final_pos = trajectory[-1]
        final_offset = (final_pos - target)
        final_offset *= weight
        return np.inner(final_offset, final_offset)

    # from scipy.optimize import minimize

    # # Constraints
    # # t1 + t2 + t3 + t4 <= 10
    # constraint = {
    #     'type': 'ineq',  # means: function(t) >= 0
    #     'fun': lambda t: 10 - sum(t)
    # }

    # # Bounds: each t >= 0
    # bounds = [(0, None)] * 4

    # # Initial guess
    # # x0 = [1, 1, 1, 1]
    # x0 = [1.25, 2.0, 1.4, 2.1]

    # # Run optimization
    # result = minimize(score_system, x0, method='SLSQP', bounds=bounds, constraints=[constraint])

    #---------------------------------------------------------------------------------------------------


    # # Constraints
    # constraints = [
    #     {'type': 'ineq', 'fun': lambda t: 10 - sum(t)},      # t1 + t2 + t3 + t4 <= 10
    #     {'type': 'ineq', 'fun': lambda t: t[0]},             # t1 >= 0
    #     {'type': 'ineq', 'fun': lambda t: t[1]},             # t2 >= 0
    #     {'type': 'ineq', 'fun': lambda t: t[2]},             # t3 >= 0
    #     {'type': 'ineq', 'fun': lambda t: t[3]}              # t4 >= 0
    # ]

    # # x0 = [1.25, 2.0, 1.4, 2.1]
    # # x0 = [1.5,1,1,2]
    # x0 = [1.61015073 + 0.01, 1.01817904 + 0.01, 0.99975761 + 0.01, 1.70355217 + 0.01]


    # result = minimize(score_system, x0, method='COBYLA', constraints=constraints)

    #---------------------------------------------------------------------------------------------------

    from scipy.optimize import differential_evolution

    def score_system(ts):
        if sum(ts) > 10:
            return 1e6  # Penalty for constraint violation
        t1, t2, t3, t4 = ts
        _trajectory, _ = simulate_system(t1, t2, t3, t4)
        final_pos = _trajectory[-1]
        final_offset = (final_pos - target)
        final_offset *= weight
        return np.inner(final_offset, final_offset)

    bounds = [(0, 10)] * 4  # Reasonable upper bound since sum must be ≤ 10

    result = differential_evolution(score_system, bounds)

    #---------------------------------------------------------------------------------------------------

    if result.success:
        print("Optimal t values:", result.x)
        print("Final score:", result.fun)
    else:
        print("Optimization failed:", result.message)
    return (score_system,)


@app.cell
def _(Controller, simulate_real_system):
    # Optimal t values: [1.15435548 0.60977315 1.46371457 1.69800872]
    # Final score: 0.0003984068848688111
    # 0 -2 with bp = 0.5

    # Optimal t values: [0.36021131 1.07647029 1.35581481 1.81170497]
    # Final score: 0.003275978508126181
    # 2 0 with bp = 0.5

    # trajectory, us = simulate_system(result.x[0], result.x[1], result.x[2], result.x[3])
    # score_system(result.x)

    #---------------------------------------------------------------------------------------------------
    controller = Controller((0, -2), [1.59411804, 0.99583488, 1.03078605, 1.66871616] )
    trajectory, us = simulate_real_system(controller)
    # score_system(result.x)
    #---------------------------------------------------------------------------------------------------

    # _ts = [1.25, 2.0, 1.40, 2.1]
    # _ts = [1.15435548, 0.60977315, 1.46371457, 1.69800872] # (0,-2)
    # _ts = [0.36021131, 1.07647029, 1.35581481, 1.81170497] # (2,0)
    # _ts = [0, 1, 0, 0]
    # _ts = [10, 0, 0, 0]
    # trajectory, us = simulate_system(_ts[0], _ts[1], _ts[2], _ts[3])
    # score_system(_ts)
    return trajectory, us


@app.cell
def _(trajectory):
    trajectory[11]
    return


@app.cell
def _(mo):
    mo.md(r"""## Plots""")
    return


@app.cell(hide_code=True)
def _(trajectory, us):
    def _():
        import plotly.graph_objects as go
        import numpy as np

        # Time vector
        dt = 0.01
        time = np.arange(trajectory.shape[0]) * dt

        # Variable names for labeling
        state_labels = ['p_x', 'p_y', 'theta', 'p_x_dot', 'p_y_dot', 'theta_dot']

        # Create figure
        fig = go.Figure()

        # Add each state variable as a line
        for i in range(trajectory.shape[1]):
            fig.add_trace(go.Scatter(
                x=time,
                y=trajectory[:, i],
                mode='lines',
                name=state_labels[i]
            ))

        fig.add_trace(go.Scatter(
            x=time,
            y=us[:, 0],
            mode='lines',
            name='u_l'
        ))
        fig.add_trace(go.Scatter(
            x=time,
            y=us[:, 1],
            mode='lines',
            name='u_r'
        ))

        # Update layout
        fig.update_layout(
            title='State Variables Over Time',
            xaxis_title='Time (s)',
            yaxis_title='Value',
            legend_title='State Variable',
            template='plotly_white',
            width=800,
            height=500
        )
        return fig


    _()
    return


@app.cell
def _(np, trajectory):
    # Plotting the trajectory
    # using plotly

    import plotly.graph_objects as go

    """Plots the trajectory of the system."""
    x_vals = [state[0] for state in trajectory]
    y_vals = [state[1] for state in trajectory]
    theta_vals = [state[2] for state in trajectory]

    fig = go.Figure()

    # Plot trajectory
    fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode='lines', name='Trajectory'))

    # Plot orientation as arrows
    for i in range(0, len(trajectory), 10):  # Plot every 10th point for clarity
        fig.add_annotation(
            x=x_vals[i],
            y=y_vals[i],
            ax=x_vals[i] + 0.1 * np.cos(theta_vals[i]),
            ay=y_vals[i] + 0.1 * np.sin(theta_vals[i]),
            showarrow=True,
            arrowhead=2,
            arrowsize=1,
            arrowwidth=1,
            axref='x',
            ayref='y'
        )

    fig.update_layout(title='Trajectory of the System', xaxis_title='X Position', yaxis_title='Y Position')

    fig
    return (go,)


@app.cell(hide_code=True)
def _(d_thrust, go, np, radius, symbol_values, trajectory, us):
    def _():
        # Simulate the system
        px = trajectory[:, 0]
        py = trajectory[:, 1]
        theta = trajectory[:, 2]

        # Parameters for drawing the box
        box_length = symbol_values[radius]
        box_width = symbol_values[radius]

        def get_box_corners(x, y, theta, L=box_length, W=box_width):
            # Box corners in local frame
            corners = np.array([
                [ L/2,  W/2],
                [ L/2, -W/2],
                [-L/2, -W/2],
                [-L/2,  W/2],
                [ L/2,  W/2]  # close the loop
            ])

            # Rotation matrix
            R = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)]
            ])

            # Rotate and translate
            rotated = corners @ R.T
            translated = rotated + np.array([x, y])
            return np.array(translated)  # ✅ force it to be a NumPy array

        # Create frames for animation
        frames = []
        for i in range(0, len(px), 5):  # Skip frames for speed
            # Box corners
            corners = get_box_corners(px[i], py[i], theta[i])

            # Base box
            box_trace = go.Scatter(
                x=corners[:, 0], y=corners[:, 1],
                mode='lines', line=dict(color='blue', width=2),
                showlegend=False
            )

            # Trajectory path
            path_trace = go.Scatter(
                x=px[:i+1], y=py[:i+1],
                mode='lines', line=dict(color='gray'),
                name='Path'
            )

            # Thruster lines
            u_l = us[i, 0]
            u_r = us[i, 1]
            scale = 0.1  # adjust for appearance

            # Compute thrust vector positions (relative to box center)
            # offset_y = -box_width / 2
            offset_y = -symbol_values[d_thrust]
            thrust_dir = -np.array([np.cos(theta[i]), np.sin(theta[i])])
            thrust_perp = np.array([-np.sin(theta[i]), np.cos(theta[i])])

            # Left thruster (at left rear of box)
            left_base = np.array([px[i], py[i]]) - offset_y * thrust_perp
            left_tip = left_base + scale * u_l * thrust_dir

            left_thruster = go.Scatter(
                x=[left_base[0], left_tip[0]],
                y=[left_base[1], left_tip[1]],
                mode='lines',
                line=dict(color='red', width=3),
                showlegend=False
            )

            # Right thruster
            right_base = np.array([px[i], py[i]]) + offset_y * thrust_perp
            right_tip = right_base + scale * u_r * thrust_dir

            right_thruster = go.Scatter(
                x=[right_base[0], right_tip[0]],
                y=[right_base[1], right_tip[1]],
                mode='lines',
                line=dict(color='green', width=3),
                showlegend=False
            )

            # Add frame
            frames.append(go.Frame(
                data=[box_trace, path_trace, left_thruster, right_thruster],
                name=str(i)
            ))

        # Initial corners and thrusts
        initial_corners = get_box_corners(px[0], py[0], theta[0])
        init_box = go.Scatter(
            x=initial_corners[:, 0], y=initial_corners[:, 1],
            mode='lines', line=dict(color='blue', width=2),
            showlegend=False
        )

        init_path = go.Scatter(
            x=[px[0]], y=[py[0]],
            mode='lines', line=dict(color='gray'),
            name='Path'
        )

        # Init thruster lines
        u_l0 = us[0, 0]
        u_r0 = us[0, 1]
        scale = 0.1
        offset_y = box_width / 2
        thrust_dir = np.array([np.cos(theta[0]), np.sin(theta[0])])
        thrust_perp = np.array([-np.sin(theta[0]), np.cos(theta[0])])

        left_base = np.array([px[0], py[0]]) - offset_y * thrust_perp
        left_tip = left_base + scale * u_l0 * thrust_dir
        init_left_thruster = go.Scatter(
            x=[left_base[0], left_tip[0]],
            y=[left_base[1], left_tip[1]],
            mode='lines',
            line=dict(color='red', width=3),
            showlegend=False
        )

        right_base = np.array([px[0], py[0]]) + offset_y * thrust_perp
        right_tip = right_base + scale * u_r0 * thrust_dir
        init_right_thruster = go.Scatter(
            x=[right_base[0], right_tip[0]],
            y=[right_base[1], right_tip[1]],
            mode='lines',
            line=dict(color='green', width=3),
            showlegend=False
        )

        # Layout with animation controls
        layout = go.Layout(
            title="Animated Box Trajectory",
            xaxis=dict(scaleanchor="y", title='p_x'),
            yaxis=dict(title='p_y'),
            width=600,
            height=600,
            updatemenus=[dict(
                type='buttons',
                showactive=False,
                buttons=[dict(label='Play',
                              method='animate',
                              args=[None, {"frame": {"duration": 30, "redraw": True},
                                           "fromcurrent": True}]),
                         dict(label='Pause',
                              method='animate',
                              args=[[None], {"frame": {"duration": 0, "redraw": False},
                                             "mode": "immediate",
                                             "transition": {"duration": 0}}])]
            )]
        )

        # Full initial data
        fig = go.Figure(
            data=[init_box, init_path, init_left_thruster, init_right_thruster],
            layout=layout,
            frames=frames
        )
        return fig

    _()
    return


if __name__ == "__main__":
    app.run()
