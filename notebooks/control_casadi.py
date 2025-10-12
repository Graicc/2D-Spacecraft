# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "casadi==3.7.2",
#     "kaleido==1.1.0",
#     "marimo",
#     "numpy==2.3.2",
#     "plotly==6.3.0",
# ]
# ///

import marimo

__generated_with = "0.16.5"
app = marimo.App(width="medium")


@app.cell
def _():
    import casadi as ca
    from casadi import MX, Function
    import numpy as np
    return Function, MX, ca, np


@app.cell
def _():
    import plotly.graph_objects as go
    return (go,)


@app.cell
def _(MX, ca):
    px = MX.sym('px')
    px_d = MX.sym('px_d')
    py = MX.sym('py')
    py_d = MX.sym('py_d')
    theta = MX.sym('theta')
    theta_d = MX.sym('theta_d')

    x = ca.vertcat(px,py,theta, px_d,py_d,theta_d)

    ul = MX.sym('ul')
    ur = MX.sym('ur')

    u = ca.vertcat(ul, ur)

    x,u
    return px_d, py_d, theta, theta_d, u, ul, ur, x


@app.cell
def _(ca, px_d, py_d, theta, theta_d, ul, ur):
    # let px_ddot = 0.2 * u[0] * cos_theta + 0.2 * u[1] * cos_theta - 0.02 * v_px;
    # let py_ddot = 0.2 * u[0] * sin_theta + 0.2 * u[1] * sin_theta - 0.02 * v_py;
    # let theta_ddot = -2.0 * u[0] + 2.0 * u[1] - 0.4 * theta_dot;

    st = ca.sin(theta)
    ct = ca.cos(theta)
    px_dd = 0.2 * ul * ct + 0.2 * ur * ct - 0.02 * px_d
    py_dd = 0.2 * ul * st + 0.2 * ur * st - 0.02 * py_d
    theta_dd = -2.0 * ul + 2.0 * ur - 0.4 * theta_d

    ode = ca.vertcat(
        px_d,py_d,theta_d,
        px_dd, py_dd, theta_dd
    )

    ode
    return (ode,)


@app.cell
def _(ca, ode, u, x):
    T = 12 # Time horizon
    N = T * 10 # Number of control intervals
    # N = 120 # Number of control intervals

    intg_options = {
        "tf": T/N,
        "simplify": True,
        "number_of_finite_elements": 4,
    }

    dae = {
        "x": x,
        "p": u,
        "ode": ode,
    }

    # Single step of the system
    intg = ca.integrator('intg', 'rk', dae, intg_options)
    print(intg)
    return N, T, intg


@app.cell
def _(intg, u, x):
    x_next = intg(x0=x, p=u)['xf']
    print(x_next)
    return (x_next,)


@app.cell
def _(Function, u, x, x_next):
    # Simplify api
    F = Function('F', [x,u], [x_next])

    F([0,0,0,0,0,0],[1,0])
    return (F,)


@app.cell
def _(F, N):
    # Turn the single step into the whole time horizon
    sim = F.mapaccum(N)

    print(sim)
    return (sim,)


@app.cell
def _(go, np):
    def plot_trajectory(xs, ys, ts):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=xs, y=ys, mode='lines'))

        for i in range(0, len(ts), 10):
            fig.add_annotation(
                x=xs[i],
                y=ys[i],
                ax = xs[i] + 0.1 * np.cos(ts[i]),
                ay = ys[i] + 0.1 * np.sin(ts[i]),
                axref='x',
                ayref='y'
            )

        return fig
    return (plot_trajectory,)


@app.cell
def _(go, np):
    def plot_trajectories(fig, xs, ys, ts):
        fig.add_trace(go.Scatter(x=xs, y=ys, mode='lines'))

        for i in range(0, len(ts), 10):
            fig.add_annotation(
                x=xs[i],
                y=ys[i],
                ax = xs[i] + 0.1 * np.cos(ts[i]),
                ay = ys[i] + 0.1 * np.sin(ts[i]),
                axref='x',
                ayref='y'
            )
    return (plot_trajectories,)


@app.cell
def _(N, T, ca, go, np, plot_trajectory, sim):
    # Example sim

    tgrid = np.linspace(0, T, N+1)
    def _():
        x0 = [0,0,0,0,0,0]

        # us = ca.repmat(ca.vertcat(1,1), 1, 20)
        # us = ca.vcat(map(lambda x: old_to_l_r(x * T/N, 1.59411804, 0.99583488, 1.03078605, 1.66871616), range(N)))
        us = ca.hcat(list(map(lambda x: np.array(old_to_l_r(x * T/N, 1.59411804, 0.99583488, 1.03078605, 1.66871616)), range(N))))

        res = np.array(sim(x0, us))

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=tgrid, y=res[0,:], mode='lines', name='px'))
        fig.add_trace(go.Scatter(x=tgrid, y=res[1,:], mode='lines', name='py'))
        fig.add_trace(go.Scatter(x=tgrid, y=res[2,:], mode='lines', name='theta'))

        return plot_trajectory(res[0,:], res[1,:], res[2,:])
    _()
    return (tgrid,)


@app.cell
def _(F, N, ca):
    # Optimal control problem

    opti = ca.Opti()

    o_x = opti.variable(6, N+1)
    o_u = opti.variable(2, N)
    o_init = opti.parameter(6,1) # First measurement
    o_final = opti.parameter(2,1) # Last measurement

    minimize_control = 3
    if minimize_control == 1:
        opti.minimize(ca.sumsqr(o_u)) # Minimize control effort
        opti.subject_to(o_x[0,-1] == o_final[0])
        opti.subject_to(o_x[1,-1] == o_final[1])

        opti.subject_to(o_x[3,-1] == 0)
        opti.subject_to(o_x[4,-1] == 0)
    elif minimize_control == 2:
        _final = o_x[:,-1]
        _err_x = ca.dot((_final[0] - o_final[0]),(_final[0] - o_final[0]))
        _err_y = ca.dot((_final[1] - o_final[1]),(_final[1] - o_final[1]))
    
        # opti.subject_to(o_x[0,-1] == o_final[0])
        # opti.subject_to(o_x[1,-1] == o_final[1])

        # opti.subject_to(o_x[3,-1] == 0)
        # opti.subject_to(o_x[4,-1] == 0)
        # opti.subject_to(o_x[5,-1] == 0)
        opti.minimize(_err_x + _err_y + _final[3] ** 2 + _final[4] ** 2 + _final[5] ** 2)
        # opti.minimize(_final[3] ** 2 + _final[4] ** 2 + _final[5] ** 2)
    elif minimize_control == 3:
        _final = o_x[:,-1]
        _err_x = ca.dot((_final[0] - o_final[0]),(_final[0] - o_final[0]))
        _err_y = ca.dot((_final[1] - o_final[1]),(_final[1] - o_final[1]))
    
        opti.subject_to(o_x[0,-1] == o_final[0])
        opti.subject_to(o_x[1,-1] == o_final[1])

        # opti.subject_to(o_x[3,-1] == 0)
        # opti.subject_to(o_x[4,-1] == 0)
        # opti.subject_to(o_x[5,-1] == 0)
        # opti.minimize(_err_x + _err_y + _final[3] ** 2 + _final[4] ** 2 + _final[5] ** 2)
        opti.minimize(_final[3] ** 2 + _final[4] ** 2 + _final[5] ** 2)

    def lerp(a, b, t):
        return a + t * (b-a)

    # Physics constraint
    for k in range(0,N):
        opti.subject_to(o_x[:,k+1] == F(o_x[:,k], o_u[:,k]))

    opti.subject_to(o_u[0,:] >= 0)
    opti.subject_to(o_u[1,:] >= 0)
    # opti.subject_to(o_u[0,:] >= -1)
    # opti.subject_to(o_u[1,:] >= -1)
    # opti.subject_to(o_u[0,:] >= -0.5)
    # opti.subject_to(o_u[1,:] >= -0.5)

    opti.subject_to(o_u[0,:] <= 1)
    opti.subject_to(o_u[1,:] <= 1)

    opti.subject_to(o_x[:,1] == o_init)

    opti
    return o_final, o_init, o_u, o_x, opti


@app.function
def old_to_l_r(t, t1, t2, t3, t4) -> (float, float):
    c1 = t1
    c2 = c1 + t1
    c3 = c2 + t2
    c4 = c3 + t3
    c5 = c4 + t3
    c6 = c5 + t4
    bp = 0.5

    v1 = (1.0, 0.0)
    v2 = (0.0, 1.0)
    v3 = (bp, bp)
    v4 = (0.0, 0.0)

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


@app.cell
def _(o_final, o_init, o_u, o_x, opti):
    opti_opts = {
        # "qpsol": "osqp",
        # "qpsol": "qpoases",
        "qpsol": "qrqp",
        # "qpsol": "proxqp",
        "print_header": False,
        "print_iteration": False,
        "print_time": False,
        "max_iter": 10000
    }

    s_opts = {
        "error_on_fail": False
    }

    opti.solver('ipopt')

    # opti.set_value(o_init, [0,0,0,0,0,0])
    opti.set_value(o_init, [0,0,0,0,0,0])

    f = opti.to_function('f', [o_final], [o_x, o_u])
    return (f,)


@app.cell
def _(o_final, opti):
    final = [3,3]
    # final = [-2,2]
    opti.set_value(o_final, final)

    # for _k in range(0,N):
    #     opti.set_initial(o_x[0,_k], lerp(0, final[0], _k/N))
    #     opti.set_initial(o_x[1,_k], lerp(0, final[1], _k/N))
    #     opti.set_initial(o_x[2,_k], lerp(0, 3.14, _k/N))
    #     # (_ul, _ur) = old_to_l_r(_k * T/N, 1.59411804, 0.99583488, 1.03078605, 1.66871616)
    #     # opti.set_initial(o_u[1,_k], _ul)
    #     # opti.set_initial(o_u[1,_k], _ur)
    sol = opti.solve()
    return (sol,)


@app.cell
def _(o_x, sol):
    sol.value(o_x)[:,-1]
    return


@app.cell
def _(np, o_x, plot_trajectory, sol):
    def _():
        xs = np.array(sol.value(o_x))[0,:]
        ys = np.array(sol.value(o_x))[1,:]
        ts = np.array(sol.value(o_x))[2,:]
        return plot_trajectory(xs, ys, ts)
    _()
    return


@app.cell
def _(f, np):
    many_solves = []
    for _x in range(-3,4):
        for _y in range(0,4):
            (states, control) = f([_x,_y])
            states = np.array(states)
            many_solves.append((states[0,:], states[1,:], states[2,:]))
    return (many_solves,)


@app.cell
def _(go, many_solves, plot_trajectories):
    fig = go.Figure()
    for (xs, ys, ts) in many_solves:
        plot_trajectories(fig, xs, ys, ts)
    fig
    return


@app.cell
def _(go, np, o_u, o_x, sol, tgrid):
    def _():
        # sol = opti.debug
        xs = np.array(sol.value(o_x))[0,:]
        ys = np.array(sol.value(o_x))[1,:]
        ts = np.array(sol.value(o_x))[2,:]
        vxs = np.array(sol.value(o_x))[3,:]
        vys = np.array(sol.value(o_x))[4,:]
        vts = np.array(sol.value(o_x))[5,:]

        uls = np.array(sol.value(o_u))[0,:]
        urs = np.array(sol.value(o_u))[1,:]
        # print(sol.value(o_x[:,100]))

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=tgrid, y=xs, mode='lines', name='px'))
        fig.add_trace(go.Scatter(x=tgrid, y=ys, mode='lines', name='py'))
        fig.add_trace(go.Scatter(x=tgrid, y=ts, mode='lines', name='theta'))
        fig.add_trace(go.Scatter(x=tgrid, y=vxs, mode='lines', name='vx'))
        fig.add_trace(go.Scatter(x=tgrid, y=vys, mode='lines', name='vy'))
        fig.add_trace(go.Scatter(x=tgrid, y=vts, mode='lines', name='vtheta'))

        fig.add_trace(go.Scatter(x=tgrid, y=uls, mode='lines', name='ul'))
        fig.add_trace(go.Scatter(x=tgrid, y=urs, mode='lines', name='ur'))

        # return plot_trajectory(xs, ys, ts)
        return fig

    _()
    return


@app.cell(hide_code=True)
def _(go, np):
    def anim_trajectory(xs, us):
        # Simulate the system
        trajectory = xs.T
        us = us.T
    
        px = trajectory[:, 0]
        py = trajectory[:, 1]
        theta = trajectory[:, 2]

        # Parameters for drawing the box
        box_length = 0.5 
        box_width = 0.5

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
        for i in range(0, len(px) - 1, 1):  # Skip frames for speed
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
            scale = 1.1  # adjust for appearance

            # Compute thrust vector positions (relative to box center)
            # offset_y = -box_width / 2
            offset_y = -box_width / 2 - 0.1
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
                name=str(i),
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

        # Compute fixed axis limits
        margin = 2
        x_min, x_max = px.min() - margin, px.max() + margin
        y_min, y_max = py.min() - margin, py.max() + margin


        # Layout with animation controls
        layout = go.Layout(
            title="Animated Box Trajectory",
            xaxis=dict(range=[x_min, x_max], scaleanchor="y", title='p_x'),
            yaxis=dict(range=[y_min, y_max], title='p_y'),
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
            frames=frames,
        )
        return fig
    return (anim_trajectory,)


@app.cell
def _(anim_trajectory, o_u, o_x, sol):
    anim_trajectory(sol.value(o_x), sol.value(o_u))
    return


@app.cell
def _(anim_trajectory, f, np):
    _xs, _us = f([3,1])
    _xs = np.array(_xs)
    _us = np.array(_us)
    anim_trajectory(_xs, _us)
    return


if __name__ == "__main__":
    app.run()
