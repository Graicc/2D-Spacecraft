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

__generated_with = "0.18.4"
app = marimo.App(width="medium")


@app.cell
def _():
    import casadi as ca
    from casadi import MX, Function
    import numpy as np
    return MX, ca, np


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
def _(ca, px_d, py_d, theta, theta_d, u, ul, ur, x):
    # let px_ddot = 0.2 * u[0] * cos_theta + 0.2 * u[1] * cos_theta - 0.02 * v_px;
    # let py_ddot = 0.2 * u[0] * sin_theta + 0.2 * u[1] * sin_theta - 0.02 * v_py;
    # let theta_ddot = -2.0 * u[0] + 2.0 * u[1] - 0.4 * theta_dot;

    st = ca.sin(theta)
    ct = ca.cos(theta)
    px_dd = 0.2 * ul * ct + 0.2 * ur * ct - 0.02 * px_d
    py_dd = 0.2 * ul * st + 0.2 * ur * st - 0.02 * py_d
    theta_dd = -2.0 * ul + 2.0 * ur - 0.4 * theta_d

    # px_dd = 0.2 * ul * ct + 0.2 * ur * ct
    # py_dd = 0.2 * ul * st + 0.2 * ur * st
    # theta_dd = -2.0 * ul + 2.0 * ur

    x_dot = ca.vertcat(
        px_d,py_d,theta_d,
        px_dd, py_dd, theta_dd
    )

    x_dot

    f = ca.Function("f", [x,u], [x_dot])
    return (f,)


@app.cell
def _(go, np):
    def plot_trajectory(xs, ys, ts, fig: go.Figure = None):
        if fig is None:
            fig = go.Figure()
        fig.add_trace(go.Scatter(x=xs, y=ys, mode='lines'))

        for i in range(0, len(ts), 1):
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
    def plot_control(ul, ur, fig: go.Figure = None):
        if fig is None:
            fig = go.Figure()

        ts = np.linspace(0,len(ul))
        fig.add_trace(go.Scatter(x=ts, y=ul, mode='lines'))
        fig.add_trace(go.Scatter(x=ts, y=ur, mode='lines'))
        return fig
    return (plot_control,)


@app.cell
def _(N, go, np):
    def plot_values(t, xs, xs_interp, us, fig: go.Figure = None):
        if fig is None:
            fig = go.Figure()

        ts = np.linspace(0, t, len(xs[0]) + len(xs_interp[0]))

        x = []
        for i in range(N):
            x.append(xs[:,i])
            x.append(xs_interp[:,i])
        x.append(xs[:,N])
        
        fig.add_trace(go.Scatter(x=ts, y=[x[0] for x in x], mode='lines', name="x"))
        fig.add_trace(go.Scatter(x=ts, y=[x[1] for x in x], mode='lines', name="y"))
        fig.add_trace(go.Scatter(x=ts, y=[x[2] for x in x], mode='lines', name="theta"))
        fig.add_trace(go.Scatter(x=ts, y=[x[3] for x in x], mode='lines', name="vx"))
        fig.add_trace(go.Scatter(x=ts, y=[x[4] for x in x], mode='lines', name="vy"))
        fig.add_trace(go.Scatter(x=ts, y=[x[5] for x in x], mode='lines', name="vt"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[0,:], mode='lines', name="y"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[1,:], mode='lines', name="y"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[2,:], mode='lines', name="theta"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[3,:], mode='lines', name="vx"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[4,:], mode='lines', name="vy"))
        # fig.add_trace(go.Scatter(x=ts, y=xs[5,:], mode='lines', name="vt"))

        return fig
    return (plot_values,)


@app.cell
def _(ca, f):
    N = 10
    opti = ca.Opti()

    o_x = opti.variable(6, N+1)
    o_x_interpolation = opti.variable(6, N) # X_k+1/2
    o_u = opti.variable(2, N+1)
    o_u_interpolation = opti.variable(2, N)
    o_t = opti.variable(1)
    o_init = opti.parameter(6,1)
    o_final = opti.parameter(2,1)

    # Objective function
    # Minimize time
    opti.minimize(o_t)


    # opti.minimize(o_t + 0.1 * ca.sumsqr(o_u))

    # Boundry constraints
    opti.subject_to(o_x[:,0] == o_init)
    opti.subject_to(o_x[0,N] == o_final[0])
    opti.subject_to(o_x[1,N] == o_final[1])
    opti.subject_to(o_x[3,N] == 0)
    opti.subject_to(o_x[4,N] == 0)
    opti.subject_to(o_x[5,N] == 0)

    h = o_t/N

    for i in range(N):
        # Interpolation constraint
        opti.subject_to(o_x_interpolation[:,i] == 1/2 * (o_x[:,i] + o_x[:, i+1]) + h/8 * (f(o_x[:,i], o_u[i]) - f(o_x[:,i+1], o_u[:,i+1])))

        # Collocation constraint
        opti.subject_to(
            h/6 * (f(o_x[:,i], o_u[:,i]) + 4 * f(o_x_interpolation[:,i], o_u_interpolation[:,i]) + f(o_x[:,i+1], o_u[:,i+1]))
            ==
            o_x[:,i+1] - o_x[:,i]
        )

    # Control constraints
    for i in range(N+1):
        opti.subject_to(o_u[:, i] <= 1)
        opti.subject_to(o_u[:, i] >= 0)
    
    for i in range(N):
        opti.subject_to(o_u_interpolation[:, i] <= 1)
        opti.subject_to(o_u_interpolation[:, i] >= 0)

    # Time constraints
    opti.subject_to(o_t > 0)
    opti.subject_to(o_t <= 10)

    opti.solver("ipopt")
    return N, o_final, o_init, o_t, o_u, o_x, o_x_interpolation, opti


@app.cell
def _(N, o_final, o_init, o_t, o_x, o_x_interpolation, opti):
    opti.set_value(o_init, [0,0,0,0,0,0])
    final_pos = [5,1]

    opti.set_value(o_final, final_pos)

    for _i in range(N):
        opti.set_initial(o_x[:, _i], [final_pos[0]*_i/N,final_pos[1]*_i/N,0,0,0,0])
        opti.set_initial(o_x_interpolation[:, _i], [final_pos[0]*_i/N,final_pos[1]*_i/N,0,0,0,0])


    opti.set_initial(o_t, 10)

    sol = opti.solve()
    return (sol,)


@app.cell
def _():
    return


@app.cell
def _(o_t, sol):
    sol_t = sol.value(o_t)
    sol_t
    return (sol_t,)


@app.cell
def _(o_x, plot_trajectory, sol):
    sol_x = sol.value(o_x)
    # sol_x = opti.debug.value(o_x)
    plot_trajectory(sol_x[0], sol_x[1], sol_x[2])
    return (sol_x,)


@app.cell
def _(o_u, plot_control, sol):
    sol_u = sol.value(o_u)
    plot_control(sol_u[0], sol_u[1])
    return (sol_u,)


@app.cell
def _(o_x_interpolation, plot_values, sol, sol_t, sol_u, sol_x):
    sol_x_interpolation = sol.value(o_x_interpolation)
    # plot_values(sol_t, sol_x_interpolation, sol_u)
    plot_values(sol_t, sol_x, sol_x_interpolation, sol_u)
    return


if __name__ == "__main__":
    app.run()
