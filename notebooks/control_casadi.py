# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "casadi==3.7.1",
#     "numpy==2.3.2",
#     "plotly==6.3.0",
# ]
# ///

import marimo

__generated_with = "0.14.10"
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
    # N = T * 100 # Number of control intervals
    N = 120 # Number of control intervals

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
def _(N, T, ca, go, np, plot_trajectory, sim):
    # Example sim

    tgrid = np.linspace(0, T, N+1)
    def _():
        x0 = [0,0,-3.14 / 2,0,0,0]

        us = ca.repmat(ca.vertcat(1,1), 1, 20)

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

    # opti.minimize(ca.sumsqr(o_u)) # Minimize control effort
    _final = o_x[:,-1]
    _err_x = ca.dot((_final[0] - o_final[0]),(_final[0] - o_final[0]))
    _err_y = ca.dot((_final[1] - o_final[1]),(_final[1] - o_final[1]))

    opti.minimize(_err_x + _err_y)

    # Physics constraint
    for k in range(0,N):
        opti.subject_to(o_x[:,k+1] == F(o_x[:,k], o_u[:,k]))

    # opti.subject_to(o_u[0,:] >= 0)
    # opti.subject_to(o_u[1,:] >= 0)
    opti.subject_to(o_u[0,:] >= -1)
    opti.subject_to(o_u[1,:] >= -1)

    opti.subject_to(o_u[0,:] <= 1)
    opti.subject_to(o_u[1,:] <= 1)
    opti.subject_to(o_x[:,1] == o_init)
    opti.subject_to(o_x[0,-1] == o_final[0])
    opti.subject_to(o_x[1,-1] == o_final[1])
    opti.subject_to(o_x[3,-1] == 0)
    opti.subject_to(o_x[4,-1] == 0)

    opti
    return o_final, o_init, o_u, o_x, opti


@app.cell
def _(o_final, o_init, opti):
    opti_opts = {
        "qpsol": "osqp",
        # "qpsol": "qrqp",
        "print_header": False,
        "print_iteration": False,
        "print_time": False,
        # "max_iter": 10000
    }

    opti.solver('sqpmethod', opti_opts)

    # opti.set_value(o_init, [0,0,0,0,0,0])
    opti.set_value(o_init, [0,0,-3.14 / 2,0,0,0])
    opti.set_value(o_final, [0,-2])
    sol = opti.solve()
    return (sol,)


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
def _(go, np, o_u, o_x, opti, tgrid):
    def _():
        sol = opti.debug
        xs = np.array(sol.value(o_x))[0,:]
        ys = np.array(sol.value(o_x))[1,:]
        ts = np.array(sol.value(o_x))[2,:]
        xds = np.array(sol.value(o_x))[3,:]
        yds = np.array(sol.value(o_x))[4,:]
        tds = np.array(sol.value(o_x))[5,:]

        uls = np.array(sol.value(o_u))[0,:]
        urs = np.array(sol.value(o_u))[1,:]
        # print(sol.value(o_x[:,100]))
    
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=tgrid, y=xs, mode='lines', name='px'))
        fig.add_trace(go.Scatter(x=tgrid, y=ys, mode='lines', name='py'))
        fig.add_trace(go.Scatter(x=tgrid, y=ts, mode='lines', name='theta'))
    
        fig.add_trace(go.Scatter(x=tgrid, y=uls, mode='lines', name='ul'))
        fig.add_trace(go.Scatter(x=tgrid, y=urs, mode='lines', name='ur'))

        # return plot_trajectory(xs, ys, ts)
        return fig

    _()
    return


@app.cell
def _():
    return


if __name__ == "__main__":
    app.run()
