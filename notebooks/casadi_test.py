# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "casadi==3.7.1",
#     "marimo",
#     "numpy==2.3.2",
#     "plotly==6.3.0",
# ]
# ///

import marimo

__generated_with = "0.15.2"
app = marimo.App(width="medium")


@app.cell
def _(mo):
    mo.md(r"""Based off of https://www.youtube.com/watch?v=JI-AyLv68Xs""")
    return


@app.cell
def _():
    import marimo as mo
    return (mo,)


@app.cell
def _():
    import casadi as ca
    from casadi import MX, SX, Function
    import numpy as np
    return Function, MX, ca, np


@app.cell
def _():
    import plotly.graph_objects as go
    return (go,)


@app.cell
def _(MX, ca):
    x1 = MX.sym('x1');
    x2 = MX.sym('x2');

    x = ca.vertcat(x1, x2)

    u = MX.sym('u');
    return u, x, x1, x2


@app.cell
def _(ca, u, x1, x2):
    # Van der Pol oscillator
    ode = ca.vertcat( (1 - x2**2)*x1 - x2 + u, x1 )
    return (ode,)


@app.cell
def _(Function, ode, u, x):
    f = Function('f', [x,u], [ode])
    f([0.2,0.8],0.1)
    # print(f)
    return


@app.cell
def _(ca, ode, u, x):
    T = 10 # Time horizon
    N = 20 # Number of control intervals

    # Integrator to discretize the system
    intg_options = {
        "tf": T/N, # Half a second
        "simplify": True,
        "number_of_finite_elements": 4,
    }

    # CasADi handles not just ODEs but DAEs, even though in this case it's a ODE
    dae = {
        "x": x, # States
        "p": u, # Parameters (fixed during integration horizion)
        "ode": ode # Expression for right hand side (could be `f(x,u)`)
    }

    # Single step of the ode
    intg = ca.integrator('intg', 'rk', dae, intg_options)
    print(intg)
    return N, T, intg


@app.cell
def _(intg):
    print(intg([0,1], 0, [], [], [], [], []))
    # Same thing as
    print(intg(x0=[0,1], p=0)['xf'])
    return


@app.cell
def _(intg, u, x):
    _res = intg(x0=x, p=u) # Symbolic evaluation is possible
    x_next = _res['xf']
    print(x_next)
    return (x_next,)


@app.cell
def _(Function, u, x, x_next):
    # Simplify api to just (x,u) -> x_next
    F = Function('F', [x,u], [x_next])

    F([0.1,0.9], 0.1)
    return (F,)


@app.cell
def _(F, N):
    # Turn the single step into the whole time horizon (since (tf = T/N) * N = T)
    sim = F.mapaccum(N)
    return (sim,)


@app.cell
def _(N, T, go, np, sim):
    x0 = [0,1]
    _res = sim(x0, np.sin(np.arange(1, N+1)))
    _res = np.array(_res)

    # Create a grid of values
    tgrid = np.linspace(0, T, N + 1)

    # Create a Plotly figure
    fig = go.Figure()

    # Add traces for x0 and res
    fig.add_trace(go.Scatter(x=tgrid, y=_res[0,:], mode='lines', name='x1'))
    fig.add_trace(go.Scatter(x=tgrid, y=_res[1,:], mode='lines', name='x2'))

    # Update layout with labels
    fig.update_layout(
        xaxis_title='t [s]',
        yaxis_title='Values',
    )

    # Show the plot
    fig.show()

    return tgrid, x0


@app.cell
def _(MX, N, ca, sim, x0):
    U = MX.sym('U', 1, N) # Symbolic representation of control matrix (1 by N)
    X1 = sim(x0, U)[1,:]

    J = ca.jacobian(X1,U)

    J.size()
    return J, U


@app.cell
def _(Function, J, U):
    Jf = Function('F', [U], [J])
    return


@app.cell
def _(F, N, ca):
    # Optimal control problem

    opti = ca.Opti()

    o_x = opti.variable(2, N+1) # Decision variable
    o_u = opti.variable(1, N)
    o_p = opti.parameter(2, 1) # Not optimized over (will be the first measurement)

    opti.minimize(ca.sumsqr(o_x) + ca.sumsqr(o_u))

    for k in range(0,N):
        opti.subject_to(o_x[:,k+1] == F(o_x[:,k],o_u[:,k]))

    opti.subject_to(-1<=o_u)
    opti.subject_to(o_u<=1)
    opti.subject_to(o_x[:,1] == o_p)

    opti
    return o_p, o_u, o_x, opti


@app.cell
def _(o_p, opti):
    # in many examples uses ipopt, but since mpc use sqp method

    opti_opts = {
        "qpsol": "osqp",
        "print_header": False,
        "print_iteration": False,
        "print_time": False,
        # "verbose": True
    }

    opti.solver('sqpmethod', opti_opts) # instead of qrqp maybe use osqp

    opti.set_value(o_p, [0,1])
    sol = opti.solve()
    return (sol,)


@app.cell
def _(go, np, o_u, o_x, sol, tgrid):
    vox1 = np.array(sol.value(o_x))[0,:]
    vox2 = np.array(sol.value(o_x))[1,:]
    vou = np.array(sol.value(o_u))

    _fig = go.Figure()

    _fig.add_trace(go.Scatter(x=tgrid, y=vox1, mode='lines', name='x1'))
    _fig.add_trace(go.Scatter(x=tgrid, y=vox2, mode='lines', name='x2'))
    _fig.add_trace(go.Bar(x=tgrid, y=vou, name='u', opacity=0.5))

    # Update layout with labels
    _fig.update_layout(
        xaxis_title='t [s]',
        yaxis_title='Values',
    )

    # Show the plot
    _fig
    return


@app.cell
def _(o_p, o_u, opti):
    M = opti.to_function('M', [o_p], [o_u[:,1]]) # MPC function from current state to best actuator command
    M
    return (M,)


@app.cell
def _(F, M, N, np):
    import random
    Xs = []
    Us = []

    _x = np.array([[0.5],[1]]);
    for i in range(0, 4*N):
        _u = M(_x)

        Us.append(np.array(_u))
        Xs.append(np.array(_x))

        _x = F(_x,_u) + [0, random.random() * 0.02] # Simulate system with noise

    Xs,Us
    return Us, Xs


@app.cell
def _(Us, Xs, go, np, tgrid):
    def _():
        vox1 = np.array(list(map(lambda x: x[0], np.array(Xs)[:,0])))
        vox2 = np.array(list(map(lambda x: x[0], np.array(Xs)[:,1])))
        vou = list(map(lambda x: x[0][0], Us))
        print(vou)

        _fig = go.Figure()

        _fig.add_trace(go.Scatter(x=tgrid, y=vox1, mode='lines', name='x1'))
        _fig.add_trace(go.Scatter(x=tgrid, y=vox2, mode='lines', name='x2'))
        _fig.add_trace(go.Bar(x=tgrid, y=vou, name='u', opacity=0.5))

        # Update layout with labels
        _fig.update_layout(
            xaxis_title='t [s]',
            yaxis_title='Values',
        )

        # Show the plot
        return _fig
    _()
    return


if __name__ == "__main__":
    app.run()
