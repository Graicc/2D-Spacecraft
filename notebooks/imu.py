# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "marimo",
#     "numpy==2.3.2",
#     "pandas==2.3.2",
#     "plotly==6.3.0",
#     "scipy==1.16.1",
# ]
# ///

import marimo

__generated_with = "0.14.10"
app = marimo.App(width="medium")


@app.cell
def _():
    import pandas as pd
    import plotly.graph_objects as go
    return go, pd


@app.cell
def _(pd):
    up = pd.read_csv("../data/MPU6000_stationary_up.csv")
    down = pd.read_csv("../data/MPU6000_stationary_down.csv")

    # Convert 16 bit to +- 16g
    up[['acc_x', 'acc_y', 'acc_z']] = up[['acc_x', 'acc_y', 'acc_z']] / 2048.0
    down[['acc_x', 'acc_y', 'acc_z']] = down[['acc_x', 'acc_y', 'acc_z']] / 2048.0

    # convert 16 bit to +-2000dps
    up['gyro_z'] = up['gyro_z'] / 16.4
    down['gyro_z'] = down['gyro_z'] / 16.4

    # make a time axis
    dt = 0.01
    time = pd.Series([i * dt for i in range(len(up))])
    up['time'] = time
    down['time'] = time
    return down, dt, up


@app.cell
def _(down, go, up):
    # Graph

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(y=up["acc_x"], mode='lines', name='accel_x_up'))
    _fig.add_trace(go.Scatter(y=up["acc_y"], mode='lines', name='accel_y_up'))
    _fig.add_trace(go.Scatter(y=up["acc_z"], mode='lines', name='accel_z_up'))
    _fig.add_trace(go.Scatter(y=down["acc_x"], mode='lines', name='accel_x_down'))
    _fig.add_trace(go.Scatter(y=down["acc_y"], mode='lines', name='accel_y_down'))
    _fig.add_trace(go.Scatter(y=down["acc_z"], mode='lines', name='accel_z_down'))
    _fig.update_layout(title='MPU6000 Stationary Data', xaxis_title='Sample', yaxis_title='Acceleration (g)')

    _fig

    return


@app.cell
def _(down, go, up):
    # Low pass

    from scipy.signal import butter, filtfilt

    def lowpass_filter(data, cutoff=5, fs=100, order=4):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y

    up['acc_x_filt'] = lowpass_filter(up['acc_x'])
    up['acc_y_filt'] = lowpass_filter(up['acc_y'])
    up['acc_z_filt'] = lowpass_filter(up['acc_z'])
    down['acc_x_filt'] = lowpass_filter(down['acc_x'])
    down['acc_y_filt'] = lowpass_filter(down['acc_y'])
    down['acc_z_filt'] = lowpass_filter(down['acc_z'])

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(y=up["acc_x_filt"], mode='lines', name='accel_x_up_filt'))
    _fig.add_trace(go.Scatter(y=up["acc_y_filt"], mode='lines', name='accel_y_up_filt'))
    # _fig.add_trace(go.Scatter(y=up["acc_z_filt"], mode='lines', name='accel_z_up_filt'))
    _fig.add_trace(go.Scatter(y=down["acc_x_filt"], mode='lines', name='accel_x_down_filt'))
    _fig.add_trace(go.Scatter(y=down["acc_y_filt"], mode='lines', name='accel_y_down_filt'))
    # _fig.add_trace(go.Scatter(y=down["acc_z_filt"], mode='lines', name='accel_z_down_filt'))
    _fig.update_layout(title='MPU6000 Stationary Data (Filtered)', xaxis_title='Sample', yaxis_title='Acceleration (g)')

    _fig
    return (lowpass_filter,)


@app.cell
def _(down, go, up):
    # Moving average

    def moving_average(data, window_size=100):
        return data.rolling(window=window_size, center=True).mean()

    up['acc_x_ma'] = moving_average(up['acc_x'])
    up['acc_y_ma'] = moving_average(up['acc_y'])
    up['acc_z_ma'] = moving_average(up['acc_z'])
    down['acc_x_ma'] = moving_average(down['acc_x'])
    down['acc_y_ma'] = moving_average(down['acc_y'])
    down['acc_z_ma'] = moving_average(down['acc_z'])

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(y=up["acc_x_ma"], mode='lines', name='accel_x_up_ma'))
    _fig.add_trace(go.Scatter(y=up["acc_y_ma"], mode='lines', name='accel_y_up_ma'))
    # _fig.add_trace(go.Scatter(y=up["acc_z_ma"], mode='lines', name='accel_z_up_ma'))
    _fig.add_trace(go.Scatter(y=down["acc_x_ma"], mode='lines', name='accel_x_down_ma'))
    _fig.add_trace(go.Scatter(y=down["acc_y_ma"], mode='lines', name='accel_y_down_ma'))
    # _fig.add_trace(go.Scatter(y=down["acc_z_ma"], mode='lines', name='accel_z_down_ma'))
    _fig.update_layout(title='MPU6000 Stationary Data (Moving Average)', xaxis_title='Sample', yaxis_title='Acceleration (g)')

    _fig
    return


@app.cell
def _(down, go, up):
    # Assume stationary for first 100 samples, use that to calibrate

    up_offset = up.iloc[:100][['acc_x', 'acc_y', 'acc_z']].mean()
    down_offset = down.iloc[:100][['acc_x', 'acc_y', 'acc_z']].mean()
    print("Up offsets:", up_offset)
    print("Down offsets:", down_offset)

    up['acc_x_cal'] = up['acc_x'] - up_offset['acc_x']
    up['acc_y_cal'] = up['acc_y'] - up_offset['acc_y']
    up['acc_z_cal'] = up['acc_z'] - up_offset['acc_z']
    down['acc_x_cal'] = down['acc_x'] - down_offset['acc_x']
    down['acc_y_cal'] = down['acc_y'] - down_offset['acc_y']
    down['acc_z_cal'] = down['acc_z'] - down_offset['acc_z']

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(y=up["acc_x_cal"], mode='lines', name='accel_x_up_cal'))
    _fig.add_trace(go.Scatter(y=up["acc_y_cal"], mode='lines', name='accel_y_up_cal'))
    # _fig.add_trace(go.Scatter(y=up["acc_z_cal"], mode='lines', name='accel_z_up_cal'))
    _fig.add_trace(go.Scatter(y=down["acc_x_cal"], mode='lines', name='accel_x_down_cal'))
    _fig.add_trace(go.Scatter(y=down["acc_y_cal"], mode='lines', name='accel_y_down_cal'))
    # _fig.add_trace(go.Scatter(y=down["acc_z_cal"], mode='lines', name='accel_z_down_cal'))
    _fig.update_layout(title='MPU6000 Stationary Data (Calibrated)', xaxis_title='Sample', yaxis_title='Acceleration (g)')

    _fig
    return


@app.cell
def _(down, lowpass_filter, up):
    # Also low pass the calibrated data
    up['acc_x_cal_filt'] = lowpass_filter(up['acc_x_cal'])
    up['acc_y_cal_filt'] = lowpass_filter(up['acc_y_cal'])
    up['acc_z_cal_filt'] = lowpass_filter(up['acc_z_cal'])
    down['acc_x_cal_filt'] = lowpass_filter(down['acc_x_cal'])
    down['acc_y_cal_filt'] = lowpass_filter(down['acc_y_cal'])
    down['acc_z_cal_filt'] = lowpass_filter(down['acc_z_cal'])
    return


@app.cell
def _(down, dt, go, up):
    # Integrate to get velocity and position

    g_to_ms2 = 9.80665

    up['vel_x'] = (up['acc_x_cal'] * g_to_ms2 * dt).cumsum()
    up['vel_y'] = (up['acc_y_cal'] * g_to_ms2 * dt).cumsum()
    up['vel_z'] = (up['acc_z_cal'] * g_to_ms2 * dt).cumsum()
    down['vel_x'] = (down['acc_x_cal'] * g_to_ms2 * dt).cumsum()
    down['vel_y'] = (down['acc_y_cal'] * g_to_ms2 * dt).cumsum()
    down['vel_z'] = (down['acc_z_cal'] * g_to_ms2 * dt).cumsum()

    up['pos_x'] = (up['vel_x'] * dt).cumsum()
    up['pos_y'] = (up['vel_y'] * dt).cumsum()
    up['pos_z'] = (up['vel_z'] * dt).cumsum()
    down['pos_x'] = (down['vel_x'] * dt).cumsum()
    down['pos_y'] = (down['vel_y'] * dt).cumsum()
    down['pos_z'] = (down['vel_z'] * dt).cumsum()

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(x=up["time"], y=up["pos_x"], mode='lines', name='pos_x_up'))
    _fig.add_trace(go.Scatter(x=up["time"], y=up["pos_y"], mode='lines', name='pos_y_up'))
    # _fig.add_trace(go.Scatter(x=up["time"], y=up["pos_z"], mode='lines', name='pos_z_up'))
    _fig.add_trace(go.Scatter(x=up["time"], y=down["pos_x"], mode='lines', name='pos_x_down'))
    _fig.add_trace(go.Scatter(x=up["time"], y=down["pos_y"], mode='lines', name='pos_y_down'))
    # _fig.add_trace(go.Scatter(x=up["time"], y=down["pos_z"], mode='lines', name='pos_z_down'))
    _fig.update_layout(title='MPU6000 Stationary Data (Position)', xaxis_title='Sample', yaxis_title='Position (m)')

    # trim to first 15 seconds
    _fig.update_xaxes(range=[0, 15])

    # trim y to fit
    _fig.update_yaxes(range=[-1, 1])

    _fig
    return (g_to_ms2,)


@app.cell
def _(down, dt, go, up):
    # 2d model, only consider x,y and gryo z
    # Integrate gyro z to get angle

    up['angle_z'] = (up['gyro_z'] * dt).cumsum()
    down['angle_z'] = (down['gyro_z'] * dt).cumsum()
    import numpy as np
    up['angle_z_rad'] = np.deg2rad(up['angle_z'])
    down['angle_z_rad'] = np.deg2rad(down['angle_z'])

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(x=up["time"], y=up["angle_z"], mode='lines', name='angle_z_up'))
    _fig.add_trace(go.Scatter(x=down["time"], y=down["angle_z"], mode='lines', name='angle_z_down'))
    _fig.update_layout(title='MPU6000 Stationary Data (Angle)', xaxis_title='Time (s)', yaxis_title='Angle Z (deg)')

    _fig
    return (np,)


@app.cell
def _(down, dt, g_to_ms2, go, np, up):
    # Calculate motion in global frame
    up['acc_x_global'] = up['acc_x_cal'] * np.cos(up['angle_z_rad']) - up['acc_y_cal'] * np.sin(up['angle_z_rad'])
    up['acc_y_global'] = up['acc_x_cal'] * np.sin(up['angle_z_rad']) + up['acc_y_cal'] * np.cos(up['angle_z_rad'])
    down['acc_x_global'] = down['acc_x_cal'] * np.cos(down['angle_z_rad']) - down['acc_y_cal'] * np.sin(down['angle_z_rad'])
    down['acc_y_global'] = down['acc_x_cal'] * np.sin(down['angle_z_rad']) + down['acc_y_cal'] * np.cos(down['angle_z_rad'])
    # Integrate to get velocity and position in global frame
    up['vel_x_global'] = (up['acc_x_global'] * g_to_ms2 * dt).cumsum()
    up['vel_y_global'] = (up['acc_y_global'] * g_to_ms2 * dt).cumsum()
    down['vel_x_global'] = (down['acc_x_global'] * g_to_ms2 * dt).cumsum()
    down['vel_y_global'] = (down['acc_y_global'] * g_to_ms2 * dt).cumsum()
    up['pos_x_global'] = (up['vel_x_global'] * dt).cumsum()
    up['pos_y_global'] = (up['vel_y_global'] * dt).cumsum()
    down['pos_x_global'] = (down['vel_x_global'] * dt).cumsum()
    down['pos_y_global'] = (down['vel_y_global'] * dt).cumsum()

    # Trim to first couple seconds
    _cutoff = 10
    up['pos_x_global'] = up['pos_x_global'][up['time'] <= _cutoff]
    up['pos_y_global'] = up['pos_y_global'][up['time'] <= _cutoff]
    down['pos_x_global'] = down['pos_x_global'][down['time'] <= _cutoff]
    down['pos_y_global'] = down['pos_y_global'][down['time'] <= _cutoff]

    _fig = go.Figure()
    _fig.add_trace(go.Scatter(x=up["pos_x_global"], y=up["pos_y_global"], mode='lines', name='pos_up'))
    _fig.add_trace(go.Scatter(x=down["pos_x_global"], y=down["pos_y_global"], mode='lines', name='pos_down'))
    _fig.update_layout(title='MPU6000 Stationary Data (2D Position)', xaxis_title='Position X (m)', yaxis_title='Position Y (m)')

    # Fixed aspect ratio
    _fig.update_yaxes(scaleanchor="x", scaleratio=1)
    _fig.update_layout(autosize=False)

    _fig
    return


if __name__ == "__main__":
    app.run()
