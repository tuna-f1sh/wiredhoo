import datetime
import math
# plotly requires pandas so I'll use it for DataFrame
import pandas as pd
import numpy as np
# scipy for filtering and curve fitting
from scipy.optimize import curve_fit
import plotly.express as px

def read_stream(file_path):
    return pd.read_csv(file_path)

def first_order(x, a, b):
    return (a * x) + b

# polyfit function, 3rd order
def third_order(x, a, b, c, d):
    return (a * x) + (b * x**2) + (c * x**3) + d

def diff_third_order(dt, a, b, c):
    return a + (b *  dt) + (2 * c * dt)

def pre_process_spindown(spindown: pd.DataFrame, exit_rps: int = 16, sample_period=100e-3):
    # remove zeros
    ret = spindown.loc[~(spindown==0).all(axis=1)]

    # find peak rps and once starts to decay, until exit rps
    maxi = ret.idxmax().rps
    # trim to start
    ret = ret[maxi:]

    # extract only change and decay in rps since only this will change ke
    ret = ret[ret.rps.diff() < 0.0]
    # trim end and anything increasing again
    ret = ret[ret.rps > exit_rps]

    # do our own calculations for omega and ke
    ret.omega = [angular_vecloity(rps) for rps in ret.rps]
    ret.ke = [kinetic_energy(omega, 0.024664) for omega in ret.omega]
    ret.speed = [trainer_speed(rps, 10, int(670 * math.pi)) for rps in ret.rps]

    # find change in ke
    ret["ke_delta"] = ret["ke"].diff()
    # check it's always decaying
    ret = ret[ret.ke_delta < 0.0]

    # since we've dropped points, put an elapsed so we can keep with respect to time
    ret["elapsed"] = (ret.index - maxi) * sample_period

    return ret

def generate_spindown_fit(spindown: pd.DataFrame):
    """
    Use spindown data to fit a curve of energy loss wrt for a given speed

    :param spindown pd.DataFrame: captured from stream and processed with `pre_process_spindown`
    """
    speed_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["rps"])
    fitted_speed = third_order(spindown["elapsed"], speed_terms[0], speed_terms[1], speed_terms[2], speed_terms[3])

    ke_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["ke"])
    fitted_ke = third_order(spindown["elapsed"], ke_terms[0], ke_terms[1], ke_terms[2], ke_terms[3])

    # differentiate for delta ke
    diff_ke = diff_third_order(spindown["elapsed"], ke_terms[0], ke_terms[1], ke_terms[2])
    # or use fitted calculated change
    ke_delta_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["ke_delta"])
    fitted_delta = third_order(spindown["elapsed"], ke_delta_terms[0], ke_delta_terms[1], ke_delta_terms[2], ke_delta_terms[3])

    spindown_terms, _ = curve_fit(third_order, fitted_speed, diff_ke)
    # spindown_terms, _ = curve_fit(third_order, fitted_speed, fitted_delta)
    spindown_curve = third_order(spindown["rps"], spindown_terms[0], spindown_terms[1], spindown_terms[2], spindown_terms[3])

    return spindown_terms, spindown_curve

def post_process_run(run: pd.DataFrame, spindown_terms, update_freq=10, system_i=0.024664):
    # ret = run.loc[~(run==0).all(axis=1)]
    ret = run
    # ret = ret[ret.rps.diff() != 0]

    # drop erronous values
    ret = ret[ret.rps < 100] # to big
    ret = ret[ret.rps.diff() < 5] # sudden change

    ret.omega = [angular_vecloity(rps) for rps in ret.rps]
    ret.ke = [kinetic_energy(omega, system_i) for omega in ret.omega]
    ret.speed = [trainer_speed(rps, 10, int(670 * math.pi)) for rps in ret.rps]

    ret["ke_delta"] = ret["ke"].diff()

    ret["elapsed"] = ret.index * 1/update_freq
    # rider power is change in energy due to speed - known losses fround with spindown wrt
    ret["rider_power"] = (ret["ke_delta"] / ret["elapsed"].diff()) - (third_order(ret["rps"], spindown_terms[0], spindown_terms[1], spindown_terms[2], spindown_terms[3]))# / ret["elapsed"].diff())

    return ret

def angular_vecloity(rps):
    return rps * math.pi * 2

def kinetic_energy(omega, system_i):
    return 0.5 * system_i * omega ** 2

def trainer_speed(rps: int, flywheel_ratio: int, wheel_circumference: int):
    """
    Returns trainer infered speed in mm/s using int same as firmware

    :param rps int: revs per second of flywheel
    :param flywheel_ratio int: flywheel to driven ratio * 100
    :param wheel_circumference int: wheel circumference in mm
    """
    return int((rps * flywheel_ratio * wheel_circumference) / 100)

spindown = read_stream('ref/spindown_b1111_3_zwift.csv')
sd = pre_process_spindown(spindown)
sd_terms, sd_curve = generate_spindown_fit(sd)

race = read_stream('ref/zwift_race.csv')
pr = post_process_run(race, sd_terms)

# raw plot
fig = px.line(spindown, y="rps", labels={"rps":"revs per second"}, title="kickr spindown light transistor sampling")
fig.show()

# plot decay
fig = px.line(sd, y="ke", x="elapsed", labels={"ke_delta":"change in ke"}, title="kickr spindown kinetic energy decay")
fig.show()

# spindown curve
# fig = px.line(y=sd["ke_delta"] * -1, x=sd_curve * -1, title="kickr spindown fitted rps and change in ke")
# fig.show()
fake_rps = np.arange(0,50,1)
ke_loss = third_order(fake_rps, sd_terms[0], sd_terms[1], sd_terms[2], sd_terms[3])
fig = px.line(y=ke_loss, x=fake_rps, labels={"y":"change in ke"}, title="kickr spindown fitted rps and change in ke for energy loss at given rps")
fig.show()

fig = px.line(pr, y=['rider_power', 'rps'], x='elapsed', title="kickr light transistor sampling power inference with spindown comp Zwift race")
fig.show()
