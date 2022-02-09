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
    return a + (b * x) + (c * x**2) + (d * x**3)

def diff_third_order(x, b, c, d):
    return b + (2 * c * x) + (3 * d * (x ** 2))

def pre_process_spindown(spindown: pd.DataFrame, exit_rps: int = 16, update_freq=10):
    # remove zeros
    ret = spindown.loc[~(spindown==0).all(axis=1)]

    # find peak rps and once starts to decay, until exit rps
    maxi = ret.idxmax().rps
    # trim to start
    ret = ret[maxi:]

    # extract only change and decay in rps since only this will change ke
    ret = ret.loc[(ret.rps.diff() < 0.0)]
    # trim end and anything increasing again
    ret = ret[ret.rps > exit_rps]

    # do our own calculations for omega and ke
    ret.omega = [angular_vecloity(rps) for rps in ret.rps]
    ret.ke = [kinetic_energy(omega, 0.024664) for omega in ret.omega]
    ret.speed = [trainer_speed(rps, 10, int(670 * math.pi)) for rps in ret.rps]

    # find change in ke
    ret["ke_delta"] = ret["ke"].diff()
    # check it's always decaying
    ret = ret.loc[(ret.ke_delta < 0.0)]

    # since we've dropped points, put an elapsed so we can keep with respect to time
    ret["elapsed"] = (ret.index - maxi) / update_freq

    return ret

def generate_spindown_fit(spindown: pd.DataFrame, ke_differential=True):
    """
    Use spindown data to fit a curve of energy loss wrt for a given speed

    :param spindown pd.DataFrame: captured from stream and processed with `pre_process_spindown`
    """
    speed_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["rps"])
    fitted_speed = third_order(spindown["elapsed"], speed_terms[0], speed_terms[1], speed_terms[2], speed_terms[3])

    ke_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["ke"])
    # fitted_ke = third_order(spindown["elapsed"], ke_terms[0], ke_terms[1], ke_terms[2], ke_terms[3])

    # differentiate for delta ke
    if ke_differential:
        diff_ke = diff_third_order(spindown["elapsed"], ke_terms[1], ke_terms[2], ke_terms[3])
        spindown_terms, _ = curve_fit(third_order, fitted_speed, diff_ke) # using differential
    else:
        # or use fitted calculated change
        ke_delta_terms, _ = curve_fit(third_order, spindown["elapsed"], spindown["ke_delta"])
        fitted_delta = third_order(spindown["elapsed"], ke_delta_terms[0], ke_delta_terms[1], ke_delta_terms[2], ke_delta_terms[3])
        spindown_terms, _ = curve_fit(third_order, fitted_speed, fitted_delta) # using delta

    spindown_curve = third_order(spindown["rps"], spindown_terms[0], spindown_terms[1], spindown_terms[2], spindown_terms[3])

    return spindown_terms, spindown_curve

def post_process_run(run: pd.DataFrame, spindown_terms, update_freq=10, system_i=0.024664, differential=True):

    def moving_average(x, w):
        return np.convolve(x, np.ones(w), 'valid') / w

    ret = run.copy()

    # drop erronous values
    ret = ret.loc[(ret.rps < 100)] # to big

    ret.omega = [angular_vecloity(rps) for rps in ret.rps]
    ret.ke = [kinetic_energy(omega, system_i) for omega in ret.omega]
    ret.speed = [trainer_speed(rps, 10, int(670 * math.pi)) for rps in ret.rps]

    ret["ke_delta"] = ret["ke"].diff()

    pos_change = ret.loc[(ret.ke_delta > 0)]
    neg_change = ret.loc[(ret.ke_delta < 0)]
    ret["rider_input"] = pos_change.loc[(pos_change.ke_delta.diff() > 0)]["ke_delta"].reindex(ret["ke_delta"].index, method='ffill') * 10 # rider input
    ret["electrical_input"] = neg_change.loc[neg_change.ke_delta.diff() < 0]["ke_delta"].reindex(ret["ke_delta"].index, method='ffill') # electrical power?

    ret["elapsed"] = ret.index * 1/update_freq

    if differential:
        ret["power_static"] = -1 * (third_order(ret["rps"], spindown_terms[0], spindown_terms[1], spindown_terms[2], spindown_terms[3]))
        # static ke loss is differential multipled by time step
        ret["ke_static"] = ret["power_static"] * ret["elapsed"].diff()
    else:
        ret["ke_static"] = -1 * (third_order(ret["rps"], spindown_terms[0], spindown_terms[1], spindown_terms[2], spindown_terms[3]))
        # power is the change in ke with time
        ret["power_static"] = (ret["ke_static"] / ret["elapsed"].diff())

    # change in energy with time
    ret["power_acceleration"] = (ret["ke_delta"] / ret["elapsed"].diff())

    filtered_ps = moving_average(ret["power_static"], 10) # 1 second
    filtered_pa = moving_average(ret["power_acceleration"], 10) # 1 second

    # match others after filter
    ret = ret[:len(filtered_pa)]
    filtered_ps = filtered_ps[:len(filtered_pa)]
    ret["filtered_pa"] = filtered_pa
    ret["filtered_ps"] = filtered_ps

    # rider power is change in energy due to speed - known losses fround with spindown wrt
    ret["rider_power"] = filtered_ps + filtered_pa

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

ke_differential = True
# spindown = read_stream('ref/spindown_b1111_3_zwift.csv')
spindown = read_stream('spindown-031221T1655.csv')
sd = pre_process_spindown(spindown, update_freq=10)
sd_terms, sd_curve = generate_spindown_fit(sd, ke_differential=ke_differential)

# race = read_stream('ref/zwift_race.csv')
# race = read_stream('ref/zwift_race2.csv')
race = read_stream('ride-031221T1655.csv')
pr = post_process_run(race, sd_terms, update_freq=10, differential=ke_differential)

print(f"Spindown coeficients (a + bx + cx^2 + dx^3), a: {sd_terms[0]}, b: {sd_terms[1]}, c: {sd_terms[2]}, d: {sd_terms[3]}")

# raw plot
fig = px.line(spindown, y="rps", labels={"rps":"revs per second"}, title="kickr spindown light transistor sampling")
fig.show()

# plot decay
# fig = px.line(sd, y="ke", x="elapsed", labels={"ke_delta":"change in ke"}, title="kickr spindown kinetic energy decay")
# fig.show()

# spindown curve
# fig = px.line(y=sd_curve * -1, title="kickr spindown fitted rps and change in ke")
# fig.show()
fake_rps = np.arange(0,50,1)
static_power = third_order(fake_rps, sd_terms[0], sd_terms[1], sd_terms[2], sd_terms[3])
fig = px.line(y=static_power, x=fake_rps, labels={"y":"power (W)"}, title="kickr spindown fitted differential ke/rps for power loss at given rps")
fig.show()

fig = px.line(pr, y=["rider_input", "electrical_input", "ke_delta", 'rider_power', 'rps', "power_acceleration", "filtered_pa", "filtered_ps", "ke_static"], x='elapsed', title="kickr light transistor sampling power inference with spindown comp Zwift race")
fig.show()
