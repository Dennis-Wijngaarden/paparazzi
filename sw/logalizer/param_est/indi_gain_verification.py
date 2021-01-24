import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from scipy import signal

# Helper functions
def bound(min_value, max_value, value):
    value = max(min_value, value)
    value = min(max_value, value)
    return value

# Parameters
avg_thrust = 5000
max_pq_cmd = 4500 # 2 because it effects pprz command half a time
min_pq_cmd = -4500
max_r_cmd = 4500
min_r_cmd = -4500
indi_params = {}
indi_params["g1_p"] = 0.01782447 / 181. * 256.
indi_params["g1_q"] = 0.01781561 / 181. * 256.
indi_params["g1_r"] = 0.00092242 / 128. * 256.
indi_params["g2_r"] = 0.00017679 / 128. * 256.
indi_params["p_gain"] = 500.
indi_params["q_gain"] = 175.
indi_params["r_gain"] = 65.
indi_params["dp_gain"] = 15.5
indi_params["dq_gain"] = 15.5
indi_params["dr_gain"] = 9.
indi_params["max_rate"] = 180. / 180. * np.pi # [rad/s]
indi_params["max_r_rate"] = 50. / 180. * np.pi # [rad/s]
indi_params["act_dyn_p"] = 0.04
indi_params["act_dyn_q"] = 0.04
indi_params["act_dyn_r"] = 0.04
indi_params["filt_cutoff"] = 5.0 # [Hz]
indi_params["r_filt_cutoff"] = 2.5 # [Hz]

# Other settings
sf = 500 # Sample frequency [Hz]
tau = 1./sf # Timestep

# Define state error(s) to plot responses (single axis only)
d_phi =     [20. / 180. * np.pi] # [rad]
d_theta =   [20. / 180. * np.pi] # [rad]
d_psi =     [90. / 180. * np.pi] # [rad]
d_qx = [d_phi[-1] / 2.]
d_qy = [d_theta[-1] / 2.]
d_qz = [d_psi[-1] / 2.]

# Define time array
t_end = 2. # [s]
t = np.arange(t_end, step = tau) # [s]

# Loop until t_end has been reached
rate_sp = {} # rad/s

angular_acc_sp = {} # rad/s2

body_rates = {}
body_rates["p"] = [0.] # rad/s
body_rates["q"] = [0.] # rad/s
body_rates["r"] = [0.] # rad/s

# Filtered body rates
fbody_rates = {}
fbody_rates["p"] = [0.] # rad/s
fbody_rates["q"] = [0.] # rad/s
fbody_rates["r"] = [0.] # rad/s

body_drates = {}
body_drates["p"] = [0.] # rad/s2
body_drates["q"] = [0.] # rad/s2
body_drates["r"] = [0.] # rad/s2

# Filtered body accelerations
fbody_drates = {}
fbody_drates["p"] = [0.] # rad/s2
fbody_drates["q"] = [0.] # rad/s2
fbody_drates["r"] = [0.] # rad/s2

u_in = {}
u_in["p"] = [0.] # PPRZ cmd 
u_in["q"] = [0.] # PPRZ cmd
u_in["r"] = [0.] # PPRZ cmd

u_act = {}
u_act["p"] = [0.] # PPRZ cmd
u_act["q"] = [0.] # PPRZ cmd
u_act["r"] = [0.] # PPRZ cmd

# Filtered actuators
fu_act = {}
fu_act["p"] = [0.] # PPRZ cmd
fu_act["q"] = [0.] # PPRZ cmd
fu_act["r"] = [0.] # PPRZ cmd

du = {} # PPRZ units
du["r"] = 0.0

# Define Butterworth noise filter
b_pq, a_pq = sp.signal.butter(2, indi_params["filt_cutoff"]/(sf/2), 'low', analog=False)
b_r, a_r = sp.signal.butter(2, indi_params["r_filt_cutoff"]/(sf/2), 'low', analog=False)

for i in range(len(t)):
    # Compute rate setpoints
    rate_sp["p"] = indi_params["p_gain"] * d_qx[-1]   / indi_params["dp_gain"]
    rate_sp["q"] = indi_params["q_gain"] * d_qy[-1] / indi_params["dq_gain"]
    rate_sp["r"] = bound(-indi_params["max_r_rate"], indi_params["max_r_rate"], indi_params["r_gain"] * d_qz[-1]   / indi_params["dr_gain"])

    # Compute reference angular accelerations
    angular_acc_sp["p"] = (rate_sp["p"] - body_rates["p"][-1]) * indi_params["dp_gain"]
    angular_acc_sp["q"] = (rate_sp["q"] - body_rates["q"][-1]) * indi_params["dq_gain"]
    angular_acc_sp["r"] = (rate_sp["r"] - body_rates["r"][-1]) * indi_params["dr_gain"]

    # Compute control inputs
    du["p"] = 1.0 / indi_params["g1_p"] * (angular_acc_sp["p"] - fbody_drates["p"][-1])
    du["q"] = 1.0 / indi_params["g1_q"] * (angular_acc_sp["q"] - fbody_drates["q"][-1])
    du["r"] = 1.0 / (indi_params["g1_r"] + indi_params["g2_r"]) * (angular_acc_sp["r"] - fbody_drates["r"][-1] + indi_params["g2_r"] * du["r"])
    
    u_in["p"].append(bound(min_pq_cmd, max_pq_cmd, fu_act["p"][-1] + du["p"]))
    u_in["q"].append(bound(min_pq_cmd, max_pq_cmd, fu_act["q"][-1] + du["q"]))
    u_in["r"].append(bound(min_r_cmd, max_r_cmd, fu_act["r"][-1] + du["r"]))

    u_act["p"].append(u_act["p"][-1] + indi_params["act_dyn_p"] * (u_in["p"][-1] - u_act["p"][-1]))
    u_act["q"].append(u_act["q"][-1] + indi_params["act_dyn_q"] * (u_in["q"][-1] - u_act["q"][-1]))
    u_act["r"].append(u_act["r"][-1] + indi_params["act_dyn_r"] * (u_in["r"][-1] - u_act["r"][-1]))

    # set new body drates
    body_drates["p"].append(body_drates["p"][-1] + (u_act["p"][-1] - u_act["p"][-2]) * indi_params["g1_p"])
    body_drates["q"].append(body_drates["q"][-1] + (u_act["q"][-1] - u_act["q"][-2]) * indi_params["g1_q"])
    body_drates["r"].append(body_drates["r"][-1] + (u_act["r"][-1] - u_act["r"][-2]) * indi_params["g1_r"])

    # set new body rates
    body_rates["p"].append(body_rates["p"][-1] + body_drates["p"][-1] * tau)
    body_rates["q"].append(body_rates["q"][-1] + body_drates["q"][-1] * tau)
    body_rates["r"].append(body_rates["r"][-1] + body_drates["r"][-1] * tau)

    # update error
    d_phi.append(     d_phi[-1]   - body_rates["p"][-1] * tau)
    d_theta.append(   d_theta[-1] - body_rates["q"][-1] * tau)
    d_psi.append(     d_psi[-1]   - body_rates["r"][-1] * tau)
    d_qx.append(d_phi[-1] / 2.)
    d_qy.append(d_theta[-1] / 2.)
    d_qz.append(d_psi[-1] / 2.)

    # Apply filters for next loop
    # Filter body rates
    fbody_rates["p"] = sp.signal.lfilter(b_pq, a_pq, body_rates["p"])
    fbody_rates["q"] = sp.signal.lfilter(b_pq, a_pq, body_rates["q"])
    fbody_rates["r"] = sp.signal.lfilter(b_r , a_r , body_rates["r"])

    # Filter body accelerations
    fbody_drates["p"].append((fbody_rates["p"][-1] - fbody_rates["p"][-2]) / tau)
    fbody_drates["q"].append((fbody_rates["q"][-1] - fbody_rates["q"][-2]) / tau)
    fbody_drates["r"].append((fbody_rates["r"][-1] - fbody_rates["r"][-2]) / tau)

    # filter actuator
    fu_act["p"] = sp.signal.lfilter(b_pq, a_pq, u_act["p"])
    fu_act["q"] = sp.signal.lfilter(b_pq, a_pq, u_act["q"])
    fu_act["r"] = sp.signal.lfilter(b_r , a_r , u_act["r"])

plt.figure()
plt.plot(t, d_phi[0:-1])

plt.figure()
plt.plot(t, body_rates["p"][0:-1])
plt.plot(t, fbody_rates["p"][0:-1])

plt.figure()
plt.plot(t, body_drates["p"][0:-1])
plt.plot(t, fbody_drates["p"][0:-1])

plt.figure()
plt.plot(t, u_in["p"][0:-1])
plt.plot(t, u_act["p"][0:-1])
plt.plot(t, fu_act["p"][0:-1])

plt.show()