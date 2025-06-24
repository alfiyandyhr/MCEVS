import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

# Case reader
cr = om.CaseReader('liftcruise_opt_cases.sql')

driver_cases = cr.list_cases('driver')

solidity = 0.13

# cur_case
hist_array = np.zeros((len(driver_cases), 12))
for i, case in enumerate(driver_cases):
    cur_case = cr.get_case(case)
    dv1 = cur_case.get_design_vars()['Mission|cruise_speed'][0]
    dv2 = cur_case.get_design_vars()['Wing|area'][0]
    dv3 = cur_case.get_design_vars()['Wing|aspect_ratio'][0]
    dv4 = cur_case.get_design_vars()['LiftRotor|radius'][0]
    dv5 = cur_case.get_design_vars()['Propeller|radius'][0]
    dv6 = cur_case.get_design_vars()['Propeller|advance_ratio'][0]
    F = cur_case.get_objectives()["Weight|takeoff"][0]
    G1 = cur_case.get_constraints()['Aero|CL_cruise'][0]
    G2 = cur_case.get_constraints()['DiskLoading|LiftRotor|segment_1'][0]
    G3 = cur_case.get_constraints()['DiskLoading|Propeller|segment_2'][0]
    G4 = cur_case.get_constraints()['Propeller|thrust_coefficient'][0] / solidity
    CV = max(G1 - 0.6, 0) + max(G2 - 600, 0) + max(G3 - 600, 0) + max(G4 - 0.14, 0)
    cur_arr = np.array([dv1, dv2, dv3, dv4, dv5, dv6, F, G1, G2, G3, CV])
    cur_arr = np.array([dv1, dv2, dv3, dv4, dv5, dv6, F, G1, G2, G3, G4, CV])
    hist_array[i] = cur_arr

plot_F = True
plot_all = True

if plot_F:
    # Plot function eval history
    plt.plot([x + 1 for x in range(len(driver_cases))], hist_array[:, 6], '-o')
    plt.ylabel('W_takeoff [kg]')
    plt.xlabel('No of function evals')
    plt.title('MTOW Iteration')
    plt.grid()
    plt.show()

if plot_all:
    ylabels = ['Wing area [m2]', 'Wing aspect ratio', 'Lift rotor radius [m]',
               'Propeller radius [m]', 'Propeller advance ratio',
               'Cruise speed [m/s]', 'CL at cruise', 'Ct/solidity',
               'Disk loading hover [N/m2]', 'Disk loading cruise [N/m2]']
    fig, axs = plt.subplots(nrows=2, ncols=5, sharex=False)
    for i in range(5):
        axs[0, i].plot([x + 1 for x in range(len(driver_cases))], hist_array[:, i + 1], '-o')
        axs[0, i].grid()
        axs[0, i].set_ylabel(ylabels[i])
    for i in range(5):
        if i == 0:
            j = 0
        if i == 1:
            j = 7
        if i == 2:
            j = 10
        if i == 3:
            j = 8
        if i == 4:
            j = 9
        axs[1, i].plot([x + 1 for x in range(len(driver_cases))], hist_array[:, j], '-o')
        axs[1, i].grid()
        axs[1, i].set_ylabel(ylabels[i + 5])
        axs[1, i].set_xlabel('Iteration')
    plt.show()
