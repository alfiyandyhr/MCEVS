import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

# Case reader
cr = om.CaseReader(f'multirotor_opt_cases.sql')

driver_cases = cr.list_cases('driver')

solidity = 1

# cur_case
hist_array = np.zeros((len(driver_cases),8))
for i, case in enumerate(driver_cases):
	cur_case = cr.get_case(case)
	dv1 = cur_case.get_design_vars()['Mission|cruise_speed'][0]
	dv2 = cur_case.get_design_vars()['LiftRotor|radius'][0]
	dv3 = cur_case.get_design_vars()['LiftRotor|advance_ratio'][0]
	F = cur_case.get_objectives()["Weight|takeoff"][0]
	G1 = cur_case.get_constraints()['DiskLoading|LiftRotor|segment_1'][0]
	G2 = cur_case.get_constraints()['DiskLoading|LiftRotor|segment_2'][0]
	G3 = cur_case.get_constraints()['LiftRotor|thrust_coefficient'][0]/solidity
	CV = max(G1-600, 0) + max(G2-600, 0) + max(G3-0.14, 0)
	cur_arr = np.array([dv1, dv2, dv3, F, G1, G2, G3, CV])
	hist_array[i] = cur_arr

plot_F = True
plot_all = True

if plot_F:
	# Plot function eval history
	plt.plot([x+1 for x in range(len(driver_cases))], hist_array[:,3], '-o')
	plt.ylabel('W_takeoff [kg]')
	plt.xlabel('No of function evals')
	plt.title('MTOW Iteration')
	plt.grid()
	plt.show()

if plot_all:
	ylabels = ['Cruise speed [m/s]', 'Lift rotor radius [m]', 'Lift rotor advance ratio',
			   'Disk loading hover [N/m2]', 'Disk loading cruise [N/m2]', 'Ct/solidity']
	fig, axs= plt.subplots(nrows=2, ncols=3, sharex=False)
	for i in range(3):
		axs[0,i].plot([x+1 for x in range(len(driver_cases))], hist_array[:,i], '-o')
		axs[0,i].grid()
		axs[0,i].set_xlabel('No of function evals')
		axs[0,i].set_ylabel(ylabels[i])
	for i in range(3):
		axs[1,i].plot([x+1 for x in range(len(driver_cases))], hist_array[:,i+4], '-o')
		axs[1,i].grid()
		axs[1,i].set_xlabel('No of function evals')
		axs[1,i].set_ylabel(ylabels[i+3])
	plt.show()







