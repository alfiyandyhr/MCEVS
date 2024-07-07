import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

# Case reader
cr = om.CaseReader(f'liftcruise_opt_cases.sql')

driver_cases = cr.list_cases('driver')

solidity = 0.13

# cur_case
hist_array = np.zeros((len(driver_cases),11))
for i, case in enumerate(driver_cases):
	cur_case = cr.get_case(case)
	dv1 = cur_case.get_design_vars()['eVTOL|Cruise_speed'][0]
	dv2 = cur_case.get_design_vars()['eVTOL|S_wing'][0]
	dv3 = cur_case.get_design_vars()['Rotor|radius_lift'][0]
	dv4 = cur_case.get_design_vars()['Rotor|radius_cruise'][0]
	dv5 = cur_case.get_design_vars()['Rotor|J'][0]
	F = cur_case.get_objectives()["eVTOL|W_takeoff"][0]
	G1 = cur_case.get_constraints()['Aero|CL_cruise'][0]
	G2 = cur_case.get_constraints()['disk_loading_hover'][0]
	G3 = cur_case.get_constraints()['disk_loading_cruise'][0]
	G4 = cur_case.get_constraints()['Rotor|Ct'][0]/solidity
	CV = max(G1-0.6, 0) + max(G2-600, 0) + max(G3-600, 0) + max(G4-0.14, 0)
	cur_arr = np.array([dv1, dv2, dv3, dv4, dv5, F, G1, G2, G3, G4, CV])
	hist_array[i] = cur_arr

plot_F = True
plot_all = True

if plot_F:
	# Plot function eval history
	plt.plot([x+1 for x in range(len(driver_cases))], hist_array[:,5], '-o')
	plt.ylabel('W_takeoff [kg]')
	plt.xlabel('No of function evals')
	plt.title('MTOW Iteration')
	plt.grid()
	plt.show()

if plot_all:
	ylabels = ['Cruise speed [m/s]', 'Propeller advance ratio', 'CL at cruise',
			   'Wing area [m2]', 'Rotor radius lift [m]', 'Rotor radius cruise [m]',
			   'Disk loading hover [N/m2]', 'Disk loading cruise [N/m2]', 'Ct/solidity']
	fig, axs= plt.subplots(nrows=3, ncols=3, sharex=False)
	for i in range(3):
		if i == 0: j = 0
		if i == 1: j = 4
		if i == 2: j = 6
		axs[0,i].plot([x+1 for x in range(len(driver_cases))], hist_array[:,j], '-o')
		axs[0,i].grid()
		axs[0,i].set_ylabel(ylabels[i])
	for i in range(3):
		axs[1,i].plot([x+1 for x in range(len(driver_cases))], hist_array[:,i+1], '-o')
		axs[1,i].grid()
		axs[1,i].set_ylabel(ylabels[i+3])
	for i in range(3):
		axs[2,i].plot([x+1 for x in range(len(driver_cases))], hist_array[:,i+7], '-o')
		axs[2,i].grid()
		axs[2,i].set_xlabel('No of function evals')
		axs[2,i].set_ylabel(ylabels[i+6])
	plt.show()









