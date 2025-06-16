import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import joblib
import numpy as np
from scipy.integrate import simpson, quad
from scipy.optimize import minimize

calc_utopian_J = False
plot_psi_for_every_year = True
compare_best_year_with_current_year = False

# 40 year lifespan of the product
year_list = np.arange(2030,2071,1)

def Boltzman_Sigmoid_Battery_Pack_GED(t, scenario):

	if scenario == 'conservative':
		models = [9.6, 659.1, 0.0436, 2042.5]
		derating_factor = 0.7
	elif scenario == 'nominal':
		models = [0.0, 801.8, 0.0607, 2030.8]
		derating_factor = 0.8
	elif scenario == 'aggresive':
		models = [32.5, 1009.0, 0.0786, 2030.6]
		derating_factor = 0.9

	a = models[0]; b = models[1]; c = models[2]; tau = models[3]
	res = a + b / (1 + np.exp(c * (tau - t)))
	
	return derating_factor*res

# Battery dictionary
battery_dict = {'conservative':None, 'nominal':None, 'aggresive':None}

for i, scenario in enumerate(battery_dict):
	battery_dict[scenario] = Boltzman_Sigmoid_Battery_Pack_GED(year_list, scenario)

def calc_psi_given_energy_year_data(energy_data:list or np.array, year_data:list or np.array, scenario:str, is_continouous:bool):
	opt_results = pd.read_csv('optimal_results_by_scenario_by_year.csv')
	opt_results = opt_results[opt_results['scenario'] == scenario]
	utopian_energy = opt_results['Energy|entire_mission'].to_numpy()
	J_tilde = simpson(utopian_energy, year_list)/(year_list[-1] - year_list[0])
	if is_continouous:
		J = simpson(energy_data, year_data)/(year_data[-1] - year_data[0])
	else:
		integrand = 0
		for i in range(len(energy_data)-1):
			integrand += (energy_data[i+1] + energy_data[i])/2 * (year_data[i+1] - year_data[i])
		J = integrand/(year_data[-1] - year_data[0])
	psi = J/J_tilde - 1
	return psi

if calc_utopian_J:

	opt_results = pd.read_csv('optimal_results_by_scenario_by_year.csv')

	for i, scenario in enumerate(battery_dict):
		opt_results_i = opt_results[opt_results['scenario'] == scenario]

		# Example data: x and y values
		y = opt_results_i['Energy|entire_mission'].to_numpy()

		# Using Simpson's rule
		time_averaged_J = simpson(y, year_list)/(year_list[-1] - year_list[0])
		print(f"Time-averaged J for scenario={scenario}: {time_averaged_J}")

	# # Results
	# time_averaged_J_conservative = 103.1168141777948
	# time_averaged_J_nominal = 88.53625756668183
	# time_averaged_J_aggresive = 83.01589005149472

if plot_psi_for_every_year:

	opt_test_results = pd.read_csv('opt_test_results.csv')

	psi_list = []

	for i, scenario in enumerate(battery_dict):

		opt_test_results_i = opt_test_results[opt_test_results['scenario']==scenario]

		psi_i = np.zeros(len(year_list))

		for j, year_opt in enumerate(year_list):

			y_array = opt_test_results_i[opt_test_results_i['year_opt']==year_opt]['Energy|entire_mission'].to_numpy()

			time_averaged_J = simpson(y_array, year_list)/(year_list[-1] - year_list[0])
			
			if scenario=='conservative': time_averaged_utopian_J = 103.1168141777948
			if scenario=='nominal': time_averaged_utopian_J = 88.53625756668183
			if scenario=='aggresive': time_averaged_utopian_J = 83.01589005149472

			psi_i[j] = time_averaged_J/time_averaged_utopian_J - 1

		psi_list.append(psi_i)

	fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=False)

	for i, scenario in enumerate(battery_dict):

		# Step 1: Find the index of the optimum value
		opt_index = np.argmin(psi_list[i])

		# Step 2: Extract the subarray starting after the optimum value
		subarray_after_opt = psi_list[i][opt_index + 1:]

		# Step 3: Find the first value greater than the first element of psi_list
		first_element = psi_list[i][0]
		for k, value in enumerate(subarray_after_opt):
			if value > first_element:
				# Step 4: Calculate the index in the original array
				threshold_index = opt_index + 1 + k
				break

		# Best year and threshold year
		year_opt_best_i = year_list[opt_index]
		year_threshold_i = year_list[threshold_index-1]

		# Plot data on each subplot
		axes[i].plot(year_list, psi_list[i], '-', markersize=4)
		axes[i].set_title(f'{scenario}')
		axes[i].set_xlabel('Optimization Year')
		axes[i].axvline(x=year_opt_best_i, color='k', linestyle='--', label=f'Optimum ({year_opt_best_i})')
		axes[i].ticklabel_format(style='sci', axis='y', scilimits=(-3,-3))
		if i==0: axes[i].set_ylabel(r'Future-aware optimality $\Psi$')
		axes[i].legend()

	plt.subplots_adjust(bottom=0.2, top=0.86, wspace=0.18)
	fig.suptitle(f'Future-aware optimality metrics under different battery projection scenarios', size=14)
	plt.show()

if compare_best_year_with_current_year:

	parameter = 'Energy|entire_mission'

	opt_test_results = pd.read_csv('opt_test_results.csv')
	utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')

	fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=False)

	for i, scenario in enumerate(battery_dict):

		opt_test_results_i = opt_test_results[opt_test_results['scenario']==scenario]
		utopian_data_i = utopian_data[utopian_data['scenario']==scenario]

		if scenario == 'conservative': best_year_i = 2034
		if scenario == 'nominal': best_year_i = 2034
		if scenario == 'aggresive': best_year_i = 2035

		best_data = opt_test_results_i[opt_test_results_i['year_opt']==best_year_i][parameter].to_numpy()
		opt_in_2030 = opt_test_results_i[opt_test_results_i['year_opt']==2030][parameter].to_numpy()
		opt_in_2070 = opt_test_results_i[opt_test_results_i['year_opt']==2070][parameter].to_numpy()

		# Psi calculations
		psi_best = calc_psi_given_energy_year_data(best_data, year_list, scenario, True)
		psi_2030 = calc_psi_given_energy_year_data(opt_in_2030, year_list, scenario, True)
		psi_2070 = calc_psi_given_energy_year_data(opt_in_2070, year_list, scenario, True)

		# Plot data on each subplot
		axes[i].plot(year_list, opt_in_2030, '-', label=fr'Opt in 2030; $\psi={np.round(psi_2030,4)}$')
		axes[i].plot(year_list, opt_in_2070, '-', label=fr'Opt in 2070; $\psi={np.round(psi_2070,4)}$')
		axes[i].plot(year_list, best_data, '-', label=fr'Min $\psi$ ({best_year_i}); $\psi={np.round(psi_best,4)}$')
		axes[i].plot(utopian_data_i['year'], utopian_data_i[parameter], 'k--', markersize=4, label=r'Utopian; $\psi=0.0$')
		axes[i].set_title(f'{scenario}')
		axes[i].set_xlabel('Sizing year')
		if i==0: axes[i].set_ylabel(r'Energy consumption $[kWh]$')
		axes[i].legend()

	# fig.legend(ncols=3,bbox_to_anchor=(0.49,0.93),loc='upper center')
	plt.subplots_adjust(bottom=0.2, top=0.86, wspace=0.18)
	fig.suptitle(f'Comparison between future-aware designs under different battery projection scenarios', size=14)
	plt.show()
