import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import simpson
from datetime import datetime

plot_time_averaged_J = True

metrics = ['weight', 'energy']
configs = ['multirotor', 'liftcruise']
scenarios = ['conservative', 'nominal', 'aggresive']

time_averaged_J_dict = {}

for metric in metrics:
	time_averaged_J_dict[metric] = {}

	for config in configs:
		time_averaged_J_dict[metric][config] = {}
		
		utopian_data = pd.read_csv(f'optimal_results_by_scenario_by_year/{metric}_opt_{config}_results.csv')
		utopian_data = utopian_data[utopian_data['success']==True]
		if metric == 'weight': metric_tag = 'Weight|takeoff'
		elif metric == 'energy': metric_tag = 'Energy|entire_mission'

		print(f'Time-averaged J for {metric}-minimal {config}:')

		for scenario in scenarios:
			year_list = utopian_data[utopian_data['scenario']==scenario]['year'].to_numpy()

			y = utopian_data[utopian_data['scenario']==scenario][metric_tag].to_numpy()

			# Using Simpson's rule
			time_averaged_J = simpson(y, year_list)/(year_list[-1] - year_list[0])

			# time_averaged_J_dict = time_averaged_J
			time_averaged_J_dict[metric][config][scenario] = time_averaged_J

			print(f'\tscenario= {scenario}; J= {time_averaged_J}')

if plot_time_averaged_J:

	fig, axes = plt.subplots(2,2,figsize=(9,6),sharex=True)

	for i, metric in enumerate(metrics):
		for j, config in enumerate(configs):
			time_averaged_J_list = list(time_averaged_J_dict[metric][config].values())
			axes[i,j].plot(scenarios, time_averaged_J_list, 'o-', markersize=4)
			if i == 1: axes[i,j].set_xlabel('Scenarios')

	axes[0,0].set_ylabel(r'Time-averaged weight $[kg]$')
	axes[1,0].set_ylabel(r'Time-averaged energy $[kWh]$')
	axes[0,0].set_title('Weight-minimal multirotor',size=11)
	axes[0,1].set_title('Weight-minimal lift+cruise',size=11)
	axes[1,0].set_title('Energy-minimal multirotor',size=11)
	axes[1,1].set_title('Energy-minimal lift+cruise',size=11)
	fig.suptitle(r'Time-averaged metric $J$ for different configurations and design objectives',size=13)
	plt.tight_layout(rect=(0,0,1,0.98))
	plt.show()






