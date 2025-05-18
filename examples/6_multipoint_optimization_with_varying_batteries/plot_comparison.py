import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

with_fixed_empty_weight = True

plot_objective_all = True
plot_objective_separately = False
plot_constraint_separately = False

battery_energy_density_list = np.arange(250,725,25)
selected_battery_energy_density_list = [250,400,550]

weight_coeffs_options = {'uniform': [0.25, 0.25, 0.25, 0.25],
						 'linear': [0.10, 0.20, 0.30, 0.40]}

folder = 'with_fixed_empty_weight' if with_fixed_empty_weight else 'without_fixed_empty_weight'

if plot_objective_all or plot_objective_separately or plot_constraint_separately:

	opt_results_list = []
	opt_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_weight/multirotor/optimization_results.csv'))
	opt_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_energy/multirotor/optimization_results.csv'))
	opt_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_weight/liftcruise/optimization_results.csv'))
	opt_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_energy/liftcruise/optimization_results.csv'))
	test_results_list = []
	test_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_weight/multirotor/test_results.csv'))
	test_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_energy/multirotor/test_results.csv'))
	test_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_weight/liftcruise/test_results.csv'))
	test_results_list.append(pd.read_csv(f'../4_suboptimal_performance_evaluation/{folder}/selection_by_energy/liftcruise/test_results.csv'))
	multipoint_test_results_list = []
	multipoint_test_results_list.append(pd.read_csv(f'{folder}/weighted_sum_of_takeoff_weight/multirotor/multipoint_test_results.csv'))
	multipoint_test_results_list.append(pd.read_csv(f'{folder}/weighted_sum_of_energy/multirotor/multipoint_test_results.csv'))
	multipoint_test_results_list.append(pd.read_csv(f'{folder}/weighted_sum_of_takeoff_weight/liftcruise/multipoint_test_results.csv'))
	multipoint_test_results_list.append(pd.read_csv(f'{folder}/weighted_sum_of_energy/liftcruise/multipoint_test_results.csv'))

	metrics = ['Weight|gross_takeoff' if with_fixed_empty_weight else 'Weight|takeoff','Energy|entire_mission']
	configs = ['Multirotor','Lift+Cruise']

	for i,opt_results in enumerate(opt_results_list):
		for metric in metrics:
			if i in [0,1]:		# multirotor
				for j in range(4):
					opt_results.loc[opt_results['Weight|residual'] > 0.1, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|Cruise|T_to_P'] > 12.01, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|Cruise|mu'] > 1, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|Cruise|CT/sigma'] > 0.141, metric] = np.nan
			elif i in [2,3]:	# lift+cruise
				for j in range(4):
					opt_results.loc[opt_results['Weight|residual']>0.1, metric] = np.nan
					opt_results.loc[opt_results['Aero|Cruise|CL']>0.91, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P']>12.01, metric] = np.nan
					opt_results.loc[opt_results['Propeller|Cruise|T_to_P']>12.01, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P']>12.01, metric] = np.nan
					opt_results.loc[opt_results['Propeller|Cruise|J']>3.01, metric] = np.nan
					opt_results.loc[opt_results['Propeller|Cruise|CT/sigma']>0.141, metric] = np.nan
					opt_results.loc[opt_results['LiftRotor|clearance_constraint']>0.1, metric] = np.nan

	for i,test_results in enumerate(test_results_list):
		if i in [0,2]: metric = 'Weight|gross_takeoff' if with_fixed_empty_weight else 'Weight|takeoff'
		if i in [1,3]: metric = 'Energy|entire_mission'
		if with_fixed_empty_weight:
			condition1 = (test_results['Weight|max_takeoff'] < test_results['Weight|gross_takeoff'])
			condition2 = (test_results['battery_energy_density_opt'] != test_results['battery_energy_density_test'])
			test_results[metric] = np.where(condition1 & condition2, np.nan, test_results[metric])
		if i in [0,1]:		# multirotor
			test_results.loc[test_results['Weight|residual'] > 0.1, metric] = np.nan
			test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
			test_results.loc[test_results['LiftRotor|Cruise|T_to_P'] > 12.01, metric] = np.nan
			test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
			test_results.loc[test_results['LiftRotor|Cruise|mu'] > 1, metric] = np.nan
			test_results.loc[test_results['LiftRotor|Cruise|CT/sigma'] > 0.141, metric] = np.nan
		elif i in [2,3]:	# lift+cruise
			test_results.loc[test_results['Weight|residual']>0.1, metric] = np.nan
			test_results.loc[test_results['Aero|Cruise|CL']>0.91, metric] = np.nan
			test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P']>12.01, metric] = np.nan
			test_results.loc[test_results['Propeller|Cruise|T_to_P']>12.01, metric] = np.nan
			test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P']>12.01, metric] = np.nan
			test_results.loc[test_results['Propeller|Cruise|J']>3.01, metric] = np.nan
			test_results.loc[test_results['Propeller|Cruise|CT/sigma']>0.141, metric] = np.nan
			test_results.loc[test_results['LiftRotor|clearance_constraint']>0.1, metric] = np.nan

	for i,multipoint_test_results in enumerate(multipoint_test_results_list):
		if i in [0,2]: metric = 'Weight|gross_takeoff' if with_fixed_empty_weight else 'Weight|takeoff'
		if i in [1,3]: metric = 'Energy|entire_mission'
		if with_fixed_empty_weight:
			condition1 = (test_results['Weight|max_takeoff'] < test_results['Weight|gross_takeoff'])
			condition2 = (test_results['battery_energy_density_opt'] != test_results['battery_energy_density_test'])
			test_results[metric] = np.where(condition1 & condition2, np.nan, test_results[metric])
		if i in [0,1]:		# multirotor
			multipoint_test_results.loc[multipoint_test_results['Weight|residual'] > 0.1, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|T_to_P'] > 12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|mu'] > 1, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|CT/sigma'] > 0.141, metric] = np.nan
		elif i in [2,3]:	# lift+cruise
			multipoint_test_results.loc[multipoint_test_results['Weight|residual']>0.1, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['Aero|Cruise|CL']>0.91, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverClimb|T_to_P']>12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['Propeller|Cruise|T_to_P']>12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverDescent|T_to_P']>12.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['Propeller|Cruise|J']>3.01, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['Propeller|Cruise|CT/sigma']>0.141, metric] = np.nan
			multipoint_test_results.loc[multipoint_test_results['LiftRotor|clearance_constraint']>0.1, metric] = np.nan

	if plot_objective_all:
		fig, axes = plt.subplots(2,2,figsize=(9,6),sharex=True)

		for i,metric in enumerate(metrics):
			for j,config in enumerate(configs):
				opt_results = opt_results_list[i+j*2] # iterating opt results based on the order of pd.read_csv()
				test_results = test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
				multipoint_test_results = multipoint_test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
				for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
					test_results_k = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][metric]
					axes[i,j].plot(battery_energy_density_list, test_results_k, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[k]} Wh/kg' if i==0 and j==0 else None)
				
				for l, coeff_type in enumerate(weight_coeffs_options):
					if coeff_type == 'uniform':
						multipoint_test_results_l = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][metric]
						axes[i,j].plot(battery_energy_density_list, multipoint_test_results_l, 'o-', ms=4.0, label=f'Multipoint coeff={coeff_type}' if i==0 and j==0 else None)
					if coeff_type == 'linear':
						multipoint_test_results_l = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][metric]
						axes[i,j].plot(battery_energy_density_list, multipoint_test_results_l, 'x-', label=f'Multipoint coeff={coeff_type}' if i==0 and j==0 else None)

				axes[i,j].plot(opt_results['battery_energy_density'], opt_results['Weight|takeoff' if metric == 'Weight|gross_takeoff' or metric == 'Weight|takeoff' else 'Energy|entire_mission'], 'k--', label=f'Optimized at test battery GED' if i==0 and j==0 else None)

				if i == 1: axes[i,j].set_xlabel(r'Test battery GED $[Wh/kg]$')
		
		# Get the handles and labels
		handles, labels = axes[0,0].get_legend_handles_labels()

		# Reorder the labels
		order = [0,1,2,5,3,4]
		handles = [handles[i] for i in order]
		labels = [labels[i] for i in order]

		axes[0,0].set_ylabel(r'Gross takeoff weight $[kg]$' if with_fixed_empty_weight else r'Maximum takeoff weight $[kg]$')
		axes[1,0].set_ylabel(r'Required energy $[kWh]$')
		axes[0,0].set_title('Weight-minimized multirotor',size=11)
		axes[0,1].set_title('Weight-minimized lift+cruise',size=11)
		axes[1,0].set_title('Energy-minimized multirotor',size=11)
		axes[1,1].set_title('Energy-minimized lift+cruise',size=11)
		fig.legend(handles=handles, labels=labels, loc='upper center', bbox_to_anchor=(0.505, 0.95), ncols=3)
		fig.suptitle('Suboptimal performance evaluation on various test battery GEDs',size=13)
		plt.tight_layout(rect=(0,0,1,0.92))
		plt.show()

	if plot_objective_separately:

		for i,metric in enumerate(metrics):
			for j,config in enumerate(configs):
				opt_results = opt_results_list[i+j*2] # iterating opt results based on the order of pd.read_csv()
				test_results = test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
				multipoint_test_results = multipoint_test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
				for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
					test_results_k = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][metric]
					plt.plot(battery_energy_density_list, test_results_k, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[k]} Wh/kg')
				
				for l, coeff_type in enumerate(weight_coeffs_options):
					if coeff_type == 'uniform':
						multipoint_test_results_l = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][metric]
						plt.plot(battery_energy_density_list, multipoint_test_results_l, 'o-', ms=4.0, label=f'Multipoint coeff={coeff_type}')
					if coeff_type == 'linear':
						multipoint_test_results_l = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][metric]
						plt.plot(battery_energy_density_list, multipoint_test_results_l, 'x-', label=f'Multipoint coeff={coeff_type}')

				plt.plot(opt_results['battery_energy_density'], opt_results['Weight|takeoff' if metric == 'Weight|gross_takeoff' or metric == 'Weight|takeoff' else 'Energy|entire_mission'], 'k--', ms=4.0, label=f'Optimized at test battery GED')
		
				plt.xlabel(r'Test battery GED $[Wh/kg]$')
				if metric == 'Weight|takeoff' or metric == 'Weight|gross_takeoff':
					plt.ylabel(r'Gross takeoff weight $[kg]$' if with_fixed_empty_weight else r'Maximum takeoff weight $[kg]$')
					plt.title(f'Suboptimal performance evaluation\nfor {config} from weight-based optimization')
				if metric == 'Energy|entire_mission':
					plt.ylabel(r'Required energy $[kWh]$')
					plt.title(f'Suboptimal performance evaluation\nfor {config} from energy-based optimization')
				plt.legend()
				plt.show()

	if plot_constraint_separately:

		for i,metric in enumerate(metrics):
			for j,config in enumerate(configs):

				test_results = test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
				multipoint_test_results = multipoint_test_results_list[i+j*2] # iterating test results based on the order of pd.read_csv()
	
				xmin = battery_energy_density_list[0]
				xmax = battery_energy_density_list[-1]

				if 2*i+j in [0,2]:

					# skewed data
					if 2*i+j == 0:
						test_results.loc[(test_results['battery_energy_density_opt']==850) & (test_results['battery_energy_density_test']==250), 'Weight|residual'] = np.nan

					ylabels = ['LiftRotor|Cruise|CT/sigma','LiftRotor|Cruise|mu','Weight|residual',
							   'LiftRotor|HoverClimb|T_to_P','LiftRotor|Cruise|T_to_P','LiftRotor|HoverDescent|T_to_P']
					ylabel_names = [r'Blade loading $CT/\sigma$', r'Advance ratio $\mu$',r'Squared residual $[kg^2]$',r'$T/P$ HoverClimb $[g/W]$',r'$T/P$ Cruise $[g/W]$',r'$T/P$ HoverDescent $[g/W]$']

					fig, axes = plt.subplots(2, 3, sharex=True, figsize=(12,7))
					axes = axes.flatten()

					for l, ylabel in enumerate(ylabels):
						for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
							test_results_k = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][ylabel]
							axes[l].plot(battery_energy_density_list, test_results_k, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if l==0 else None)
							if ylabel == 'LiftRotor|Cruise|CT/sigma': axes[l].hlines(0.14,xmin=xmin,xmax=xmax,linestyles='--',color='k',label='Constraint upper limit' if k==2 else None)
							if ylabel == 'LiftRotor|Cruise|mu': axes[l].hlines(1.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|HoverClimb|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|Cruise|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|HoverDescent|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if l in [3,4,5]: axes[l].set_xlabel('Test battery GED [Wh/kg]')
							axes[l].set_ylabel(ylabel_names[l])
					
						for m, coeff_type in enumerate(weight_coeffs_options):
							if coeff_type == 'uniform':
								multipoint_test_results_m = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][ylabel]
								axes[l].plot(battery_energy_density_list, multipoint_test_results_m, 'o-', ms=3.0, label=f'Multipoint coeff={coeff_type}' if l==0 else None)
							if coeff_type == 'linear':
								multipoint_test_results_m = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][ylabel]
								axes[l].plot(battery_energy_density_list, multipoint_test_results_m, 'x-', label=f'Multipoint coeff={coeff_type}' if l==0 else None)

					axes[2].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
					if 2*i+j == 0: fig.suptitle('Constraint evaluation with varying battery GED for Multirotor from weight-based optimization')
					if 2*i+j == 2: fig.suptitle('Constraint evaluation with varying battery GED for Multirotor from energy-based optimization')
					fig.legend(ncols=3,bbox_to_anchor=(0.49,0.95),loc='upper center')
					plt.tight_layout(rect=(0,0,1,0.93))
					plt.show()

				if 2*i+j in [1,3]:

					ylabels = ['Aero|Cruise|CL','Propeller|Cruise|CT/sigma','Propeller|Cruise|J','Weight|residual',
							   'LiftRotor|HoverClimb|T_to_P','Propeller|Cruise|T_to_P','LiftRotor|HoverDescent|T_to_P','LiftRotor|clearance_constraint']
					ylabel_names = [r'Lift coeff $C_{L}$',r'Prop blade loading $CT/\sigma$',r'Prop advance ratio $J$',r'Squared residual $[kg^2]$',
									r'$T/P$ HoverClimb $[g/W]$',r'$T/P$ Cruise $[g/W]$',r'$T/P$ HoverDescent $[g/W]$',r'Clearance constraint $[m]$']

					fig, axes = plt.subplots(2, 4, sharex=True, figsize=(14,7))
					axes = axes.flatten()

					for l, ylabel in enumerate(ylabels):
						for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
							test_results_k = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][ylabel]
							axes[l].plot(battery_energy_density_list, test_results_k, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if l==0 else None)
							
							if ylabel == 'Aero|Cruise|CL': axes[l].hlines(0.9,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'Propeller|Cruise|CT/sigma': axes[l].hlines(0.14,xmin=xmin,xmax=xmax,linestyles='--',color='k',label='Constraint upper limit' if k==1 else None)
							if ylabel == 'Propeller|Cruise|J': axes[l].hlines(3.001,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|HoverClimb|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'Propeller|Cruise|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|HoverDescent|T_to_P': axes[l].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
							if ylabel == 'LiftRotor|clearance_constraint': axes[l].hlines(1e-5,xmin=xmin,xmax=xmax,linestyles='--',color='k')

							if l in [4,5,6,7]: axes[l].set_xlabel('Test battery GED [Wh/kg]')
							axes[l].set_ylabel(ylabel_names[l])

						for m, coeff_type in enumerate(weight_coeffs_options):
							if coeff_type == 'uniform':
								multipoint_test_results_m = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][ylabel]
								axes[l].plot(battery_energy_density_list, multipoint_test_results_m, 'o-', ms=3.0, label=f'Multipoint coeff={coeff_type}' if l==1 else None)
							if coeff_type == 'linear':
								multipoint_test_results_m = multipoint_test_results[multipoint_test_results['coeff_type']==coeff_type][ylabel]
								axes[l].plot(battery_energy_density_list, multipoint_test_results_m, 'x-', label=f'Multipoint coeff={coeff_type}' if l==1 else None)

					axes[3].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
					if 2*i+j == 1: fig.suptitle('Constraint evaluation with varying battery GED for Lift+Cruise from weight-based optimization')
					if 2*i+j == 3: fig.suptitle('Constraint evaluation with varying battery GED for Lift+Cruise from energy-based optimization')
					fig.legend(ncols=3,bbox_to_anchor=(0.50,0.95),loc='upper center')
					plt.tight_layout(rect=(0,0,1,0.93))
					plt.show()