import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import simpson, quad
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.core.problem import Problem
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.repair.rounding import RoundingRepair
from pymoo.operators.sampling.rnd import IntegerRandomSampling
from pymoo.optimize import minimize

base_plan_years = [2030, 2040, 2050, 2060, 2070] # uniform spacing

optimal_plan_years = {'conservative': [2030, 2036, 2044, 2055, 2070],
					  'nominal': [2030, 2036, 2044, 2055, 2070],
					  'aggresive': [2030, 2036, 2043, 2053, 2070]} # GA-searched spacing

optimize_plan = False
plot_upgrade_plan = True

scenario_list = ['conservative', 'nominal', 'aggresive']

# design_years = ['utopian', 'utopian', 'utopian']
design_years = ['2044', '2043', '2041']

# 40 year lifespan of the product
year_list = np.arange(2030,2071,1)

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

def create_plan_data_given_plan_years_and_continous_plan_data(plan_years:list, parameter:str, continuous_plan_data:pd.DataFrame):
	"""
	continuous_plan_data = pd.DataFrame({'year_test':[...], 'parameter':[...]})
	"""
	year_data = []
	plan_data = []
	for j, year in enumerate(year_list):
		if year in plan_years:
			val = continuous_plan_data[continuous_plan_data['year_test']==year][parameter].to_numpy()[0]
			if j != 0:
				year_data.append(year)
				plan_data.append(prev_val)
			prev_val = val
		year_data.append(year)
		plan_data.append(val)

	year_data = np.array(year_data)
	plan_data = np.array(plan_data)

	return year_data, plan_data

if optimize_plan:
	
	def calc_psi_given_intermediate_plan_years(plan_years:np.array, scenario:str, continuous_plan_data:pd.DataFrame):
		
		if plan_years.ndim == 1: plan_years = np.concatenate(([2030], plan_years, [2070]))
		else: raise ValueError('Check your input dimension!')

		year_data, plan_data = create_plan_data_given_plan_years_and_continous_plan_data(plan_years, 'Energy|entire_mission', continuous_plan_data)
		psi = calc_psi_given_energy_year_data(plan_data, year_data, scenario, False)
		return psi

	class OptimalUpgradePlanProblem(Problem):

		def __init__(self, scenario:str, continuous_plan_data:pd.DataFrame):
			super().__init__(n_var=3, n_obj=1, n_ieq_constr=2, xl=2031, xu=2069, vtype=int)

			self.scenario = scenario
			self.continuous_plan_data = continuous_plan_data

		def _evaluate(self, x, out, *args, **kwargs):

			F = np.zeros((len(x), 1))
			G = np.zeros((len(x), 2))

			for i in range(len(x)):
				F[i][0] = calc_psi_given_intermediate_plan_years(x[i], self.scenario, self.continuous_plan_data)
				G[i][0] = x[i][0] - x[i][1]
				G[i][1] = x[i][1] - x[i][2]

			out["F"] = F
			out["G"] = G

	scenario = 'conservative'; design_year = 2044
	# scenario = 'nominal'; design_year = 2043
	# scenario = 'aggresive'; design_year = 2041

	opt_test_data = pd.read_csv('opt_test_results.csv')
	continuous_plan_data = opt_test_data[(opt_test_data['scenario']==scenario) & (opt_test_data['year_opt']==design_year)]

	# Test
	x = np.array([2040, 2050, 2060])
	psi = calc_psi_given_intermediate_plan_years(x, scenario, continuous_plan_data)
	# print(np.round(psi,4))

	# Optimization
	problem = OptimalUpgradePlanProblem(scenario, continuous_plan_data)

	method = GA(pop_size=20,
				sampling=IntegerRandomSampling(),
				crossover=SBX(prob=1.0, eta=3.0, vtype=float, repair=RoundingRepair()),
				mutation=PM(prob=1.0, eta=3.0, vtype=float, repair=RoundingRepair()),
				eliminate_duplicates=True)

	res = minimize(problem, method, termination=('n_gen', 100), seed=1, save_history=True, verbose=True)

	print("Best solution found: %s" % res.X)
	print("Function value: %s" % res.F)
	print("Constraint violation: %s" % res.CV)

if plot_upgrade_plan:

	parameter = 'Energy|entire_mission'

	utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')
	opt_test_data = pd.read_csv('opt_test_results.csv')

	fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=True)

	for i, scenario in enumerate(scenario_list):

		if design_years[i] == 'utopian':
			data = utopian_data[utopian_data['scenario']==scenario]
			continuous_psi = calc_psi_given_energy_year_data(data[parameter], year_list, scenario, True)
			axes[i].plot(data['year'], data[parameter], 'k--', markersize=4, label=fr'Utopian; $\psi={continuous_psi}$')
			data = data.rename(columns={'year':'year_test'})
		
		else:
			data = utopian_data[utopian_data['scenario']==scenario]
			continuous_psi = calc_psi_given_energy_year_data(data[parameter], year_list, scenario, True)
			axes[i].plot(data['year'], data[parameter], 'k--', markersize=4, label=fr'Utopian; $\psi={continuous_psi}$')

			data = opt_test_data[(opt_test_data['scenario']==scenario) & (opt_test_data['year_opt']==int(design_years[i]))]
			continuous_psi = calc_psi_given_energy_year_data(data[parameter], year_list, scenario, True)
			axes[i].plot(data['year_test'], data[parameter], '-', color='grey', markersize=4, label=fr'Opt {design_years[i]}; $\psi={np.round(continuous_psi,4)}$')

		# Base plan
		base_year_data, base_plan_data = create_plan_data_given_plan_years_and_continous_plan_data(base_plan_years, parameter, data)
		base_psi = calc_psi_given_energy_year_data(base_plan_data, base_year_data, scenario, False)
		axes[i].plot(base_year_data, base_plan_data, 'r-', markersize=4, label=fr'Uniform; $\psi={np.round(base_psi,4)}$')

		# Optimal plan
		optimal_year_data, optimal_plan_data = create_plan_data_given_plan_years_and_continous_plan_data(optimal_plan_years[scenario], parameter, data)
		optimal_psi = calc_psi_given_energy_year_data(optimal_plan_data, optimal_year_data, scenario, False)
		axes[i].plot(optimal_year_data, optimal_plan_data, 'b-', markersize=4, label=fr'Optimal; $\psi={np.round(optimal_psi,4)}$')

		axes[i].legend()
		axes[i].set_title(f'{scenario}')
		axes[i].set_xlabel('Sizing year')
		if i==0: axes[i].set_ylabel(r'Energy consumption $[kWh]$')

	plt.subplots_adjust(bottom=0.2, top=0.86, wspace=0.08)
	fig.suptitle(f'Optimal upgrade timing plan under different battery projection scenarios', size=14)
	plt.show()















