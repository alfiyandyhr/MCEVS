from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.problems import get_problem
from pymoo.optimize import minimize
import numpy as np
from pymoo.core.problem import Problem

class MyProblem(Problem):

	def __init__(self, n_var, n_obj, xl, xu, DP, dv_names):
		super().__init__(n_var=n_var,
						 n_obj=n_obj,
						 xl=xl,
						 xu=xu)
		self.DP = DP
		self.dv_names = dv_names

	def _evaluate(self, x, out, *args, **kwargs):
		# F = 100 * (x[:, 0]**2 + x[:, 1]**2)

		F = np.zeros(len(x))

		for i in range(len(x)):
			design_var = {}
			for n, dv_name in enumerate(self.dv_names):
				design_var[dv_name] = x[i, n]
			mtow = self.DP.evaluate_one_sample(design_var)
			F[i] = mtow

		print(f'mtow = {np.min(F)}')

		out["F"] = np.column_stack([F])

def run_gradient_free_optimization(DP:object):

	n_var = len(DP.design_variables)
	# n_obj = len(DP.objectives)

	xl = np.zeros(n_var)
	xu = np.zeros(n_var)
	dv_names = []
	for i, dv_name in enumerate(DP.design_variables):
		if dv_name == 'Mission|cruise_speed': dv_names.append('cruise_speed')
		elif dv_name == 'Wing|area': dv_names.append('wing_area')
		elif dv_name == 'Wing|aspect_ratio': dv_names.append('wing_aspect_ratio')
		elif dv_name == 'LiftRotor|radius': dv_names.append('r_lift_rotor')
		elif dv_name == 'Propeller|radius': dv_names.append('r_propeller')
		elif dv_name == 'Propeller|advance_ratio': dv_names.append('propeller_advance_ratio')
		
		xl[i] = DP.design_variables[dv_name][0]
		xu[i] = DP.design_variables[dv_name][1]

	problem = MyProblem(n_var=n_var, n_obj=1, xl=xl, xu=xu, DP=DP, dv_names=dv_names)

	algorithm = NSGA2(pop_size=20)

	res = minimize(problem,
				   algorithm,
				   ('n_gen', 50),
				   seed=1,
				   verbose=True)

