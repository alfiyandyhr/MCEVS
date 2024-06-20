import numpy as np
import openmdao.api as om

from AVTOL.Weights.Structure.Fuselage import FuselageWeight
from AVTOL.Weights.Structure.Landing_Gear import LandingGearWeight
from AVTOL.Weights.Structure.Wing import WingWeight

class StructureWeight(om.Group):
	"""
	Calculates structure weight
		W_structure = W_fuselage + W_landing_gear + W_wing + W_tails
	"""
	def initialize(self):
		self.options.declare('params', types=dict, desc='A dictionary that includes important parameters')

	def setup(self):
		params = self.options['params']

		# Fuselage weight
		self.add_subsystem('fuselage_weight',
							FuselageWeight(n_pax=params['n_pax'],l_fuse=params['l_fuse'],p_max=np.pi*params['d_fuse_max']),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Fuselage'])

		# Landing gear weight
		self.add_subsystem('landing_gear_weight',
							LandingGearWeight(l_sm=params['l_sm'],n_ult=params['n_ult_lg']),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Landing_gear'])

		# Wing weight
		# wing is possessed by lift+cruise, tiltrotor, and tiltiwing configurations
		if params['evtol_config'] == 'lift+cruise':
			self.add_subsystem('wing_weight',
								WingWeight(wing_AR=params['wing_AR'],n_ult=params['n_ult_wing']),
								promotes_inputs=['eVTOL|W_takeoff', 'eVTOL|S_wing'],
								promotes_outputs=['Weights|Wing'])

		# Tails weight
		pass

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weights|Structure',
							input_names=['Weights|Fuselage', 'Weights|Landing_gear', 'Weights|Wing'],
							units='kg',
							scaling_factors=[1., 1., 1.])
		self.add_subsystem('structure_sum_weight',
							adder,
							promotes_inputs=['Weights|*'],
							promotes_outputs=['Weights|Structure'])