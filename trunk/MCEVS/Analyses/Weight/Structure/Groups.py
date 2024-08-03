import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Structure.Fuselage import FuselageWeight
from MCEVS.Analyses.Weight.Structure.Landing_Gear import LandingGearWeight
from MCEVS.Analyses.Weight.Structure.Wing import WingWeight

class StructureWeight(om.Group):
	"""
	Calculates structure weight
		W_structure = W_fuselage + W_landing_gear + W_wing + W_tails
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')

	def setup(self):

		vehicle = self.options['vehicle']

		# Fuselage parameter
		n_pax 	= vehicle.fuselage.number_of_passenger
		l_fuse 	= vehicle.fuselage.length
		d_max	= vehicle.fuselage.max_diameter
		
		# Landing gear parameter
		l_sm 		= vehicle.landing_gear.strut_length
		n_ult_lg 	= vehicle.landing_gear.ultimate_load_factor

		# Wing parameter
		if vehicle.configuration == 'LiftPlusCruise':
			n_ult_wing 	= vehicle.wing.ultimate_load_factor

		# Fuselage weight
		self.add_subsystem('fuselage_weight',
							FuselageWeight(n_pax=n_pax,l_fuse=l_fuse,p_max=np.pi*d_max),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|fuselage'])

		# Landing gear weight
		self.add_subsystem('landing_gear_weight',
							LandingGearWeight(l_sm=l_sm,n_ult=n_ult_lg),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|landing_gear'])

		# Wing weight
		# wing is possessed by lift+cruise, tiltrotor, and tiltiwing configurations
		if vehicle.configuration == 'LiftPlusCruise':
			self.add_subsystem('wing_weight',
								WingWeight(n_ult=n_ult_wing),
								promotes_inputs=['Weight|takeoff', 'Wing|area', 'Wing|aspect_ratio'],
								promotes_outputs=['Weight|wing'])

		# Tails weight
		pass

		# Sum up
		if vehicle.configuration == 'Multirotor':
			input_names_list = ['Weight|fuselage', 'Weight|landing_gear']
			sfs = [1., 1.,]
		elif vehicle.configuration == 'LiftPlusCruise':
			input_names_list = ['Weight|fuselage', 'Weight|landing_gear', 'Weight|wing']	
			sfs = [1., 1., 1.]
		adder = om.AddSubtractComp()
		adder.add_equation('Weight|structure',
							input_names=input_names_list,
							units='kg',
							scaling_factors=sfs)
		self.add_subsystem('structure_sum_weight',
							adder,
							promotes_inputs=['Weight|*'],
							promotes_outputs=['Weight|structure'])