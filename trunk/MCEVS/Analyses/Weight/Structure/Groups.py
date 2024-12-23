import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Structure.Fuselage import FuselageWeight
from MCEVS.Analyses.Weight.Structure.Landing_Gear import LandingGearWeightNDARCFractionalMethod
from MCEVS.Analyses.Weight.Structure.Wing import WingWeight
from MCEVS.Analyses.Weight.Structure.Tail import HorizontalTailWeight, VerticalTailWeight
from MCEVS.Analyses.Weight.Structure.Boom import BoomWeight

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
		gear_type 	= vehicle.landing_gear.gear_type
		l_sm 		= vehicle.landing_gear.strut_length
		n_ult_lg 	= vehicle.landing_gear.ultimate_load_factor

		# Wing parameter
		if vehicle.configuration == 'LiftPlusCruise':
			n_ult_wing 	= vehicle.wing.ultimate_load_factor

		# Horizontal tail parameter
		if vehicle.configuration == 'LiftPlusCruise':
			htail_AR 	= vehicle.horizontal_tail.aspect_ratio
			t_rh 		= vehicle.horizontal_tail.max_root_thickness
			htail_area 	= vehicle.horizontal_tail.area

		# Vertical tail parameter
		if vehicle.configuration == 'LiftPlusCruise':
			vtail_AR 	= vehicle.vertical_tail.aspect_ratio
			t_rv 		= vehicle.vertical_tail.max_root_thickness
			vtail_sweep = vehicle.vertical_tail.sweep_angle
			vtail_area 	= vehicle.vertical_tail.area

		# Fuselage weight
		self.add_subsystem('fuselage_weight',
							FuselageWeight(n_pax=n_pax,l_fuse=l_fuse,p_max=np.pi*d_max),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|fuselage'])

		# Landing gear weight
		self.add_subsystem('landing_gear_weight',
							LandingGearWeightNDARCFractionalMethod(gear_type=gear_type),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|landing_gear'])

		# Boom weight
		self.add_subsystem('boom_weight',
							BoomWeight(),
							promotes_inputs=['total_req_takeoff_power'],
							promotes_outputs=['Weight|boom'])

		# Wing weight
		# wing is possessed by lift+cruise, tiltrotor, and tiltiwing configurations
		if vehicle.configuration == 'LiftPlusCruise':
			self.add_subsystem('wing_weight',
								WingWeight(n_ult=n_ult_wing),
								promotes_inputs=['Weight|takeoff', 'Wing|area', 'Wing|aspect_ratio'],
								promotes_outputs=['Weight|wing'])

		# Tails weight
		if vehicle.configuration == 'LiftPlusCruise':
			indep = self.add_subsystem('tail_geom', om.IndepVarComp())
			indep.add_output('htail_area', val=htail_area, units='m**2')
			indep.add_output('vtail_area', val=vtail_area, units='m**2')
			self.add_subsystem('htail_weight',
								HorizontalTailWeight(htail_AR=htail_AR, t_rh=t_rh),
								promotes_inputs=['Weight|takeoff', ('Htail|area','tail_geom.htail_area')],
								promotes_outputs=['Weight|horizontal_tail'])
			self.add_subsystem('vtail_weight',
								VerticalTailWeight(vtail_AR=vtail_AR, t_rv=t_rv, vtail_sweep=vtail_sweep),
								promotes_inputs=['Weight|takeoff', ('Vtail|area','tail_geom.vtail_area')],
								promotes_outputs=['Weight|vertical_tail'])

		# Sum up
		if vehicle.configuration == 'Multirotor':
			input_names_list = ['Weight|fuselage', 'Weight|landing_gear', 'Weight|boom']
			sfs = [1., 1., 1.]
		elif vehicle.configuration == 'LiftPlusCruise':
			input_names_list = ['Weight|fuselage', 'Weight|landing_gear', 'Weight|wing', 'Weight|horizontal_tail', 'Weight|vertical_tail', 'Weight|boom']
			sfs = [1., 1., 1., 1., 1., 1.]
		adder = om.AddSubtractComp()
		adder.add_equation('Weight|structure',
							input_names=input_names_list,
							units='kg',
							scaling_factors=sfs)
		self.add_subsystem('structure_sum_weight',
							adder,
							promotes_inputs=['Weight|*'],
							promotes_outputs=['Weight|structure'])