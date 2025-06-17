import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Structure.Fuselage import FuselageWeightRoskam, FuselageWeightM4ModelsForNASALPC
from MCEVS.Analyses.Weight.Structure.Landing_Gear import LandingGearWeightNDARCFractionalMethod
from MCEVS.Analyses.Weight.Structure.Wing import WingWeightRoskam, WingWeightM4ModelsForNASALPC
from MCEVS.Analyses.Weight.Structure.Tail import HorizontalTailWeightRoskam, VerticalTailWeightRoskam, EmpennageWeightM4ModelsForNASALPC
from MCEVS.Analyses.Weight.Structure.Boom import BoomWeightRoskam, BoomWeightM4ModelsForNASALPC

class StructureWeight(om.Group):
	"""
	Calculates structure weight
		W_structure = W_fuselage + W_landing_gear + W_wing + W_tails
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('tf_structure', types=float, desc='Technology factor for structure systems')

	def setup(self):

		vehicle = self.options['vehicle']
		fidelity = self.options['fidelity']
		tf_structure = self.options['tf_structure']

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

		if fidelity['weight_model']['structure'] == 'Roskam':

			# Fuselage weight
			self.add_subsystem('fuselage_weight',
								FuselageWeightRoskam(n_pax=n_pax,l_fuse=l_fuse,p_max=np.pi*d_max, tf=tf_structure),
								promotes_inputs=['Weight|takeoff'],
								promotes_outputs=['Weight|fuselage'])

			# Boom weight
			self.add_subsystem('boom_weight',
								BoomWeightRoskam(tf=tf_structure),
								promotes_inputs=['total_req_takeoff_power'],
								promotes_outputs=['Weight|boom'])

			# Wing weight
			# wing is possessed by lift+cruise, tiltrotor, and tiltiwing configurations
			if vehicle.configuration == 'LiftPlusCruise':
				self.add_subsystem('wing_weight',
									WingWeightRoskam(n_ult=n_ult_wing, tf=tf_structure),
									promotes_inputs=['Weight|takeoff', 'Wing|area', 'Wing|aspect_ratio'],
									promotes_outputs=['Weight|wing'])

			# Tails weight
			if vehicle.configuration == 'LiftPlusCruise':
				self.add_subsystem('htail_weight',
									HorizontalTailWeightRoskam(tf=tf_structure),
									promotes_inputs=['Weight|takeoff', 'HorizontalTail|area', 'HorizontalTail|aspect_ratio', 'HorizontalTail|max_root_thickness'],
									promotes_outputs=['Weight|horizontal_tail'])
				self.add_subsystem('vtail_weight',
									VerticalTailWeightRoskam(tf=tf_structure),
									promotes_inputs=['Weight|takeoff', 'VerticalTail|area', 'VerticalTail|aspect_ratio', 'VerticalTail|max_root_thickness', 'VerticalTail|sweep_angle'],
									promotes_outputs=['Weight|vertical_tail'])

		# Landing gear weight
		self.add_subsystem('landing_gear_weight',
							LandingGearWeightNDARCFractionalMethod(gear_type=gear_type, tf=tf_structure),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|landing_gear'])

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