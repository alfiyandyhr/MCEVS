import numpy as np
import openmdao.api as om

class LiftRotorClearanceConstraint(om.ExplicitComponent):
	"""
	Computes the spanwise spacing constraint of lift rotors
	Parameter:
		N_rotor					: number of (lift) rotors
		max_d_fuse 				: fuselage max diameter [m]
		percent_max_span 		: maximum span percentage to accommodate lift rotors and their clearance (default=95)
	Inputs:
		LiftRotor|radius		: lift rotor's radius [m]
		Wing|area 				: wing area [m**2]
		Wing|aspect_ratio 		: wing aspect ratio
	Outputs:
		clearance_constraint
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of (lift) rotors')
		self.options.declare('max_d_fuse', types=float, desc='Fuselage max diameter')
		self.options.declare('percent_max_span', types=float, desc='Maximum span percentage to accommodate lift rotors and their clearance')

	def setup(self):
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_input('Wing|area', units='m**2', desc='Wing area')
		self.add_input('Wing|aspect_ratio', units=None, desc='Wing aspect ratio')
		self.add_output('clearance_constraint', desc='Spanwise clearance constraint')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		max_d_fuse = self.options['max_d_fuse']
		percent_max_span = self.options['percent_max_span']
		r_lift_rotor = inputs['LiftRotor|radius']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		b = np.sqrt(S_wing * AR_wing) # in [m]

		outputs['clearance_constraint'] = (N_rotor/2 - 1) * 2*r_lift_rotor + max_d_fuse - percent_max_span/100 * b

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		max_d_fuse = self.options['max_d_fuse']
		percent_max_span = self.options['percent_max_span']
		r_lift_rotor = inputs['LiftRotor|radius']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		b = np.sqrt(S_wing * AR_wing) # in [m]

		db_dS = 0.5 * (S_wing * AR_wing)**(-0.5) * AR_wing
		db_dAR = 0.5 * (S_wing * AR_wing)**(-0.5) * S_wing

		partials['clearance_constraint', 'LiftRotor|radius'] = (N_rotor/2 - 1) * 2
		partials['clearance_constraint', 'Wing|area'] = - percent_max_span/100 * db_dS
		partials['clearance_constraint', 'Wing|aspect_ratio'] = - percent_max_span/100 * db_dAR