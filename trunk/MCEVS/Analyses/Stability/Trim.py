import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Aerodynamics.LiftingLineTheory import FiniteWingLiftCoefficientCurveSlope


class MultirotorConstantClimbTrim(om.ExplicitComponent):
	"""
	Computes the thrust and rotor tilt angle during climb
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during climb [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
		Aero|total_drag : drag of a multirotor body [N]
	Outputs:
		Thrust 			: total thrust required as a vehicle [N]
		Body|cos_delta	: angle between thrust and x-axis
	"""
	def initialize(self):
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during climb')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Drag of a multirotor body')
		self.add_output('Thrust', units='N', desc='Thrust required as a vehicle')
		self.add_output('Body|cos_delta', desc='Angle between thrust and x-axis')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		D = inputs['Aero|total_drag']			# in [N]
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		thrust = np.sqrt((D * np.cos(gamma))**2 + (D * np.sin(gamma) + W_takeoff * g)**2)

		outputs['Thrust'] = thrust
		outputs['Body|cos_delta'] = D * np.cos(gamma) / thrust

	def compute_partials(self, inputs, partials):
		D = inputs['Aero|total_drag']			# in [N]
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		thrust = np.sqrt((D * np.cos(gamma))**2 + (D * np.sin(gamma) + W_takeoff * g)**2)
		dt_dm = (D * np.sin(gamma) + W_takeoff * g) * g / thrust
		dt_dD = (D * np.cos(gamma) * np.cos(gamma) + (D * np.sin(gamma) + W_takeoff * g) * np.sin(gamma)) / thrust

		partials['Thrust', 'Weight|takeoff'] = dt_dm
		partials['Thrust', 'Aero|total_drag'] = dt_dD
		partials['Body|cos_delta', 'Weight|takeoff'] = - D * np.cos(gamma) / (thrust**2) * dt_dm
		partials['Body|cos_delta', 'Aero|total_drag'] = np.cos(gamma) / thrust - D * np.cos(gamma) / (thrust**2) * dt_dD


class MultirotorConstantDescentTrim(om.ExplicitComponent):
	"""
	Computes the thrust and rotor tilt angle during descent
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during climb [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
		Aero|total_drag : drag of a multirotor body [N]
	Outputs:
		Thrust 			: total thrust required as a vehicle [N]
		Body|cos_delta	: angle between thrust and x-axis
	"""
	def initialize(self):
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during climb')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Drag of a multirotor body')
		self.add_output('Thrust', units='N', desc='Thrust required as a vehicle')
		self.add_output('Body|cos_delta', desc='Angle between thrust and x-axis')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		D = inputs['Aero|total_drag']			# in [N]
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		thrust = np.sqrt((D * np.cos(gamma))**2 + (W_takeoff * g - D * np.sin(gamma))**2)

		outputs['Thrust'] = thrust
		outputs['Body|cos_delta'] = D * np.cos(gamma) / thrust

	def compute_partials(self, inputs, partials):
		D = inputs['Aero|total_drag']			# in [N]
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		thrust = np.sqrt((D * np.cos(gamma))**2 + (W_takeoff * g - D * np.sin(gamma))**2)
		dt_dm = (W_takeoff * g - D * np.sin(gamma)) * g / thrust
		dt_dD = (D * np.cos(gamma) * np.cos(gamma) - (W_takeoff * g - D * np.sin(gamma)) * np.sin(gamma)) / thrust

		partials['Thrust', 'Weight|takeoff'] = dt_dm
		partials['Thrust', 'Aero|total_drag'] = dt_dD
		partials['Body|cos_delta', 'Weight|takeoff'] = - D * np.cos(gamma) / (thrust**2) * dt_dm
		partials['Body|cos_delta', 'Aero|total_drag'] = np.cos(gamma) / thrust - D * np.cos(gamma) / (thrust**2) * dt_dD


class MultirotorConstantCruiseTrim(om.ExplicitComponent):
	"""
	Computes the body tilt angle for wingless multirotor in cruise
	Inputs:
		Weight|takeoff 		: total take-off weight [kg]
		Aero|total_drag 	: drag of a multirotor body [N]
	Outputs:
		Thrust 				: total thrust required as a vehicle [N]
		Body|sin_beta		: sin(beta), beta = body incidence angle [rad]
	"""
	def initialize(self):
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Drag of a multirotor body')
		self.add_output('Thrust', units='N', desc='Thrust required as a vehicle')
		self.add_output('Body|sin_beta', desc='Sin of body incidence angle')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		D = inputs['Aero|total_drag']			# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff * g)**2 + D**2)

		outputs['Thrust'] = thrust
		outputs['Body|sin_beta'] = (W_takeoff * g) / thrust

	def compute_partials(self, inputs, partials):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		D = inputs['Aero|total_drag']			# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff * g)**2 + D**2)

		partials['Thrust', 'Weight|takeoff'] = (W_takeoff * g**2) / thrust
		partials['Thrust', 'Aero|total_drag'] = D / thrust
		partials['Body|sin_beta', 'Weight|takeoff'] = g / thrust - (W_takeoff**2 * g**3) / (thrust**3)
		partials['Body|sin_beta', 'Aero|total_drag'] = - (W_takeoff * g * D) / (thrust**3)


class WingedTrimOfAoA(om.Group):
	"""
	Computes the AoA using Prandtl lifting line theory for finite wing
	"""
	def initialize(self):
		self.options.declare('v_sound', types=float, desc='Sound speed')

	def setup(self):

		v_sound = self.options['v_sound']

		self.add_subsystem('finite_wing_CL_alpha',
							FiniteWingLiftCoefficientCurveSlope(v_sound=v_sound),
							promotes_inputs=['Wing|airfoil|CL_alpha', 'Wing|aspect_ratio', 'Aero|speed'],
							promotes_outputs=['Wing|CL_alpha'])

		self.add_subsystem('AoA_calc',
							om.ExecComp('AoA = (CL - CL0)/CL_alpha * 180.0/pi', AoA={'units': 'deg'}, CL={'units': None}, CL0={'units': None}, CL_alpha={'units': '1/rad'}),
							promotes_inputs=['CL', 'CL0', ('CL_alpha', 'Wing|CL_alpha')],
							promotes_outputs=['AoA'])


class WingedConstantClimbTrimOfLift(om.ExplicitComponent):
	"""
	Computes the required lift during climb
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during climb [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
	Outputs:
		Aero|lift 		: aerodynamic lift [N]
	"""
	def initialize(self):
		self.options.declare('g', desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during climb')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Aero|lift', units='N', desc='Aerodynamic lift')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]
		
		outputs['Aero|lift'] = W_takeoff * g * np.cos(gamma)

	def compute_partials(self, inputs, partials):
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		partials['Aero|lift', 'Weight|takeoff'] = g * np.cos(gamma)


class WingedConstantClimbTrimOfThrust(om.ExplicitComponent):
	"""
	Computes the required thrust during climb
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during climb [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
		Aero|total_drag	: total aerodynamic drag [N]
	Outputs:
		Thrust_all 		: total required thrust [N]
	"""
	def initialize(self):
		self.options.declare('g', desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during climb')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Total aerodynamic drag')
		self.add_output('Thrust_all', units='N', desc='Total required thrust')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		Drag = inputs['Aero|total_drag']		# in [N]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]
		
		outputs['Thrust_all'] = Drag + W_takeoff * g * np.sin(gamma)

	def compute_partials(self, inputs, partials):
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		partials['Thrust_all', 'Weight|takeoff'] = g * np.sin(gamma)
		partials['Thrust_all', 'Aero|total_drag'] = 1.0


class WingedConstantDescentTrimOfLift(om.ExplicitComponent):
	"""
	Computes the required lift during descent
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during descent [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
	Outputs:
		Aero|lift 		: aerodynamic lift [N]
	"""
	def initialize(self):
		self.options.declare('g', desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during descent')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Aero|lift', units='N', desc='Aerodynamic lift')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]
		
		outputs['Aero|lift'] = W_takeoff * g * np.cos(gamma)

	def compute_partials(self, inputs, partials):
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		partials['Aero|lift', 'Weight|takeoff'] = g * np.cos(gamma)


class WingedConstantDescentTrimOfThrust(om.ExplicitComponent):
	"""
	Computes the required thrust during descent
	Parameters:
		g 				: gravitational acceleration [m/s**2]
		gamma			: flight path angle during descent [rad]
	Inputs:
		Weight|takeoff 	: total take-off weight [kg]
		Aero|total_drag	: total aerodynamic drag [N]
	Outputs:
		Thrust_all 		: total required thrust [N]
	"""
	def initialize(self):
		self.options.declare('g', desc='Gravitational acceleration')
		self.options.declare('gamma', desc='Flight path angle during descent')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Total aerodynamic drag')
		self.add_output('Thrust_all', units='N', desc='Total required thrust')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff'] 	# in [kg]
		Drag = inputs['Aero|total_drag']		# in [N]
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]
		
		outputs['Thrust_all'] = Drag - W_takeoff * g * np.sin(gamma)

	def compute_partials(self, inputs, partials):
		gamma = self.options['gamma']			# in [rad]
		g = self.options['g']					# in [m/s**2]

		partials['Thrust_all', 'Weight|takeoff'] = - g * np.sin(gamma)
		partials['Thrust_all', 'Aero|total_drag'] = 1.0
