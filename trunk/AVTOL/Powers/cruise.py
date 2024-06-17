import numpy as np
import openmdao.api as om

from AVTOL.Aerodynamics.empirical import MultirotorParasiteDrag

class PowerForwardEdgewise(om.Group):
	"""
	Computes the power required in edgewise forward flight (cruise of wingless multirotor)
	Inputs:
		eVTOL|W_takeoff 	: total take-off weight [kg]
		eVTOL|Cruise_peed	: cruising speed of the eVTOL [m/s]
		Rotor|radius		: rotor radius [m]
		Rotor|mu 	 		: rotor's advance ratio
		Rotor|alpha			: rotor tilt angle [rad]
	Outputs:
		power_forward 		: required power for cruise [W]
		Rotor|Thrust		: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of lifting rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rotor_sigma', types=float, desc='Rotor solidity')
		self.options.declare('rho_air', default=1.225, desc='Air density')

	def setup(self):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rotor_sigma = self.options['rotor_sigma']
		rho_air = self.options['rho_air']

		# Step 1: Calculate BodyDrag() for the multirotor in cruise
		self.add_subsystem('body_drag',
							MultirotorParasiteDrag(),
							promotes_inputs=['eVTOL|*', 'Body|sin_beta'],
							promotes_outputs=['Aero|Drag'])
		
		# Step 2: Calculate thrust required for trim and the body tilt angle
		self.add_subsystem('trim',
							MultiRotorTrim(),
							promotes_inputs=['eVTOL|W_takeoff', 'Aero|Drag'],
							promotes_outputs=[('Thrust', 'Thrust_all'), 'Body|sin_beta'])

		# Step 3: Convert Body|sin_beta into Rotor|alpha
		self.add_subsystem('beta2alpha',
							om.ExecComp('alpha = arccos(sin_beta)', alpha={'units':'rad'}),
							promotes_inputs=[('sin_beta', 'Body|sin_beta')],
							promotes_outputs=[('alpha', 'Rotor|alpha')])

		# Step 4: Calculate the thrust required by each rotor
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=['Thrust_all'],
							promotes_outputs=['Rotor|Thrust'])

		# Step 5: Calculate rotor omega given the advance ratio mu
		self.add_subsystem('rotor_revolution',
							RotorRevolutionFromAdvanceRatio(),
							promotes_inputs=['Rotor|radius', 'Rotor|*', ('v_inf', 'eVTOL|Cruise_peed')],
							promotes_outputs=['Rotor|omega'])
		self.set_input_defaults('Rotor|mu', 0.15)

		# Step 6: Calculate the thrust coefficient Ct
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=['Rotor|radius', 'Rotor|*'],
							promotes_outputs=['Rotor|Ct'])

		# Step 7: Calculate profile power
		self.add_subsystem('profile_power',
							ProfilePower(rho_air=rho_air, sigma=rotor_sigma),
							promotes_inputs=['Rotor|radius', 'Rotor|*'],
							promotes_outputs=['Rotor|Profile_power'])

		# Step 8: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|lambda'])
		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=['Rotor|radius', 'Rotor|*', ('v_inf', 'eVTOL|Cruise_peed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=['Rotor|radius', 'Rotor|*'],
							promotes_outputs=['Rotor|kappa'])

		# Step 9: Calculate total power required
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor),
							promotes_inputs=['Rotor|*', 'v_induced', ('v_inf', 'eVTOL|Cruise_peed')],
							promotes_outputs=['power_forward'])
