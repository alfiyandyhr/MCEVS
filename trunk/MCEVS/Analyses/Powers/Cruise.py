import numpy as np
import openmdao.api as om

from MCEVS.Aerodynamics.Empirical import MultirotorParasiteDrag
from MCEVS.Aerodynamics.Empirical import WingedParasiteDrag
from MCEVS.Aerodynamics.Parabolic import WingedCruiseDrag

from MCEVS.Stability.Trim import MultiRotorTrim

from MCEVS.Aerodynamics.Rotor import ThrustOfEachRotor
from MCEVS.Aerodynamics.Rotor import RotorRevolutionFromAdvanceRatio
from MCEVS.Aerodynamics.Rotor import PropellerRevolutionFromAdvanceRatio
from MCEVS.Aerodynamics.Rotor import RotorAdvanceRatio
from MCEVS.Aerodynamics.Rotor import ThrustCoefficient
from MCEVS.Aerodynamics.Rotor import RotorInflow
from MCEVS.Aerodynamics.Rotor import InducedVelocity

from MCEVS.Powers.Rotor import RotorProfilePower
from MCEVS.Powers.Rotor import InducedPowerFactor
from MCEVS.Powers.Rotor import PowerForwardComp


class PowerForwardEdgewise(om.Group):
	"""
	Computes the power required in edgewise forward flight (cruise of wingless multirotor)
	Parameters:
		N_rotor				: number or rotors
		hover_FM			: hover figure of merit
		rotor_sigma 		: rotor's solidity
		rho_air				: air density [kg/m**3]
	Inputs:
		eVTOL|W_takeoff 	: total take-off weight [kg]
		eVTOL|Cruise_speed	: cruising speed of the eVTOL [m/s]
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

		# Step 1: Calculate the drag for the multirotor in cruise
		self.add_subsystem('parasite_drag',
							MultirotorParasiteDrag(N_rotor=N_rotor, rho_air=rho_air),
							promotes_inputs=['eVTOL|W_takeoff', 'eVTOL|Cruise_speed', 'Rotor|radius'],
							promotes_outputs=['Aero|Drag', 'Aero|Cd0'])
		
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
							promotes_inputs=['Rotor|*', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['Rotor|omega'])

		# Step 6: Calculate the thrust coefficient Ct
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|Ct'])

		# Step 7: Calculate profile power
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, sigma=rotor_sigma),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|Profile_power'])

		# Step 8: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|lambda'])
		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=['Rotor|*', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|kappa'])

		# Step 9: Calculate total power required
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor),
							promotes_inputs=['Rotor|*', 'v_induced', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['power_forward'])

class PowerForwardWithWing(om.Group):
	"""
	Computes the power required in winged forward flight (cruise of winged config)
	Parameters:
		N_rotor				: number or rotors
		hover_FM			: hover figure of merit
		rho_air				: air density [kg/m**3]
		rotor_sigma 		: rotor's solidity
		g 					: gravitational acceleration [m/s**2]
		AoA 				: aircraft's angle of attack [deg]
	Inputs:
		eVTOL|W_takeoff 	: total take-off weight [kg]
		eVTOL|S_wing 		: wing area [m**2]
		eVTOL|AR_wing		: wing aspect ratio 
		eVTOL|Cruise_speed	: cruising speed of the eVTOL [m/s]
		Rotor|radius		: rotor radius [m]
		Rotor|J				: propeller's advance ratio
		Rotor|alpha 		: rotor tilt angle [rad]
	Outputs:
		power_forward 		: required power for cruise [W]
		Rotor|Thrust		: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of cruising rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', default=1.225, desc='Air density')
		self.options.declare('rotor_sigma', types=float, desc='Rotor solidity')
		self.options.declare('g', default=9.81, desc='Gravitational acceleration')
		self.options.declare('AoA', desc='Aircraft angle of attack')

	def setup(self):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		rotor_sigma = self.options['rotor_sigma']
		g = self.options['g']
		AoA = self.options['AoA']

		# Step 1: Lift should be equal to total weight
		indep = self.add_subsystem('gravity', om.IndepVarComp())
		indep.add_output('g', val=g)
		lift_comp = om.ExecComp('lift = weight * g', lift={'units':'N'}, weight={'units':'kg'})
		self.add_subsystem('lift',
							lift_comp,
							promotes_inputs=[('weight', 'eVTOL|W_takeoff'), ('g', 'gravity.g')],
							promotes_outputs=['lift'])

		# Step 2: Calculate drag in cruise using simple polar equations
		self.add_subsystem('parasite_drag',
							WingedParasiteDrag(rho_air=rho_air),
							promotes_inputs=['eVTOL|*'],
							promotes_outputs=['Aero|Cd0', 'Aero|Parasite_drag'])

		self.add_subsystem('total_drag',
							WingedCruiseDrag(rho_air=rho_air),
							promotes_inputs=[('Aero|Lift', 'lift'), 'eVTOL|*', 'Aero|Cd0'],
							promotes_outputs=['Aero|Drag', 'Aero|CL_cruise'])

		# Step 3: Calculate thrust required by each rotor (thrust = drag)
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=[('Thrust_all', 'Aero|Drag')],
							promotes_outputs=['Rotor|Thrust'])

		# Step 4: Calculate rotor omega given propeller advance ratio J;
		# freestream speed = eVTOL cruise speed
		self.add_subsystem('prop_revolution',
							PropellerRevolutionFromAdvanceRatio(),
							promotes_inputs=['Rotor|radius', 'Rotor|J', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['Rotor|omega'])
		self.set_input_defaults('Rotor|J', 1.0) # default J

		# Step 5: Calculate rotor advance ratio mu and thrust coefficient Ct
		self.add_subsystem('mu',
							RotorAdvanceRatio(),
							promotes_inputs=['Rotor|*', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['Rotor|mu'])
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|Ct'])

		# Step 6: Calculate profile power of a rotor
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, sigma=rotor_sigma),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|Profile_power'])

		# Step 7: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|lambda'])
		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=['Rotor|*', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=['Rotor|*'],
							promotes_outputs=['Rotor|kappa'])

		# Step 8: Calculate total power required for winged forward flight
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor),
							promotes_inputs=['Rotor|*', 'v_induced', ('v_inf', 'eVTOL|Cruise_speed')],
							promotes_outputs=['power_forward'])

		# Assume the rotor tilt angle is 85, or AoA = 5
		self.set_input_defaults('Rotor|alpha', val=90.0-AoA, units='deg')
