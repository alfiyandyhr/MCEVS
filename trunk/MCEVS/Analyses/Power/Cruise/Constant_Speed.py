import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Aerodynamics.Empirical import MultirotorParasiteDrag
from MCEVS.Analyses.Aerodynamics.Empirical import WingedParasiteDrag
from MCEVS.Analyses.Aerodynamics.Parabolic import WingedAeroDrag

from MCEVS.Analyses.Stability.Trim import MultiRotorTrim

from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor
from MCEVS.Analyses.Aerodynamics.Rotor import RotorRevolutionFromAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import PropellerRevolutionFromAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import RotorAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustCoefficient
from MCEVS.Analyses.Aerodynamics.Rotor import RotorInflow
from MCEVS.Analyses.Aerodynamics.Rotor import InducedVelocity

from MCEVS.Analyses.Power.Rotor import RotorProfilePower, PowerForwardComp, InducedPowerFactor

class PowerCruiseConstantSpeedEdgewise(om.Group):
	"""
	Computes the power required in edgewise forward flight (cruise of wingless multirotor)
	Parameters:
		N_rotor				: number or rotors
		hover_FM			: hover figure of merit
		rotor_sigma 		: rotor's solidity
		rho_air				: air density [kg/m**3]
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
		Rotor|radius			: rotor radius [m]
		Rotor|mu 	 			: rotor's advance ratio
		Rotor|alpha				: rotor tilt angle [rad]
	Outputs:
		Power|CruiseConstantSpeed 	: required power for cruise [W]
		Rotor|thrust				: thrust of a rotor [N]
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
							promotes_inputs=['Weight|takeoff', 'Mission|cruise_speed', ('Rotor|radius', 'LiftRotor|radius')],
							promotes_outputs=['Aero|total_drag', 'Aero|Cd0'])
		
		# Step 2: Calculate thrust required for trim and the body tilt angle
		self.add_subsystem('trim',
							MultiRotorTrim(),
							promotes_inputs=['Weight|takeoff', 'Aero|total_drag'],
							promotes_outputs=[('Thrust', 'Thrust_all'), 'Body|sin_beta'])

		# Step 3: Convert Body|sin_beta into Rotor|alpha
		self.add_subsystem('beta2alpha',
							om.ExecComp('alpha = arccos(sin_beta)', alpha={'units':'rad'}),
							promotes_inputs=[('sin_beta', 'Body|sin_beta')],
							promotes_outputs=[('alpha', 'LiftRotor|alpha')])

		# Step 4: Calculate the thrust required by each rotor
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=['Thrust_all'],
							promotes_outputs=[('Rotor|thrust', 'LiftRotor|thrust')])

		# Step 5: Calculate rotor omega given the advance ratio mu
		self.add_subsystem('rotor_revolution',
							RotorRevolutionFromAdvanceRatio(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|alpha'),
											 ('Rotor|mu',		'LiftRotor|advance_ratio'),
											 ('v_inf',			'Mission|cruise_speed')],
							promotes_outputs=[('Rotor|omega', 	'LiftRotor|omega')])

		# Step 6: Calculate the thrust coefficient Ct
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',	'LiftRotor|thrust'),
											 ('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|omega',	'LiftRotor|omega')],
							promotes_outputs=[('Rotor|thrust_coefficient','LiftRotor|thrust_coefficient')])

		# Step 7: Calculate profile power
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, sigma=rotor_sigma),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|mu',		'LiftRotor|advance_ratio'),
											 ('Rotor|omega',	'LiftRotor|omega')],
							promotes_outputs=[('Rotor|profile_power', 'LiftRotor|profile_power')])

		# Step 8: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=[('Rotor|mu', 					'LiftRotor|advance_ratio'),
											 ('Rotor|alpha', 				'LiftRotor|alpha'),
											 ('Rotor|thrust_coefficient', 	'LiftRotor|thrust_coefficient'),],
							promotes_outputs=[('Rotor|lambda', 				'LiftRotor|lambda')])
		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|alpha'),
											 ('Rotor|omega',	'LiftRotor|omega'),
											 ('Rotor|lambda',	'LiftRotor|lambda'),
											 ('v_inf', 			'Mission|cruise_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',			'LiftRotor|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|profile_power'),
											 ('Rotor|radius',			'LiftRotor|radius'),],
							promotes_outputs=[('Rotor|kappa',			'LiftRotor|kappa')])

		# Step 9: Calculate total power required
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor),
							promotes_inputs=[('Rotor|thrust',			'LiftRotor|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|profile_power'),
											 ('Rotor|alpha',			'LiftRotor|alpha'),
											 ('Rotor|kappa',			'LiftRotor|kappa'),
											 'v_induced', ('v_inf', 'Mission|cruise_speed')],
							promotes_outputs=[('Power|forward','Power|CruiseConstantSpeed')])

class PowerCruiseConstantSpeedWithWing(om.Group):
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
		Weight|takeoff 	: total take-off weight [kg]
		eVTOL|S_wing 		: wing area [m**2]
		eVTOL|AR_wing		: wing aspect ratio 
		Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
		Rotor|radius		: rotor radius [m]
		Rotor|J				: propeller's advance ratio
		Rotor|alpha 		: rotor tilt angle [rad]
	Outputs:
		Power|CruiseConstantSpeed 	: required power for cruise [W]
		Rotor|thrust				: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of cruising rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('rotor_sigma', types=float, desc='Rotor solidity')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
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
							promotes_inputs=[('weight', 'Weight|takeoff'), ('g', 'gravity.g')],
							promotes_outputs=[('lift', 'Aero|Cruise|lift')])

		# Step 2: Calculate drag in cruise using simple polar equations
		self.add_subsystem('parasite_drag',
							WingedParasiteDrag(rho_air=rho_air),
							promotes_inputs=['Weight|takeoff', 'Wing|area', ('Aero|speed', 'Mission|cruise_speed')],
							promotes_outputs=[('Aero|Cd0','Aero|Cruise|Cd0'), ('Aero|parasite_drag','Aero|Cruise|parasite_drag')])

		self.add_subsystem('total_drag',
							WingedAeroDrag(rho_air=rho_air),
							promotes_inputs=[('Aero|Cd0','Aero|Cruise|Cd0'), ('Aero|lift','Aero|Cruise|lift'), 'Wing|*', ('Aero|speed', 'Mission|cruise_speed')],
							promotes_outputs=[('Aero|total_drag','Aero|Cruise|total_drag'), ('Aero|CL','Aero|Cruise|CL')])

		# Step 3: Calculate thrust required by each propeller (thrust = drag)
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=[('Thrust_all', 'Aero|Cruise|total_drag')],
							promotes_outputs=[('Rotor|thrust','Propeller|Cruise|thrust')])

		# Step 4: Calculate rotor omega given propeller advance ratio J;
		# freestream speed = eVTOL cruise speed
		self.add_subsystem('prop_revolution',
							PropellerRevolutionFromAdvanceRatio(),
							promotes_inputs=['Propeller|radius', 'Propeller|advance_ratio', ('v_inf', 'Mission|cruise_speed')],
							promotes_outputs=[('Propeller|omega','Propeller|Cruise|omega')])

		# Step 5: Calculate rotor advance ratio mu and thrust coefficient Ct
		# Treating propeller as a rotor
		self.add_subsystem('mu',
							RotorAdvanceRatio(),
							promotes_inputs=[('Rotor|radius',	'Propeller|radius'),
											 ('Rotor|alpha',	'Propeller|Cruise|alpha'),
											 ('Rotor|omega',	'Propeller|Cruise|omega'),
											 ('v_inf',			'Mission|cruise_speed')],
							promotes_outputs=[('Rotor|mu',		'Propeller|Cruise|mu')])
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust', 				'Propeller|Cruise|thrust'),
											 ('Rotor|radius', 				'Propeller|radius'),
											 ('Rotor|omega',  				'Propeller|Cruise|omega')],
							promotes_outputs=[('Rotor|thrust_coefficient',  'Propeller|Cruise|thrust_coefficient')])

		# Step 6: Calculate profile power of a rotor
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, sigma=rotor_sigma),
							promotes_inputs=[('Rotor|radius',			'Propeller|radius'),
											 ('Rotor|mu', 	  			'Propeller|Cruise|mu'),
											 ('Rotor|omega',  			'Propeller|Cruise|omega')],
							promotes_outputs=[('Rotor|profile_power', 	'Propeller|Cruise|profile_power')])

		# Step 7: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=[('Rotor|mu',   			  'Propeller|Cruise|mu'),
											 ('Rotor|alpha',			  'Propeller|Cruise|alpha'),
											 ('Rotor|thrust_coefficient', 'Propeller|Cruise|thrust_coefficient')],
							promotes_outputs=[('Rotor|lambda',			  'Propeller|Cruise|lambda')])

		# Assume the rotor tilt angle is 85, or AoA = 5
		self.set_input_defaults('Propeller|Cruise|alpha', val=90.0-AoA, units='deg')

		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=[('Rotor|radius',	'Propeller|radius'),
											 ('Rotor|alpha',	'Propeller|Cruise|alpha'),
											 ('Rotor|omega',	'Propeller|Cruise|omega'),
											 ('Rotor|lambda',	'Propeller|Cruise|lambda'),
											 ('v_inf',			'Mission|cruise_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',			'Propeller|Cruise|thrust'),
											 ('Rotor|profile_power',	'Propeller|Cruise|profile_power'),
											 ('Rotor|radius',			'Propeller|radius'),],
							promotes_outputs=[('Rotor|kappa','Propeller|Cruise|kappa')])

		# Step 8: Calculate total power required for winged forward flight
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor),
							promotes_inputs=[('Rotor|thrust', 'Propeller|Cruise|thrust'),
											 ('Rotor|profile_power', 'Propeller|Cruise|profile_power'),
											 ('Rotor|alpha', 'Propeller|Cruise|alpha'),
											 ('Rotor|kappa', 'Propeller|Cruise|kappa'),
											 'v_induced', ('v_inf', 'Mission|cruise_speed')],
							promotes_outputs=[('Power|forward','Power|CruiseConstantSpeed')])
