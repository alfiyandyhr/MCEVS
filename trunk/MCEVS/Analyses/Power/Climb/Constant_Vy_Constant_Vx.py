import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Stability.Trim import WingedConstantClimbTrimOfLift, WingedConstantClimbTrimOfThrust
from MCEVS.Analyses.Aerodynamics.Empirical import WingedParasiteDrag
from MCEVS.Analyses.Aerodynamics.Parabolic import WingedAeroDrag
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor, ThrustCoefficient
from MCEVS.Analyses.Aerodynamics.Rotor import PropellerRevolutionFromAdvanceRatio, RotorAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import RotorInflow, InducedVelocity
from MCEVS.Analyses.Power.Rotor import RotorProfilePower, InducedPowerFactor, PowerForwardComp


class PowerClimbConstantVyConstantVxWithWing(om.Group):
	"""
	Computes the power required in winged climb phase
	Parameters:
		N_propeller			: number of propellers
		rho_air				: air density [kg/m**3]
		prop_sigma 			: propeller's solidity
		hover_FM			: hover figure of merit
		g 					: gravitational acceleration [m/s**2]
		AoA 				: aircraft's angle of attack [deg]
		climb_airspeed		: climb air speed [m/s]
		gamma				: flight path angle during climb/descent
	Inputs:
		Weight|takeoff 		: total take-off weight [kg]
		eVTOL|S_wing 		: wing area [m**2]
		eVTOL|AR_wing		: wing aspect ratio
		Rotor|radius		: rotor radius [m]
		Rotor|J				: propeller's advance ratio
		Rotor|alpha 		: rotor tilt angle [rad]
	Outputs:
		Power|CruiseConstantSpeed 	: required power for cruise [W]
		Rotor|thrust				: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('N_propeller', types=int, desc='Number of propellers')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('prop_sigma', types=float, desc='Propeller solidity')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('AoA', desc='Aircraft angle of attack')
		self.options.declare('climb_airspeed', desc='Climb air speed')
		self.options.declare('gamma', desc='Flight path angle during climb/descent')

	def setup(self):
		N_propeller = self.options['N_propeller']
		hover_FM = self.options['hover_FM']
		prop_sigma = self.options['prop_sigma']
		rho_air = self.options['rho_air']
		g = self.options['g']
		AoA = self.options['AoA']
		gamma = self.options['gamma']
		climb_airspeed = self.options['climb_airspeed']

		# Step 1: Trim analysis
		self.add_subsystem('trim_lift',
							WingedConstantClimbTrimOfLift(g=g, gamma=gamma),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=[('Aero|lift','Aero|Climb|lift')])

		# Step 2: Calculate drag in climb using simple polar equations
		indep = self.add_subsystem('climb', om.IndepVarComp())
		indep.add_output('climb_airspeed', val=climb_airspeed, units='m/s')
		self.add_subsystem('parasite_drag',
							WingedParasiteDrag(rho_air=rho_air),
							promotes_inputs=['Weight|takeoff', 'Wing|area', ('Aero|speed', 'climb.climb_airspeed')],
							promotes_outputs=[('Aero|Cd0','Aero|Climb|Cd0'), ('Aero|parasite_drag','Aero|Climb|parasite_drag')])

		self.add_subsystem('total_drag',
							WingedAeroDrag(rho_air=rho_air),
							promotes_inputs=[('Aero|Cd0','Aero|Climb|Cd0'), ('Aero|lift','Aero|Climb|lift'), ('Aero|speed', 'climb.climb_airspeed'), 'Wing|*'],
							promotes_outputs=[('Aero|total_drag','Aero|Climb|total_drag'), ('Aero|CL','Aero|Climb|CL')])

		# Step 3: Calculate thrust required by each propeller after trimming
		self.add_subsystem('trim_thrust',
							WingedConstantClimbTrimOfThrust(g=g, gamma=gamma),
							promotes_inputs=['Weight|takeoff', ('Aero|total_drag','Aero|Climb|total_drag')],
							promotes_outputs=['Thrust_all'])
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_propeller),
							promotes_inputs=['Thrust_all'],
							promotes_outputs=[('Rotor|thrust','Propeller|Climb|thrust')])

		# Step 4: Calculate rotor omega given propeller advance ratio J;
		# freestream speed = eVTOL cruise speed
		self.add_subsystem('prop_revolution',
							PropellerRevolutionFromAdvanceRatio(),
							promotes_inputs=['Propeller|radius', 'Propeller|advance_ratio', ('v_inf', 'climb.climb_airspeed')],
							promotes_outputs=[('Propeller|omega','Propeller|Climb|omega')])

		# Step 5: Calculate rotor advance ratio mu and thrust coefficient Ct
		# Treating propeller as a rotor
		self.add_subsystem('mu',
							RotorAdvanceRatio(),
							promotes_inputs=[('Rotor|radius',	'Propeller|radius'),
											 ('Rotor|alpha',	'Propeller|Climb|alpha'),
											 ('Rotor|omega',	'Propeller|Climb|omega'),
											 ('v_inf',			'climb.climb_airspeed')],
							promotes_outputs=[('Rotor|mu',		'Propeller|Climb|mu')])
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust', 				'Propeller|Climb|thrust'),
											 ('Rotor|radius', 				'Propeller|radius'),
											 ('Rotor|omega',  				'Propeller|Climb|omega')],
							promotes_outputs=[('Rotor|thrust_coefficient',  'Propeller|Climb|thrust_coefficient')])

		# Step 6: Calculate profile power of a rotor
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, sigma=prop_sigma),
							promotes_inputs=[('Rotor|radius',			'Propeller|radius'),
											 ('Rotor|mu', 	  			'Propeller|Climb|mu'),
											 ('Rotor|omega',  			'Propeller|Climb|omega')],
							promotes_outputs=[('Rotor|profile_power', 	'Propeller|Climb|profile_power')])

		# Step 7: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=[('Rotor|mu',   			  'Propeller|Climb|mu'),
											 ('Rotor|alpha',			  'Propeller|Climb|alpha'),
											 ('Rotor|thrust_coefficient', 'Propeller|Climb|thrust_coefficient')],
							promotes_outputs=[('Rotor|lambda',			  'Propeller|Climb|lambda')])

		# Assume the rotor tilt angle is 85, or AoA = 5
		self.set_input_defaults('Propeller|Climb|alpha', val=90.0-AoA, units='deg')

		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=[('Rotor|radius',	'Propeller|radius'),
											 ('Rotor|alpha',	'Propeller|Climb|alpha'),
											 ('Rotor|omega',	'Propeller|Climb|omega'),
											 ('Rotor|lambda',	'Propeller|Climb|lambda'),
											 ('v_inf',			'climb.climb_airspeed')],
							promotes_outputs=[('v_induced','Propeller|Climb|v_induced')])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',			'Propeller|Climb|thrust'),
											 ('Rotor|profile_power',	'Propeller|Climb|profile_power'),
											 ('Rotor|radius',			'Propeller|radius'),],
							promotes_outputs=[('Rotor|kappa','Propeller|Climb|kappa')])

		# Step 8: Calculate total power required for winged forward flight
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_propeller),
							promotes_inputs=[('Rotor|thrust', 'Propeller|Climb|thrust'),
											 ('Rotor|profile_power', 'Propeller|Climb|profile_power'),
											 ('Rotor|alpha', 'Propeller|Climb|alpha'),
											 ('Rotor|kappa', 'Propeller|Climb|kappa'),
											 ('v_induced','Propeller|Climb|v_induced'),
											 ('v_inf', 'climb.climb_airspeed')],
							promotes_outputs=[('Power|forward','Power|ClimbConstantVyConstantVx')])


