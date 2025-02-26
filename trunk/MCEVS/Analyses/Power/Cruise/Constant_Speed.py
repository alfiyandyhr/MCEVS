import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Aerodynamics.Parasite import ParasiteDragFidelityOne
from MCEVS.Analyses.Aerodynamics.Empirical import MultirotorParasiteDrag
from MCEVS.Analyses.Aerodynamics.Empirical import WingedParasiteDrag
from MCEVS.Analyses.Aerodynamics.Parabolic import WingedAeroDrag

from MCEVS.Analyses.Stability.Trim import MultirotorConstantCruiseTrim

from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor
from MCEVS.Analyses.Aerodynamics.Rotor import RotorRevolutionFromAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import PropellerRevolutionFromAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import RotorAdvanceRatio, PropellerAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustCoefficient
from MCEVS.Analyses.Aerodynamics.Rotor import RotorInflow
from MCEVS.Analyses.Aerodynamics.Rotor import InducedVelocity

from MCEVS.Analyses.Power.Rotor import RotorProfilePower, PowerForwardComp, InducedPowerFactor

class PowerCruiseConstantSpeedEdgewise(om.Group):
	"""
	Computes the power required in edgewise forward flight (cruise of wingless multirotor)
	Parameters:
		N_rotor		: number or rotors
		n_blade 	: number of blades per rotor
		Cd0 		: rotor's parasite drag coefficient
		hover_FM	: hover figure of merit
		rho_air		: air density [kg/m**3]
		g 			: gravitational acceleration [m/s**2]
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
		Rotor|radius			: rotor radius [m]
		Rotor|chord 			: rotor chord length [m]
		Rotor|mu 	 			: rotor's advance ratio
		Rotor|alpha				: rotor tilt angle [rad]
	Outputs:
		Power|CruiseConstantSpeed 	: required power for cruise [W]
		Rotor|thrust				: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('N_rotor', types=int, desc='Number of lifting rotors')
		self.options.declare('n_blade', types=int, desc='Number of blades per rotor')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

	def setup(self):
		vehicle = self.options['vehicle']
		N_rotor = self.options['N_rotor']
		n_blade = self.options['n_blade']
		hover_FM = self.options['hover_FM']
		Cd0 = self.options['Cd0']
		rho_air = self.options['rho_air']
		mu_air = self.options['mu_air']
		g = self.options['g']
		fidelity = self.options['fidelity']

		# Step 1: Calculate the drag for the multirotor in cruise
		if fidelity['aero'] == 0:
			self.add_subsystem('parasite_drag',
								MultirotorParasiteDrag(N_rotor=N_rotor, rho_air=rho_air),
								promotes_inputs=['Weight|takeoff', ('Aero|speed','Mission|cruise_speed'), ('Rotor|radius', 'LiftRotor|radius')],
								promotes_outputs=[('Aero|total_drag','Aero|Cruise|total_drag'), ('Aero|Cd0','Aero|Cruise|Cd0')])
		elif fidelity['aero'] == 1:
			if vehicle.Cd0['cruise'] is None:
				self.add_subsystem('parasite_drag',
						ParasiteDragFidelityOne(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='cruise'),
						promotes_inputs=['Weight|takeoff', ('Aero|speed', 'Mission|cruise_speed'), ('Rotor|radius', 'LiftRotor|radius')],
						promotes_outputs=[('Aero|Cd0', 'Aero|Cruise|Cd0'), ('Aero|parasite_drag','Aero|Cruise|total_drag'), ('Aero|f_total','Aero|Cruise|f_total'),('Aero|f_fuselage','Aero|Cruise|f_fuselage'),('Aero|f_rotor_hub','Aero|Cruise|f_rotor_hub')])
		
		# Step 2: Calculate thrust required for trim and the body tilt angle
		self.add_subsystem('trim',
							MultirotorConstantCruiseTrim(g=g),
							promotes_inputs=['Weight|takeoff', ('Aero|total_drag','Aero|Cruise|total_drag')],
							promotes_outputs=[('Thrust', 'Thrust_all_cruise'), ('Body|sin_beta','Body|Cruise|sin_beta')])

		# Step 3: Convert Body|sin_beta into Rotor|alpha
		self.add_subsystem('beta2alpha',
							om.ExecComp('alpha = arccos(sin_beta)', alpha={'units':'rad'}),
							promotes_inputs=[('sin_beta', 'Body|Cruise|sin_beta')],
							promotes_outputs=[('alpha', 'LiftRotor|Cruise|alpha')])

		# Step 4: Calculate the thrust required by each rotor
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=[('Thrust_all','Thrust_all_cruise')],
							promotes_outputs=[('Rotor|thrust', 'LiftRotor|Cruise|thrust')])

		# Step 5: Calculate rotor omega from RPM
		self.add_subsystem('rpm2omega',
							om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units':'rad/s'}, rpm={'units':'rpm'}),
							promotes_inputs=[('rpm','LiftRotor|Cruise|RPM')],
							promotes_outputs=[('omega','LiftRotor|Cruise|omega')])

		self.add_subsystem('mu',
							RotorAdvanceRatio(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|Cruise|alpha'),
											 ('Rotor|omega',	'LiftRotor|Cruise|omega'),
											 ('v_inf',			'Mission|cruise_speed')],
							promotes_outputs=[('Rotor|mu',		'LiftRotor|Cruise|mu')])

		# Step 6: Calculate the thrust coefficient Ct
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',	'LiftRotor|Cruise|thrust'),
											 ('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|omega',	'LiftRotor|Cruise|omega')],
							promotes_outputs=[('Rotor|thrust_coefficient','LiftRotor|Cruise|thrust_coefficient')])

		# Step 7: Calculate profile power
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|chord',	'LiftRotor|chord'),
											 ('Rotor|mu',		'LiftRotor|Cruise|mu'),
											 ('Rotor|omega',	'LiftRotor|Cruise|omega')],
							promotes_outputs=[('Rotor|profile_power', 'LiftRotor|Cruise|profile_power')])

		# Step 8: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=[('Rotor|mu', 					'LiftRotor|Cruise|mu'),
											 ('Rotor|alpha', 				'LiftRotor|Cruise|alpha'),
											 ('Rotor|thrust_coefficient', 	'LiftRotor|Cruise|thrust_coefficient'),],
							promotes_outputs=[('Rotor|lambda', 				'LiftRotor|Cruise|lambda')])
		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|Cruise|alpha'),
											 ('Rotor|omega',	'LiftRotor|Cruise|omega'),
											 ('Rotor|lambda',	'LiftRotor|Cruise|lambda'),
											 ('v_inf', 			'Mission|cruise_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',			'LiftRotor|Cruise|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|Cruise|profile_power'),
											 ('Rotor|radius',			'LiftRotor|radius'),],
							promotes_outputs=[('Rotor|kappa',			'LiftRotor|Cruise|kappa')])

		# Step 9: Calculate total power required
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor, g=g),
							promotes_inputs=[('Rotor|thrust',			'LiftRotor|Cruise|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|Cruise|profile_power'),
											 ('Rotor|alpha',			'LiftRotor|Cruise|alpha'),
											 ('Rotor|kappa',			'LiftRotor|Cruise|kappa'),
											 'v_induced', ('v_inf', 'Mission|cruise_speed')],
							promotes_outputs=[('Power|forward','Power|CruiseConstantSpeed'), ('Rotor|T_to_P','LiftRotor|Cruise|T_to_P')])

class PowerCruiseConstantSpeedWithWing(om.Group):
	"""
	Computes the power required in winged forward flight (cruise of winged config)
	Parameters:
		N_propeller	: number or propellers
		n_blade 	: number of blades per propeller
		Cd0 		: rotor's parasite drag coefficient
		hover_FM	: hover figure of merit
		rho_air		: air density [kg/m**3]
		g 			: gravitational acceleration [m/s**2]
		AoA 		: aircraft's angle of attack [deg]
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		eVTOL|S_wing 			: wing area [m**2]
		eVTOL|AR_wing			: wing aspect ratio 
		Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
		Rotor|radius			: rotor radius [m]
		Rotor|J					: propeller's advance ratio
		Rotor|alpha 			: rotor tilt angle [rad]
	Outputs:
		Power|CruiseConstantSpeed 	: required power for cruise [W]
		Rotor|thrust				: thrust of a rotor [N]
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('N_propeller', types=int, desc='Number of propellers')
		self.options.declare('n_blade', types=int, desc='Number of blades per propeller')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('AoA', desc='Aircraft angle of attack')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

	def setup(self):
		vehicle = self.options['vehicle']
		N_propeller = self.options['N_propeller']
		n_blade = self.options['n_blade']
		Cd0 = self.options['Cd0']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		mu_air = self.options['mu_air']
		g = self.options['g']
		AoA = self.options['AoA']
		fidelity = self.options['fidelity']

		# Step 1: Lift should be equal to total weight
		indep = self.add_subsystem('cruise', om.IndepVarComp())
		indep.add_output('g', val=g)
		lift_comp = om.ExecComp('lift = weight * g', lift={'units':'N'}, weight={'units':'kg'})
		self.add_subsystem('lift',
							lift_comp,
							promotes_inputs=[('weight', 'Weight|takeoff'), ('g', 'cruise.g')],
							promotes_outputs=[('lift', 'Aero|Cruise|lift')])

		# Step 2: Calculate drag in cruise using simple polar equations
		if fidelity['aero'] == 0:
			self.add_subsystem('parasite_drag',
								WingedParasiteDrag(rho_air=rho_air),
								promotes_inputs=['Weight|takeoff', 'Wing|area', ('Aero|speed', 'Mission|cruise_speed')],
								promotes_outputs=[('Aero|Cd0','Aero|Cruise|Cd0'), ('Aero|parasite_drag','Aero|Cruise|parasite_drag')])

		elif fidelity['aero'] == 1:
			self.add_subsystem('parasite_drag',
								ParasiteDragFidelityOne(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='cruise'),
								promotes_inputs=['Weight|takeoff', ('Aero|speed', 'Mission|cruise_speed'),'Wing|area'],
								promotes_outputs=[('Aero|Cd0', 'Aero|Cruise|Cd0'), ('Aero|parasite_drag','Aero|Cruise|parasite_drag'),
												  ('Aero|f_fuselage','Aero|Cruise|f_fuselage'),('Aero|f_rotor_hub','Aero|Cruise|f_rotor_hub')])

		self.add_subsystem('total_drag',
							WingedAeroDrag(rho_air=rho_air),
							promotes_inputs=[('Aero|Cd0','Aero|Cruise|Cd0'), ('Aero|lift','Aero|Cruise|lift'), 'Wing|*', ('Aero|speed', 'Mission|cruise_speed')],
							promotes_outputs=[('Aero|total_drag','Aero|Cruise|total_drag'), ('Aero|CL','Aero|Cruise|CL'), ('Aero|f_total', 'Aero|Cruise|f_total')])

		# Step 3: Calculate thrust required by each propeller (thrust = drag)
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_propeller),
							promotes_inputs=[('Thrust_all', 'Aero|Cruise|total_drag')],
							promotes_outputs=[('Rotor|thrust','Propeller|Cruise|thrust')])

		# Step 4: Calculate rotor omega from RPM and propeller advance ratio
		self.add_subsystem('rpm2omega',
							om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units':'rad/s'}, rpm={'units':'rpm'}),
							promotes_inputs=[('rpm','Propeller|Cruise|RPM')],
							promotes_outputs=[('omega','Propeller|Cruise|omega')])

		self.add_subsystem('J',
							PropellerAdvanceRatio(),
							promotes_inputs=[('v_inf', 'Mission|cruise_speed'), 'Propeller|radius', ('Propeller|omega', 'Propeller|Cruise|omega')],
							promotes_outputs=[('Propeller|advance_ratio', 'Propeller|Cruise|J')])

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
							RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
							promotes_inputs=[('Rotor|radius',			'Propeller|radius'),
											 ('Rotor|chord',			'Propeller|chord'),
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
							PowerForwardComp(N_rotor=N_propeller, g=g),
							promotes_inputs=[('Rotor|thrust', 'Propeller|Cruise|thrust'),
											 ('Rotor|profile_power', 'Propeller|Cruise|profile_power'),
											 ('Rotor|alpha', 'Propeller|Cruise|alpha'),
											 ('Rotor|kappa', 'Propeller|Cruise|kappa'),
											 'v_induced', ('v_inf', 'Mission|cruise_speed')],
							promotes_outputs=[('Power|forward','Power|CruiseConstantSpeed'), ('Rotor|T_to_P','Propeller|Cruise|T_to_P')])
