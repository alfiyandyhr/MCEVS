import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor, RotorAdvanceRatio, ThrustCoefficient, RotorInflow, InducedVelocity
from MCEVS.Analyses.Power.Rotor import RotorProfilePower, PowerForwardComp, InducedPowerFactor

from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOMGroup
from MCEVS.Analyses.Aerodynamics.BEMT.SectionOM import SectionLocalPitch, SectionLocalRadiusChordWidth

class PowerHoverClimbConstantSpeedFidelityZero(om.ExplicitComponent):
	"""
	Computes the power required for hover climb with constant speed
	Parameters:
		N_rotor		 : number or lift rotors
		hover_FM	 : hover figure of merit
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
	Inputs:
		Weight|takeoff  			: total take-off weight [kg]
		LiftRotor|radius			: lift rotor radius [m]
		Mission|hover_climb_speed 	: hover climb speed [m/s]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_input('Mission|hover_climb_speed', units='m/s', desc='Hover climb speed')
		self.add_output('Power|HoverClimbConstantSpeed', units='W', desc='Power required for hover climb')
		self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
		self.add_output('LiftRotor|T_to_P', units='g/W', desc='Thrust to power ratio of a single rotor/propeller')
		self.add_output('FM', desc='Hover climb figure of merit')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius']
		v_climb = inputs['Mission|hover_climb_speed']

		S_disk = np.pi * r**2

		P_total = (W_takeoff*g)/hover_FM * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		outputs['Power|HoverClimbConstantSpeed'] = P_total
		outputs['LiftRotor|thrust'] = (W_takeoff*g)/N_rotor
		outputs['LiftRotor|T_to_P'] = ( W_takeoff * 1000.0 ) / P_total
		outputs['FM'] = hover_FM

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius']
		v_climb = inputs['Mission|hover_climb_speed']

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		P_total = (W_takeoff*g)/hover_FM * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		dP_dW = g*v_climb/(2*hover_FM) + g/hover_FM * np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) + (W_takeoff*g**2)/(4*hover_FM*rho_air*S_disk*N_rotor) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5)
		dP_dr = - (W_takeoff*g)**2 / (4*hover_FM*rho_air*N_rotor*S_disk**2) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*N_rotor*S_disk) )**(-0.5) * dSdisk_dr
		dP_dv = (W_takeoff*g)/hover_FM * ( (1/2) + 0.5*( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5) * (v_climb/2) )

		dTP_P = ( W_takeoff * 1000.0 ) * (-1/P_total**2)

		partials['Power|HoverClimbConstantSpeed', 'Weight|takeoff'] = dP_dW
		partials['Power|HoverClimbConstantSpeed', 'LiftRotor|radius'] = dP_dr
		partials['Power|HoverClimbConstantSpeed', 'Mission|hover_climb_speed'] = dP_dv
		partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor
		partials['LiftRotor|thrust', 'LiftRotor|radius'] = 0.0
		partials['LiftRotor|thrust', 'Mission|hover_climb_speed'] = 0.0
		partials['LiftRotor|T_to_P', 'Weight|takeoff'] = ( 1 * 1000.0 ) / P_total + dTP_P * dP_dW
		partials['LiftRotor|T_to_P', 'LiftRotor|radius'] = dTP_P * dP_dr
		partials['LiftRotor|T_to_P', 'Mission|hover_climb_speed'] = dTP_P * dP_dv
		partials['FM', 'Weight|takeoff'] = 0.0
		partials['FM', 'LiftRotor|radius'] = 0.0
		partials['FM', 'Mission|hover_climb_speed'] = 0.0

class PowerHoverClimbConstantSpeedFidelityOne(om.Group):
	"""
	Computes the power required for hover climb with constant speed
	Parameters:
		N_rotor		 	: number or lift rotors
		hover_FM	 	: hover figure of merit
		rho_air		 	: air density [kg/m**3]
		g 			 	: gravitational acceleration [m/s**2]
	Inputs:
		Weight|takeoff  			: total take-off weight [kg]
		LiftRotor|radius			: lift rotor radius [m]
		Mission|hover_climb_speed 	: hover climb speed [m/s]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('n_blade', types=int, desc='Number of blades per rotor')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):
		N_rotor = self.options['N_rotor']
		n_blade = self.options['n_blade']
		Cd0 = self.options['Cd0']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		# Step 1: Total thrust is equal to weight (drag is considered negligible)
		indep = self.add_subsystem('hover_climb', om.IndepVarComp())
		indep.add_output('g', val=g)
		thrust_comp = om.ExecComp('thrust = weight * g', thrust={'units':'N'}, weight={'units':'kg'})
		self.add_subsystem('total_thrust',
							thrust_comp,
							promotes_inputs=[('weight', 'Weight|takeoff'), ('g', 'hover_climb.g')],
							promotes_outputs=[('thrust', 'Thrust_all')])

		# Step 2: Calculate thrust required by each rotor
		self.add_subsystem('thrust_each',
							ThrustOfEachRotor(N_rotor=N_rotor),
							promotes_inputs=['Thrust_all'],
							promotes_outputs=[('Rotor|thrust','LiftRotor|HoverClimb|thrust')])

		# Step 3: Calculate rotor omega from RPM
		self.add_subsystem('rpm2omega',
							om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units':'rad/s'}, rpm={'units':'rpm'}),
							promotes_inputs=[('rpm','LiftRotor|HoverClimb|RPM')],
							promotes_outputs=[('omega','LiftRotor|HoverClimb|omega')])

		# Step 4: Rotor advance ratio
		self.add_subsystem('mu',
							RotorAdvanceRatio(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|HoverClimb|alpha'),
											 ('Rotor|omega',	'LiftRotor|HoverClimb|omega'),
											 ('v_inf',			'Mission|hover_climb_speed')],
							promotes_outputs=[('Rotor|mu',		'LiftRotor|HoverClimb|mu')])

		# Rotor tilt angle is 90 deg
		self.set_input_defaults('LiftRotor|HoverClimb|alpha', val=90.0, units='deg')

		# Step 5: Calculate thrust coefficient
		self.add_subsystem('Ct',
							ThrustCoefficient(rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust', 				'LiftRotor|HoverClimb|thrust'),
											 ('Rotor|radius', 				'LiftRotor|radius'),
											 ('Rotor|omega',  				'LiftRotor|HoverClimb|omega')],
							promotes_outputs=[('Rotor|thrust_coefficient',  'LiftRotor|HoverClimb|thrust_coefficient')])

		# Step 6: Calculate profile power of a rotor
		self.add_subsystem('profile_power',
							RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
							promotes_inputs=[('Rotor|radius',			'LiftRotor|radius'),
											 ('Rotor|chord',			'LiftRotor|chord'),
											 ('Rotor|mu', 	  			'LiftRotor|HoverClimb|mu'),
											 ('Rotor|omega',  			'LiftRotor|HoverClimb|omega')],
							promotes_outputs=[('Rotor|profile_power', 	'LiftRotor|HoverClimb|profile_power')])

		# Step 7: Calculate induced power
		self.add_subsystem('rotor_inflow',
							RotorInflow(),
							promotes_inputs=[('Rotor|mu',   			  'LiftRotor|HoverClimb|mu'),
											 ('Rotor|alpha',			  'LiftRotor|HoverClimb|alpha'),
											 ('Rotor|thrust_coefficient', 'LiftRotor|HoverClimb|thrust_coefficient')],
							promotes_outputs=[('Rotor|lambda',			  'LiftRotor|HoverClimb|lambda')])

		self.add_subsystem('v_induced',
							InducedVelocity(),
							promotes_inputs=[('Rotor|radius',	'LiftRotor|radius'),
											 ('Rotor|alpha',	'LiftRotor|HoverClimb|alpha'),
											 ('Rotor|omega',	'LiftRotor|HoverClimb|omega'),
											 ('Rotor|lambda',	'LiftRotor|HoverClimb|lambda'),
											 ('v_inf',			'Mission|hover_climb_speed')],
							promotes_outputs=['v_induced'])
		self.add_subsystem('kappa',
							InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=[('Rotor|thrust',			'LiftRotor|HoverClimb|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|HoverClimb|profile_power'),
											 ('Rotor|radius',			'LiftRotor|radius'),],
							promotes_outputs=[('Rotor|kappa','LiftRotor|HoverClimb|kappa')])

		# Step 8: Calculate total power required for winged forward flight
		self.add_subsystem('power_req',
							PowerForwardComp(N_rotor=N_rotor, g=g),
							promotes_inputs=[('Rotor|thrust', 			'LiftRotor|HoverClimb|thrust'),
											 ('Rotor|profile_power',	'LiftRotor|HoverClimb|profile_power'),
											 ('Rotor|alpha', 			'LiftRotor|HoverClimb|alpha'),
											 ('Rotor|kappa', 			'LiftRotor|HoverClimb|kappa'),
											 'v_induced', ('v_inf', 'Mission|hover_climb_speed')],
							promotes_outputs=[('Power|forward','Power|HoverClimbConstantSpeed'), ('Rotor|T_to_P','LiftRotor|HoverClimb|T_to_P')])

		# # Step 9: Calculate figure of merit
		self.add_subsystem('FM_calc',
							FigureOfMerit(N_rotor=N_rotor, rho_air=rho_air, g=g),
							promotes_inputs=[('P_calculated', 'Power|HoverClimbConstantSpeed'),
											  'Weight|takeoff', 'LiftRotor|radius', 'Mission|hover_climb_speed'],
							promotes_outputs=[('FM','LiftRotor|HoverClimb|FM')])

class PowerHoverClimbConstantSpeedFidelityTwo(om.Group):
	"""
	Computes the power required for hover climb with constant speed using BEMT
	Parameters:
		vehicle 	 : vehicle object
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
	Inputs:
		Weight|takeoff  			: total take-off weight [kg]
		LiftRotor|radius			: lift rotor radius [m]
		Mission|hover_climb_speed 	: hover climb speed [m/s]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):

		vehicle = self.options['vehicle']

		N_rotor = vehicle.lift_rotor.n_rotor
		n_blade = vehicle.lift_rotor.n_blade
		airfoil_list = vehicle.lift_rotor.n_section * [vehicle.lift_rotor.airfoil]

		rho_air = self.options['rho_air']
		g = self.options['g']

		indep = self.add_subsystem('hover_climb', om.IndepVarComp())
		indep.add_output('g', val=g, units='m/s**2')
		indep.add_output('N_rotor', val=N_rotor)

		self.add_subsystem('T_req_calc',
							om.ExecComp('T_req = W_takeoff * g / N_rotor', T_req={'units':'N'}, W_takeoff={'units':'kg'}, g={'units':'m/s**2'}),
							promotes_inputs=[('W_takeoff', 'Weight|takeoff'), ('g','hover_climb.g'), ('N_rotor', 'hover_climb.N_rotor')],
							promotes_outputs=[('T_req','LiftRotor|thrust')])

		input_list = [('T_req','LiftRotor|thrust'),
					  ('rpm', 'LiftRotor|HoverClimb|RPM'),
					  ('v_inf','Mission|hover_climb_speed'),
					  ('blade_radius','LiftRotor|radius'),
					  ('hub_radius','LiftRotor|hub_radius'),
					  ('global_twist','LiftRotor|global_twist')]

		for i in range(len(airfoil_list)):

			# Convert pitch linear grad distribution to local pitch list
			if vehicle.lift_rotor.pitch_linear_grad is not None:
				self.add_subsystem(f'local_pitch{i+1}_calc',
									SectionLocalPitch(),
									promotes_inputs=[('gradient', 'LiftRotor|pitch_linear_grad'), ('r_to_R', f'LiftRotor|Section{i+1}|r_to_R')],
									promotes_outputs=[('local_pitch', f'LiftRotor|Section{i+1}|pitch')])

			# Convert r_to_R, c_to_R, and w_to_R into local radius, local chord, and local width
			self.add_subsystem(f'local_blade_section{i+1}',
								SectionLocalRadiusChordWidth(),
								promotes_inputs=[('r_to_R', f'LiftRotor|Section{i+1}|r_to_R'), ('c_to_R', f'LiftRotor|Section{i+1}|c_to_R'), ('w_to_R', f'LiftRotor|Section{i+1}|w_to_R'), ('rotor_radius', 'LiftRotor|radius')],
								promotes_outputs=[('local_radius', f'LiftRotor|Section{i+1}|radius'), ('local_chord', f'LiftRotor|Section{i+1}|chord'), ('local_width', f'LiftRotor|Section{i+1}|width')])

			input_list.append( (f'Section{i+1}|radius', f'LiftRotor|Section{i+1}|radius') )
			input_list.append( (f'Section{i+1}|chord', f'LiftRotor|Section{i+1}|chord') )
			input_list.append( (f'Section{i+1}|pitch', f'LiftRotor|Section{i+1}|pitch') )
			input_list.append( (f'Section{i+1}|width', f'LiftRotor|Section{i+1}|width') )

		output_list = ['Power','thrust_residual_square', 'FM', 'CT', 'J']

		self.add_subsystem('BEMT_Solver',
							BEMTSolverOMGroup(nblades=n_blade,
						  				      airfoil_list=airfoil_list,
						  				      rho=rho_air,
						  				      trim_rpm=True),
							promotes_inputs=input_list,
							promotes_outputs=output_list)
		
		self.add_subsystem('hover_climb_total_power',
							om.ExecComp('P_hover_climb = N_rotor * power_each', P_hover_climb={'units':'W'}, power_each={'units':'W'}),
							promotes_inputs=[('N_rotor', 'hover_climb.N_rotor'), ('power_each', 'Power')],
							promotes_outputs=[('P_hover_climb', 'Power|HoverClimbConstantSpeed')])

class FigureOfMerit(om.ExplicitComponent):
	"""
	Computes the hover climb figure of merit given calculated power
	Parameters:
		N_rotor		 : number or lift rotors
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
	Inputs:
		P_calculated 				: calculated power required for hover climb [W]
		Weight|takeoff  			: total take-off weight [kg]
		LiftRotor|radius			: lift rotor radius [m]
		Mission|hover_climb_speed 	: hover climb speed [m/s]
	Outputs:
		FM : figure of merit
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('P_calculated', units='W', desc='Calculated power required for hover climb')
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_input('Mission|hover_climb_speed', units='m/s', desc='Hover climb speed')
		self.add_output('FM', desc='Hover climb figure of merit')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		rho_air = self.options['rho_air']
		g = self.options['g']

		P_calculated = inputs['P_calculated']
		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius']
		v_climb = inputs['Mission|hover_climb_speed']

		S_disk = np.pi * r**2

		P_ideal = (W_takeoff*g) * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		
		outputs['FM'] = P_ideal / P_calculated

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		rho_air = self.options['rho_air']
		g = self.options['g']

		P_calculated = inputs['P_calculated']
		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius']
		v_climb = inputs['Mission|hover_climb_speed']

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		P_ideal = (W_takeoff*g) * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		
		dP_dW = g*v_climb/2 + g * np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) + (W_takeoff*g**2)/(4*rho_air*S_disk*N_rotor) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5)
		dP_dr = - (W_takeoff*g)**2 / (4*rho_air*N_rotor*S_disk**2) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*N_rotor*S_disk) )**(-0.5) * dSdisk_dr
		dP_dv = (W_takeoff*g) * ( (1/2) + 0.5*( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5) * (v_climb/2) )

		partials['FM', 'P_calculated'] = - P_ideal / P_calculated**2
		partials['FM', 'Weight|takeoff'] = dP_dW/P_calculated
		partials['FM', 'LiftRotor|radius'] = dP_dr/P_calculated
		partials['FM', 'Mission|hover_climb_speed'] = dP_dv/P_calculated
		 
