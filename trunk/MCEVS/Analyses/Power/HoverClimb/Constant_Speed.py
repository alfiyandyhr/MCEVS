import numpy as np
import openmdao.api as om
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
		v_climb 	 : climb speed [m/s]
	Inputs:
		Weight|takeoff  	: total take-off weight [kg]
		LiftRotor|radius	: lift rotor radius [m]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('v_climb', types=float, desc='Hover climb speed')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_output('Power|HoverClimbConstantSpeed', units='W', desc='Power required for hover climb')
		self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
		self.add_output('FM', desc='Hover climb figure of merit')
		self.add_output('LiftRotor|T_to_P', units='g/W', desc='Thrust to power ratio of a single rotor/propeller')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']
		v_climb = self.options['v_climb']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

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
		v_climb = self.options['v_climb']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		P_total = (W_takeoff*g)/hover_FM * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		dP_dW = g*v_climb/(2*hover_FM) + g/hover_FM * np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) + (W_takeoff*g**2)/(4*hover_FM*rho_air*S_disk*N_rotor) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5)
		dP_dr = - (W_takeoff*g)**2 / (4*hover_FM*rho_air*N_rotor*S_disk**2) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*N_rotor*S_disk) )**(-0.5) * dSdisk_dr

		partials['Power|HoverClimbConstantSpeed', 'Weight|takeoff'] = dP_dW
		partials['Power|HoverClimbConstantSpeed', 'LiftRotor|radius'] = dP_dr
		partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor
		partials['LiftRotor|thrust', 'LiftRotor|radius'] = 0.0
		partials['LiftRotor|T_to_P', 'Weight|takeoff'] = ( 1 * 1000.0 ) / P_total + ( W_takeoff * 1000.0 ) * (-1/P_total**2) * dP_dW
		partials['LiftRotor|T_to_P', 'LiftRotor|radius'] = ( W_takeoff * 1000.0 ) * (-1/P_total**2) * dP_dr
		partials['FM', 'Weight|takeoff'] == 0.0
		partials['FM', 'LiftRotor|radius'] == 0.0

class PowerHoverClimbConstantSpeedFidelityTwo(om.Group):
	"""
	Computes the power required for hover climb with constant speed using BEMT
	Parameters:
		vehicle 	 : vehicle object
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
		v_climb 	 : climb speed [m/s]
	Inputs:
		Weight|takeoff  	: total take-off weight [kg]
		LiftRotor|radius	: lift rotor radius [m]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('v_climb', types=float, desc='Hover climb speed')

	def setup(self):

		vehicle = self.options['vehicle']

		N_rotor = vehicle.lift_rotor.n_rotor
		n_blade = vehicle.lift_rotor.n_blade
		airfoil_list = vehicle.lift_rotor.n_section * [vehicle.lift_rotor.airfoil]

		rho_air = self.options['rho_air']
		g = self.options['g']
		v_climb = self.options['v_climb']

		indep = self.add_subsystem('hover_climb', om.IndepVarComp())
		indep.add_output('v_inf', val=v_climb, units='m/s')
		indep.add_output('g', val=g, units='m/s**2')
		indep.add_output('N_rotor', val=N_rotor)

		self.add_subsystem('T_req_calc',
							om.ExecComp('T_req = W_takeoff * g / N_rotor', T_req={'units':'N'}, W_takeoff={'units':'kg'}, g={'units':'m/s**2'}),
							promotes_inputs=[('W_takeoff', 'Weight|takeoff'), ('g','hover_climb.g'), ('N_rotor', 'hover_climb.N_rotor')],
							promotes_outputs=[('T_req','LiftRotor|thrust')])

		input_list = [('T_req','LiftRotor|thrust'),
					  ('rpm', 'LiftRotor|HoverClimb|RPM'),
					  ('v_inf','hover_climb.v_inf'),
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
