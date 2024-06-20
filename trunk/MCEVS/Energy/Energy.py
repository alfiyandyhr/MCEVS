import numpy as np
import openmdao.api as om

from MCEVS.Powers.Hover import PowerHover
from MCEVS.Powers.Cruise import PowerForwardEdgewise, PowerForwardWithWing

class EnergyConsumption(om.Group):
	"""
	Computes the energy consumption of an eVTOL given the vehicle specifications and mission requirements.
	It also computes disk loading in hover and cruise.
	Inputs:
	(mission requirements)
		mission_range		: mission range [m]
		hover_time 			: total hover time [s]
	(eVTOL design variables)
		eVTOL|W_takeoff 	: total take-off weight [kg]
		eVTOL|Cruise_speed 	: cruising speed of the eVTOL [m/s]
		Rotor|radius_lift	: lifting rotor radius [m]
		Rotor|radius_cruise	: cruising rotor radius [m] 		(for lift+cruise only)
		eVTOL|S_wing 		: wing area [m**2]					(for lift+cruise only)
		Rotor|mu 			: rotor advance ratio				(for multirotor only)
		Rotor|J 			: propeller advance ratio			(for lift+cruise only)
	Outputs:
	(major performances)
		power_hover
		power_cruise
	(for some constraints)
		disk_loading_hover
		disk_loading_cruise
		Rotor|Ct (in cruise)
		CL_cruise
	"""
	def initialize(self):
		self.options.declare('evtol_options', types=dict, desc='Dict containing all option parameters')

	def setup(self):
		params = self.options['evtol_options']

		# Unpacking options
		eVTOL_config = params['evtol_config']
		N_rotors_lift = params['N_rotors_lift']		# number of lifting rotors
		rotor_sigma = params['rotor_lift_solidity']	# solidity of lifting rotors 
		hover_FM = params['hover_FM']				# hover figure of merit
		rho_air = params['rho_air']					# air density
		g = params['gravitational_accel']			# gravitational acceleration

		if eVTOL_config == 'multirotor':
			pass
		elif eVTOL_config == 'lift+cruise':
			N_rotors_cruise = params['N_rotors_cruise']		# number of cruising rotors
			# Cd0 = params['Cd0'] 							# minimum drag of the drag polar
			wing_AR = params['wing_AR']						# wing aspect ratio
			# wing_e = params['wing_e']						# Oswald efficiency
			prop_sigma = params['rotor_cruise_solidity']	# solidty of cruising rotors
			AoA = params['AoA_cruise'] 						# angle of attack during cruise
		else:
			raise RuntimeError('eVTOL configuration is not available.')


		# --- Calculate power consumptions for each flight segment --- #
		# power in hover
		self.add_subsystem('power_hover',
							PowerHover(N_rotor=N_rotors_lift, hover_FM=hover_FM, rho_air=rho_air, g=g),
							promotes_inputs=['eVTOL|W_takeoff', ('Rotor|radius', 'Rotor|radius_lift')],
							promotes_outputs=['power_hover'])

		# power in cruise
		if eVTOL_config == 'multirotor':
			input_list = ['eVTOL|W_takeoff', 'eVTOL|Cruise_speed', 'Rotor|mu', ('Rotor|radius', 'Rotor|radius_lift')]
			self.add_subsystem('power_forward_edgewise',
								PowerForwardEdgewise(N_rotor=N_rotors_lift, hover_FM=hover_FM, rotor_sigma=rotor_sigma),
								promotes_inputs=input_list,
								promotes_outputs=['*'])

		elif eVTOL_config == 'lift+cruise':
			input_list = ['eVTOL|*', 'Rotor|J', ('Rotor|radius', 'Rotor|radius_cruise')]
			self.add_subsystem('power_forward_wing',
								PowerForwardWithWing(N_rotor=N_rotors_cruise, hover_FM=hover_FM, wing_AR=wing_AR, rotor_sigma=prop_sigma, g=g, AoA=AoA),
								promotes_inputs=input_list,
								promotes_outputs=['*'])

		# --- Calculate energy consumption --- #
		# energy = power_hover * hover_time + power_cruise * cruise_time
		energy_comp = om.ExecComp('energy_cnsmp = (power_hover * hover_time) + (power_forward * flight_distance / speed)',
								   energy_cnsmp={'units': 'W * s'},
								   power_hover={'units': 'W'},
								   power_forward={'units': 'W'},
								   hover_time={'units': 's'},
								   flight_distance={'units': 'm'},
								   speed={'units': 'm/s'})
		self.add_subsystem('energy', energy_comp,
							promotes_inputs=['power_hover', 'power_forward', 'hover_time', 'flight_distance', ('speed', 'eVTOL|Cruise_speed')],
							promotes_outputs=['energy_cnsmp'])

		# --- Calculate disk loadings --- #
		# in hover
		disk_loading_comp_1 = om.ExecComp('disk_loading = thrust / (pi * r**2)',
										   disk_loading={'units': 'N/m**2'},
										   thrust={'units': 'N'},
										   r={'units': 'm'})
		self.add_subsystem('disk_loading_hover', disk_loading_comp_1,
							promotes_inputs=[('r', 'Rotor|radius_lift')],
							promotes_outputs=[('disk_loading', 'disk_loading_hover')])
		self.connect('power_hover.thrust_each', 'disk_loading_hover.thrust')

		# in cruise
		disk_loading_comp_2 = om.ExecComp('disk_loading = thrust / (pi * r**2)',
										   disk_loading={'units': 'N/m**2'},
										   thrust={'units': 'N'},
										   r={'units': 'm'})
		self.add_subsystem('disk_loading_cruise', disk_loading_comp_2,
							promotes_outputs=[('disk_loading', 'disk_loading_cruise')])
		if eVTOL_config == 'multirotor':
			self.promotes('disk_loading_cruise', inputs=[('r', 'Rotor|radius_lift')])
		elif eVTOL_config == 'lift+cruise':
			self.promotes('disk_loading_cruise', inputs=[('r', 'Rotor|radius_cruise')])
		self.connect('Rotor|Thrust', 'disk_loading_cruise.thrust')


		# --- Add nonlinear solvers for implicit equations --- #
		self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=30, iprint=0, rtol=1e-10)
		self.nonlinear_solver.options['err_on_non_converge'] = True
		self.nonlinear_solver.options['reraise_child_analysiserror'] = True
		self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
		self.nonlinear_solver.linesearch.options['maxiter'] = 10
		self.nonlinear_solver.linesearch.options['iprint'] = 0
		self.linear_solver = om.DirectSolver(assemble_jac=True)













