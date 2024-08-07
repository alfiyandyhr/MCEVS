import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Power.Hover.Stay import PowerHoverStay
from MCEVS.Analyses.Power.HoverClimb.Constant_Speed import PowerHoverClimbConstantSpeed
from MCEVS.Analyses.Power.HoverDescent.Constant_Speed import PowerHoverDescentConstantSpeed
from MCEVS.Analyses.Power.Climb.Constant_Vy_Constant_Vx import PowerClimbConstantVyConstantVxWithWing, PowerClimbConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Descent.Constant_Vy_Constant_Vx import PowerDescentConstantVyConstantVxWithWing, PowerDescentConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Cruise.Constant_Speed import PowerCruiseConstantSpeedEdgewise, PowerCruiseConstantSpeedWithWing

class PowerAnalysis(object):
	"""
	docstring for PowerAnalysis
	"""
	def __init__(self, vehicle:object, mission:object, constants:object):
		super(PowerAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.constants = constants

	def evaluate(self):
		# print('### --- Solving for energy requirement --- ###')

		# MTOW should be defined if not in sizing mode
		mtow = self.vehicle.weight.max_takeoff 		# kg

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			rotor_advance_ratio 	= self.vehicle.lift_rotor.advance_ratio

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_propeller 			= self.vehicle.propeller.radius 		# m
			wing_area 				= self.vehicle.wing.area 				# m**2
			wing_aspect_ratio 		= self.vehicle.wing.aspect_ratio
			propeller_advance_ratio = self.vehicle.propeller.advance_ratio

		for segment in self.mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				cruise_speed = segment.speed 			# m/s

		# --- OpenMDAO probolem --- #
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		indeps.add_output('Weight|takeoff', mtow, units='kg')
		indeps.add_output('Mission|cruise_speed', cruise_speed, units='m/s')

		if self.vehicle.configuration == 'Multirotor':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|advance_ratio', rotor_advance_ratio)

		elif self.vehicle.configuration == 'LiftPlusCruise':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('Propeller|radius', r_propeller, units='m')
			indeps.add_output('Wing|area', wing_area, units='m**2')
			indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)
			indeps.add_output('Propeller|advance_ratio', propeller_advance_ratio)
		
		prob.model.add_subsystem('energy_model',
								  PowerRequirement(mission=self.mission,
								  				   vehicle=self.vehicle,
								  				   constants=self.constants),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()
		
		return prob

class PowerRequirement(om.Group):
	"""
	docstring for PowerRequirement
	"""
	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('constants', types=object, desc='Constants object')

	def setup(self):

		# Unpacking option objects
		mission 	= self.options['mission']
		vehicle 	= self.options['vehicle']
		constants 	= self.options['constants']

		# Unpacking constants
		rho_air = constants['rho'] 	# air density
		g 		= constants['g']	# gravitational acceleration

		# Unpacking cruise AoA
		for segment in mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				AoA = segment.AoA
			if segment.kind == 'HoverClimbConstantSpeed':
				v_climb = segment.speed
			if segment.kind == 'HoverDescentConstantSpeed':
				v_descent = segment.speed

		# Unpacking vehicle parameters
		N_lift_rotor 	= vehicle.lift_rotor.n_rotor			# number of lift rotors
		rotor_sigma 	= vehicle.lift_rotor.solidity			# solidity of lift rotors 
		hover_FM 		= vehicle.lift_rotor.figure_of_merit	# hover figure of merit

		if vehicle.configuration == 'Multirotor':
			pass
		elif vehicle.configuration == 'LiftPlusCruise':
			N_propeller = vehicle.propeller.n_propeller		# number of propellers
			prop_sigma 	= vehicle.propeller.solidity  		# solidity of cruising rotors
		else:
			raise RuntimeError('eVTOL configuration is not available.')

		# -------------------------------------------------------------#
		# --- Calculate power consumptions for each flight segment --- #
		# -------------------------------------------------------------#

		for segment in mission.segments:

			# LiftPlusCruise's propellers do not work during hover, hoverclimb, or hoverdescent
			# and its lift rotor does not work during cruise, climb, or descent
			if vehicle.configuration == 'LiftPlusCruise':
				if segment.kind in ['HoverStay', 'HoverClimbConstantSpeed', 'HoverDescentConstantSpeed']:
					zero_p = om.IndepVarComp(f'Power|Propeller|segment_{segment.id}', val=0.0, units='W')
					self.add_subsystem(f'zero_p_{segment.id}', zero_p, promotes=['*'])
					zero_t = om.IndepVarComp(f'Propeller|thrust_each|segment_{segment.id}', val=0.0, units='N')
					self.add_subsystem(f'zero_t_{segment.id}', zero_t, promotes=['*'])
				if segment.kind in ['CruiseConstantSpeed', 'ClimbConstantVyConstantVx', 'DescentConstantVyConstantVx']:
					zero_p = om.IndepVarComp(f'Power|LiftRotor|segment_{segment.id}', val=0.0, units='W')
					self.add_subsystem(f'zero_p_{segment.id}', zero_p, promotes=['*'])
					zero_t = om.IndepVarComp(f'LiftRotor|thrust_each|segment_{segment.id}', val=0.0, units='N')
					self.add_subsystem(f'zero_t_{segment.id}', zero_t, promotes=['*'])

			if segment.kind == 'HoverStay':
				self.add_subsystem(f'segment_{segment.id}_power',
									PowerHoverStay(N_rotor=N_lift_rotor, hover_FM=hover_FM, rho_air=rho_air, g=g),
									promotes_inputs=['Weight|takeoff', 'LiftRotor|radius'],
									promotes_outputs=[('Power|HoverStay',f'Power|LiftRotor|segment_{segment.id}'),
													  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])

			if segment.kind == 'HoverClimbConstantSpeed':
				self.add_subsystem(f'segment_{segment.id}_power',
									PowerHoverClimbConstantSpeed(N_rotor=N_lift_rotor, hover_FM=hover_FM, rho_air=rho_air, g=g, v_climb=v_climb),
									promotes_inputs=['Weight|takeoff', 'LiftRotor|radius'],
									promotes_outputs=[('Power|HoverClimbConstantSpeed',f'Power|LiftRotor|segment_{segment.id}'),
													  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])

			if segment.kind == 'HoverDescentConstantSpeed':
				self.add_subsystem(f'segment_{segment.id}_power',
									PowerHoverDescentConstantSpeed(N_rotor=N_lift_rotor, hover_FM=hover_FM, rho_air=rho_air, g=g, v_descent=v_descent),
									promotes_inputs=['Weight|takeoff', 'LiftRotor|radius'],
									promotes_outputs=[('Power|HoverDescentConstantSpeed',f'Power|LiftRotor|segment_{segment.id}'),
													  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])
			
			if segment.kind == 'ClimbConstantVyConstantVx':
				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerClimbConstantVyConstantVxEdgewise(N_rotor=N_lift_rotor, hover_FM=hover_FM, rotor_sigma=rotor_sigma, rho_air=rho_air, g=g, climb_airspeed=segment.speed, gamma=segment.gamma),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Climb|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerClimbConstantVyConstantVxWithWing(N_propeller=N_propeller, hover_FM=hover_FM, prop_sigma=prop_sigma, rho_air=rho_air, g=g, AoA=AoA, gamma=segment.gamma, climb_airspeed=segment.speed),
										promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
														  ('Propeller|Climb|thrust',f'Propeller|thrust_each|segment_{segment.id}')])

			if segment.kind == 'DescentConstantVyConstantVx':
				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerDescentConstantVyConstantVxEdgewise(N_rotor=N_lift_rotor, hover_FM=hover_FM, rotor_sigma=rotor_sigma, rho_air=rho_air, g=g, descent_airspeed=segment.speed, gamma=segment.gamma),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|DescentConstantVyConstantVx',f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Descent|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerDescentConstantVyConstantVxWithWing(N_propeller=N_propeller, hover_FM=hover_FM, prop_sigma=prop_sigma, rho_air=rho_air, g=g, AoA=AoA, gamma=segment.gamma, descent_airspeed=segment.speed),
										promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|DescentConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
														  ('Propeller|Descent|thrust',f'Propeller|thrust_each|segment_{segment.id}')])

			if segment.kind == 'CruiseConstantSpeed':

				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerCruiseConstantSpeedEdgewise(N_rotor=N_lift_rotor, hover_FM=hover_FM, rotor_sigma=rotor_sigma, rho_air=rho_air, g=g),
										promotes_inputs=['Weight|*', 'Mission|*', 'LiftRotor|*'],
										promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Cruise|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerCruiseConstantSpeedWithWing(N_rotor=N_propeller, rho_air=rho_air, hover_FM=hover_FM, rotor_sigma=prop_sigma, g=g, AoA=AoA),
										promotes_inputs=['Weight|*', 'Mission|*', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|Propeller|segment_{segment.id}'),
														   'Aero|Cruise|CL', 'Propeller|Cruise|thrust_coefficient',
														  ('Propeller|Cruise|thrust',f'Propeller|thrust_each|segment_{segment.id}')])

		# ------------------------------------------#
		# ---- Writing maximum power equations ---- #
		# ------------------------------------------#
		# p_max = max(p_segment_1, ..., p_segment_n)

		if vehicle.configuration == 'Multirotor':
			max_power_eq = 'p_max = '
			kwargs_p = {'p_max': {'units':'W'}}
			for i in range(1, len(mission.segments)+1):
				if i == len(mission.segments):
					max_power_eq += f'p_segment_{i}' + (len(mission.segments)-1)*')'
				else:
					max_power_eq += f'maximum(p_segment_{i}, '
				kwargs_p[f'p_segment_{i}'] = {'units': 'W'}

		elif vehicle.configuration == 'LiftPlusCruise':
			max_power_liftrotor_eq = 'p1_max = '
			max_power_propeller_eq = 'p2_max = '
			kwargs_p1 = {'p1_max':{'units':'W'}}
			kwargs_p2 = {'p2_max':{'units':'W'}}
			for i in range(1, len(mission.segments)+1):
				if i == len(mission.segments):
					max_power_liftrotor_eq += f'p1_segment_{i}' + (len(mission.segments)-1)*')'
					max_power_propeller_eq += f'p2_segment_{i}' + (len(mission.segments)-1)*')'
				else:
					max_power_liftrotor_eq += f'maximum(p1_segment_{i}, '
					max_power_propeller_eq += f'maximum(p2_segment_{i}, '
				kwargs_p1[f'p1_segment_{i}'] = {'units': 'W'}
				kwargs_p2[f'p2_segment_{i}'] = {'units': 'W'}

		# ---------------------------------------------------------------------------- #
		# --- Calculate maximum power requirement per component for sizing purpose --- #
		# ---------------------------------------------------------------------------- #
		if vehicle.configuration == 'Multirotor':
			max_power_comp = om.ExecComp(max_power_eq, **kwargs_p)
			self.add_subsystem('max_power_req', max_power_comp,
								promotes_outputs=[('p_max','Power|LiftRotor|maximum')])

			for i in range(1, len(mission.segments)+1):
				self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_req.p_segment_{i}')

		elif vehicle.configuration == 'LiftPlusCruise':
			max_power_liftrotor_comp = om.ExecComp(max_power_liftrotor_eq, **kwargs_p1)
			max_power_propeller_comp = om.ExecComp(max_power_propeller_eq, **kwargs_p2)
			self.add_subsystem('max_power_liftrotor_req', max_power_liftrotor_comp,
								promotes_outputs=[('p1_max','Power|LiftRotor|maximum')])
			self.add_subsystem('max_power_propeller_req', max_power_propeller_comp,
								promotes_outputs=[('p2_max','Power|Propeller|maximum')])
			
			for i in range(1, len(mission.segments)+1):
				self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_liftrotor_req.p1_segment_{i}')
				self.connect(f'Power|Propeller|segment_{i}', f'max_power_propeller_req.p2_segment_{i}')

		# ------------------------------------------------------#
		# --- Calculate total power requirement per segment --- #
		# ------------------------------------------------------#
		if vehicle.configuration == 'Multirotor':
			for i in range(1, len(mission.segments)+1):
				kwargs = {f'power_segment_{i}': {'units':'W'}, f'p_{i}': {'units':'W'}}
				segment_power_comp = om.ExecComp(f'power_segment_{i} = p_{i}', **kwargs)
				self.add_subsystem(f'power_segment_req_{i}', segment_power_comp,
									promotes_outputs=[(f'power_segment_{i}',f'Power|segment_{i}')])
				self.connect(f'Power|LiftRotor|segment_{i}', f'power_segment_req_{i}.p_{i}')

		elif vehicle.configuration == 'LiftPlusCruise':
			for i in range(1, len(mission.segments)+1):
				adder = om.AddSubtractComp()
				adder.add_equation(f'Power|segment_{i}',
									input_names=[f'Power|LiftRotor|segment_{i}', f'Power|Propeller|segment_{i}'],
									units='W',
									scaling_factors=[1., 1.])
				self.add_subsystem(f'p_total_segment_{i}',
									adder,
									promotes_inputs=[f'Power|LiftRotor|segment_{i}', f'Power|Propeller|segment_{i}'],
									promotes_outputs=[f'Power|segment_{i}'])

		# -----------------------------------------------------#
		# --- Calculate disk loading per rotor per segment --- #
		# -----------------------------------------------------#
		if vehicle.configuration == 'Multirotor':
			for i in range(1, len(mission.segments)+1):
				DL_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
									   disk_loading 	= {'units': 'N/m**2'},
									   thrust 			= {'units': 'N'},
									   r 				= {'units': 'm'})
				self.add_subsystem(f'DL_segment_{i}', DL_comp,
									promotes_inputs = [('r', 'LiftRotor|radius')],
									promotes_outputs= [('disk_loading', f'DiskLoading|LiftRotor|segment_{i}')] )
				self.connect(f'LiftRotor|thrust_each|segment_{i}', f'DL_segment_{i}.thrust')

		elif vehicle.configuration == 'LiftPlusCruise':
			for i in range(1, len(mission.segments)+1):
				DL_1_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
										 disk_loading 	= {'units': 'N/m**2'},
										 thrust 		= {'units': 'N'},
										 r 				= {'units': 'm'})
				DL_2_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
										 disk_loading 	= {'units': 'N/m**2'},
										 thrust 		= {'units': 'N'},
										 r 				= {'units': 'm'})
				self.add_subsystem(f'DL_1_segment_{i}', DL_1_comp,
									promotes_inputs = [('r', 'LiftRotor|radius')],
									promotes_outputs= [('disk_loading', f'DiskLoading|LiftRotor|segment_{i}')] )
				self.connect(f'LiftRotor|thrust_each|segment_{i}', f'DL_1_segment_{i}.thrust')
				self.add_subsystem(f'DL_2_segment_{i}', DL_2_comp,
									promotes_inputs = [('r', 'Propeller|radius')],
									promotes_outputs= [('disk_loading', f'DiskLoading|Propeller|segment_{i}')] )
				self.connect(f'Propeller|thrust_each|segment_{i}', f'DL_2_segment_{i}.thrust')

		# -----------------------------------------------------#
		# --- Add nonlinear solvers for implicit equations --- #
		# -----------------------------------------------------#

		self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=30, iprint=0, rtol=1e-10)
		self.nonlinear_solver.options['err_on_non_converge'] = True
		self.nonlinear_solver.options['reraise_child_analysiserror'] = True
		self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
		self.nonlinear_solver.linesearch.options['maxiter'] = 10
		self.nonlinear_solver.linesearch.options['iprint'] = 0
		self.linear_solver = om.DirectSolver(assemble_jac=True)












