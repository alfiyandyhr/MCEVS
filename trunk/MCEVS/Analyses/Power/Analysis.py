import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Power.Hover.Stay import PowerHoverStay
from MCEVS.Analyses.Power.HoverClimb.Constant_Speed import PowerHoverClimbConstantSpeedFidelityZero, PowerHoverClimbConstantSpeedFidelityOne
from MCEVS.Analyses.Power.HoverDescent.Constant_Speed import PowerHoverDescentConstantSpeed
from MCEVS.Analyses.Power.Climb.Constant_Vy_Constant_Vx import PowerClimbConstantVyConstantVxWithWing, PowerClimbConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Descent.Constant_Vy_Constant_Vx import PowerDescentConstantVyConstantVxWithWing, PowerDescentConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Cruise.Constant_Speed import PowerCruiseConstantSpeedEdgewise, PowerCruiseConstantSpeedWithWing
from MCEVS.Analyses.Power.Others.Constant_Power import PowerConstantFractionOfMaxPower
from MCEVS.Utils.Performance import record_performance_by_segments

class PowerAnalysis(object):
	"""
	docstring for PowerAnalysis
	"""
	def __init__(self, vehicle:object, mission:object, fidelity:dict):
		super(PowerAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.fidelity = fidelity

	def evaluate(self, record=False):
		# print('### --- Solving for power requirement --- ###')

		# MTOW should be defined if not in sizing mode
		mtow = self.vehicle.weight.max_takeoff 		# kg

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			c_lift_rotor 			= self.vehicle.lift_rotor.chord 		# m
			rotor_advance_ratio 	= self.vehicle.lift_rotor.advance_ratio

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			c_lift_rotor 			= self.vehicle.lift_rotor.chord 		# m
			r_propeller 			= self.vehicle.propeller.radius 		# m
			c_propeller 			= self.vehicle.propeller.chord  		# m
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
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|chord', c_lift_rotor, units='m')
			indeps.add_output('LiftRotor|advance_ratio', rotor_advance_ratio)

		elif self.vehicle.configuration == 'LiftPlusCruise':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|chord', c_lift_rotor, units='m')
			indeps.add_output('Propeller|radius', r_propeller, units='m')
			indeps.add_output('Propeller|chord', c_propeller, units='m')
			indeps.add_output('Wing|area', wing_area, units='m**2')
			indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)
			indeps.add_output('Propeller|advance_ratio', propeller_advance_ratio)

		# Lift rotor variables needed for BEMT
		if self.fidelity['hover_climb'] == 1:
			n_sections = self.vehicle.lift_rotor.n_section
			radius_list = np.array(self.vehicle.lift_rotor.r_to_R_list) * self.vehicle.lift_rotor.radius
			chord_list = np.array(self.vehicle.lift_rotor.c_to_R_list) * self.vehicle.lift_rotor.radius
			pitch_list = np.array(self.vehicle.lift_rotor.pitch_list)
			for i in range(n_sections):
				if i == 0:
					width = 2*(radius_list[i] - r_hub_lift_rotor)
				elif i == n_sections-1:
					width = 2*(r_lift_rotor - radius_list[i])
				else:
					width = radius_list[i] - radius_list[i-1]
				indeps.add_output(f'LiftRotor|Section{i+1}|radius', radius_list[i], units='m')
				indeps.add_output(f'LiftRotor|Section{i+1}|chord', chord_list[i], units='m')
				indeps.add_output(f'LiftRotor|Section{i+1}|pitch', pitch_list[i], units='deg')
				indeps.add_output(f'LiftRotor|Section{i+1}|width', width, units='m')
			indeps.add_output('LiftRotor|HoverClimb|RPM', 1000.0, units='rpm') # rpm guess

		prob.model.add_subsystem('power_model',
								  PowerRequirement(mission=self.mission,
								  				   vehicle=self.vehicle,
								  				   fidelity=self.fidelity),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		if self.fidelity['hover_climb'] == 0:
			prob.setup(check=False)
			prob.run_model()

		elif self.fidelity['hover_climb'] == 1:
			prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=False)
			prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10, upper=5000)
			prob.model.add_objective('LiftRotor|HoverClimb|thrust_residual_square')
			prob.setup(check=False)
			prob.run_driver()
		
		if record:
			record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

		return prob

class PowerRequirement(om.Group):
	"""
	docstring for PowerRequirement
	"""
	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

	def setup(self):

		# Unpacking option objects
		mission 	 = self.options['mission']
		vehicle 	 = self.options['vehicle']
		fidelity 	 = self.options['fidelity']

		# Unpacking cruise AoA
		for segment in mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				AoA = segment.AoA
			if segment.kind == 'HoverClimbConstantSpeed':
				v_climb = segment.speed
			if segment.kind == 'HoverDescentConstantSpeed':
				v_descent = segment.speed

		# Unpacking vehicle parameters
		N_lift_rotor 	   		= vehicle.lift_rotor.n_rotor			# number of lift rotors
		n_blade_lift_rotor 		= vehicle.lift_rotor.n_blade 			# number of blades per rotor

		if vehicle.configuration == 'Multirotor':
			Cd0 			  = vehicle.lift_rotor.Cd0
			hover_FM 		  = vehicle.lift_rotor.figure_of_merit		# hover figure of merit
		elif vehicle.configuration == 'LiftPlusCruise':
			N_propeller 	  = vehicle.propeller.n_propeller			# number of propellers
			n_blade_propeller = vehicle.propeller.n_blade 				# number of blades per propeller
			Cd0 			  = vehicle.propeller.Cd0
			hover_FM 		  = vehicle.propeller.figure_of_merit		# hover figure of merit
		else:
			raise RuntimeError('eVTOL configuration is not available.')

		# -------------------------------------------------------------#
		# --- Calculate power consumptions for each flight segment --- #
		# -------------------------------------------------------------#

		ids_for_max_p = []

		for segment in mission.segments:

			# Unpacking constants for each segment that needs them
			if segment.kind not in ['ConstantPower','NoCreditDescent','ReserveCruise']:
				ids_for_max_p.append(segment.id) 	# segment id for calculating maximum power
				rho_air = segment.constants['rho'] 	# air density
				mu_air 	= segment.constants['mu'] 	# air dynamic viscosity
				g 		= segment.constants['g']	# gravitational acceleration

			# LiftPlusCruise's propellers do not work during hover, hoverclimb, or hoverdescent
			# and its lift rotor does not work during cruise, climb, descent, or constant power segment
			if vehicle.configuration == 'LiftPlusCruise':
				if segment.kind in ['HoverStay', 'HoverClimbConstantSpeed', 'HoverDescentConstantSpeed', 'ConstantPower', 'NoCreditDescent']:
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
				if fidelity['hover_climb'] == 0:
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerHoverClimbConstantSpeedFidelityZero(N_rotor=N_lift_rotor, hover_FM=hover_FM, rho_air=rho_air, g=g, v_climb=v_climb),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|HoverClimbConstantSpeed',f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])
				elif fidelity['hover_climb'] == 1:
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerHoverClimbConstantSpeedFidelityOne(vehicle=vehicle, rho_air=rho_air, g=g, v_climb=v_climb),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|HoverClimbConstantSpeed',f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}'),
														  ('thrust_residual_square',f'LiftRotor|HoverClimb|thrust_residual_square'),
														  ('FM','LiftRotor|HoverClimb|FM')])

			if segment.kind == 'HoverDescentConstantSpeed':
				self.add_subsystem(f'segment_{segment.id}_power',
									PowerHoverDescentConstantSpeed(N_rotor=N_lift_rotor, hover_FM=hover_FM, rho_air=rho_air, g=g, v_descent=v_descent),
									promotes_inputs=['Weight|takeoff', 'LiftRotor|radius'],
									promotes_outputs=[('Power|HoverDescentConstantSpeed',f'Power|LiftRotor|segment_{segment.id}'),
													  ('LiftRotor|thrust',f'LiftRotor|thrust_each|segment_{segment.id}')])
			
			if segment.kind == 'ClimbConstantVyConstantVx':
				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerClimbConstantVyConstantVxEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0, hover_FM=hover_FM, rho_air=rho_air, mu_air=mu_air, g=g, climb_airspeed=segment.speed, gamma=segment.gamma, fidelity=fidelity),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Climb|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerClimbConstantVyConstantVxWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, Cd0=Cd0, hover_FM=hover_FM, rho_air=rho_air, mu_air=mu_air, g=g, AoA=AoA, gamma=segment.gamma, climb_airspeed=segment.speed, fidelity=fidelity),
										promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
														  ('Propeller|Climb|thrust',f'Propeller|thrust_each|segment_{segment.id}')])

			if segment.kind == 'DescentConstantVyConstantVx':
				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerDescentConstantVyConstantVxEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0, hover_FM=hover_FM, rho_air=rho_air, mu_air=mu_air, g=g, descent_airspeed=segment.speed, gamma=segment.gamma, fidelity=fidelity),
										promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
										promotes_outputs=[('Power|DescentConstantVyConstantVx',f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Descent|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerDescentConstantVyConstantVxWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, Cd0=Cd0, hover_FM=hover_FM, rho_air=rho_air, mu_air=mu_air, g=g, AoA=AoA, gamma=segment.gamma, descent_airspeed=segment.speed, fidelity=fidelity),
										promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|DescentConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
														  ('Propeller|Descent|thrust',f'Propeller|thrust_each|segment_{segment.id}')])

			if segment.kind == 'NoCreditDescent':

				self.add_subsystem(f'segment_{segment.id}_power',
									om.ExecComp(['zero_power = 0.0','zero_thrust = 0.0'], zero_power={'units':'W'}, zero_thrust={'units':'N'}),
									promotes_outputs=[('zero_power', f'Power|LiftRotor|segment_{segment.id}'), ('zero_thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

			if segment.kind == 'CruiseConstantSpeed':
				cruise_segment_id = segment.id
				if vehicle.configuration == 'Multirotor':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerCruiseConstantSpeedEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0, hover_FM=hover_FM, rho_air=rho_air, mu_air=mu_air, g=g, fidelity=fidelity),
										promotes_inputs=['Weight|*', 'Mission|*', 'LiftRotor|*'],
										promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
														  ('LiftRotor|Cruise|thrust',f'LiftRotor|thrust_each|segment_{segment.id}'),
														  'Aero|Cruise|total_drag','Aero|Cruise|f_total','Aero|Cruise|f_fuselage','Aero|Cruise|f_rotor_hub'])

				elif vehicle.configuration == 'LiftPlusCruise':
					self.add_subsystem(f'segment_{segment.id}_power',
										PowerCruiseConstantSpeedWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, rho_air=rho_air, mu_air=mu_air, Cd0=Cd0, hover_FM=hover_FM, g=g, AoA=AoA, fidelity=fidelity),
										promotes_inputs=['Weight|*', 'Mission|*', 'Wing|*', 'Propeller|*'],
										promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|Propeller|segment_{segment.id}'),
														   'Aero|Cruise|CL', 'Propeller|Cruise|thrust_coefficient',
														  ('Propeller|Cruise|thrust',f'Propeller|thrust_each|segment_{segment.id}'),
														  'Aero|Cruise|total_drag','Aero|Cruise|f_total','Aero|Cruise|f_fuselage','Aero|Cruise|f_rotor_hub'])
			if segment.kind == 'ConstantPower':

				self.add_subsystem(f'segment_{segment.id}_power',
									PowerConstantFractionOfMaxPower(percent_max_power=segment.percent_max_power),
									promotes_inputs=[('max_power','Power|LiftRotor|maximum')],
									promotes_outputs=[('fractional_power', f'Power|LiftRotor|segment_{segment.id}'), ('zero_thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

			if segment.kind == 'ReserveCruise':
				if vehicle.configuration == 'Multirotor': component = 'LiftRotor'
				elif vehicle.configuration == 'LiftPlusCruise': component = 'Propeller'

				self.add_subsystem(f'reserve_segment_power',
									PowerConstantFractionOfMaxPower(percent_max_power=100.0),
									promotes_inputs=[('max_power',f'Power|{component}|segment_{cruise_segment_id}')],
									promotes_outputs=[('fractional_power', f'Power|reserve_segment')])

		# ------------------------------------------ #
		# ---- Writing maximum thrust equations ---- #
		# ------------------------------------------ #
		# T_max = max(T_segment_1, ..., T_segment_n)

		if vehicle.configuration == 'Multirotor':
			max_thrust_eq = 'T_max = '
			kwargs_T = {'T_max': {'units':'N'}}
			for ctr, i in enumerate(ids_for_max_p):
				if ctr == len(ids_for_max_p)-1:
					max_thrust_eq += f'T_segment_{i}' + (len(ids_for_max_p)-1)*')'
				else:
					max_thrust_eq += f'maximum(T_segment_{i}, '
				kwargs_T[f'T_segment_{i}'] = {'units': 'N'}

		elif vehicle.configuration == 'LiftPlusCruise':
			max_thrust_liftrotor_eq = 'T1_max = '
			max_thrust_propeller_eq = 'T2_max = '
			kwargs_T1 = {'T1_max':{'units':'N'}}
			kwargs_T2 = {'T2_max':{'units':'N'}}
			for ctr, i in enumerate(ids_for_max_p):
				if ctr == len(ids_for_max_p)-1:
					max_thrust_liftrotor_eq += f'T1_segment_{i}' + (len(ids_for_max_p)-1)*')'
					max_thrust_propeller_eq += f'T2_segment_{i}' + (len(ids_for_max_p)-1)*')'
				else:
					max_thrust_liftrotor_eq += f'maximum(T1_segment_{i}, '
					max_thrust_propeller_eq += f'maximum(T2_segment_{i}, '
				kwargs_T1[f'T1_segment_{i}'] = {'units': 'N'}
				kwargs_T2[f'T2_segment_{i}'] = {'units': 'N'}

		# ----------------------------------------- #
		# ---- Writing maximum power equations ---- #
		# ----------------------------------------- #
		# p_max = max(p_segment_1, ..., p_segment_n)

		if vehicle.configuration == 'Multirotor':
			max_power_eq = 'p_max = '
			kwargs_p = {'p_max': {'units':'W'}}
			for ctr, i in enumerate(ids_for_max_p):
				if ctr == len(ids_for_max_p)-1:
					max_power_eq += f'p_segment_{i}' + (len(ids_for_max_p)-1)*')'
				else:
					max_power_eq += f'maximum(p_segment_{i}, '
				kwargs_p[f'p_segment_{i}'] = {'units': 'W'}

		elif vehicle.configuration == 'LiftPlusCruise':
			max_power_liftrotor_eq = 'p1_max = '
			max_power_propeller_eq = 'p2_max = '
			kwargs_p1 = {'p1_max':{'units':'W'}}
			kwargs_p2 = {'p2_max':{'units':'W'}}
			for ctr, i in enumerate(ids_for_max_p):
				if ctr == len(ids_for_max_p)-1:
					max_power_liftrotor_eq += f'p1_segment_{i}' + (len(ids_for_max_p)-1)*')'
					max_power_propeller_eq += f'p2_segment_{i}' + (len(ids_for_max_p)-1)*')'
				else:
					max_power_liftrotor_eq += f'maximum(p1_segment_{i}, '
					max_power_propeller_eq += f'maximum(p2_segment_{i}, '
				kwargs_p1[f'p1_segment_{i}'] = {'units': 'W'}
				kwargs_p2[f'p2_segment_{i}'] = {'units': 'W'}

		# ----------------------------------------------------------------------------- #
		# --- Calculate maximum thrust requirement per component for sizing purpose --- #
		# ----------------------------------------------------------------------------- #
		if vehicle.configuration == 'Multirotor':
			max_thrust_comp = om.ExecComp(max_thrust_eq, **kwargs_T)
			self.add_subsystem('max_thrust_req', max_thrust_comp,
								promotes_outputs=[('T_max','LiftRotor|thrust_each|maximum')])

			for ctr, i in enumerate(ids_for_max_p):
				self.connect(f'LiftRotor|thrust_each|segment_{i}', f'max_thrust_req.T_segment_{i}')

		elif vehicle.configuration == 'LiftPlusCruise':
			max_thrust_liftrotor_comp = om.ExecComp(max_thrust_liftrotor_eq, **kwargs_T1)
			max_thrust_propeller_comp = om.ExecComp(max_thrust_propeller_eq, **kwargs_T2)
			self.add_subsystem('max_thrust_liftrotor_req', max_thrust_liftrotor_comp,
								promotes_outputs=[('T1_max','LiftRotor|thrust_each|maximum')])
			self.add_subsystem('max_thrust_propeller_req', max_thrust_propeller_comp,
								promotes_outputs=[('T2_max','Propeller|thrust_each|maximum')])
			
			for ctr, i in enumerate(ids_for_max_p):
				self.connect(f'LiftRotor|thrust_each|segment_{i}', f'max_thrust_liftrotor_req.T1_segment_{i}')
				self.connect(f'Propeller|thrust_each|segment_{i}', f'max_thrust_propeller_req.T2_segment_{i}')

		# ---------------------------------------------------------------------------- #
		# --- Calculate maximum power requirement per component for sizing purpose --- #
		# ---------------------------------------------------------------------------- #
		if vehicle.configuration == 'Multirotor':
			max_power_comp = om.ExecComp(max_power_eq, **kwargs_p)
			self.add_subsystem('max_power_req', max_power_comp,
								promotes_outputs=[('p_max','Power|LiftRotor|maximum')])

			for ctr, i in enumerate(ids_for_max_p):
				self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_req.p_segment_{i}')

		elif vehicle.configuration == 'LiftPlusCruise':
			max_power_liftrotor_comp = om.ExecComp(max_power_liftrotor_eq, **kwargs_p1)
			max_power_propeller_comp = om.ExecComp(max_power_propeller_eq, **kwargs_p2)
			self.add_subsystem('max_power_liftrotor_req', max_power_liftrotor_comp,
								promotes_outputs=[('p1_max','Power|LiftRotor|maximum')])
			self.add_subsystem('max_power_propeller_req', max_power_propeller_comp,
								promotes_outputs=[('p2_max','Power|Propeller|maximum')])
			
			for ctr, i in enumerate(ids_for_max_p):
				self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_liftrotor_req.p1_segment_{i}')
				self.connect(f'Power|Propeller|segment_{i}', f'max_power_propeller_req.p2_segment_{i}')

		# ------------------------------------------------------#
		# --- Calculate total power requirement per segment --- #
		# ------------------------------------------------------#
		if vehicle.configuration == 'Multirotor':
			for i in range(1, mission.n_segments+1):
				kwargs = {f'power_segment_{i}': {'units':'W'}, f'p_{i}': {'units':'W'}}
				segment_power_comp = om.ExecComp(f'power_segment_{i} = p_{i}', **kwargs)
				self.add_subsystem(f'power_segment_req_{i}', segment_power_comp,
									promotes_outputs=[(f'power_segment_{i}',f'Power|segment_{i}')])
				self.connect(f'Power|LiftRotor|segment_{i}', f'power_segment_req_{i}.p_{i}')

		elif vehicle.configuration == 'LiftPlusCruise':
			for i in range(1, mission.n_segments+1):
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
			for i in range(1, mission.n_segments+1):
				DL_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
									   disk_loading 	= {'units': 'N/m**2'},
									   thrust 			= {'units': 'N'},
									   r 				= {'units': 'm'})
				self.add_subsystem(f'DL_segment_{i}', DL_comp,
									promotes_inputs = [('r', 'LiftRotor|radius')],
									promotes_outputs= [('disk_loading', f'DiskLoading|LiftRotor|segment_{i}')] )
				self.connect(f'LiftRotor|thrust_each|segment_{i}', f'DL_segment_{i}.thrust')

		elif vehicle.configuration == 'LiftPlusCruise':
			for i in range(1, mission.n_segments+1):
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

		self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=100, iprint=0, rtol=1e-3)
		self.nonlinear_solver.options['err_on_non_converge'] = False
		self.nonlinear_solver.options['reraise_child_analysiserror'] = False
		self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
		self.nonlinear_solver.linesearch.options['maxiter'] = 10
		self.nonlinear_solver.linesearch.options['iprint'] = 0
		self.linear_solver = om.DirectSolver(assemble_jac=True)












