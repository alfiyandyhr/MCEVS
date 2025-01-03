import numpy as np
import openmdao.api as om
from scipy.optimize import bisect
from MCEVS.Analyses.Aerodynamics.BEMT.Rotor import initialize_rotor, RotorPerformanceCoeffs
from MCEVS.Analyses.Aerodynamics.BEMT.Section import initialize_sections
from MCEVS.Analyses.Aerodynamics.BEMT.SectionOM import SectionSolverOM, SectionForces

class BEMTSolver:
	"""
	Solver object
		Inputs:
			rotorDict
			sectionDict
			fluidDict
		Output: rotor performance
	"""
	def __init__(self, rotorDict:dict, sectionDict:dict, fluidDict:dict):

		# Check input dictionaries
		rotorDict_keys = ['nblades', 'diameter', 'hub_radius', 'global_twist']
		sectionDict_keys = ['airfoil_list', 'radius_list', 'chord_list', 'pitch_list']
		fluidDict_keys = ['rho', 'mu']

		if all(key in rotorDict for key in rotorDict_keys): pass
		else: raise SyntaxError('Keys in rotorDict are not complete.')

		if all(key in sectionDict for key in sectionDict_keys): pass
		else: raise SyntaxError('Keys in sectionDict are not complete.')

		if all(key in fluidDict for key in fluidDict_keys): pass
		else: raise SyntaxError('Keys in fluidDict are not complete.')

		self.sectionDict = sectionDict
		self.fluidDict = fluidDict

		# Initialization
		self.rotor = initialize_rotor(rotorDict)
		self.section_list = initialize_sections(sectionDict, rotorDict)

	def run(self, v_inf, rpm):

		# Thrust and torque initialization
		T = 0.0
		Q = 0.0

		# Rotor operating point
		omega = rpm*2*np.pi/60.0
		self.rotor.op['v_inf'] = v_inf
		self.rotor.op['rpm'] = rpm
		self.rotor.op['omega'] = omega
		self.rotor.op['n'] = rpm/60.0
		self.rotor.op['J'] = v_inf*60.0/(self.rotor.diameter*rpm)

		nblades = self.rotor.nblades
		blade_radius = self.rotor.blade_radius
		hub_radius = self.rotor.hub_radius
		rho = self.fluidDict['rho']
		mu = self.fluidDict['mu']

		for section in self.section_list:
			try:
				phi = bisect(section._calc_inflow_angle_residual,
							 0.01*np.pi, 0.9*np.pi,
							 args=(v_inf, omega, nblades, blade_radius, hub_radius))
			except ValueError:
				raise ValueError('Solver run unsuccessful.')

			dT, dQ = section._calc_forces(phi, v_inf, omega, rho, mu, nblades, blade_radius, hub_radius)

			# Integrate
			T += dT
			Q += dQ

		# Power = torque * omega
		P = Q * omega

		# Rotor performance
		FM, CT, CQ, CP, eta = self.rotor._calc_performance(T, Q, P, rho)

		# Results book-keeping
		results = {'T':T, 'Q':Q, 'P':P, 'FM':FM,
				   'CT':CT, 'CQ':CQ, 'CP':CP, 'eta':eta,
				   'rpm':rpm, 'J': self.rotor.op['J']}

		return results

	def trim_rpm(self, T_req, v_inf, rpm_bounds=[100,2000]):

		def thrust_residual(rpm, T_req, v_inf):
			T_calc = self.run(v_inf, rpm)['T']
			return (T_req - T_calc)

		try:
			trimmed_rpm = bisect(thrust_residual, rpm_bounds[0], rpm_bounds[1], args=(T_req, v_inf))
		except:
			raise ValueError('Solver trim unsuccessful. Might need to change the rpm_bounds.')

		results = self.run(v_inf, trimmed_rpm)

		results['T_residual'] = T_req - results['T']

		return results


class BEMTSolverOM(object):
	"""
	docstring for BEMTSolverOM
	"""
	def __init__(self, rotorDict:dict, sectionDict:dict, fluidDict:dict):
		super(BEMTSolverOM, self).__init__()
		self.rotorDict = rotorDict
		self.sectionDict = sectionDict
		self.fluidDict = fluidDict

	def run(self, v_inf, rpm):

		# --- OpenMDAO probolem --- #
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		# --- Design parameters --- #
		indeps.add_output('v_inf', v_inf, units='m/s')
		indeps.add_output('rpm', rpm, units='rpm')
		indeps.add_output('diameter', self.rotorDict['diameter'], units='m')
		indeps.add_output('blade_radius', self.rotorDict['diameter']/2, units='m')
		indeps.add_output('hub_radius', self.rotorDict['hub_radius'], units='m')
		indeps.add_output('global_twist', self.rotorDict['global_twist'], units='deg')
		
		for i in range(len(self.sectionDict['airfoil_list'])):
			if i == 0:
				width = self.sectionDict['radius_list'][i] - self.rotorDict['hub_radius']
			else:
				width = self.sectionDict['radius_list'][i] - self.sectionDict['radius_list'][i-1]	
			indeps.add_output(f'Section{i+1}|radius', self.sectionDict['radius_list'][i], units='m')
			indeps.add_output(f'Section{i+1}|chord', self.sectionDict['chord_list'][i], units='m')
			indeps.add_output(f'Section{i+1}|pitch', self.sectionDict['pitch_list'][i], units='deg')
			indeps.add_output(f'Section{i+1}|width', width, units='m')

		prob.model.add_subsystem('BEMT_Solver',
								  BEMTSolverOMGroup(nblades=self.rotorDict['nblades'],
								  				    airfoil_list=self.sectionDict['airfoil_list'],
								  				    rho=self.fluidDict['rho'],
								  				    mu=self.fluidDict['mu'],
								  				    trim_rpm=False),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])
	
		prob.setup(check=False)
		prob.run_model()
		# prob.check_partials(compact_print=True)

		# Results book-keeping
		T = prob.get_val('Thrust','N')[0]
		Q = prob.get_val('Torque','N*m')[0]
		P = prob.get_val('Power','W')[0]
		FM = prob.get_val('FM')[0]
		CT = prob.get_val('CT')[0]
		CQ = prob.get_val('CQ')[0]
		CP = prob.get_val('CP')[0]
		eta = prob.get_val('eta')[0]
		rpm = prob.get_val('rpm')[0]
		omega = prob.get_val('omega','rad/s')[0]
		J = prob.get_val('J')[0]

		results = {'T':T, 'Q':Q, 'P':P, 'FM':FM,
				   'CT':CT, 'CQ':CQ, 'CP':CP, 'eta':eta,
				   'rpm':rpm, 'J': J}

		return results

	def trim_rpm(self, T_req, v_inf, rpm_bounds=[100, 2000]):

		# --- OpenMDAO probolem --- #
		prob = om.Problem()

		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		# --- Design parameters --- #
		indeps.add_output('T_req', T_req, units='N')
		indeps.add_output('v_inf', v_inf, units='m/s')
		indeps.add_output('rpm', 0.5*(rpm_bounds[0]+rpm_bounds[1]), units='rpm')
		indeps.add_output('diameter', self.rotorDict['diameter'], units='m')
		indeps.add_output('blade_radius', self.rotorDict['diameter']/2, units='m')
		indeps.add_output('hub_radius', self.rotorDict['hub_radius'], units='m')
		indeps.add_output('global_twist', self.rotorDict['global_twist'], units='deg')
		
		for i in range(len(self.sectionDict['airfoil_list'])):
			if i == 0:
				width = self.sectionDict['radius_list'][i] - self.rotorDict['hub_radius']
			else:
				width = self.sectionDict['radius_list'][i] - self.sectionDict['radius_list'][i-1]	
			indeps.add_output(f'Section{i+1}|radius', self.sectionDict['radius_list'][i], units='m')
			indeps.add_output(f'Section{i+1}|chord', self.sectionDict['chord_list'][i], units='m')
			indeps.add_output(f'Section{i+1}|pitch', self.sectionDict['pitch_list'][i], units='deg')
			indeps.add_output(f'Section{i+1}|width', width, units='m')
		
		prob.model.add_subsystem('BEMT_Solver',
								  BEMTSolverOMGroup(nblades=self.rotorDict['nblades'],
								  				    airfoil_list=self.sectionDict['airfoil_list'],
								  				    rho=self.fluidDict['rho'],
								  				    mu=self.fluidDict['mu'],
								  				    trim_rpm=True),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		# Setup as an optimization problem
		prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=False)
		prob.model.add_design_var('rpm', lower=rpm_bounds[0], upper=rpm_bounds[1])
		prob.model.add_objective('thrust_residual_square')
	
		prob.setup(check=False)
		prob.run_driver()
		# prob.check_partials(compact_print=True)

		# Results book-keeping
		T = prob.get_val('Thrust','N')[0]
		Q = prob.get_val('Torque','N*m')[0]
		P = prob.get_val('Power','W')[0]
		FM = prob.get_val('FM')[0]
		CT = prob.get_val('CT')[0]
		CQ = prob.get_val('CQ')[0]
		CP = prob.get_val('CP')[0]
		eta = prob.get_val('eta')[0]
		rpm = prob.get_val('rpm')[0]
		omega = prob.get_val('omega','rad/s')[0]
		J = prob.get_val('J')[0]
		T_residual = np.sqrt(prob.get_val('thrust_residual_square')[0])

		results = {'T':T, 'Q':Q, 'P':P, 'FM':FM,
				   'CT':CT, 'CQ':CQ, 'CP':CP, 'eta':eta,
				   'rpm':rpm, 'J': J, 'T_residual': T_residual}

		return results

class BEMTSolverOMGroup(om.Group):
	"""
	docstring for BEMTSolverOMGroup
	"""
	def initialize(self):
		self.options.declare('nblades', types=int, desc='Number of blades per rotor')
		self.options.declare('airfoil_list', types=list, desc='List of sectional airfoils')
		self.options.declare('rho', types=float, desc='Air density')
		self.options.declare('mu', types=float, desc='Air dynamic viscosity')
		self.options.declare('trim_rpm', types=bool, desc='Whether to use in trim mode (find trimmed_rpm)')

	def setup(self):
		nblades = self.options['nblades']
		airfoil_list = self.options['airfoil_list']
		rho = self.options['rho']
		mu = self.options['mu']
		trim_rpm = self.options['trim_rpm']

		self.add_subsystem(f'rpm2omega',
							om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units':'rad/s'}, rpm={'units':'rpm'}),
							promotes_inputs=['rpm'],
							promotes_outputs=['omega'])

		# --- Solver for each section --- #

		for i in range(len(airfoil_list)):

			input_list = ['v_inf','omega','blade_radius','hub_radius',
						   ('radius',f'Section{i+1}|radius'),
						   ('pitch',f'Section{i+1}|pitch'),
						   ('chord',f'Section{i+1}|chord')]

			output_list = [('dr_tip',f'Section{i+1}|dr_tip'),
						   ('dr_hub',f'Section{i+1}|dr_hub'),
						   ('AoA',f'Section{i+1}|AoA'),
						   ('Cl',f'Section{i+1}|Cl'),
						   ('Cd',f'Section{i+1}|Cd'),
						   ('CT',f'Section{i+1}|CT'),
						   ('CQ',f'Section{i+1}|CQ'),
						   ('kappa',f'Section{i+1}|kappa'),
						   ('kappap',f'Section{i+1}|kappap'),
						   ('a',f'Section{i+1}|a'),
						   ('ap',f'Section{i+1}|ap'),
						   ('phi_residual',f'Section{i+1}|phi_residual'),
						   ('phi',f'Section{i+1}|phi')]

			self.add_subsystem(f'section{i+1}_solver',
								SectionSolverOM(airfoil=airfoil_list[i], nblades=nblades, rho=rho, mu=mu),
								promotes_inputs=input_list,
								promotes_outputs=output_list)

			self.add_subsystem(f'section{i+1}_forces',
								SectionForces(nblades=nblades, rho=rho, mu=mu),
								promotes_inputs=['v_inf','omega',
												 ('a',f'Section{i+1}|a'),
												 ('ap',f'Section{i+1}|ap'),
												 ('CT',f'Section{i+1}|CT'),
												 ('CQ',f'Section{i+1}|CQ'),
												 ('radius',f'Section{i+1}|radius'),
												 ('width',f'Section{i+1}|width'),
												 ('chord',f'Section{i+1}|chord')],
								promotes_outputs=[('dT',f'Section{i+1}|dT'),
												  ('dQ',f'Section{i+1}|dQ')])

		# --- Thrust and Torque integration --- #

		thrust_result_eq = 'Thrust = '
		thrust_input_list = []
		kwargs_t = {'Thrust':{'units':'N'}}
		for i in range(len(airfoil_list)):
			kwargs_t[f'thrust_{i+1}'] = {'units': 'N'}
			if i == len(airfoil_list)-1:
				thrust_result_eq += f'thrust_{i+1}'
			else:
				thrust_result_eq += f'thrust_{i+1} + '
			thrust_input_list.append((f'thrust_{i+1}',f'Section{i+1}|dT'))

		torque_result_eq = 'Torque = '
		torque_input_list = []
		kwargs_tq = {'Torque':{'units':'N*m'}}
		for i in range(len(airfoil_list)):
			kwargs_tq[f'torque_{i+1}'] = {'units': 'N*m'}
			if i == len(airfoil_list)-1:
				torque_result_eq += f'torque_{i+1}'
			else:
				torque_result_eq += f'torque_{i+1} + '
			torque_input_list.append((f'torque_{i+1}',f'Section{i+1}|dQ'))

		self.add_subsystem('thrust_resultant',
							om.ExecComp(thrust_result_eq, **kwargs_t),
							promotes_inputs=thrust_input_list,
							promotes_outputs=['Thrust'])

		self.add_subsystem('torque_resultant',
							om.ExecComp(torque_result_eq, **kwargs_tq),
							promotes_inputs=torque_input_list,
							promotes_outputs=['Torque'])

		self.add_subsystem('power_calc',
							om.ExecComp('Power = Torque * omega', Power={'units':'W'}, Torque={'units':'N*m'}, omega={'units':'rad/s'}),
							promotes_inputs=['Torque','omega'],
							promotes_outputs=['Power'])

		if trim_rpm:
			thrust_residual_eqn = 'thrust_residual_square = (Thrust - T_req)**2'
			self.add_subsystem('thrust_residual_comp',
								om.ExecComp(thrust_residual_eqn, Thrust={'units':'N'}, T_req={'units':'N'}),
								promotes_inputs=['Thrust', 'T_req'],
								promotes_outputs=['thrust_residual_square'])

		# --- Rotor performance --- #
		self.add_subsystem('rotor_performance_coeffs',
							RotorPerformanceCoeffs(rho=rho),
							promotes_inputs=[('T','Thrust'),('Q','Torque'),('P','Power'),'v_inf','omega','blade_radius'],
							promotes_outputs=['CT','CQ','CP','eta','FM','J'])
