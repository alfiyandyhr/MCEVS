from MCEVS.Analyses.Aerodynamics.BEMT.Airfoil import AirfoilCoeffs
import warnings
import openmdao.api as om
import numpy as np

class SectionSolverOM(om.Group):
	"""
	docstring for SectionSolverOM that runs under OpenMDAO framework
	"""
	def initialize(self):
		self.options.declare('airfoil', types=str, desc='Airfoil name')
		self.options.declare('nblades', types=int, desc='Number of blades per rotor')
		self.options.declare('rho', types=float, desc='Air density')
	
	def setup(self):
		airfoil = self.options['airfoil']
		nblades = self.options['nblades']
		rho = self.options['rho']

		self.add_subsystem(f'dr_tip_calc',
							om.ExecComp('dr_tip = blade_radius - radius', dr_tip={'units':'m'}, blade_radius={'units':'m'}, radius={'units':'m'}),
							promotes_inputs=['blade_radius','radius'],
							promotes_outputs=['dr_tip'])
		self.add_subsystem(f'dr_hub_calc',
							om.ExecComp('dr_hub = radius - hub_radius', dr_hub={'units':'m'}, hub_radius={'units':'m'}, radius={'units':'m'}),
							promotes_inputs=['hub_radius','radius'],
							promotes_outputs=['dr_hub'])

		self.add_subsystem('tip_and_hub_loss_factor_group',
							TipAndHubLossFactor(nblades=nblades),
							promotes_inputs=['dr_tip','dr_hub','phi','radius'],
							promotes_outputs=['F'])

		self.add_subsystem('AoA_calc',
							om.ExecComp('AoA = pitch - phi * 180/pi', AoA={'units':'deg'}, pitch={'units':'deg'}, phi={'units':'rad'}),
							promotes_inputs=['pitch','phi'],
							promotes_outputs=['AoA'])

		self.add_subsystem('airfoil_coeffs',
							AirfoilCoeffs(airfoil=airfoil),
							promotes_inputs=['AoA'],
							promotes_outputs=['Cl','Cd'])

		self.add_subsystem('thrust_torque_coeffs',
							ThrustTorqueCoeffs(),
							promotes_inputs=['Cl','Cd','phi'],
							promotes_outputs=['CT','CQ'])

		self.add_subsystem('induction_factors_kappa',
							InductionFactorsKappa(nblades=nblades),
							promotes_inputs=['F','phi','chord','radius','CT','CQ'],
							promotes_outputs=['kappa','kappap'])

		self.add_subsystem('induction_factors',
							InductionFactors(),
							promotes_inputs=['kappa','kappap'],
							promotes_outputs=['a','ap'])

		phi_residual_eqn = 'phi_residual = sin(phi)/(1.0 + a) - v_inf*cos(phi)/(omega*radius*(1.0 - ap))'
		self.add_subsystem('phi_residual_comp',
							om.ExecComp(phi_residual_eqn, phi={'units':'rad'},v_inf={'units':'m/s'},radius={'units':'m'},omega={'units':'rad/s'}),
							promotes_inputs=['phi', 'a', 'ap', 'v_inf', 'omega', 'radius'],
							promotes_outputs=['phi_residual'])

		# This drives phi_residual = 0 by varying phi. LB and UB of phi should be given.
		residual_balance = om.BalanceComp('phi',
										   units='rad',
										   eq_units=None,
										   lower=0*np.pi,
										   upper=2*np.pi,
										   val=(0.01+0.9)/2*np.pi,
										   rhs_val=0.0,
										   use_mult=False)
		self.add_subsystem('phi_residual_balance',
							residual_balance,
							promotes_inputs=[('lhs:phi', 'phi_residual')],
							promotes_outputs=['phi'])

		# Add solvers for implicit relations
		self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=200, iprint=0, rtol=1e-3)
		self.nonlinear_solver.options['err_on_non_converge'] = False
		self.nonlinear_solver.options['reraise_child_analysiserror'] = True
		self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
		self.nonlinear_solver.linesearch.options['maxiter'] = 10
		self.nonlinear_solver.linesearch.options['iprint'] = 0
		self.linear_solver = om.DirectSolver(assemble_jac=True)

class SectionForces(om.ExplicitComponent):
	"""
	Parameters: nblades, rho
	Inputs: a, ap, CT, CQ, v_inf, omega, radius, width, chord
	Outputs: dT, dQ
	"""
	def initialize(self):
		self.options.declare('nblades', types=int)
		self.options.declare('rho', types=float)

	def setup(self):
		self.add_input('a', units=None)
		self.add_input('ap', units=None)
		self.add_input('CT', units=None)
		self.add_input('CQ', units=None)
		self.add_input('v_inf', units='m/s')
		self.add_input('omega', units='rad/s')
		self.add_input('radius', units='m')
		self.add_input('width', units='m')
		self.add_input('chord', units='m')
		self.add_output('dT', units='N')
		self.add_output('dQ', units='N*m')
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		nblades = self.options['nblades']
		rho = self.options['rho']
		a = inputs['a']
		ap = inputs['ap']
		CT = inputs['CT']
		CQ = inputs['CQ']
		v_inf = inputs['v_inf']
		omega = inputs['omega']
		radius = inputs['radius']
		width = inputs['width']
		chord = inputs['chord']

		solidity = nblades*chord/(2*np.pi*radius)

		v = (1.0 + a) * v_inf
		vp = (1.0 - ap) * omega * radius

		U = np.sqrt(v**2 + vp**2)

		outputs['dT'] = solidity * np.pi * rho * U**2 * CT * radius * width
		outputs['dQ'] = solidity * np.pi * rho * U**2 * CQ * radius**2 * width

	def compute_partials(self, inputs, partials):
		nblades = self.options['nblades']
		rho = self.options['rho']
		a = inputs['a']
		ap = inputs['ap']
		CT = inputs['CT']
		CQ = inputs['CQ']
		v_inf = inputs['v_inf']
		omega = inputs['omega']
		radius = inputs['radius']
		width = inputs['width']
		chord = inputs['chord']

		v = (1.0 + a) * v_inf
		vp = (1.0 - ap) * omega * radius

		U = np.sqrt(v**2 + vp**2)
		dU_da = 0.5/np.sqrt(v**2 + vp**2) * (2*v) * v_inf
		dU_dvinf = 0.5/np.sqrt(v**2 + vp**2) * (2*v) * (1.0 + a)
		dU_dap = 0.5/np.sqrt(v**2 + vp**2) * (2*vp) * (-omega*radius)
		dU_domega = 0.5/np.sqrt(v**2 + vp**2) * (2*vp) * (1.0 - ap)*radius
		dU_dradius = 0.5/np.sqrt(v**2 + vp**2) * (2*vp) * (1.0 - ap)*omega
		
		partials['dT','a'] = 0.5 * nblades * chord * rho * CT * width * (2*U) * dU_da
		partials['dT','ap'] = 0.5 * nblades * chord * rho * CT * width * (2*U) * dU_dap
		partials['dT','CT'] = 0.5 * nblades * chord * rho * U**2 * width
		partials['dT','CQ'] = 0.0
		partials['dT','v_inf'] = 0.5 * nblades * chord * rho * CT * width * (2*U) * dU_dvinf
		partials['dT','omega'] = 0.5 * nblades * chord * rho * CT * width * (2*U) * dU_domega
		partials['dT','radius'] = 0.5 * nblades * chord * rho * CT * width * (2*U) * dU_dradius
		partials['dT','width'] = 0.5 * nblades * chord * rho * U**2 * CT
		partials['dT','chord'] = 0.5 * nblades * rho * U**2 * CT * width
		partials['dQ','a'] = 0.5 * nblades * chord * rho * CQ * radius * width * (2*U) * dU_da
		partials['dQ','ap'] = 0.5 * nblades * chord * rho * CQ * radius * width * (2*U) * dU_dap
		partials['dQ','CT'] = 0.0
		partials['dQ','CQ'] = 0.5 * nblades * chord * rho * U**2 * radius * width
		partials['dQ','v_inf'] = 0.5 * nblades * chord * rho * CQ * radius * width * (2*U) * dU_dvinf
		partials['dQ','omega'] = 0.5 * nblades * chord * rho * CQ * radius * width * (2*U) * dU_domega
		partials['dQ','radius'] = (0.5*nblades*chord*rho*U**2*CQ*width) + (0.5*nblades*chord*rho*CQ*radius*width*(2*U)*dU_dradius)
		partials['dQ','width'] = 0.5 * nblades * chord * rho * U**2 * CQ * radius
		partials['dQ','chord'] = 0.5 * nblades * rho * U**2 * CQ * radius * width

class InductionFactors(om.ExplicitComponent):
	"""
	Inputs: kappa, kappap
	Outputs: a, ap
	"""
	def setup(self):
		self.add_input('kappa', units=None)
		self.add_input('kappap', units=None)
		self.add_output('a', units=None)
		self.add_output('ap', units=None)
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		kappa = inputs['kappa']
		kappap = inputs['kappap']

		outputs['a'] = 1.0 / (kappa - 1.0)
		outputs['ap'] = 1.0 / (kappap + 1.0)
		
	def compute_partials(self, inputs, partials):
		kappa = inputs['kappa']
		kappap = inputs['kappap']
		partials['a','kappa'] = - 1.0 / (kappa - 1.0)**2
		partials['a','kappap'] = 0.0
		partials['ap','kappa'] = 0.0
		partials['ap','kappap'] = - 1.0 / (kappap + 1.0)**2

class InductionFactorsKappa(om.ExplicitComponent):
	"""
	Parameter: nblades
	Inputs: F, phi, chord, radius, CT, CQ
	Outputs: kappa, kappap
	"""
	def initialize(self):
		self.options.declare('nblades', types=int)

	def setup(self):
		self.add_input('F', units=None)
		self.add_input('phi', units='rad')
		self.add_input('chord', units='m')
		self.add_input('radius', units='m')
		self.add_input('CT', units=None)
		self.add_input('CQ', units=None)
		self.add_output('kappa', units=None)
		self.add_output('kappap', units=None)
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		nblades = self.options['nblades']
		F = inputs['F']
		phi = inputs['phi']
		chord = inputs['chord']
		radius = inputs['radius']
		CT = inputs['CT']
		CQ = inputs['CQ']

		solidity = nblades*chord/(2*np.pi*radius)

		outputs['kappa'] = 4*F*np.sin(phi)**2/(solidity*CT)
		outputs['kappap'] = 4*F*np.sin(phi)*np.cos(phi)/(solidity*CQ)

	def compute_partials(self, inputs, partials):
		nblades = self.options['nblades']
		F = inputs['F']
		phi = inputs['phi']
		chord = inputs['chord']
		radius = inputs['radius']
		CT = inputs['CT']
		CQ = inputs['CQ']
		solidity = nblades*chord/(2*np.pi*radius)
		dsolidity_dchord = nblades/(2*np.pi*radius)
		dsolidity_dradius = nblades*chord/(2*np.pi) * (-1/radius**2)

		partials['kappa','F'] = 4*np.sin(phi)**2/(solidity*CT)
		partials['kappa','phi'] = 4*F*2*np.sin(phi)*np.cos(phi)/(solidity*CT)
		partials['kappa','chord'] = 4*F*np.sin(phi)**2/CT * (-1/solidity**2) * dsolidity_dchord
		partials['kappa','radius'] = 4*F*np.sin(phi)**2/CT * (-1/solidity**2) * dsolidity_dradius
		partials['kappa','CT'] = 4*F*np.sin(phi)**2/solidity * (-1/CT**2)
		partials['kappa','CQ'] = 0.0
		partials['kappap','F'] = 4*np.sin(phi)*np.cos(phi)/(solidity*CQ)
		partials['kappap','phi'] = 4*F*np.cos(2*phi)/(solidity*CQ)
		partials['kappap','chord'] = 4*F*np.sin(phi)*np.cos(phi)/CQ * (-1/solidity**2) * dsolidity_dchord
		partials['kappap','radius'] = 4*F*np.sin(phi)*np.cos(phi)/CQ * (-1/solidity**2) * dsolidity_dradius
		partials['kappap','CT'] = 0.0
		partials['kappap','CQ'] = 4*F*np.sin(phi)*np.cos(phi)/solidity * (-1/CQ**2)

class ThrustTorqueCoeffs(om.ExplicitComponent):
	"""
	Inputs: Cl, Cd, phi
	Output: CT, CQ
	"""
	def setup(self):
		self.add_input('Cl', units=None)
		self.add_input('Cd', units=None)
		self.add_input('phi', units='rad')
		self.add_output('CT', units=None)
		self.add_output('CQ', units=None)
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		Cl = inputs['Cl']
		Cd = inputs['Cd']
		phi = inputs['phi']
		outputs['CT'] = Cl*np.cos(phi) - Cd*np.sin(phi)
		outputs['CQ'] = Cl*np.sin(phi) + Cd*np.cos(phi)

	def compute_partials(self, inputs, partials):
		Cl = inputs['Cl']
		Cd = inputs['Cd']
		phi = inputs['phi']
		partials['CT','Cl'] = np.cos(phi)
		partials['CT','Cd'] = -np.sin(phi)
		partials['CT','phi'] = -Cl*np.sin(phi) - Cd*np.cos(phi)
		partials['CQ','Cl'] = np.sin(phi)
		partials['CQ','Cd'] = np.cos(phi)
		partials['CQ','phi'] = Cl*np.cos(phi) - Cd*np.sin(phi)

class TipAndHubLossFactor(om.Group):
	"""
	Parameters: nblades
	Inputs: phi, radius, dr
	Output: F
	"""
	def initialize(self):
		self.options.declare('nblades', types=int)

	def setup(self):
		nblades = self.options['nblades']

		# Ftip
		self.add_subsystem('tip_loss_factor',
						   Prandtl(nblades=nblades),
						   promotes_inputs=[('dr','dr_tip'),'radius','phi'],
						   promotes_outputs=[('F', 'Ftip')])
		# Fhub
		self.add_subsystem('hub_loss_factor',
						   Prandtl(nblades=nblades),
						   promotes_inputs=[('dr','dr_hub'),'radius','phi'],
						   promotes_outputs=[('F', 'Fhub')])

		# F total 
		self.add_subsystem('tip_and_hub_loss_factor',
							om.ExecComp('F = Ftip * Fhub', F={'units':None}),
							promotes_inputs=['Ftip', 'Fhub'],
							promotes_outputs=['F'])

class Prandtl(om.ExplicitComponent):
	"""
	Parameter: nblades
	Inputs: dr, r, phi
	Output: F
	"""
	def initialize(self):
		self.options.declare('nblades', types=int)

	def setup(self):
		self.add_input('dr', units='m')
		self.add_input('radius', units='m')
		self.add_input('phi', units='rad')
		self.add_output('F', units=None)
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		nblades = self.options['nblades']
		dr = inputs['dr']
		r = inputs['radius']
		phi = inputs['phi']

		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			# --- sometimes phi can be let to zero --- #
			f = nblades * dr / (2*r*np.sin(phi))

		if (-f > 500): # exp can overflow for very large numbers
			outputs['F'] = 1.0
		else:
			outputs['F'] = 2/np.pi * np.arccos(min(1.0, np.exp(-f)))

	def compute_partials(self, inputs, partials):
		nblades = self.options['nblades']
		dr = inputs['dr']
		r = inputs['radius']
		phi = inputs['phi']

		f = nblades * dr / (2*r*np.sin(phi))
		df_ddr = nblades / (2*r*np.sin(phi))
		df_dr = nblades * dr / (2*np.sin(phi)) * (-1/r**2)
		df_dphi = nblades * dr / (2*r) * (-1/(np.sin(phi)*np.sin(phi)))*np.cos(phi)
		if (-f > 500): # exp can overflow for very large numbers
			partials['F','dr'] = 0.0
			partials['F','radius'] = 0.0
			partials['F','phi'] = 0.0
		else:
			F = 2/np.pi * np.arccos(min(1.0, np.exp(-f)))
			if np.exp(-f)<1:
				partials['F','dr'] = np.exp(-f) / np.sqrt(1 - np.exp(-2*f)) * df_ddr
				partials['F','radius'] = np.exp(-f) / np.sqrt(1 - np.exp(-2*f)) * df_dr
				partials['F','phi'] = np.exp(-f) / np.sqrt(1 - np.exp(-2*f)) * df_dphi
			else:
				partials['F','dr'] = 0.0
				partials['F','radius'] = 0.0
				partials['F','phi'] = 0.0