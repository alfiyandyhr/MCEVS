import numpy as np
from scipy import optimize
from MCEVS.Analyses.Aerodynamics.BEMT.Rotor import initialize_rotor
from MCEVS.Analyses.Aerodynamics.BEMT.Section import initialize_sections

class BEMTSolver:
	"""
	Solver object
		Inputs:
			caseDict
			rotorDict
			sectionDict
			fluidDict
		Output: rotor performance
	"""
	def __init__(self, caseDict:dict, rotorDict:dict, sectionDict:dict, fluidDict:dict):

		# Check input dictionaries
		caseDict_keys = ['v_inf', 'rpm']
		rotorDict_keys = ['nblades', 'diameter', 'hub_radius', 'global_twist']
		sectionDict_keys = ['airfoil_list', 'radius_list', 'chord_list', 'pitch_list']
		fluidDict_keys = ['rho', 'mu']

		if all(key in caseDict for key in caseDict_keys): pass
		else: raise SyntaxError('Keys in caseDict are not complete.')

		if all(key in rotorDict for key in rotorDict_keys): pass
		else: raise SyntaxError('Keys in rotorDict are not complete.')

		if all(key in sectionDict for key in sectionDict_keys): pass
		else: raise SyntaxError('Keys in sectionDict are not complete.')

		if all(key in fluidDict for key in fluidDict_keys): pass
		else: raise SyntaxError('Keys in fluidDict are not complete.')

		self.caseDict = caseDict
		self.sectionDict = sectionDict
		self.fluidDict = fluidDict

		# Initialization
		self.rotor = initialize_rotor(rotorDict, caseDict)
		self.section_list = initialize_sections(sectionDict, rotorDict)

	def run(self):

		# Thrust and torque initialization
		T = 0.0
		Q = 0.0

		v_inf = self.rotor.op['v_inf']
		omega = self.rotor.op['omega']
		nblades = self.rotor.nblades
		blade_radius = self.rotor.blade_radius
		hub_radius = self.rotor.hub_radius
		rho = self.fluidDict['rho']
		mu = self.fluidDict['mu']

		for section in self.section_list:
			try:
				phi = optimize.bisect(section._calc_inflow_angle_residual,
									  0.01*np.pi, 0.9*np.pi,
									  args=(v_inf, omega, nblades, blade_radius, hub_radius))
			except ValueError:
				print('Solver run unsuccessful.')

			dT, dQ = section._calc_forces(phi, v_inf, omega, rho, mu, nblades, blade_radius, hub_radius)

			# Integrate
			T += dT
			Q += dQ

		# Power = torque * omega
		P = Q * omega

		# Rotor performance
		FM, CT, CQ, CP, eta = self.rotor._calc_performance(T, Q, P, rho)

		# Rotor operating point
		rpm = self.caseDict['rpm']
		J = self.rotor.op['J']

		# Results book-keeping
		results = {'T':T, 'Q':Q, 'P':P, 'FM':FM,
				   'CT':CT, 'CQ':CQ, 'CP':CP, 'eta':eta,
				   'rpm':rpm, 'J': J}

		return results










