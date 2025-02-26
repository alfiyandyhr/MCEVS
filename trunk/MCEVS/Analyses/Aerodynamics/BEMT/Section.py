import numpy as np
from scipy.interpolate import Akima1DInterpolator
from MCEVS.Analyses.Aerodynamics.BEMT.Airfoil import load_airfoil

class Section(object):
	"""
	Section object
	Notes:
		> should be called with "initialize_sections" function
	"""
	def __init__(self, airfoil, width, radius, chord, pitch, solidity):
		super(Section, self).__init__()
		
		self.airfoil = airfoil
		self.width = width
		self.radius = radius
		self.chord = chord
		self.pitch = pitch
		self.solidity = solidity

		self.loss_factor = None
		self.AoA = None
		self.Cl = None
		self.Cd = None
		self.Re = None

		self.dT = None
		self.dQ = None

	def _calc_inflow_angle_residual(self, phi, v_inf, omega, nblades, blade_radius, hub_radius):
		"""
		Given local inflow angle phi in rad
		"""
		a, ap = self._calc_induction_factors(phi, nblades, blade_radius, hub_radius)

		residual = np.sin(phi)/(1.0 + a) - v_inf*np.cos(phi)/(omega*self.radius*(1.0 - ap))

		return residual

	def _calc_induction_factors(self, phi, nblades, blade_radius, hub_radius):
		"""
		Calculation of axial and tangential induction factors
		"""
		F = self._calc_tip_and_hub_loss(phi, nblades, blade_radius, hub_radius)

		CT, CQ = self._calc_airfoil_forces(phi)

		kappa = 4*F*np.sin(phi)**2/(self.solidity*CT)
		kappap = 4*F*np.sin(phi)*np.cos(phi)/(self.solidity*CQ)

		a = 1.0 / (kappa - 1.0)
		ap = 1.0 / (kappap + 1.0)

		return a, ap

	def _calc_tip_and_hub_loss(self, phi, nblades, blade_radius, hub_radius):
		"""
		The total loss factor is F = F_tip * F_hub, where F is the prandtl loss factor
		"""
		if phi == 0:
			F = 1.0
		else:
			Ftip = prandtl(nblades, blade_radius - self.radius, self.radius, phi)
			Fhub = prandtl(nblades, self.radius - hub_radius, self.radius, phi)
			F = Ftip * Fhub
		self.loss_factor = F
		return F

	def _calc_airfoil_forces(self, phi):

		self.AoA = (self.pitch - phi) * 180.0 / np.pi

		self.Cl = self.airfoil.eval_Cl(self.AoA)
		self.Cd = self.airfoil.eval_Cd(self.AoA)

		CT = self.Cl*np.cos(phi) - self.Cd*np.sin(phi)
		CQ = self.Cl*np.sin(phi) + self.Cd*np.cos(phi)

		return CT, CQ

	def _calc_forces(self, phi, v_inf, omega, rho, mu, nblades, blade_radius, hub_radius):

		a, ap = self._calc_induction_factors(phi, nblades, blade_radius, hub_radius)
		CT, CQ = self._calc_airfoil_forces(phi)

		v = (1.0 + a) * v_inf
		vp = (1.0 - ap) * omega * self.radius
		U = np.sqrt(v**2 + vp**2)

		self.Re = rho * U * self.chord / mu

		# Blade element theory
		self.dT = self.solidity * np.pi * rho * U**2 * CT * self.radius * self.width
		self.dQ = self.solidity * np.pi * rho * U**2 * CQ * self.radius**2 * self.width

		return self.dT, self.dQ

def initialize_sections(sectionDict, rotorDict):

	# Interpolation
	dr = (rotorDict['diameter']/2 - rotorDict['hub_radius']) / sectionDict['n_sections']
	r_i = rotorDict['hub_radius'] + dr/2
	r_f = rotorDict['diameter']/2 - dr/2
	radius_list = np.linspace(r_i, r_f, sectionDict['n_sections'])
	chord_list = Akima1DInterpolator(sectionDict['radius_list'], sectionDict['chord_list'], method='makima', extrapolate=True)(radius_list)
	pitch_list = Akima1DInterpolator(sectionDict['radius_list'], sectionDict['pitch_list'], method='makima', extrapolate=True)(radius_list)

	section_list = []

	for i in range(sectionDict['n_sections']):
		
		airfoil = load_airfoil(sectionDict['airfoil_list'][i])

		if i==0:
			width = radius_list[i] - rotorDict['hub_radius']
		else:
			width = radius_list[i] - radius_list[i-1]

		radius = radius_list[i]

		chord = chord_list[i]

		pitch = (pitch_list[i] + rotorDict['global_twist']) * np.pi/180.0

		solidity = rotorDict['nblades'] * chord_list[i] / (2 * np.pi * radius_list[i])

		section = Section(airfoil=airfoil,
						  width=width,
						  radius=radius,
						  chord=chord,
						  pitch=pitch,
						  solidity=solidity)

		section_list.append(section)
	
	return section_list

def prandtl(nblades, dr, r, phi):
	"""
	Prandtl tip and hub loss factor, defined as:
		F = 2 / pi * arccos(exp(-f))
		f = B / 2 * (R-r) / (r*sin(phi))
	"""
	f = nblades * dr / (2*r*np.sin(phi))
	if (-f > 500): # exp can overflow for very large numbers
		F = 1.0
	else:
		F = 2/np.pi * np.arccos(min(1.0, np.exp(-f)))
	return F