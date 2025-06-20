import numpy as np
import openmdao.api as om

class FiniteWingLiftCoefficientCurveSlope(om.ExplicitComponent):
	"""
	Computes the finite wing lift coefficient curve slope from infinite 2D airfoil curve slope corrected for compressibility
	Parameters:
		v_sound 				: sound speed (m/s)
	Inputs:
		Wing|airfoil|CL_alpha	: 2D airfoil lift curve slope (1/rad)
		Wing|aspect_ratio		: wing aspect ratio
		Aero|speed 				: air speed (m/s)
	Outputs:
		Wing|CL_alpha			: finite wing lift curve slope (1/rad)
	Notes:
		> Airfoil CL_alpha is lift-curve slope of 2D infinite airfoil
		> The lifting line theory introduces an induced AoA caused by the downwash from the wing’s trailing vortices
		> Includes compressibility correction (should be used for Mach < 0.7)
	Sources:
		1. Leishman, G., “Introduction to Aerospace Flight Vehicles,” Embry-Riddle Aeronautical University, 2022.
		   https://doi.org/10.15394/eaglepub.2022.1066
		2. https://eaglepubs.erau.edu/introductiontoaerospaceflightvehicles/chapter/lifting-line-theory/
		3. https://webstor.srmist.edu.in/web_assets/srm_mainsite/files/downloads/class4-2012.pdf
	"""
	def initialize(self):
		self.options.declare('v_sound', types=float, desc='Speed of sound')

	def setup(self):
		self.add_input('Wing|airfoil|CL_alpha', units='1/rad', desc='Airfoil lift-curve slope')
		self.add_input('Aero|speed', units='m/s', desc='Air speed')
		self.add_input('Wing|aspect_ratio', desc='Wing aspect ratio')
		self.add_output('Wing|CL_alpha', units='1/rad', desc='Finite wing lift-curve slope')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		v_sound = self.options['v_sound']
		v = inputs['Aero|speed']
		airfoil_CL_alpha = inputs['Wing|airfoil|CL_alpha']
		AR_wing = inputs['Wing|aspect_ratio']

		# Raymer's formula for non-swept wing (Oswald efficiency)
		wing_e = 1.78 * (1 - 0.045 * AR_wing**0.68) - 0.64

		# Compressibility correction
		Mach = v/v_sound
		compressibility = np.sqrt(1 - Mach**2)

		outputs['Wing|CL_alpha'] = airfoil_CL_alpha / (compressibility + airfoil_CL_alpha/(np.pi * wing_e * AR_wing))

	def compute_partials(self, inputs, partials):
		v_sound = self.options['v_sound']
		v = inputs['Aero|speed']
		airfoil_CL_alpha = inputs['Wing|airfoil|CL_alpha']
		AR_wing = inputs['Wing|aspect_ratio']

		# Raymer's formula for non-swept wing (Oswald efficiency)
		wing_e = 1.78 * (1 - 0.045 * AR_wing**0.68) - 0.64
		de_dAR = - 0.68 * 1.78 * 0.045 * AR_wing**(-0.32)

		# Compressibility correction
		cp = np.sqrt(1 - (v/v_sound)**2)
		dcp_dv = 0.5*(1- (v/v_sound)**2)**(-0.5) * (-2*v/v_sound**2)

		x = np.pi * wing_e * AR_wing
		dx_dAR = np.pi * de_dAR * AR_wing + np.pi * wing_e

		partials['Wing|CL_alpha', 'Wing|airfoil|CL_alpha'] = 1 / (cp + airfoil_CL_alpha/x) - airfoil_CL_alpha/(cp + airfoil_CL_alpha/x)**2 * (1/x)
		partials['Wing|CL_alpha', 'Wing|aspect_ratio'] = - airfoil_CL_alpha/(cp + airfoil_CL_alpha/x)**2 * (- airfoil_CL_alpha/x**2)
		partials['Wing|CL_alpha', 'Aero|speed'] = - airfoil_CL_alpha / (cp + airfoil_CL_alpha/x)**2 * dcp_dv
		