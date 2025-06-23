import numpy as np

def calc_CL_characteristics_given_polar_data(alpha_array:np.array, CL_array:np.array):
	"""
	Calculate lift-curve slope and CL_0 using linear fitting
	Note:
		We force CL_0 to match CL(alpha=0), fit for slope only.
	Inputs:
		alpha_array	: array of alphas (AoAs)
		CL_array	: array of CLs
	Outputs:
		CL_alpha 	: dCL/da, lift-curve slope (1/rad)
		CL_0 		: CL at alpha=0 deg
	"""
	# Convert alpha from degrees to radians for the slope calculation
	alpha_rad = np.deg2rad(alpha_array)

	# Linear fit: CL = CL_alpha * alpha + CL_0
	coeffs = np.polyfit(alpha_rad, CL_array, 1)  # degree-1 polyfit

	CL_alpha = coeffs[0]  # Slope (dCL/dalpha)
	CL_0 = coeffs[1]      # Intercept (CL at alpha=0)

	return CL_alpha, CL_0
