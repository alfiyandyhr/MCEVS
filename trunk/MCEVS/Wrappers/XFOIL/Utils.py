import os
import re

def _remove_BL_file():
	# Remove unwanted Boundary Layer (BL) file if it exists
	bl_file = ':00.bl'
	if os.path.exists(bl_file):
		os.remove(bl_file)

def _check_and_warn_solver_parameter(airfoil_name, path_to_airfoil_data, is_viscous, is_compressible, Re, Mach):

	# Check airfoil
	if airfoil_name[:4] != 'NACA' and path_to_airfoil_data is None:
		raise ValueError(f'Airfoil {airfoil_name} is not in database. Please specify "XFOIL.path_to_airfoil_data"!')

	# Check compressibility
	if not is_viscous and is_compressible:
		print("Warning: is_compressible is set to True but will be ignored in inviscid mode.")

	# Check Re requirement
	if is_viscous and Re is None:
		raise ValueError("Re must be provided for viscous calculations.")

	# Check Mach requirement
	if is_compressible and Mach is None:
		raise ValueError("Mach must be provided for compressible calculations (only reliable for Mach < 0.3).")

	# Warn if parameters are provided but not needed
	if not is_compressible and Mach is not None:
		print("Warning: Mach number is provided but will be ignored in incompressible mode.")
	if not is_viscous and Re is not None:
		print("Warning: Reynolds number is provided but will be ignored in inviscid mode.")

	# Warn if Mach is above recommended value for XFOIL
	if is_compressible and Mach is not None and Mach > 0.3:
		print("Warning: Mach > 0.3; XFOIL results are unreliable at higher Mach numbers.")

def _extract_cl_cd_cm_cdf_cdp_from_stdout_of_single_point_analysis(stdout):
	# Regex patterns
	cl_pattern   = re.compile(r'a\s*=\s*[\d\.\-]+.*CL\s*=\s*([\d\.\-Ee]+)')
	cd_cm_cdf_cdp_pattern = re.compile(
		r'Cm\s*=\s*([\d\.\-Ee]+).*CD\s*=\s*([\d\.\-Ee]+).*CDf\s*=\s*([\d\.\-Ee]+).*CDp\s*=\s*([\d\.\-Ee]+)'
	)

	cl = cd = cm = cdf = cdp = None
	lines = stdout.splitlines()
	for i in range(len(lines)):
		line = lines[i]
		cl_match = cl_pattern.search(line)
		if cl_match and i+1 < len(lines):
			next_line = lines[i+1]
			cdcmcdfcdp_match = cd_cm_cdf_cdp_pattern.search(next_line)
			if cdcmcdfcdp_match:
				cl   = float(cl_match.group(1))
				cm   = float(cdcmcdfcdp_match.group(1))
				cd   = float(cdcmcdfcdp_match.group(2))
				cdf  = float(cdcmcdfcdp_match.group(3))
				cdp  = float(cdcmcdfcdp_match.group(4))
	return cl, cd, cm, cdf, cdp