from MCEVS.Applications.XFOIL import XFOIL
from MCEVS.Methods.Aerodynamics.Polar import calc_CL_characteristics_given_polar_data
import matplotlib.pyplot as plt
import numpy as np

run_single_point = False
run_polar = True
plot = True

airfoil_name = 'LS417'
# airfoil_name = 'NACA2412'
# airfoil_name = 'NACA0012'
Re=4E6; Mach=0.2

xfoil = XFOIL(airfoil_name)
xfoil.path_to_xfoilexe = '/Users/alfiyandyhr/Codes/Xfoil/bin/xfoil'
if airfoil_name[:4] != 'NACA':
	xfoil.path_to_airfoil_data = f'{airfoil_name}.dat'

if run_single_point:
	sp_res1, sp_res2 = xfoil.run_single_point(is_viscous=True, is_compressible=True, AoA=5, Re=6E6, Mach=0.2, verbose=False)
	print(sp_res1)

	if plot:
		fig, ax = plt.subplots(figsize=(6, 4))
		ax.plot(sp_res2['x'], sp_res2['Cp'], 'o', ms=5, ls='-', label=f'Re={Re}, Mach={Mach}')
		ax.invert_yaxis() # Invert y-axis (standard for Cp plots)
		ax.set_xlabel(r'$x/c$')
		ax.set_ylabel(r'$C_{p}$')
		ax.set_title(f'Pressure coefficient distribution for {airfoil_name}')
		ax.legend()
		plt.tight_layout()
		plt.show()

if run_polar:
	AoA_seq = [-20.0, 25.0, 0.5] # start= -20 deg; end= 25 deg; increment= 0.5 deg
	polar_df = xfoil.run_polar(is_viscous=True, is_compressible=True, AoA_seq=AoA_seq, Re=Re, Mach=Mach, save_polar=False, verbose=False)
	print(polar_df)

	# CL linear characteristics
	linear_alpha = polar_df[(polar_df['alpha']<=10.0) & (polar_df['alpha']>=-10.0)]['alpha'].to_numpy()
	linear_CL = polar_df[(polar_df['alpha']<=10.0) & (polar_df['alpha']>=-10.0)]['CL'].to_numpy()
	CL_alpha, CL_0 = calc_CL_characteristics_given_polar_data(linear_alpha, linear_CL)
	print(f'CL_alpha= {CL_alpha}; CL_0= {CL_0}')

	if plot:
		AoAs = np.arange(-10,11,1)
		fig, ax = plt.subplots(figsize=(6, 4))
		ax.plot(polar_df['alpha'], polar_df['CL'], '-', label=f'XFOIL; Re={np.round(Re/1000000,1)}E6; Mach={Mach}')
		ax.plot(AoAs, AoAs*CL_alpha*np.pi/180 + CL_0, 'r-', label=fr'$dC_{{\mathrm{{L}}}}/d\alpha$= {np.round(CL_alpha,4)} /rad; $C_{{\mathrm{{L0}}}}$= {np.round(CL_0,4)}')
		ax.set_xlabel('AoA (deg)')
		ax.set_ylabel(r'$C_{L}$')
		ax.set_title(f'Lift coefficients for {airfoil_name}')
		ax.legend()
		plt.tight_layout()
		plt.show()
