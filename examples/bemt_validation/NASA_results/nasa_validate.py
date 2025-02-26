from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOM, BEMTSolver
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

raw_data = pd.read_csv('raw_data.csv')

# RPM 350
# plt.plot(raw_data.CT_sigma[0:7],raw_data.FM[0:7],'o-')
# plt.grid()
# plt.xlabel(r'$C_{T}/\sigma$')
# plt.ylabel(r'FM')
# plt.show()

# RPM 400
# plt.plot(raw_data.CT_sigma[7:13],raw_data.FM[7:13],'o-')
# plt.grid()
# plt.xlabel(r'$C_{T}/\sigma$')
# plt.ylabel(r'FM')
# plt.show()

# RPM 469
# plt.plot(raw_data.CT_sigma[13:19],raw_data.FM[13:19],'o-')
# plt.grid()
# plt.xlabel(r'$C_{T}/\sigma$')
# plt.ylabel(r'FM')
# plt.show()

# RPM 500
# plt.plot(raw_data.CT_sigma[19:25],raw_data.FM[19:25],'o-')
# plt.grid()
# plt.xlabel(r'$C_{T}/\sigma$')
# plt.ylabel(r'FM')
# plt.show()

constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=0.0)

n_discrete = 15
r_lift_rotor = 9.1 * 0.3048
r_list = np.array([0.120, 0.175, 0.285, 0.395, 0.505, 0.615, 0.725, 0.835, 0.945, 1.000])
c_list = (-0.194318182*r_list+0.708318182) * 0.3048
p_list = -12.0*r_list + 9.0

# mean_c = 1.44 * 0.3048
mean_c = np.mean(c_list)
solidity = 3 * mean_c / (np.pi * r_lift_rotor)
global_twist_list = np.arange(5, 13, 1)

FM_res = np.zeros(len(global_twist_list))
CT_sigma_res = np.zeros(len(global_twist_list))

# print(c_list * 9.1)
# print(0.57657/0.76875)
# print(global_twist_list)
# print(r_list)
# print(c_list)
# print(solidity)

for i in range(len(global_twist_list)):
	print(f'Solving for collective pitch = {global_twist_list[i]}')

	rotorDict = {'nblades': 3, 'diameter': 2*r_lift_rotor, 'hub_radius': 0.12*r_lift_rotor, 'global_twist': global_twist_list[i]}
	sectionDict = {'n_sections': n_discrete,
				   'airfoil_list': n_discrete*['BOEING_VERTOL_VR12_with_rotation'],
				   'radius_list': r_lift_rotor * r_list,
				   'chord_list': c_list,
				   'pitch_list': p_list}
	fluidDict = {'rho': constants['rho'], 'mu': constants['mu']}

	solverOM = BEMTSolver(rotorDict, sectionDict, fluidDict)
	resultsOM = solverOM.run(v_inf=0.01, rpm=350.0)

	FM_res[i] = resultsOM['FM']
	CT_sigma_res[i] = resultsOM['CT']/solidity

# RPM 350
plt.plot(raw_data.CT_sigma[0:7],raw_data.FM[0:7],'o-')
plt.plot(CT_sigma_res, FM_res,'o-')
plt.grid()
plt.xlabel(r'$C_{T}/\sigma$')
plt.ylabel(r'FM')
plt.show()

# print(resultsOM)
# calc_P = 4 * resultsOM['P']/1000
# calc_v_tip = resultsOM['rpm'] * r_lift_rotor * 2*np.pi/60
# calc_FM = resultsOM['FM']


# # trim_resultsOM = solverOM.trim_rpm(T_req=T_req_one_rotor, v_inf=100*0.3048/60)
# # # print(trim_resultsOM)
# # calc_P = 4 * trim_resultsOM['P']/1000
# # calc_v_tip = trim_resultsOM['rpm'] * r_lift_rotor * 2*np.pi/60
# # calc_FM = trim_resultsOM['FM']
# # print(f'P_hover_total: published= 345.1525714; calculated= {calc_P}; diff= {(calc_P-345.1525714)/345.1525714*100}%')
# # print(f'Hover tip speed: published= 167.64; calculated= {calc_v_tip}; diff= {(calc_v_tip-167.64)/167.64*100}%')
# # print(f'Figure of merit: published= 0.7; calculated= {calc_FM}; diff= {(calc_FM-0.7)/0.7*100}%')