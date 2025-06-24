from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOM  # noqa: F401
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
import numpy as np

constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=6025 * 0.3048)

r_lift_rotor = 13.1 * 0.3048
T_req_one_rotor = 2939.27616 * constants['g'] / 4

n_discrete = 50

dr = (1.0 - 0.12) / n_discrete
r_i = 0.12 + dr / 2
r_f = 1.0 - dr / 2

r_list = np.linspace(r_i, r_f, n_discrete)
c_list = -0.018636364 * r_list + 0.07473636
p_list = -12 * r_list + 9

rotorDict = {'nblades': 2, 'diameter': 2 * r_lift_rotor, 'hub_radius': 0.12 * r_lift_rotor, 'global_twist': 4.0}
sectionDict = {'airfoil_list': n_discrete * ['BOEING_VERTOL_VR12'],
               'radius_list': r_lift_rotor * r_list,
               'chord_list': r_lift_rotor * c_list,
               'pitch_list': p_list}
fluidDict = {'rho': constants['rho'], 'mu': constants['mu']}
print(p_list)
# solverOM = BEMTSolverOM(rotorDict, sectionDict, fluidDict)
# # resultsOM = solverOM.run(v_inf=100*0.3048/60, rpm=167.64/r_lift_rotor*9.549297)
# # print(resultsOM)
# # calc_P = 4 * resultsOM['P']/1000
# # calc_v_tip = resultsOM['rpm'] * r_lift_rotor * 2*np.pi/60
# # calc_FM = resultsOM['FM']


# trim_resultsOM = solverOM.trim_rpm(T_req=T_req_one_rotor, v_inf=100*0.3048/60)
# # print(trim_resultsOM)
# calc_P = 4 * trim_resultsOM['P']/1000
# calc_v_tip = trim_resultsOM['rpm'] * r_lift_rotor * 2*np.pi/60
# calc_FM = trim_resultsOM['FM']
# print(f'P_hover_total: published= 345.1525714; calculated= {calc_P}; diff= {(calc_P-345.1525714)/345.1525714*100}%')
# print(f'Hover tip speed: published= 167.64; calculated= {calc_v_tip}; diff= {(calc_v_tip-167.64)/167.64*100}%')
# print(f'Figure of merit: published= 0.7; calculated= {calc_FM}; diff= {(calc_FM-0.7)/0.7*100}%')
