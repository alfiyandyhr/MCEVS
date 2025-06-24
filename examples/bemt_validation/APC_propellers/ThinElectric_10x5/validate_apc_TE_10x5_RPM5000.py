from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOM, BEMTSolver  # noqa: F401
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
# from scipy.interpolate import Akima1DInterpolator
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=0.0)

n_discrete = 15
r_lift_rotor = 10.0 / 2 * 0.0254
r_list = np.arange(0.15, 1.05, 0.05)
c_list = np.array([0.130, 0.149, 0.173, 0.189, 0.197, 0.201, 0.200, 0.194, 0.186, 0.174, 0.160, 0.145, 0.128, 0.112, 0.096, 0.081, 0.061, 0.041])
p_list = np.array([32.76, 37.19, 33.54, 29.25, 25.64, 22.54, 20.27, 18.46, 17.05, 15.97, 14.87, 14.09, 13.39, 12.84, 12.25, 11.37, 10.19, 8.99])

# Sweep the v_inf from 2.5 to 14 m/s
v_infs = np.arange(2.5, 14, 0.5)

results_dict = {'J': [], 'CT': [], 'CP': [], 'eta': []}

for i in range(len(v_infs)):
    print(f'Solving for v_inf = {v_infs[i]} m/s')

    rotorDict = {'nblades': 2, 'diameter': 2 * r_lift_rotor, 'hub_radius': 0.10 * r_lift_rotor, 'global_twist': 0.0}
    sectionDict = {'n_sections': n_discrete,
                   'airfoil_list': n_discrete * ['NACA_4412_with_rotation'],
                   'radius_list': r_lift_rotor * r_list,
                   'chord_list': r_lift_rotor * c_list,
                   'pitch_list': p_list}
    fluidDict = {'rho': constants['rho'], 'mu': constants['mu']}

    solver = BEMTSolverOM(rotorDict, sectionDict, fluidDict)
    results = solver.run(v_inf=v_infs[i], rpm=5400.0)

    results_dict['J'].append(results['J'])
    results_dict['CT'].append(results['CT'])
    results_dict['CP'].append(results['CP'])
    results_dict['eta'].append(results['eta'])

res_data = pd.DataFrame(results_dict)

exp_data = pd.read_csv('apc_TE_10x5_RPM5000.csv')

fig, ax = plt.subplots()
ax2 = ax.twinx()
ax.plot(exp_data.J, exp_data.CT, 'ro')
ax.plot(res_data.J, res_data.CT, 'r-')
ax.plot(exp_data.J, exp_data.CP, 'bo')
ax.plot(res_data.J, res_data.CP, 'b-')
ax2.plot(exp_data.J, exp_data.eta, 'go')
ax2.plot(res_data.J, res_data.eta, 'g-')
ax.set_ylabel(r'$C_{T}, C_{P}$')
ax2.set_ylabel(r'$\eta$')
ax.set_xlabel(r'Advance ratio $J$')
ax.grid()
fig.suptitle('Experiment vs BEMT')
fig.legend([r'$C_T$-exp', r'$C_T$-bemt', r'$C_P$-exp', r'$C_P$-bemt', r'$\eta$-exp', r'$\eta$-bemt'],
           ncols=3, loc='upper center', bbox_to_anchor=(0.5, 0.94))
plt.subplots_adjust(left=0.12, bottom=0.1, right=0.9, top=0.79, hspace=0.1)
plt.show()
