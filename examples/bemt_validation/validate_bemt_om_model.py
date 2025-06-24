from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOM
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# BEMT_OM: with grads, solver: newton solver, implemented using OpenMDAO

# Sweep the v_inf from 0 to 45 m/s
v_infs = np.arange(0, 46, 1)

results_dict = {'J': [], 'CT': [], 'CP': [], 'eta': []}
resultsOM_dict = {'J': [], 'CT': [], 'CP': [], 'eta': []}

rotorDict = {'nblades': 3, 'diameter': 3.054, 'hub_radius': 0.375, 'global_twist': 0.0}
sectionDict = {'airfoil_list': ['CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY'],
               'radius_list': [0.525, 0.675, 0.825, 0.975, 1.125, 1.275, 1.425],
               'chord_list': [0.18, 0.225, 0.225, 0.21, 0.1875, 0.1425, 0.12],
               'pitch_list': [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]}
fluidDict = {'rho': 1.225, 'mu': 1.81E-5}

for v_inf in v_infs:
    print(f'Solving for v_inf = {v_inf} m/s')
    solverOM = BEMTSolverOM(rotorDict, sectionDict, fluidDict)
    resultsOM = solverOM.run(v_inf=float(v_inf), rpm=1100.0)

    resultsOM_dict['J'].append(resultsOM['J'])
    resultsOM_dict['CT'].append(resultsOM['CT'])
    resultsOM_dict['CP'].append(resultsOM['CP'])
    resultsOM_dict['eta'].append(resultsOM['eta'])

bemt_data = pd.DataFrame(results_dict)
bemtOM_data = pd.DataFrame(resultsOM_dict)
experiment_data = pd.read_csv('experiment_data.csv')

fig, ax = plt.subplots()
ax2 = ax.twinx()
ax.plot(experiment_data.J, experiment_data.CT, 'ro')
ax.plot(bemtOM_data.J, bemtOM_data.CT, 'r-')
ax.plot(experiment_data.J, experiment_data.CP, 'bo')
ax.plot(bemtOM_data.J, bemtOM_data.CP, 'b-')
ax2.plot(experiment_data.J, experiment_data.eta, 'go')
ax2.plot(bemtOM_data.J, bemtOM_data.eta, 'g-')
ax.set_ylabel(r'$C_{T}, C_{P}$')
ax2.set_ylabel(r'$\eta$')
ax.set_xlabel(r'Advance ratio $J$')
ax.grid()
fig.suptitle('Experiment vs BEMT')
fig.legend([r'$C_T$-exp', r'$C_T$-bemt', r'$C_P$-exp', r'$C_P$-bemt', r'$\eta$-exp', r'$\eta$-bemt'],
           ncols=3, loc='upper center', bbox_to_anchor=(0.5, 0.94))
plt.subplots_adjust(left=0.12, bottom=0.1, right=0.9, top=0.80, hspace=0.1)
plt.show()
# plt.savefig('bemt_om_validation.pdf',format='pdf',dpi=600)

# References
# [1] Theodorsen T., Stickle G.W. and Brevoort, M.J. “Characteristics of six propellers including the high-speed range.”
#     Report no. 594-National Advisory Committee for Aeronautics (1937).
# [2] Morgado J., Silvestre M.A.R. and Páscoa J.C. “Validation of new formulations for propeller analysis.”
#     Journal of Propulsion and Power 31.1 (2015): 467-477.
