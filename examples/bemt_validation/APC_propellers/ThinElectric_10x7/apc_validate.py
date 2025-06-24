from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolverOM, BEMTSolver
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plot_static = False
plot_rpm4007 = True

constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=0.0)

r_lift_rotor = 10.0 / 2 * 0.0254

n_discrete = 15

dr = (1.0 - 0.15) / n_discrete
r_i = 0.15 + dr / 2
r_f = 1.0 - dr / 2

r_list = np.linspace(r_i, r_f, n_discrete)
c_list = 2.381059 * r_list**6 - 11.069854 * r_list**5 + 19.308968 * r_list**4 - 15.630064 * r_list**3 + 5.342020 * r_list**2 - 0.414998 * r_list + 0.123051
p_list = -6697.117926 * r_list**6 + 25184.415974 * r_list**5 - 37946.404426 * r_list**4 + 29071.647212 * r_list**3 - 11759.969513 * r_list**2 + 2278.364278 * r_list - 119.686081

if plot_static:

    # Sweep the RPM from 1900 to 6800
    RPMs = np.arange(1900, 6800, 300)

    results_dict = {'RPM': [], 'CT': [], 'CP': []}

    for i in range(len(RPMs)):
        print(f'Solving for RPM = {RPMs[i]}')

        rotorDict = {'nblades': 2, 'diameter': 2 * r_lift_rotor, 'hub_radius': 0.15 * r_lift_rotor, 'global_twist': 0.0}
        sectionDict = {'airfoil_list': n_discrete * ['CLARKY'],
                       'radius_list': r_lift_rotor * r_list,
                       'chord_list': r_lift_rotor * c_list,
                       'pitch_list': p_list}
        fluidDict = {'rho': constants['rho'], 'mu': constants['mu']}

        solver = BEMTSolverOM(rotorDict, sectionDict, fluidDict)
        results = solver.run(v_inf=15.0, rpm=RPMs[i])

        results_dict['RPM'].append(RPMs[i])
        results_dict['CT'].append(results['CT'])
        results_dict['CP'].append(results['CP'])

    res_data = pd.DataFrame(results_dict)

    exp_data = pd.read_csv('apc_TE_10x7_Static.csv')

    fig, ax = plt.subplots()
    ax.plot(exp_data.RPM, exp_data.CT, 'ro')
    ax.plot(res_data.RPM, res_data.CT, 'r-')
    ax.plot(exp_data.RPM, exp_data.CP, 'bo')
    ax.plot(res_data.RPM, res_data.CP, 'b-')
    ax.set_ylabel(r'$C_{T}, C_{P}$')
    ax.set_xlabel(r'$RPM$')
    ax.grid()
    fig.suptitle('Experiment vs BEMT')
    fig.legend([r'$C_T$-exp', r'$C_T$-bemt', r'$C_P$-exp', r'$C_P$-bemt'],
               ncols=3, loc='upper center', bbox_to_anchor=(0.5, 0.94))
    plt.subplots_adjust(left=0.12, bottom=0.1, right=0.9, top=0.79, hspace=0.1)
    plt.show()

if plot_rpm4007:

    # Sweep the v_inf from 2 to 14 m/s
    v_infs = np.arange(2, 14, 1)

    results_dict = {'J': [], 'CT': [], 'CP': [], 'eta': []}

    for i in range(len(v_infs)):
        print(f'Solving for v_inf = {v_infs[i]} m/s')

        rotorDict = {'nblades': 2, 'diameter': 2 * r_lift_rotor, 'hub_radius': 0.15 * r_lift_rotor, 'global_twist': 0.0}
        sectionDict = {'airfoil_list': n_discrete * ['NACA_4412'],
                       'radius_list': r_lift_rotor * r_list,
                       'chord_list': r_lift_rotor * c_list,
                       'pitch_list': p_list}
        fluidDict = {'rho': constants['rho'], 'mu': constants['mu']}

        solver = BEMTSolver(rotorDict, sectionDict, fluidDict)
        results = solver.run(v_inf=v_infs[i], rpm=4007.0)

        results_dict['J'].append(results['J'])
        results_dict['CT'].append(results['CT'])
        results_dict['CP'].append(results['CP'])
        results_dict['eta'].append(results['eta'])

    res_data = pd.DataFrame(results_dict)

    exp_data = pd.read_csv('apc_TE_10x7_RPM4007.csv')

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
