from MCEVS.Analyses.Aerodynamics.BEMT.Solver import BEMTSolver

caseDict = {'v_inf': 1.0, 'rpm': 1100.0}
rotorDict = {'nblades': 3, 'diameter': 3.054, 'hub_radius': 0.375, 'global_twist':0.0}
sectionDict = {'airfoil_list': ['CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY', 'CLARKY'],
			   'radius_list': [0.525, 0.675, 0.825, 0.975, 1.125, 1.275, 1.425],
			   'chord_list': [0.18, 0.225, 0.225, 0.21, 0.1875, 0.1425, 0.12],
			   'pitch_list': [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]}
fluidDict = {'rho': 1.225, 'mu':1.81E-5}

solver = BEMTSolver(caseDict, rotorDict, sectionDict, fluidDict)
results = solver.run()
print(results)