from MCEVS.Analyses.Aerodynamics.BEMT.Airfoil import load_airfoil
import numpy as np

airfoil = load_airfoil('NACA_4412_with_rotation')

alpha1 = np.linspace(-180.0, -9.5, 50).reshape(-1, 1)
alpha2 = np.linspace(-9.5, 16.25, 100).reshape(-1, 1)
alpha3 = np.linspace(16.25, 180.0, 50).reshape(-1, 1)

Cl1 = airfoil.eval_Cl(alpha1).reshape(-1, 1)
Cl2 = airfoil.eval_Cl(alpha2).reshape(-1, 1)
Cl3 = airfoil.eval_Cl(alpha3).reshape(-1, 1)

Cd1 = airfoil.eval_Cd(alpha1).reshape(-1, 1)
Cd2 = airfoil.eval_Cd(alpha2).reshape(-1, 1)
Cd3 = airfoil.eval_Cd(alpha3).reshape(-1, 1)


data1 = np.concatenate((alpha3, Cl3, Cd3), axis=1)

np.savetxt('data.dat', data1, delimiter=',')

# print(data1)
