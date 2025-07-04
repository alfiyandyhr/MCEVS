import numpy as np
import openmdao.api as om


class WingedAeroDragViaParabolicDragPolar(om.ExplicitComponent):
    """
    Computes the drag of a winged configuration in flight (cruise, climb, descent).
    Parameters:
            rho_air					: air density [kg/m**3]
    Inputs:
            Aero|Cd0				: minimum Cd of the polar drag (coefficient of parasitic drag)
            Aero|lift				: aerodynamic lift [N]
            Wing|area 				: wing area [m**2]
            Wing|aspect_ratio		: wing aspect ratio
            Aero|speed 				: air speed of the eVTOL [m/s]
    Outputs:
            Aero|total_drag 		: aerodynamic drag [N]
            Aero|CL 				: aerodynamic coefficient of lift
            Aero|CD 				: aerodynamic coefficient of drag
            Aero|f_total 			: total equivalent flat plate area [m**2]
    Notes:
            > Based on a simple parabolic drag polar equations
            > Oswald efficiency is in the function of wing aspect ratio (typically ~0.8)
            > The wing should be un-swept.
    Source:
            Raymer, D. P. Aircraft Design: A Conceptual Approach. Reston, Virginia: American Institute of Aeronautics and Astronautics, Inc., 2006.
    """
    def initialize(self):
        self.options.declare('rho_air', types=float, desc='Air density')

    def setup(self):
        self.add_input('Aero|Cd0', desc='Minimum Cd of the polar drag')
        self.add_input('Aero|lift', units='N', desc='Lift generated by the wing')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_input('Wing|area', units='m**2', desc='Wing reference area')
        self.add_input('Wing|aspect_ratio', desc='Wing aspect ratio')
        self.add_output('Aero|total_drag', units='N', desc='Drag of a winged configuration')
        self.add_output('Aero|CL', desc='Lift coefficient')
        self.add_output('Aero|CD', desc='Drag coefficient')
        self.add_output('Aero|f_total', units='m**2', desc='Total equivalent flat plate area')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        CD0 = inputs['Aero|Cd0']
        L = inputs['Aero|lift']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']
        AR_wing = inputs['Wing|aspect_ratio']

        # Raymer's formula for non-swept wing (Oswald efficiency)
        wing_e = 1.78 * (1 - 0.045 * AR_wing**0.68) - 0.64

        q = 0.5 * rho_air * v**2 	 				   # dynamic pressure
        CL = L / (q * S_wing)						   # lift coefficient
        CD = CD0 + CL**2 / (np.pi * wing_e * AR_wing)  # drag coefficient
        f_total = CD * S_wing 						   # total equivalent flat plate area

        outputs['Aero|total_drag'] = q * S_wing * CD  # drag
        outputs['Aero|CL'] = CL
        outputs['Aero|CD'] = CD
        outputs['Aero|f_total'] = f_total

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        CD0 = inputs['Aero|Cd0']
        L = inputs['Aero|lift']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']
        AR_wing = inputs['Wing|aspect_ratio']

        # Raymer's formula for non-swept wing (Oswald efficiency)
        wing_e = 1.78 * (1 - 0.045 * AR_wing**0.68) - 0.64

        de_dAR = - 1.78 * 0.045 * 0.68 * AR_wing**(-0.32)

        q = 0.5 * rho_air * v**2  # dynamic pressure
        CL = L / (q * S_wing)  # lift coefficient
        dq_dv = rho_air * v
        dCL_dL = 1 / (q * S_wing)
        dCL_dq = -L / (S_wing * q**2)
        dCL_dS = -L / (q * S_wing**2)

        CD = CD0 + CL**2 / (np.pi * wing_e * AR_wing)  # drag coefficient
        dCD_dCD0 = 1
        dCD_dCL = 2 * CL / (np.pi * wing_e * AR_wing)
        dCD_dL = dCD_dCL * dCL_dL
        dCD_dS = dCD_dCL * dCL_dS
        dCD_dAR = CL**2 / np.pi * (- 1 / (wing_e**2 * AR_wing) * de_dAR - 1 / (wing_e * AR_wing**2))

        dD_dq = S_wing * CD
        dD_dS = q * CD
        dD_dCD = q * S_wing

        partials['Aero|total_drag', 'Aero|Cd0'] = q * S_wing * dCD_dCD0
        partials['Aero|total_drag', 'Aero|lift'] = dD_dCD * dCD_dCL * dCL_dL
        partials['Aero|total_drag', 'Wing|area'] = dD_dS + dD_dCD * dCD_dCL * dCL_dS
        partials['Aero|total_drag', 'Wing|aspect_ratio'] = q * S_wing * CL**2 / np.pi * -(wing_e * AR_wing)**(-2) * (de_dAR * AR_wing + wing_e)
        partials['Aero|total_drag', 'Aero|speed'] = (dD_dCD * dCD_dCL * dCL_dq + dD_dq) * dq_dv
        partials['Aero|CL', 'Aero|Cd0'] = 0
        partials['Aero|CL', 'Aero|lift'] = dCL_dL
        partials['Aero|CL', 'Wing|area'] = dCL_dS
        partials['Aero|CL', 'Wing|aspect_ratio'] = 0
        partials['Aero|CL', 'Aero|speed'] = dCL_dq * dq_dv
        partials['Aero|CD', 'Aero|Cd0'] = dCD_dCD0
        partials['Aero|CD', 'Aero|lift'] = dCD_dL
        partials['Aero|CD', 'Wing|area'] = dCD_dS
        partials['Aero|CD', 'Wing|aspect_ratio'] = dCD_dAR
        partials['Aero|CD', 'Aero|speed'] = dCD_dCL * dCL_dq * dq_dv
        partials['Aero|f_total', 'Aero|Cd0'] = dCD_dCD0 * S_wing
        partials['Aero|f_total', 'Aero|lift'] = dCD_dL * S_wing
        partials['Aero|f_total', 'Wing|area'] = dCD_dS * S_wing + CD
        partials['Aero|f_total', 'Wing|aspect_ratio'] = dCD_dAR * S_wing
        partials['Aero|f_total', 'Aero|speed'] = dCD_dCL * dCL_dq * dq_dv * S_wing
