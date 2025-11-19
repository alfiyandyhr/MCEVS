import numpy as np
import openmdao.api as om


class LiftRotorClearanceConstraintTypeOne(om.ExplicitComponent):
    """
    Computes the spanwise spacing constraint of lift rotors; the outer rotors are placed at the wing tip
    Parameter:
            N_rotor					: number of (lift) rotors
            max_d_fuse 				: fuselage max diameter [m]
            percent_max_span 		: maximum span percentage to accommodate lift rotors and their clearance (default=95)
    Inputs:
            LiftRotor|radius		: lift rotor's radius [m]
            Wing|area 				: wing area [m**2]
            Wing|aspect_ratio 		: wing aspect ratio
    Outputs:
            clearance_constraint
    """
    def initialize(self):
        self.options.declare('N_rotor', types=int, desc='Number of (lift) rotors')
        self.options.declare('max_d_fuse', types=float, desc='Fuselage max diameter')
        self.options.declare('percent_max_span', types=float, desc='Maximum span percentage to accommodate lift rotors and their clearance')

    def setup(self):
        self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
        self.add_input('Wing|area', units='m**2', desc='Wing area')
        self.add_input('Wing|aspect_ratio', units=None, desc='Wing aspect ratio')
        self.add_output('clearance_constraint', units='m', desc='Spanwise clearance constraint')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        N_rotor = self.options['N_rotor']
        max_d_fuse = self.options['max_d_fuse']
        percent_max_span = self.options['percent_max_span']
        r_lift_rotor = inputs['LiftRotor|radius']
        S_wing = inputs['Wing|area']
        AR_wing = inputs['Wing|aspect_ratio']
        b = np.sqrt(S_wing * AR_wing)  # in [m]

        outputs['clearance_constraint'] = (N_rotor / 2 - 1) * 2 * r_lift_rotor + max_d_fuse - percent_max_span / 100 * b

    def compute_partials(self, inputs, partials):
        N_rotor = self.options['N_rotor']
        percent_max_span = self.options['percent_max_span']
        S_wing = inputs['Wing|area']
        AR_wing = inputs['Wing|aspect_ratio']

        db_dS = 0.5 * (S_wing * AR_wing)**(-0.5) * AR_wing
        db_dAR = 0.5 * (S_wing * AR_wing)**(-0.5) * S_wing

        partials['clearance_constraint', 'LiftRotor|radius'] = (N_rotor / 2 - 1) * 2
        partials['clearance_constraint', 'Wing|area'] = - percent_max_span / 100 * db_dS
        partials['clearance_constraint', 'Wing|aspect_ratio'] = - percent_max_span / 100 * db_dAR


class LiftRotorClearanceConstraintTypeTwo(om.ExplicitComponent):
    """
    Spanwise clearance constraints for two y-stations per semi-span:
      - Inner station at y = beta * b/2
      - Outer station at y = alpha * b/2 (may overhang; no tip constraint enforced)
    Assumes an even number of lift rotors (N_rotor), arranged in two chordwise rows,
    so spanwise interference is governed by the two stations per side.

    Parameters (options):
        N_rotor         : int   Number of lift rotors (must be even). Used only for validation/logging.
        max_d_fuse      : float Fuselage max diameter D [m]
        alpha           : float Outer station nondimensional span location (0 < beta < alpha; alpha can be >= 1 if overhang allowed)
        beta            : float Inner station nondimensional span location (> 0)
        clearance_c     : float Fixed extra clearance c [m] (recommended 0.10)
    Inputs:
        LiftRotor|radius    r [m]
        Wing|area           S [m^2]
        Wing|aspect_ratio   AR [-]
    Outputs (residual form; must be <= 0 to be feasible):
        clearance_inner_fuselage   =  (2*r + 2*c + D) - beta * b
        clearance_inner_outer      =  (4*r + 2*c) - (alpha - beta) * b
    """

    def initialize(self):
        self.options.declare('N_rotor', types=int, desc='Number of lift rotors (even)')
        self.options.declare('max_d_fuse', types=float, desc='Fuselage max diameter D [m]')
        self.options.declare('alpha', types=float, desc='Outer station fraction of semi-span')
        self.options.declare('beta', types=float, desc='Inner station fraction of semi-span')
        self.options.declare('clearance_c', types=float, default=0.10, desc='Fixed clearance c [m]')

    def setup(self):
        self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius r')
        self.add_input('Wing|area', units='m**2', desc='Wing planform area S')
        self.add_input('Wing|aspect_ratio', units=None, desc='Wing aspect ratio AR')
        self.add_output('clearance_inner_fuselage', units='m', desc='Residual: inner vs fuselage (<= 0 feasible)')
        self.add_output('clearance_inner_outer', units='m', desc='Residual: inner vs outer (<= 0 feasible)')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        # Options
        N_rotor = self.options['N_rotor']
        D = self.options['max_d_fuse']
        alpha = self.options['alpha']
        beta = self.options['beta']
        c = self.options['clearance_c']

        # Validate basics (no hard failure to keep optimizer running; warn instead)
        if N_rotor % 2 != 0:
            raise ValueError(f'LiftRotorClearanceConstraintTypeTwo: N_rotor is not even ({N_rotor}).')
        if not (beta > 0.0 and alpha > beta):
            raise ValueError(f'LiftRotorClearanceConstraintTypeTwo: Require 0 < beta < alpha. Got alpha={alpha}, beta={beta}')

        # Inputs
        r = inputs['LiftRotor|radius']
        S = inputs['Wing|area']
        AR = inputs['Wing|aspect_ratio']

        # Wingspan
        b = np.sqrt(S * AR)

        # Residual forms (<= 0 is feasible)
        # 1) Inner vs fuselage: beta*b - D - 2r - 2c >= 0  => residual = (2r + 2c + D) - beta*b
        outputs['clearance_inner_fuselage'] = (2.0 * r + 2.0 * c + D) - beta * b

        # 2) Inner vs outer: (alpha - beta)*b - 4r - 2c >= 0  => residual = (4r + 2c) - (alpha - beta)*b
        outputs['clearance_inner_outer'] = (4.0 * r + 2.0 * c) - (alpha - beta) * b

    def compute_partials(self, inputs, partials):
        alpha = self.options['alpha']
        beta = self.options['beta']

        S = inputs['Wing|area']
        AR = inputs['Wing|aspect_ratio']

        # b = sqrt(S*AR); db/dS and db/dAR
        b = np.sqrt(S * AR)
        # Guard against zero
        if b <= 0.0:
            db_dS = 0.0
            db_dAR = 0.0
        else:
            db_dS = 0.5 * (AR / b)
            db_dAR = 0.5 * (S / b)

        # Partials for clearance_inner_fuselage = (2r + 2c + D) - beta*b
        partials['clearance_inner_fuselage', 'LiftRotor|radius'] = 2.0
        partials['clearance_inner_fuselage', 'Wing|area'] = - beta * db_dS
        partials['clearance_inner_fuselage', 'Wing|aspect_ratio'] = - beta * db_dAR

        # Partials for clearance_inner_outer = (4r + 2c) - (alpha - beta)*b
        partials['clearance_inner_outer', 'LiftRotor|radius'] = 4.0
        partials['clearance_inner_outer', 'Wing|area'] = - (alpha - beta) * db_dS
        partials['clearance_inner_outer', 'Wing|aspect_ratio'] = - (alpha - beta) * db_dAR
