import numpy as np
import openmdao.api as om


class LiftRotorClearanceConstraintTypeOne(om.ExplicitComponent):
    """
    Computes the spanwise spacing constraint of lift rotors;
    the outer rotors are placed at the wing tip, and the inner rotors are placed at the mid span
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
      - Inner station at y = s_inner * b/2
      - Outer station at y = s_outer * b/2 (no tip constraint enforced)
    Assumes an even number of lift rotors (N_rotor), arranged in two chordwise rows,
    so spanwise interference is governed by the two stations per side.

    Parameters (options):
        N_rotor         : int   Number of lift rotors (must be even). Used only for validation/logging.
        max_d_fuse      : float Fuselage max diameter D [m]
        s_outer         : float Outer station nondimensional span location (0 < s_inner < s_outer < 1)
        s_inner         : float Inner station nondimensional span location (> 0)
        clearance_c     : float Fixed extra clearance c [m] (recommended 0.10)
    Inputs:
        LiftRotor|radius    r [m]
        Wing|area           S [m^2]
        Wing|aspect_ratio   AR [-]
    Outputs (residual form; must be <= 0 to be feasible):
        clearance_inner_fuselage   =  (2*r + 2*c + D) - s_inner * b
        clearance_inner_outer      =  (4*r + 2*c) - (s_outer - s_inner) * b
    """

    def initialize(self):
        self.options.declare('N_rotor', types=int, desc='Number of lift rotors (even)')
        self.options.declare('max_d_fuse', types=float, desc='Fuselage max diameter D [m]')
        self.options.declare('s_outer', types=float, desc='Outer station fraction of semi-span')
        self.options.declare('s_inner', types=float, desc='Inner station fraction of semi-span')
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
        s_outer = self.options['s_outer']
        s_inner = self.options['s_inner']
        c = self.options['clearance_c']

        # Validate basics (no hard failure to keep optimizer running; warn instead)
        if N_rotor % 2 != 0:
            raise ValueError(f'LiftRotorClearanceConstraintTypeTwo: N_rotor is not even ({N_rotor}).')
        if not (s_inner > 0.0 and s_outer > s_inner):
            raise ValueError(f'LiftRotorClearanceConstraintTypeTwo: Require 0 < s_inner < s_outer. Got s_outer={s_outer}, s_inner={s_inner}')

        # Inputs
        r = inputs['LiftRotor|radius']
        S = inputs['Wing|area']
        AR = inputs['Wing|aspect_ratio']

        # Wingspan
        b = np.sqrt(S * AR)

        # Residual forms (<= 0 is feasible)
        # 1) Inner vs fuselage: s_inner*b - D - 2r - 2c >= 0  => residual = (2r + 2c + D) - s_inner*b
        outputs['clearance_inner_fuselage'] = (2.0 * r + 2.0 * c + D) - s_inner * b

        # 2) Inner vs outer: (s_outer - s_inner)*b - 4r - 2c >= 0  => residual = (4r + 2c) - (s_outer - s_inner)*b
        outputs['clearance_inner_outer'] = (4.0 * r + 2.0 * c) - (s_outer - s_inner) * b

    def compute_partials(self, inputs, partials):
        s_outer = self.options['s_outer']
        s_inner = self.options['s_inner']

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

        # Partials for clearance_inner_fuselage = (2r + 2c + D) - s_inner*b
        partials['clearance_inner_fuselage', 'LiftRotor|radius'] = 2.0
        partials['clearance_inner_fuselage', 'Wing|area'] = - s_inner * db_dS
        partials['clearance_inner_fuselage', 'Wing|aspect_ratio'] = - s_inner * db_dAR

        # Partials for clearance_inner_outer = (4r + 2c) - (s_outer - s_inner)*b
        partials['clearance_inner_outer', 'LiftRotor|radius'] = 4.0
        partials['clearance_inner_outer', 'Wing|area'] = - (s_outer - s_inner) * db_dS
        partials['clearance_inner_outer', 'Wing|aspect_ratio'] = - (s_outer - s_inner) * db_dAR


class LiftRotorClearanceConstraintTypeThree(om.ExplicitComponent):
    """
    Variable spanwise stations for lift rotors.

    Two stations per semi-span:
        s_inner in [0,1], at y = ± (b/2) * s_inner
        s_outer in [0,1], at y = ± (b/2) * s_outer

    Constraints (residual <= 0 is feasible):
        inner vs fuselage:  (2r + 2c + D) - s_inner * b <= 0
        tip margin (outer): (2r + 2c)     - (1 - s_outer) * b <= 0
        inner-outer gap:    (4r + 2c)     - (s_outer - s_inner) * b <= 0
        ordering:           s_inner - s_outer <= 0

    Options:
        max_d_fuse: D [m]
        clearance_c: c [m]

    Inputs:
        LiftRotor|radius  r [m]
        Wing|area         S [m^2]
        Wing|aspect_ratio AR [-]
        LiftRotor|s_inner s_inner [-]
        LiftRotor|s_outer s_outer [-]

    Outputs:
        g_inner_fuse, g_tip_outer, g_inner_outer, g_order
    """

    def initialize(self):
        self.options.declare('max_d_fuse', types=float, desc='Fuselage max diameter D [m]')
        self.options.declare('clearance_c', types=float, default=0.10, desc='Fixed clearance c [m]')

    def setup(self):
        self.add_input('LiftRotor|radius', units='m')
        self.add_input('Wing|area', units='m**2')
        self.add_input('Wing|aspect_ratio', units=None)
        self.add_input('LiftRotor|s_inner', units=None)
        self.add_input('LiftRotor|s_outer', units=None)

        self.add_output('g_inner_fuse', units='m')
        self.add_output('g_tip_outer', units='m')
        self.add_output('g_inner_outer', units='m')
        self.add_output('g_order', units=None)

        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        D = self.options['max_d_fuse']
        c = self.options['clearance_c']

        r = inputs['LiftRotor|radius']
        S = inputs['Wing|area']
        AR = inputs['Wing|aspect_ratio']
        s_in = inputs['LiftRotor|s_inner']
        s_out = inputs['LiftRotor|s_outer']

        b = np.sqrt(S * AR)

        # Residuals (<= 0 feasible)
        outputs['g_inner_fuse'] = (2.0 * r + 2.0 * c + D) - s_in * b
        outputs['g_tip_outer'] = (2.0 * r + 2.0 * c) - (1.0 - s_out) * b
        outputs['g_inner_outer'] = (4.0 * r + 2.0 * c) - (s_out - s_in) * b

        # Ordering only (box bounds handled by design var limits)
        outputs['g_order'] = s_in - s_out

    def compute_partials(self, inputs, partials):
        S = inputs['Wing|area']
        AR = inputs['Wing|aspect_ratio']
        s_in = inputs['LiftRotor|s_inner']
        s_out = inputs['LiftRotor|s_outer']

        b = np.sqrt(S * AR)
        if b <= 0.0:
            db_dS = 0.0
            db_dAR = 0.0
        else:
            db_dS = 0.5 * (AR / b)
            db_dAR = 0.5 * (S / b)

        # g_inner_fuse = (2r+2c+D) - s_in * b
        partials['g_inner_fuse', 'LiftRotor|radius'] = 2.0
        partials['g_inner_fuse', 'LiftRotor|s_inner'] = -b
        partials['g_inner_fuse', 'Wing|area'] = - s_in * db_dS
        partials['g_inner_fuse', 'Wing|aspect_ratio'] = - s_in * db_dAR

        # g_tip_outer = (2r+2c) - (1 - s_out)*b = (2r+2c) - b + s_out*b
        partials['g_tip_outer', 'LiftRotor|radius'] = 2.0
        partials['g_tip_outer', 'LiftRotor|s_outer'] = b
        partials['g_tip_outer', 'Wing|area'] = - db_dS + s_out * db_dS
        partials['g_tip_outer', 'Wing|aspect_ratio'] = - db_dAR + s_out * db_dAR

        # g_inner_outer = (4r+2c) - (s_out - s_in) * b
        partials['g_inner_outer', 'LiftRotor|radius'] = 4.0
        partials['g_inner_outer', 'LiftRotor|s_inner'] = b
        partials['g_inner_outer', 'LiftRotor|s_outer'] = -b
        partials['g_inner_outer', 'Wing|area'] = - (s_out - s_in) * db_dS
        partials['g_inner_outer', 'Wing|aspect_ratio'] = - (s_out - s_in) * db_dAR

        # Ordering
        partials['g_order', 'LiftRotor|s_inner'] = 1.0
        partials['g_order', 'LiftRotor|s_outer'] = -1.0
