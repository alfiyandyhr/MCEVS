import openmdao.api as om


class CL_Calculation(om.ExplicitComponent):
    """
    Computes CL
    CL = lift / (0.5 * rho * v**2 * S_ref)
    """

    def initialize(self):
        self.options.declare('rho_air', types=float, desc='Air density')

    def setup(self):
        self.add_input('Aero|lift', units='N', desc='Aerodynamic lift')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_input('Wing|area', units='m**2', desc='Wing reference area')
        self.add_output('Aero|CL', units=None, desc='Lift coefficient')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        L = inputs['Aero|lift']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']

        q = 0.5 * rho_air * v**2

        outputs['Aero|CL'] = L / (q * S_wing)

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        L = inputs['Aero|lift']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']

        q = 0.5 * rho_air * v**2
        dq_dv = rho_air * v

        partials['Aero|CL', 'Aero|lift'] = 1.0 / (q * S_wing)
        partials['Aero|CL', 'Aero|speed'] = - L / (S_wing * q**2) * dq_dv
        partials['Aero|CL', 'Wing|area'] = - L / (q * S_wing**2)


class Drag_Calculation(om.ExplicitComponent):
    """
    Computes Drag
    Drag = 0.5 * rho * v**2 * S_ref * CD
    """

    def initialize(self):
        self.options.declare('rho_air', types=float, desc='Air density')

    def setup(self):
        self.add_input('Aero|CD', units=None, desc='Drag coefficient')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_input('Wing|area', units='m**2', desc='Wing reference area')
        self.add_output('Aero|drag', units='N', desc='Aerodynamic drag')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        CD = inputs['Aero|CD']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']

        q = 0.5 * rho_air * v**2

        outputs['Aero|drag'] = q * S_wing * CD

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        CD = inputs['Aero|CD']
        v = inputs['Aero|speed']
        S_wing = inputs['Wing|area']

        q = 0.5 * rho_air * v**2
        dq_dv = rho_air * v

        partials['Aero|drag', 'Aero|CD'] = q * S_wing
        partials['Aero|drag', 'Aero|speed'] = dq_dv * S_wing * CD
        partials['Aero|drag', 'Wing|area'] = q * CD
