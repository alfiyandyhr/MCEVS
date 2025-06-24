import openmdao.api as om


class LandingGearWeightNDARCFractional(om.ExplicitComponent):
    """
    Computes landing gear weight via NDARC method
    Parameters:
            gear_type : landing gear type; either "wheeled" or "skid"
            tf 		  : technology factor (a reduction due to the use of composites, e.g., 0.8)
    Inputs:
            Weight|takeoff : total take-off weight [kg]
    Outputs:
            Weight|structure|landing_gear : landing gear weight [kg]
    Notes:
            >
    Source:
            Johnson, W. NDARC - NASA Design and Analysis of Rotorcraft. NASA/TP–20220000355/APPX-A. Moffett Field, California. March 2023.
    """
    def initialize(self):
        self.options.declare('gear_type', types=str, desc='Gear type; either "wheeled" or "skid".')
        self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

    def setup(self):
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_output('Weight|structure|landing_gear', units='kg', desc='Landing gear weight')
        self.declare_partials('Weight|structure|landing_gear', 'Weight|takeoff')

    def compute(self, inputs, outputs):
        gear_type = self.options['gear_type']
        tf = self.options['tf']
        W_takeoff = inputs['Weight|takeoff']

        # Calculating W_landing_gear
        if gear_type == 'wheeled':
            f_LG = 0.0325
        elif gear_type == 'skid':
            f_LG = 0.014
        else:
            raise ValueError('Gear type should be either "wheeled" or "skid".')

        W_landing_gear = f_LG * W_takeoff

        outputs['Weight|structure|landing_gear'] = tf * W_landing_gear  # in [kg]

    def compute_partials(self, inputs, partials):
        gear_type = self.options['gear_type']
        tf = self.options['tf']

        # Calculating W_landing_gear
        if gear_type == 'wheeled':
            f_LG = 0.0325
        elif gear_type == 'skid':
            f_LG = 0.014
        else:
            raise ValueError('Gear type should be either "wheeled" or "skid".')

        partials['Weight|structure|landing_gear', 'Weight|takeoff'] = tf * f_LG


class LandingGearWeightRoskam(om.ExplicitComponent):
    """
    Computes landing gear weight via Roskam method
    Parameters:
            l_sm 	: shock strut length for main gear [m]
            n_ult	: design ultimate load factor for landing (=5.7)
            tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
    Inputs:
            Weight|takeoff : total take-off weight [kg]
    Outputs:
            Weight|structure|landing_gear : landing gear weight [kg]
    Notes:
            > Class II USAF Method for General Aviation airplanes
            > Used for light and utility type airplanes
            > Maximum cruise speed <= 300 kts (556 kph, Mach 0.45)
            > n_ult may be taken as 5.7
            > This method includes nose gear weight
    Source:
            Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
    """
    def initialize(self):
        self.options.declare('l_sm', types=float, desc='Main landing gear strut length')
        self.options.declare('n_ult', types=float, default=5.7, desc='Design ultimate load factor for landing')
        self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

    def setup(self):
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_output('Weight|structure|landing_gear', units='kg', desc='Landing gear weight')
        self.declare_partials('Weight|structure|landing_gear', 'Weight|takeoff')

    def compute(self, inputs, outputs):
        l_sm = self.options['l_sm']
        n_ult = self.options['n_ult']
        tf = self.options['tf']
        W_takeoff = inputs['Weight|takeoff']

        # Calculating W_landing_gear
        kg_to_lb = 2.20462**0.684
        m_to_ft = 3.28084**0.501
        lb_to_kg = 0.453592

        W_landing_gear = 0.054 * l_sm**0.501 * (W_takeoff * n_ult)**0.684 * kg_to_lb * m_to_ft * lb_to_kg

        outputs['Weight|structure|landing_gear'] = tf * W_landing_gear  # in [kg]

    def compute_partials(self, inputs, partials):
        l_sm = self.options['l_sm']
        n_ult = self.options['n_ult']
        W_takeoff = inputs['Weight|takeoff']
        tf = self.options['tf']

        # Calculating dWlg_dWtakeoff
        kg_to_lb = 2.20462**0.684
        m_to_ft = 3.28084**0.501
        lb_to_kg = 0.453592

        dWlg_dWtakeoff = 0.054 * l_sm**0.501 * 0.684 * W_takeoff**(-0.316) * n_ult**0.684 * kg_to_lb * m_to_ft * lb_to_kg

        partials['Weight|structure|landing_gear', 'Weight|takeoff'] = tf * dWlg_dWtakeoff
