import numpy as np
import openmdao.api as om


class CylindricalBodyDrag(om.ExplicitComponent):
    """
    Computes the drag of a cylindrical body
    Parameters:
            rho_air				: air density [kg/m**3]
    Inputs:
            Rotor|radius			: rotor radius [m]
            Aero|speed 				: air speed of the eVTOL [m/s]
            Body|sin_beta 			: body/fuselage tilt angle
    Outputs:
            Aero|total_drag 		: aerodynamic drag [N]
    Notes:
            > Assumes that the body/fuselage is a cylinder whose radius is 58% of rotor radius
            > The length-to-diameter ratio is assumed to be 2.5
    Source:
            B. Govindarajan and A. Sridharan, “Conceptual Sizing of Vertical Lift Package Delivery Platforms,”
            Journal of Aircraft, vol. 57, no. 6, pp. 1170–1188, Nov. 2020, doi: 10.2514/1.C035805.
    """
    def initialize(self):
        self.options.declare('rho_air', types=float, default=1.225, desc='Air density')

    def setup(self):
        self.add_input('Rotor|radius', units='m', desc='Rotor radius')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_input('Body|sin_beta', desc='sin(beta), beta: incidence angle of body')
        self.add_output('Aero|total_drag', units='N', desc='Drag of the body')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        r = inputs['Rotor|radius']
        v = inputs['Aero|speed']
        sin_beta = inputs['Body|sin_beta']

        S_body = 1.682 * r**2 				# body area
        CD_body = 0.1 + 0.2 * sin_beta**3 	# body drag coefficient

        outputs['Aero|total_drag'] = 0.5 * rho_air * v * v * S_body * CD_body

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        r = inputs['Rotor|radius']
        v = inputs['Aero|speed']
        sin_beta = inputs['Body|sin_beta']

        S_body = 1.682 * r**2 				# body area
        dS_dr = 1.682 * 2 * r
        CD_body = 0.1 + 0.2 * sin_beta**3 	# body drag coefficient
        dCD_dsinB = 0.2 * 3 * sin_beta**2

        partials['Aero|total_drag', 'Rotor|radius'] = 0.5 * rho_air * v * v * CD_body * dS_dr
        partials['Aero|total_drag', 'Aero|speed'] = rho_air * v * S_body * CD_body
        partials['Aero|total_drag', 'Body|sin_beta'] = 0.5 * rho_air * v * v * S_body * dCD_dsinB


class MultirotorParasiteDragViaWeightBasedRegression(om.ExplicitComponent):
    """
    Computes the drag of a multirotor body
    Parameters:
            N_rotor				: number or rotors
            rho_air				: air density [kg/m**3]
    Inputs:
            Weight|takeoff 			: total take-off weight [kg]
            Aero|speed 				: cruising speed of the eVTOL [m/s]
            Rotor|radius			: rotor radius [m]
    Outputs:
            Aero|Cd0				: parasite drag coefficient
            Aero|total_drag 		: aerodynamic drag [N]
    Notes:
            > Based on an empirical equivalent flat plat drag area
            > As a function of gross weight
    Source:
            1. Russell, C., et al., “Wind Tunnel and Hover Performance Test Results for Multicopter UAS Vehicles,”
               Report TM-2018-219758, NASA, February 2018.
            2. A. R. Kadhiresan and M. J. Duffy, “Conceptual Design and Mission Analysis for eVTOL Urban Air Mobility Flight Vehicle Configurations,”
               in AIAA Aviation 2019 Forum, Dallas, Texas: American Institute of Aeronautics and Astronautics, Jun. 2019. doi: 10.2514/6.2019-2873.
    """
    def initialize(self):
        self.options.declare('N_rotor', types=int, desc='Number of rotors')
        self.options.declare('rho_air', types=float, desc='Air density')

    def setup(self):
        self.add_input('Rotor|radius', units='m', desc='Rotor radius')
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_output('Aero|Cd0', desc='Parasite drag coefficient')
        self.add_output('Aero|total_drag', units='N', desc='Drag of a multirotor body')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        N_rotor = self.options['N_rotor']
        r = inputs['Rotor|radius']			  # in [m]
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']		# in [m/s**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**0.8903
        ft2_to_m2 = 0.3048 * 0.3048

        f = 0.0327 * W_takeoff**0.8903 * kg_to_lb * ft2_to_m2

        # Parasite drag coefficient
        S_ref = N_rotor * np.pi * r**2  # total rotor area
        CD0 = f / S_ref

        outputs['Aero|Cd0'] = CD0
        outputs['Aero|total_drag'] = 0.5 * rho_air * v * v * S_ref * CD0

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        N_rotor = self.options['N_rotor']
        r = inputs['Rotor|radius']			  # in [m]
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']		# in [m/s**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**0.8903
        ft2_to_m2 = 0.3048 * 0.3048

        f = 0.0327 * W_takeoff**0.8903 * kg_to_lb * ft2_to_m2
        df_dW = 0.0327 * 0.8903 * W_takeoff**(-0.1097) * kg_to_lb * ft2_to_m2

        partials['Aero|Cd0', 'Rotor|radius'] = f / (N_rotor * np.pi) * (-2 / r**3)
        partials['Aero|Cd0', 'Weight|takeoff'] = 1 / (N_rotor * np.pi * r**2) * df_dW
        partials['Aero|Cd0', 'Aero|speed'] = 0
        partials['Aero|total_drag', 'Rotor|radius'] = 0.0
        partials['Aero|total_drag', 'Weight|takeoff'] = 0.5 * rho_air * v**2 * df_dW
        partials['Aero|total_drag', 'Aero|speed'] = rho_air * v * f


class WingedParasiteDragViaWeightBasedRegression(om.ExplicitComponent):
    """
    Computes the drag of a winged configuration
    Parameters:
            rho_air				: air density [kg/m**3]
    Inputs:
            Weight|takeoff 		: total take-off weight [kg]
            Aero|speed 			: air speed of the eVTOL [m/s]
            Wing|area 			: wing area [m**2]
    Outputs:
            Aero|Cd0			: parasite drag coefficient
            Aero|parasite_drag	: parasite drag [N]
    Notes:
            > Based on an empirical equivalent flat plat drag area
            > As a function of gross weight
    Source:
            1. Harris, F. D., “Introduction to Autogyros, Helicopters, and Other V/STOL Aircraft,”
               National Aeronautics and Space Administration, Ames Research Center, Moffett Field, Calif, 2011.
            2. A. R. Kadhiresan and M. J. Duffy, “Conceptual Design and Mission Analysis for eVTOL Urban Air Mobility Flight Vehicle Configurations,”
               in AIAA Aviation 2019 Forum, Dallas, Texas: American Institute of Aeronautics and Astronautics, Jun. 2019. doi: 10.2514/6.2019-2873.
    """
    def initialize(self):
        self.options.declare('rho_air', types=float, default=1.225, desc='Air density')

    def setup(self):
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_input('Wing|area', units='m**2', desc='Wing reference area')
        self.add_output('Aero|Cd0', desc='Parasite drag coefficient')
        self.add_output('Aero|parasite_drag', units='N', desc='Parasite drag of a winged config')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        rho_air = self.options['rho_air']
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']			  # in [m/s**2]
        S_wing = inputs['Wing|area']		  # in [m**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**(2 / 3)
        ft2_to_m2 = 0.3048 * 0.3048

        f = 1.6 * (W_takeoff / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2

        # Parasite drag coefficient
        S_ref = S_wing 	# ref area is the wing area
        CD0 = f / S_ref

        outputs['Aero|Cd0'] = CD0
        outputs['Aero|parasite_drag'] = 0.5 * rho_air * v * v * S_ref * CD0

    def compute_partials(self, inputs, partials):
        rho_air = self.options['rho_air']
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']			  # in [m/s**2]
        S_wing = inputs['Wing|area']		  # in [m**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**(2 / 3)
        ft2_to_m2 = 0.3048 * 0.3048

        f = 1.6 * (W_takeoff / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
        df_dW = 1.6 * (2 / 3) * W_takeoff**(-1 / 3) * (1 / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
        S_ref = S_wing

        partials['Aero|Cd0', 'Weight|takeoff'] = 1 / S_ref * df_dW
        partials['Aero|Cd0', 'Aero|speed'] = 0
        partials['Aero|Cd0', 'Wing|area'] = -f / (S_ref**2)
        partials['Aero|parasite_drag', 'Weight|takeoff'] = 0.5 * rho_air * v**2 * df_dW
        partials['Aero|parasite_drag', 'Aero|speed'] = rho_air * v * f
        partials['Aero|parasite_drag', 'Wing|area'] = 0


class RotorHubParasiteDragFidelityZero(om.ExplicitComponent):
    """
    Computes the parasite drag of rotor hubs of wingless and winged configurations
    Parameters:
            vehicle			: MCEVS vehicle object
            rho_air			: air density [kg/m**3]
    Inputs:
            Weight|takeoff 	: total take-off weight [kg]
            Aero|speed 		: air speed of the eVTOL [m/s]
    Outputs:
            Aero|Cd0_rotor_hub				: parasite drag coefficient
            Aero|rotor_hub_parasite_drag	: parasite drag [N]
    Notes:
            > Based on an empirical equivalent flat plat drag area as a function of gross weight
            > Adapted from compound helicopter drag data
    Source:
            1. Harris, F. D., “Introduction to Autogyros, Helicopters, and Other V/STOL Aircraft,”
               National Aeronautics and Space Administration, Ames Research Center, Moffett Field, Calif, 2011.
            2. Yeo, H., and Johnson, W., “Aeromechanics Analysis of a Heavy Lift Slowed-Rotor Compound Helicopter,”
               Journal of Aircraft, Vol. 44, No. 2, 2007, pp. 501–508. https://doi.org/10.2514/1.23905
    """
    def initialize(self):
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('rho_air', types=float, desc='Air density')

    def setup(self):
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_input('Aero|speed', units='m/s', desc='Air speed')
        self.add_output('Aero|f_rotor_hub', units='m**2', desc='Rotor hub equivalent flat plate area')
        self.add_output('Aero|parasite_drag_rotor_hub', units='N', desc='Rotor hub parasite drag of a winged config')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        vehicle = self.options['vehicle']
        rho_air = self.options['rho_air']
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']			  # in [m/s**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**(2 / 3)
        ft2_to_m2 = 0.3048 * 0.3048

        if vehicle.configuration in ['Multirotor']:
            N_rotor = vehicle.lift_rotor.n_rotor
            f = N_rotor * 0.85 * ((W_takeoff / N_rotor) / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
        elif vehicle.configuration in ['LiftPlusCruise']:
            N_rotor = vehicle.lift_rotor.n_rotor + vehicle.propeller.n_propeller
            f = N_rotor * 0.40 * ((W_takeoff / N_rotor) / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2

        outputs['Aero|f_rotor_hub'] = f
        outputs['Aero|parasite_drag_rotor_hub'] = 0.5 * rho_air * v * v * f

    def compute_partials(self, inputs, partials):
        vehicle = self.options['vehicle']
        rho_air = self.options['rho_air']
        W_takeoff = inputs['Weight|takeoff']  # in [kg]
        v = inputs['Aero|speed']			  # in [m/s**2]

        # Equivalent flat plate area "f"
        kg_to_lb = 2.20462**(2 / 3)
        ft2_to_m2 = 0.3048 * 0.3048

        if vehicle.configuration in ['Multirotor']:
            N_rotor = vehicle.lift_rotor.n_rotor
            f = N_rotor * 0.85 * ((W_takeoff / N_rotor) / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
            df_dW = N_rotor * 0.85 * (2 / 3) * W_takeoff**(-1 / 3) * (1 / N_rotor / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
        elif vehicle.configuration in ['LiftPlusCruise']:
            N_rotor = vehicle.lift_rotor.n_rotor + vehicle.propeller.n_propeller
            f = N_rotor * 0.40 * ((W_takeoff / N_rotor) / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2
            df_dW = N_rotor * 0.40 * (2 / 3) * W_takeoff**(-1 / 3) * (1 / N_rotor / 1000)**(2 / 3) * kg_to_lb * ft2_to_m2

        partials['Aero|f_rotor_hub', 'Weight|takeoff'] = df_dW
        partials['Aero|f_rotor_hub', 'Aero|speed'] = 0
        partials['Aero|parasite_drag_rotor_hub', 'Weight|takeoff'] = 0.5 * rho_air * v**2 * df_dW
        partials['Aero|parasite_drag_rotor_hub', 'Aero|speed'] = rho_air * v * f
