import numpy as np
import openmdao.api as om


class PowerHoverDescentConstantSpeed(om.ExplicitComponent):
    """
    Computes the power required for hover descent with constant speed
    Parameters:
            N_rotor		 : number or lift rotors
            hover_FM	 : hover figure of merit
            rho_air		 : air density [kg/m**3]
            g 			 : gravitational acceleration [m/s**2]
    Inputs:
            Weight|takeoff  			: total take-off weight [kg]
            LiftRotor|radius			: lift rotor radius [m]
            Mission|hover_descent_speed : hover climb speed [m/s]
    Outputs:
            Power|HoverDescentConstantSpeed	: power required for hover descent [W]
            LiftRotor|thrust 				: thrust produced by each rotor during hover descent [N]
    """

    def initialize(self):
        self.options.declare('N_rotor', types=int, desc='Number of rotors')
        self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
        self.options.declare('rho_air', types=float, desc='Air density')
        self.options.declare('g', types=float, desc='Gravitational acceleration')

    def setup(self):
        self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
        self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
        self.add_input('Mission|hover_descent_speed', units='m/s', desc='Hover descent speed')
        self.add_output('Power|HoverDescentConstantSpeed', units='W', desc='Power required for hover descent')
        self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
        self.add_output('LiftRotor|T_to_P', units='g/W', desc='Thrust to power ratio of a single rotor/propeller')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        N_rotor = self.options['N_rotor']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        g = self.options['g']

        W_takeoff = inputs['Weight|takeoff']
        r = inputs['LiftRotor|radius']
        v_descent = inputs['Mission|hover_descent_speed']

        S_disk = np.pi * r**2

        # If v_descent is less than 2*v_hoverstay, use original hover descent model, otherwise use hover stay model
        if v_descent >= 2.0 * np.sqrt((W_takeoff * g) / (2 * rho_air * S_disk * N_rotor)):
            P_total = (W_takeoff * g) / hover_FM * ((v_descent / 2) - np.sqrt((v_descent / 2)**2 - (W_takeoff * g) / (2 * rho_air * S_disk * N_rotor)))
            outputs['Power|HoverDescentConstantSpeed'] = P_total
        else:
            P_total = 1 / hover_FM * np.sqrt(((W_takeoff * g)**3) / (2 * rho_air * S_disk * N_rotor))
            outputs['Power|HoverDescentConstantSpeed'] = P_total

        outputs['LiftRotor|thrust'] = (W_takeoff * g) / N_rotor
        outputs['LiftRotor|T_to_P'] = (W_takeoff * 1000.0) / P_total

    def compute_partials(self, inputs, partials):
        N_rotor = self.options['N_rotor']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        g = self.options['g']

        W_takeoff = inputs['Weight|takeoff']
        r = inputs['LiftRotor|radius']
        v_descent = inputs['Mission|hover_descent_speed']

        S_disk = np.pi * r**2
        dSdisk_dr = 2 * np.pi * r

        # If v_descent is less than 2*v_hoverstay, use original hover descent model, otherwise use hover stay model
        if v_descent >= 2.0 * np.sqrt((W_takeoff * g) / (2 * rho_air * S_disk * N_rotor)):
            P_total = (W_takeoff * g) / hover_FM * ((v_descent / 2) - np.sqrt((v_descent / 2)**2 - (W_takeoff * g) / (2 * rho_air * S_disk * N_rotor)))
            dP_dW = g * v_descent / (2 * hover_FM) - g / hover_FM * np.sqrt((v_descent / 2)**2 - (W_takeoff * g) / (2 * rho_air * S_disk * N_rotor)) + (W_takeoff * g**2) / (4 * hover_FM * rho_air * S_disk * N_rotor) * ((v_descent / 2)**2 - (W_takeoff * g) / (2 * rho_air * S_disk * N_rotor))**(-0.5)
            dP_dr = - (W_takeoff * g)**2 / (4 * hover_FM * rho_air * N_rotor * S_disk**2) * ((v_descent / 2)**2 - (W_takeoff * g) / (2 * rho_air * N_rotor * S_disk))**(-0.5) * dSdisk_dr
            partials['Power|HoverDescentConstantSpeed', 'Weight|takeoff'] = dP_dW
            partials['Power|HoverDescentConstantSpeed', 'LiftRotor|radius'] = dP_dr
            partials['Power|HoverDescentConstantSpeed', 'Mission|hover_descent_speed'] = 0.0
        else:
            P_total = 1 / hover_FM * np.sqrt(((W_takeoff * g)**3) / (2 * rho_air * S_disk * N_rotor))
            dP_dW = 1.5 / hover_FM * np.sqrt((W_takeoff * g**3) / (2 * rho_air * S_disk * N_rotor))
            dP_dr = - 0.5 / hover_FM * np.sqrt(((W_takeoff * g)**3) / (2 * rho_air * N_rotor * S_disk**3)) * dSdisk_dr
            partials['Power|HoverDescentConstantSpeed', 'Weight|takeoff'] = dP_dW
            partials['Power|HoverDescentConstantSpeed', 'LiftRotor|radius'] = dP_dr
            partials['Power|HoverDescentConstantSpeed', 'Mission|hover_descent_speed'] = 0.0

        partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor
        partials['LiftRotor|thrust', 'LiftRotor|radius'] = 0.0
        partials['LiftRotor|thrust', 'Mission|hover_descent_speed'] = 0.0
        partials['LiftRotor|T_to_P', 'Weight|takeoff'] = (1 * 1000.0) / P_total + (W_takeoff * 1000.0) * (-1 / P_total**2) * dP_dW
        partials['LiftRotor|T_to_P', 'LiftRotor|radius'] = (W_takeoff * 1000.0) * (-1 / P_total**2) * dP_dr
        partials['LiftRotor|T_to_P', 'Mission|hover_descent_speed'] = 0.0
