import numpy as np


class LiftRotor(object):
    """
    docstring for LiftRotor
    """

    def __init__(self, kwargs: dict):
        super(LiftRotor, self).__init__()
        self.kwargs = kwargs
        self.name = 'lift_rotor'

        # Geometric information
        self.n_rotor = None
        self.n_blade = None
        self.solidity = None
        self.radius = None
        self.mean_c_to_R = None

        # Section info
        self.airfoil = None
        self.n_section = None
        self.r_to_R_list = None
        self.c_to_R_list = None
        self.w_to_R_list = None
        self.global_twist = 0.0
        self.pitch_list = None
        self.pitch_linear_grad = None

        # Weight and performance
        self.weight = None
        self.figure_of_merit = None
        self.RPM = None
        self.Cd0 = None

        # Hub info
        self.hub_radius = None
        self.hub_length = None
        self.hub_max_diameter = None

        # Use of new material that reduces weight?
        self.technology_factor = None

        # Attached motor
        self.motor_power_margin = 50.0  # %; default value

    def _initialize(self):
        for item in list(self.kwargs):
            if item == 'n_rotor':
                self.n_rotor = int(self.kwargs[item])
            elif item == 'n_blade':
                self.n_blade = int(self.kwargs[item])
            elif item == 'n_section':
                self.n_section = int(self.kwargs[item])
            elif item == 'airfoil':
                self.airfoil = self.kwargs[item]
            elif item == 'r_to_R_list':
                self.r_to_R_list = self.kwargs[item]
            elif item == 'c_to_R_list':
                self.c_to_R_list = self.kwargs[item]
            elif item == 'w_to_R_list':
                self.w_to_R_list = self.kwargs[item]
            elif item == 'pitch_list':
                self.pitch_list = self.kwargs[item]
            elif item == 'pitch_linear_grad':
                self.pitch_linear_grad = float(self.kwargs[item])
            elif item == 'global_twist':
                self.global_twist = float(self.kwargs[item])
            elif item == 'radius':
                self.radius = float(self.kwargs[item])
            elif item == 'hub_radius':
                self.hub_radius = float(self.kwargs[item])
            elif item == 'mean_c_to_R':
                self.mean_c_to_R = float(self.kwargs[item])
            elif item == 'hub_length':
                self.hub_length = float(self.kwargs[item])
            elif item == 'hub_max_diameter':
                self.hub_max_diameter = float(self.kwargs[item])
            elif item == 'Cd0':
                self.Cd0 = float(self.kwargs[item])
            elif item == 'figure_of_merit':
                self.figure_of_merit = float(self.kwargs[item])
            elif item == 'RPM':
                self.RPM = self.kwargs[item]
            elif item == 'technology_factor':
                self.technology_factor = float(self.kwargs[item])
            elif item == 'motor_power_margin':
                self.motor_power_margin = float(self.kwargs[item])

        self.solidity = self.n_blade * self.mean_c_to_R / np.pi

    def _calculate_weight_given_max_power(self, p_max):
        k_prop = 0.144  # for piston engines; 0.108 for turboprops
        W_rotor = k_prop * (2 * self.radius * 3.28084)**0.782
        W_rotor *= (p_max * 0.00134101 / self.n_rotor)**0.782
        W_rotor *= (self.n_blade**0.5)**0.782
        self.weight = self.n_rotor * W_rotor * 0.453592 * self.technology_factor

    def _info(self):
        info = '\tComponent name: Lift Rotor\n'
        info += f'\t\tNumber of rotor(s) = {self.n_rotor}\n'
        info += f'\t\tNumber of blade(s) per rotor = {self.n_blade}\n'
        info += f'\t\tNumber of section(s) per blade = {self.n_section}\n'
        info += f'\t\tSection airfoil = {self.airfoil}\n'
        info += f'\t\tSection r/R = {np.round(self.r_to_R_list,4)}\n'
        info += f'\t\tSection c/R = {np.round(self.c_to_R_list,4)}\n'
        info += f'\t\tSection w/R = {np.round(self.w_to_R_list,4)}\n'
        info += f'\t\tSection pitch = {np.round(self.pitch_list,4)}\n'
        info += f'\t\tPitch linear grad = {self.pitch_linear_grad}\n'
        info += f'\t\tGlobal twist = {np.round(self.global_twist,4)}\n'
        info += f'\t\tSolidity = {self.solidity}\n'
        info += f'\t\tRadius = {self.radius} m\n'
        info += f'\t\tHub radius = {self.hub_radius} m\n'
        info += f'\t\tMean c_to_R = {self.mean_c_to_R}\n'
        info += f'\t\tCd0 = {self.Cd0}\n'
        info += f'\t\tFigure of Merit = {self.figure_of_merit}\n'
        info += f'\t\tRPM = {self.RPM}\n'
        info += f'\t\tMotor power margin = {self.motor_power_margin}%'
        return info

    def print_info(self):
        print('Component name: Lift Rotor')
        print(f'\tNumber of rotor(s) = {self.n_rotor}')
        print(f'\tNumber of blade(s) per rotor = {self.n_blade}')
        print(f'\tNumber of section(s) per blade = {self.n_section}')
        print(f'\tSection airfoil = {self.airfoil}')
        print(f'\tSection r/R = {np.round(self.r_to_R_list,4)}')
        print(f'\tSection c/R = {np.round(self.c_to_R_list,4)}')
        print(f'\tSection w/R = {np.round(self.w_to_R_list,4)}')
        print(f'\tSection pitch = {np.round(self.pitch_list,4)}')
        print(f'\tPitch linear grad = {self.self.pitch_linear_grad}')
        print(f'\tGlobal twist = {np.round(self.global_twist,4)}')
        print(f'\tSolidity = {self.solidity}')
        print(f'\tRadius = {self.radius} m')
        print(f'\tHub radius = {self.hub_radius} m')
        print(f'\tMean c_to_R = {self.mean_c_to_R}')
        print(f'\tCd0 = {self.Cd0}')
        print(f'\tFigure of Merit = {self.figure_of_merit}')
        print(f'\tRPM = {self.RPM}')
        print(f'\tMotor power margin = {self.motor_power_margin}%')


class Propeller(object):
    """
    docstring for Propeller
    """

    def __init__(self, kwargs: dict):
        super(Propeller, self).__init__()
        self.kwargs = kwargs
        self.name = 'propeller'

        # Geometric information
        self.n_propeller = None
        self.n_blade = None
        self.solidity = None
        self.radius = None
        self.mean_c_to_R = None

        # Section info
        self.airfoil = None
        self.n_section = None
        self.r_to_R_list = None
        self.c_to_R_list = None
        self.w_to_R_list = None
        self.pitch_list = None
        self.global_twist = 0.0
        self.pitch_linear_grad = None

        # Weight and performance
        self.weight = None
        self.Cd0 = None
        self.figure_of_merit = None
        self.RPM = None

        # Hub info
        self.hub_radius = None
        self.hub_length = None
        self.hub_max_diameter = None

        # Use of new material that reduces weight?
        self.technology_factor = None

        # Attached motor
        self.motor_power_margin = 50.0  # %; default value

    def _initialize(self):
        for item in list(self.kwargs):
            if item == 'n_propeller':
                self.n_propeller = int(self.kwargs[item])
            elif item == 'n_blade':
                self.n_blade = int(self.kwargs[item])
            elif item == 'n_section':
                self.n_section = int(self.kwargs[item])
            elif item == 'airfoil':
                self.airfoil = self.kwargs[item]
            elif item == 'r_to_R_list':
                self.r_to_R_list = self.kwargs[item]
            elif item == 'c_to_R_list':
                self.c_to_R_list = self.kwargs[item]
            elif item == 'w_to_R_list':
                self.w_to_R_list = self.kwargs[item]
            elif item == 'pitch_list':
                self.pitch_list = self.kwargs[item]
            elif item == 'pitch_linear_grad':
                self.pitch_linear_grad = float(self.kwargs[item])
            elif item == 'global_twist':
                self.global_twist = float(self.kwargs[item])
            elif item == 'radius':
                self.radius = float(self.kwargs[item])
            elif item == 'hub_radius':
                self.hub_radius = float(self.kwargs[item])
            elif item == 'mean_c_to_R':
                self.mean_c_to_R = float(self.kwargs[item])
            elif item == 'hub_length':
                self.hub_length = float(self.kwargs[item])
            elif item == 'hub_max_diameter':
                self.hub_max_diameter = float(self.kwargs[item])
            elif item == 'Cd0':
                self.Cd0 = float(self.kwargs[item])
            elif item == 'figure_of_merit':
                self.figure_of_merit = float(self.kwargs[item])
            elif item == 'RPM':
                self.RPM = self.kwargs[item]
            elif item == 'technology_factor':
                self.technology_factor = float(self.kwargs[item])
            elif item == 'motor_power_margin':
                self.motor_power_margin = float(self.kwargs[item])

        self.solidity = self.n_blade * self.mean_c_to_R / np.pi

    def _calculate_weight_given_max_power(self, p_max):
        k_prop = 0.144  # for piston engines; 0.108 for turboprops
        W_prop = k_prop * (2 * self.radius * 3.28084)**0.782
        W_prop *= (p_max * 0.00134101 / self.n_propeller)**0.782
        W_prop *= (self.n_blade**0.5)**0.782
        self.weight = self.n_propeller * W_prop * 0.453592 * self.technology_factor

    def _info(self):
        info = '\tComponent name: Propeller\n'
        info += f'\t\tNumber of propeller(s) = {self.n_propeller}\n'
        info += f'\t\tNumber of blade(s) per propeller = {self.n_blade}\n'
        info += f'\t\tNumber of section(s) per blade = {self.n_section}\n'
        info += f'\t\tSection airfoil = {self.airfoil}\n'
        info += f'\t\tSection r/R = {np.round(self.r_to_R_list,4)}\n'
        info += f'\t\tSection c/R = {np.round(self.c_to_R_list,4)}\n'
        info += f'\t\tSection w/R = {np.round(self.w_to_R_list,4)}\n'
        info += f'\t\tSection pitch = {np.round(self.pitch_list,4)}\n'
        info += f'\t\tPitch linear grad = {self.pitch_linear_grad}\n'
        info += f'\t\tGlobal twist = {np.round(self.global_twist,4)}\n'
        info += f'\t\tSolidity = {self.solidity}\n'
        info += f'\t\tRadius = {self.radius} m\n'
        info += f'\t\tHub radius = {self.hub_radius} m\n'
        info += f'\t\tMean c_to_R = {self.mean_c_to_R}\n'
        info += f'\t\tCd0 = {self.Cd0}\n'
        info += f'\t\tFigure of Merit = {self.figure_of_merit}\n'
        info += f'\t\tRPM = {self.RPM}\n'
        info += f'\t\tMotor power margin = {self.motor_power_margin}%'
        return info

    def print_info(self):
        print('Component name: Propeller')
        print(f'\tNumber of propeller(s) = {self.n_propeller} m')
        print(f'\tNumber of blade(s) per propeller = {self.n_blade}')
        print(f'\tNumber of section(s) per blade = {self.n_section}')
        print(f'\tSection airfoil = {self.airfoil}')
        print(f'\tSection r/R = {np.round(self.r_to_R_list,4)}')
        print(f'\tSection c/R = {np.round(self.c_to_R_list,4)}')
        print(f'\tSection w/R = {np.round(self.w_to_R_list,4)}')
        print(f'\tSection pitch = {np.round(self.pitch_list,4)}')
        print(f'\tPitch linear grad = {np.round(self.self.pitch_linear_grad,4)}')
        print(f'\tGlobal twist = {np.round(self.global_twist,4)}')
        print(f'\tSolidity = {self.solidity} m')
        print(f'\tRadius = {self.radius} m')
        print(f'\tHub radius = {self.hub_radius} m')
        print(f'\tMean c_to_R = {self.mean_c_to_R}')
        print(f'\tCd0 = {self.Cd0}')
        print(f'\tFigure of Merit = {self.figure_of_merit}')
        print(f'\tRPM = {self.RPM}')
        print(f'\tMotor power margin = {self.motor_power_margin}%')
