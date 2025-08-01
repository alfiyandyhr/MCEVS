from MCEVS.Vehicles.Components.Wing import Wing
from MCEVS.Vehicles.Components.Fuselage import Fuselage
from MCEVS.Vehicles.Components.Landing_Gear import LandingGear
from MCEVS.Vehicles.Components.Rotors import LiftRotor, Propeller
from MCEVS.Vehicles.Components.Battery import Battery
from MCEVS.Vehicles.Components.Tails import HorizontalTail, VerticalTail
from MCEVS.Vehicles.Components.Boom import Boom

from MCEVS.Analyses.Weight.Analysis import VehicleWeight


class LiftPlusCruiseEVTOL(object):
    """
    docstring for LiftPlusCruiseEVTOL
    This is a type of eVTOL that has a fixed-wing and separate propulsion for hover and cruise.
    """

    def __init__(self, mtow=None, tf_structure=0.8, tf_equipment=0.8, tf_propulsion=0.8):
        super(LiftPlusCruiseEVTOL, self).__init__()

        self.configuration = 'LiftPlusCruise'
        self.n_passenger = None  # including the pilot

        # Components
        self.fuselage = None
        self.wing = None
        self.landing_gear = None
        self.empennage = None
        self.lift_rotor = None
        self.propeller = None
        self.battery = None
        self.horizontal_tail = None
        self.vertical_tail = None
        self.boom = None

        # Weight
        self.weight = VehicleWeight(mtow)

        # Aero performance, used when evaluated using fidelity > 0
        self.f_total_non_hub_non_wing = {'climb': None, 'cruise': None, 'descent': None}

        # Wetted area (an array of wetted area for all components) called by calc_wetted_area
        self.S_wetted = None

        # Whether or not this vehicle has been sized
        self.is_sized = False

        # Technology factors
        self.tf_propulsion = tf_propulsion
        self.tf_structure = tf_structure
        self.tf_equipment = tf_equipment

    def add_component(self, kind: str, **kwargs):

        if kind == 'battery':
            self.battery = Battery(kwargs)
            self.battery._initialize()

        elif kind == 'wing':
            self.wing = Wing(kwargs)
            self.wing._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.wing._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'fuselage':
            self.fuselage = Fuselage(kwargs)
            self.fuselage._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.fuselage._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'landing_gear':
            self.landing_gear = LandingGear(kwargs)
            self.landing_gear._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.landing_gear._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'lift_rotor':
            self.lift_rotor = LiftRotor(kwargs)
            self.lift_rotor._initialize()
            # self.lift_rotor._calculate_weight_given_max_power(183068.9599)

        elif kind == 'propeller':
            self.propeller = Propeller(kwargs)
            self.propeller._initialize()
            # self.propeller._calculate_weight_given_max_power(35669.31421)

        elif kind == 'horizontal_tail':
            self.horizontal_tail = HorizontalTail(kwargs)
            self.horizontal_tail._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.horizontal_tail._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'vertical_tail':
            self.vertical_tail = VerticalTail(kwargs)
            self.vertical_tail._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.vertical_tail._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'boom':
            self.boom = Boom(kwargs)
            self.boom._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.boom._calculate_weight_given_mtow(self.weight.max_takeoff)

    def print_info(self):
        print('Vehicle type: Lift+Cruise eVTOL')
        print('Technology factors:')
        print(f'\tTF propulsion = {self.tf_propulsion}')
        print(f'\tTF structure = {self.tf_structure}')
        print(f'\tTF equipment = {self.tf_equipment}')
        print('List of components:')
        print(self.battery._info())
        print(self.fuselage._info())
        print(self.wing._info())
        print(self.horizontal_tail._info())
        print(self.vertical_tail._info())
        print(self.landing_gear._info())
        print(self.lift_rotor._info())
        print(self.propeller._info())
        print(self.boom._info())


class MultirotorEVTOL(object):
    """
    docstring for MultirotorEVTOL
    This is a type of eVTOL that has a fixed-wing and separate propulsion for hover and cruise.
    """

    def __init__(self, mtow=None, tf_structure=0.8, tf_equipment=0.8, tf_propulsion=0.8):
        super(MultirotorEVTOL, self).__init__()

        self.configuration = 'Multirotor'
        self.n_passenger = None  # including the pilot

        # Components
        self.fuselage = None
        self.landing_gear = None
        self.lift_rotor = None
        self.battery = None
        self.boom = None

        # Weight
        self.weight = VehicleWeight(mtow)

        # Aero performance, used when evaluated using fidelity > 0
        self.f_total_non_hub = {'climb': None, 'cruise': None, 'descent': None}

        # Wetted area (an array of wetted area for all components) called by calc_wetted_area
        self.S_wetted = None

        # Whether or not this vehicle has been sized
        self.is_sized = False

        # Technology factors
        self.tf_propulsion = tf_propulsion
        self.tf_structure = tf_structure
        self.tf_equipment = tf_equipment

    def add_component(self, kind: str, **kwargs):

        if kind == 'battery':
            self.battery = Battery(kwargs)
            self.battery._initialize()

        elif kind == 'fuselage':
            self.fuselage = Fuselage(kwargs)
            self.fuselage._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.fuselage._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'landing_gear':
            self.landing_gear = LandingGear(kwargs)
            self.landing_gear._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.landing_gear._calculate_weight_given_mtow(self.weight.max_takeoff)

        elif kind == 'lift_rotor':
            self.lift_rotor = LiftRotor(kwargs)
            self.lift_rotor._initialize()
            # self.lift_rotor._calculate_weight_given_max_power(151102.2955)

        elif kind == 'boom':  # boom is represented as a wing object in multirotor
            self.boom = Boom(kwargs)
            self.boom._initialize()
            # if self.weight.max_takeoff is not None:
            # 	self.boom._calculate_weight_given_mtow(self.weight.max_takeoff)

    def print_info(self):
        print('Vehicle type: Multirotor eVTOL')
        print('Technology factors:')
        print(f'\tTF propulsion = {self.tf_propulsion}')
        print(f'\tTF structure = {self.tf_structure}')
        print(f'\tTF equipment = {self.tf_equipment}')
        print('List of components:')
        print(self.battery._info())
        print(self.fuselage._info())
        print(self.boom._info())
        print(self.landing_gear._info())
        print(self.lift_rotor._info())
