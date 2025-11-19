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

    def compute_component_weights(self):

        if self.weight.max_takeoff is None:
            raise ValueError("MTOW is not defined on vehicle.weight.max_takeoff")

        mtow = self.weight.max_takeoff

        # Apply group tf factors to each component
        self._apply_technology_factors()

        # Structure
        if self.fuselage:
            self.fuselage._calculate_weight_given_mtow(mtow)
        if self.wing:
            self.wing._calculate_weight_given_mtow(mtow)
        if self.horizontal_tail:
            self.horizontal_tail._calculate_weight_given_mtow(mtow)
        if self.vertical_tail:
            self.vertical_tail._calculate_weight_given_mtow(mtow)
        if self.landing_gear:
            self.landing_gear._calculate_weight_given_mtow(mtow)
        if self.boom:
            self.boom._calculate_weight_given_mtow(mtow)

        # # Propulsion (power-based for now)
        # if self.lift_rotor and p_max_lift is not None:
        #     self.lift_rotor.calculate_weight(p_max_lift)
        # if self.propeller and p_max_cruise is not None:
        #     self.propeller.calculate_weight(p_max_cruise)

        # # Totals
        # self.weight.structure = sum(filter(None, [
        #     getattr(self.fuselage, 'weight', None),
        #     getattr(self.wing, 'weight', None),
        #     getattr(self.horizontal_tail, 'weight', None),
        #     getattr(self.vertical_tail, 'weight', None),
        #     getattr(self.landing_gear, 'weight', None),
        #     getattr(self.boom, 'weight', None),
        # ])) or 0.0

        # self.weight.propulsion = sum(filter(None, [
        #     getattr(self.lift_rotor, 'weight', None),
        #     getattr(self.propeller, 'weight', None),
        # ])) or 0.0

        # self.weight.battery = getattr(self.battery, 'weight', None) or 0.0
        # self.weight.gross_takeoff = (self.weight.structure +
        #                              self.weight.propulsion +
        #                              self.weight.battery +
        #                              (self.weight.payload or 0.0))

    def _apply_technology_factors(self):

        # Structure group
        for comp in [
            getattr(self, 'fuselage', None),
            getattr(self, 'wing', None),
            getattr(self, 'horizontal_tail', None),
            getattr(self, 'vertical_tail', None),
            getattr(self, 'landing_gear', None),
            getattr(self, 'boom', None),
        ]:
            if comp is not None:
                # Only set if not already set by user kwargs
                if getattr(comp, 'technology_factor', None) is None:
                    comp.technology_factor = self.tf_structure

        # Propulsion group
        for comp in [
            getattr(self, 'lift_rotor', None),
            getattr(self, 'propeller', None),
            # motors, controllers, etc. in the future
        ]:
            if comp is not None:
                if getattr(comp, 'technology_factor', None) is None:
                    comp.technology_factor = self.tf_propulsion

        # Equipment group (if/when you add avionics, anti-icing, furnishings, etc.)
        for comp in [
            # getattr(self, 'avionics', None),
            # getattr(self, 'anti_icing', None),
            # getattr(self, 'flight_control', None),
            # getattr(self, 'furnishings', None),
        ]:
            if comp is not None:
                if getattr(comp, 'technology_factor', None) is None:
                    comp.technology_factor = self.tf_equipment


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
