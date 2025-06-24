import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Power.Hover.Stay import PowerHoverStay
from MCEVS.Analyses.Power.HoverClimb.Constant_Speed import PowerHoverClimbConstantSpeedMT, PowerHoverClimbConstantSpeedMMT, PowerHoverClimbConstantSpeedBEMT
from MCEVS.Analyses.Power.HoverDescent.Constant_Speed import PowerHoverDescentConstantSpeed
from MCEVS.Analyses.Power.Climb.Constant_Vy_Constant_Vx import PowerClimbConstantVyConstantVxWithWing, PowerClimbConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Descent.Constant_Vy_Constant_Vx import PowerDescentConstantVyConstantVxWithWing, PowerDescentConstantVyConstantVxEdgewise
from MCEVS.Analyses.Power.Cruise.Constant_Speed import PowerCruiseConstantSpeedEdgewise, PowerCruiseConstantSpeedWithWing
from MCEVS.Analyses.Power.Others.Constant_Power import PowerConstantFractionOfMaxPower
from MCEVS.Analyses.Geometry.Rotor import MeanChord
from MCEVS.Utils.Performance import record_performance_by_segments
from MCEVS.Utils.Checks import check_fidelity_dict


class PowerAnalysis(object):
    """
    docstring for PowerAnalysis
    """

    def __init__(self, vehicle: object, mission: object, fidelity: dict):
        super(PowerAnalysis, self).__init__()
        self.vehicle = vehicle
        self.mission = mission
        self.fidelity = fidelity

        # Check solver fidelity
        check_fidelity_dict(self.fidelity, self.vehicle.configuration)

    def evaluate(self, record=False):
        # print('### --- Solving for power requirement --- ###')

        # --- Design parameters --- #

        if self.vehicle.configuration == 'Multirotor':
            r_lift_rotor = self.vehicle.lift_rotor.radius 					# m
            r_hub_lift_rotor = self.vehicle.lift_rotor.hub_radius 				# m
            mean_c_to_R_lift_rotor = self.vehicle.lift_rotor.mean_c_to_R
            global_twist_lift_rotor = self.vehicle.lift_rotor.global_twist  			# deg

        elif self.vehicle.configuration == 'LiftPlusCruise':
            r_lift_rotor = self.vehicle.lift_rotor.radius 					# m
            r_hub_lift_rotor = self.vehicle.lift_rotor.hub_radius 				# m
            mean_c_to_R_lift_rotor = self.vehicle.lift_rotor.mean_c_to_R
            global_twist_lift_rotor = self.vehicle.lift_rotor.global_twist  			# deg
            r_propeller = self.vehicle.propeller.radius 					# m
            mean_c_to_R_propeller = self.vehicle.propeller.mean_c_to_R
            wing_area = self.vehicle.wing.area 							# m**2
            wing_aspect_ratio = self.vehicle.wing.aspect_ratio
            wing_airfoil_CL_alpha = self.vehicle.wing.airfoil.CL_alpha 				# 1/rad
            wing_airfoil_CL_0 = self.vehicle.wing.airfoil.CL_0
            htail_area = self.vehicle.horizontal_tail.area 				# m**2
            htail_aspect_ratio = self.vehicle.horizontal_tail.aspect_ratio
            htail_max_root_thickness = self.vehicle.horizontal_tail.max_root_thickness 	# m
            vtail_area = self.vehicle.vertical_tail.area 					# m**2
            vtail_aspect_ratio = self.vehicle.vertical_tail.aspect_ratio
            vtail_max_root_thickness = self.vehicle.vertical_tail.max_root_thickness 	# m
            vtail_sweep_angle = self.vehicle.vertical_tail.sweep_angle 			# deg
            l_fuse = self.vehicle.fuselage.length 						# m

        # --- OpenMDAO probolem --- #
        prob = om.Problem()
        indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

        # MTOW should be defined since this is not in sizing mode
        if self.vehicle.weight.max_takeoff is None:
            raise ValueError('Vehicle MTOW should be defined, since PowerAnalysis() is never in sizing mode!')
        else:
            indeps.add_output('Weight|takeoff', self.vehicle.weight.max_takeoff, units='kg')

        # Hover climb RPM should be defined since this is not in sizing mode
        if self.vehicle.lift_rotor.RPM['hover_climb'] is None:
            raise ValueError('Hover climb RPM should be defined, since PowerAnalysis() is never in sizing mode!')
        else:
            indeps.add_output('LiftRotor|HoverClimb|RPM', self.vehicle.lift_rotor.RPM['hover_climb'], units='rpm')

        for segment in self.mission.segments:
            if segment.kind not in ['ConstantPower', 'NoCreditClimb', 'NoCreditDescent', 'ReserveCruise']:
                indeps.add_output(f'Mission|segment_{segment.id}|speed', segment.speed, units='m/s')
                indeps.add_output(f'Mission|segment_{segment.id}|distance', segment.distance, units='m')
            if segment.kind == 'CruiseConstantSpeed':
                if self.vehicle.configuration == 'Multirotor':
                    indeps.add_output('LiftRotor|Cruise|RPM', self.vehicle.lift_rotor.RPM['cruise'], units='rpm')
                elif self.vehicle.configuration == 'LiftPlusCruise':
                    indeps.add_output('Propeller|Cruise|RPM', self.vehicle.propeller.RPM['cruise'], units='rpm')
            if segment.kind == 'ClimbConstantVyConstantVx':
                if self.vehicle.configuration == 'Multirotor':
                    indeps.add_output('LiftRotor|Climb|RPM', self.vehicle.lift_rotor.RPM['climb'], units='rpm')
                elif self.vehicle.configuration == 'LiftPlusCruise':
                    indeps.add_output('Propeller|Climb|RPM', self.vehicle.propeller.RPM['climb'], units='rpm')
            if segment.kind == 'DescentConstantVyConstantVx':
                if self.vehicle.configuration == 'Multirotor':
                    indeps.add_output('LiftRotor|Descent|RPM', self.vehicle.lift_rotor.RPM['descent'], units='rpm')
                elif self.vehicle.configuration == 'LiftPlusCruise':
                    indeps.add_output('Propeller|Descent|RPM', self.vehicle.propeller.RPM['descent'], units='rpm')

        if self.vehicle.configuration == 'Multirotor':
            indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
            indeps.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
            indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
            indeps.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')

        elif self.vehicle.configuration == 'LiftPlusCruise':
            indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
            indeps.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
            indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
            indeps.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')
            indeps.add_output('Propeller|radius', r_propeller, units='m')
            indeps.add_output('Propeller|mean_c_to_R', mean_c_to_R_propeller, units=None)
            indeps.add_output('Wing|area', wing_area, units='m**2')
            indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)
            indeps.add_output('Wing|airfoil|CL_alpha', wing_airfoil_CL_alpha, units='1/rad')
            indeps.add_output('Wing|airfoil|CL_0', wing_airfoil_CL_0)
            indeps.add_output('HorizontalTail|area', htail_area, units='m**2')
            indeps.add_output('HorizontalTail|aspect_ratio', htail_aspect_ratio)
            indeps.add_output('HorizontalTail|max_root_thickness', htail_max_root_thickness, units='m')
            indeps.add_output('VerticalTail|area', vtail_area, units='m**2')
            indeps.add_output('VerticalTail|aspect_ratio', vtail_aspect_ratio)
            indeps.add_output('VerticalTail|max_root_thickness', vtail_max_root_thickness, units='m')
            indeps.add_output('VerticalTail|sweep_angle', vtail_sweep_angle, units='deg')
            indeps.add_output('Fuselage|length', l_fuse, units='m')

        # Variables needed for BEMT
        if self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
            n_sections = self.vehicle.lift_rotor.n_section
            r_to_R_list = self.vehicle.lift_rotor.r_to_R_list
            c_to_R_list = self.vehicle.lift_rotor.c_to_R_list
            w_to_R_list = self.vehicle.lift_rotor.w_to_R_list
            if self.vehicle.lift_rotor.pitch_linear_grad is not None:
                indeps.add_output('LiftRotor|pitch_linear_grad', self.vehicle.lift_rotor.pitch_linear_grad, units='deg')
            else:
                pitch_list = np.array(self.vehicle.lift_rotor.pitch_list)
                for i in range(n_sections):
                    indeps.add_output(f'LiftRotor|Section{i+1}|pitch', pitch_list[i], units='deg')
            for i in range(n_sections):
                indeps.add_output(f'LiftRotor|Section{i+1}|r_to_R', r_to_R_list[i], units=None)
                indeps.add_output(f'LiftRotor|Section{i+1}|c_to_R', c_to_R_list[i], units=None)
                indeps.add_output(f'LiftRotor|Section{i+1}|w_to_R', w_to_R_list[i], units=None)

        # Geometric analysis
        if self.vehicle.configuration == 'Multirotor':
            # Convert mean_c_to_R into mean_chord
            prob.model.add_subsystem('chord_calc_lift_rotor',
                                     MeanChord(),
                                     promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
                                     promotes_outputs=[('mean_chord', 'LiftRotor|chord')])

        elif self.vehicle.configuration == 'LiftPlusCruise':
            # Convert mean_c_to_R into mean_chord
            prob.model.add_subsystem('chord_calc_lift_rotor',
                                     MeanChord(),
                                     promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
                                     promotes_outputs=[('mean_chord', 'LiftRotor|chord')])
            prob.model.add_subsystem('chord_calc_propeller',
                                     MeanChord(),
                                     promotes_inputs=[('mean_c_to_R', 'Propeller|mean_c_to_R'), ('R', 'Propeller|radius')],
                                     promotes_outputs=[('mean_chord', 'Propeller|chord')])

        # Core power module
        prob.model.add_subsystem('power_model',
                                 PowerRequirement(mission=self.mission,
                                                  vehicle=self.vehicle,
                                                  fidelity=self.fidelity),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

        # Run the model (not in sizing mode !!!)
        prob.setup(check=False)
        prob.run_model()

        if record:
            record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

        return prob


class PowerRequirement(om.Group):
    """
    docstring for PowerRequirement
    """

    def initialize(self):
        self.options.declare('mission', types=object, desc='Mission object')
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
        self.options.declare('rhs_checking', types=bool, desc='rhs_checking in OpenMDAO linear solver')

    def setup(self):

        # Unpacking option objects
        mission = self.options['mission']
        vehicle = self.options['vehicle']
        fidelity = self.options['fidelity']
        rhs_checking = self.options['rhs_checking']

        # Unpacking cruise AoA
        for segment in mission.segments:
            if segment.kind == 'CruiseConstantSpeed':
                AoA = segment.AoA
            if segment.kind == 'HoverClimbConstantSpeed':
                # v_climb = segment.speed
                pass
            if segment.kind == 'HoverDescentConstantSpeed':
                # v_descent = segment.speed
                pass

        # Unpacking vehicle parameters
        N_lift_rotor = vehicle.lift_rotor.n_rotor			# number of lift rotors
        n_blade_lift_rotor = vehicle.lift_rotor.n_blade 			# number of blades per rotor
        Cd0_lift_rotor = vehicle.lift_rotor.Cd0 				# rotor's drag coefficient
        hover_FM_lift_rotor = vehicle.lift_rotor.figure_of_merit 	# hover figure of merit

        if vehicle.configuration == 'Multirotor':
            pass

        elif vehicle.configuration == 'LiftPlusCruise':
            N_propeller = vehicle.propeller.n_propeller			# number of propellers
            n_blade_propeller = vehicle.propeller.n_blade 			# number of blades per propeller
            Cd0_propeller = vehicle.propeller.Cd0 				# propeller's drag coefficient
            hover_FM_propeller = vehicle.propeller.figure_of_merit		# hover figure of merit
        else:
            raise RuntimeError('eVTOL configuration is not available.')

        # -------------------------------------------------------------#
        # --- Calculate power consumptions for each flight segment --- #
        # -------------------------------------------------------------#

        ids_for_max_p = []

        for segment in mission.segments:

            # Unpacking constants for each segment that needs them
            if segment.kind not in ['ConstantPower', 'NoCreditClimb', 'NoCreditDescent', 'ReserveCruise']:
                ids_for_max_p.append(segment.id) 		# segment id for calculating maximum power
                rho_air = segment.constants['rho'] 		# air density
                mu_air = segment.constants['mu'] 		# air dynamic viscosity
                v_sound = segment.constants['v_sound']  # sound speed
                g = segment.constants['g']		# gravitational acceleration

            # LiftPlusCruise's propellers do not work during hover, hoverclimb, or hoverdescent
            # and its lift rotor does not work during cruise, climb, descent, or constant power segment
            if vehicle.configuration == 'LiftPlusCruise':
                if segment.kind in ['HoverStay', 'HoverClimbConstantSpeed', 'HoverDescentConstantSpeed', 'ConstantPower', 'NoCreditClimb', 'NoCreditDescent']:
                    zero_p = om.IndepVarComp(f'Power|Propeller|segment_{segment.id}', val=0.0, units='W')
                    self.add_subsystem(f'zero_p_{segment.id}', zero_p, promotes=['*'])
                    zero_t = om.IndepVarComp(f'Propeller|thrust_each|segment_{segment.id}', val=0.0, units='N')
                    self.add_subsystem(f'zero_t_{segment.id}', zero_t, promotes=['*'])
                if segment.kind in ['CruiseConstantSpeed', 'ClimbConstantVyConstantVx', 'DescentConstantVyConstantVx']:
                    zero_p = om.IndepVarComp(f'Power|LiftRotor|segment_{segment.id}', val=0.0, units='W')
                    self.add_subsystem(f'zero_p_{segment.id}', zero_p, promotes=['*'])
                    zero_t = om.IndepVarComp(f'LiftRotor|thrust_each|segment_{segment.id}', val=0.0, units='N')
                    self.add_subsystem(f'zero_t_{segment.id}', zero_t, promotes=['*'])

            if segment.kind == 'HoverStay':
                self.add_subsystem(f'segment_{segment.id}_power',
                                   PowerHoverStay(N_rotor=N_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, g=g),
                                   promotes_inputs=['Weight|takeoff', 'LiftRotor|radius'],
                                   promotes_outputs=[('Power|HoverStay', f'Power|LiftRotor|segment_{segment.id}'),
                                                     ('LiftRotor|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

            if segment.kind == 'HoverClimbConstantSpeed':
                if fidelity['power_model']['hover_climb'] == 'MomentumTheory':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerHoverClimbConstantSpeedMT(N_rotor=N_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, g=g),
                                       promotes_inputs=['Weight|takeoff', ('Mission|hover_climb_speed', f'Mission|segment_{segment.id}|speed'), 'LiftRotor|*'],
                                       promotes_outputs=[('Power|HoverClimbConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
                                                         ('LiftRotor|thrust', f'LiftRotor|thrust_each|segment_{segment.id}'),
                                                         ('FM', 'LiftRotor|HoverClimb|FM'), ('LiftRotor|T_to_P', 'LiftRotor|HoverClimb|T_to_P')])

                elif fidelity['power_model']['hover_climb'] == 'ModifiedMomentumTheory':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerHoverClimbConstantSpeedMMT(N_rotor=N_lift_rotor, hover_FM=hover_FM_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0_lift_rotor, rho_air=rho_air, g=g),
                                       promotes_inputs=['Weight|takeoff', ('Mission|hover_climb_speed', f'Mission|segment_{segment.id}|speed'), 'LiftRotor|*'],
                                       promotes_outputs=[('Power|HoverClimbConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
                                                         ('LiftRotor|HoverClimb|thrust', f'LiftRotor|thrust_each|segment_{segment.id}'),
                                                         'LiftRotor|HoverClimb|FM', 'LiftRotor|HoverClimb|thrust_coefficient'])

                elif fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerHoverClimbConstantSpeedBEMT(vehicle=vehicle, rho_air=rho_air, g=g),
                                       promotes_inputs=['Weight|takeoff', ('Mission|hover_climb_speed', f'Mission|segment_{segment.id}|speed'), 'LiftRotor|*'],
                                       promotes_outputs=[('Power|HoverClimbConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
                                                         ('LiftRotor|thrust', f'LiftRotor|thrust_each|segment_{segment.id}'),
                                                         ('thrust_residual_square', 'LiftRotor|HoverClimb|thrust_residual_square'), ('J', 'LiftRotor|HoverClimb|J'),
                                                         ('FM', 'LiftRotor|HoverClimb|FM'), ('CT', 'LiftRotor|HoverClimb|thrust_coefficient')])

            if segment.kind == 'HoverDescentConstantSpeed':
                self.add_subsystem(f'segment_{segment.id}_power',
                                   PowerHoverDescentConstantSpeed(N_rotor=N_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, g=g),
                                   promotes_inputs=['Weight|takeoff', ('Mission|hover_descent_speed', f'Mission|segment_{segment.id}|speed'), 'LiftRotor|radius'],
                                   promotes_outputs=[('Power|HoverDescentConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'),
                                                     ('LiftRotor|thrust', f'LiftRotor|thrust_each|segment_{segment.id}'), ('LiftRotor|T_to_P', 'LiftRotor|HoverDescent|T_to_P')])

            if segment.kind == 'ClimbConstantVyConstantVx':
                if vehicle.configuration == 'Multirotor':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerClimbConstantVyConstantVxEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, mu_air=mu_air, g=g, climb_airspeed=segment.speed, gamma=segment.gamma, fidelity=fidelity),
                                       promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
                                       promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|LiftRotor|segment_{segment.id}'),
                                                         ('LiftRotor|Climb|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

                elif vehicle.configuration == 'LiftPlusCruise':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerClimbConstantVyConstantVxWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, Cd0=Cd0_propeller, hover_FM=hover_FM_propeller, rho_air=rho_air, mu_air=mu_air, g=g, AoA=AoA, gamma=segment.gamma, climb_airspeed=segment.speed, fidelity=fidelity),
                                       promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
                                       promotes_outputs=[('Power|ClimbConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
                                                         ('Propeller|Climb|thrust', f'Propeller|thrust_each|segment_{segment.id}')])

            if segment.kind == 'DescentConstantVyConstantVx':
                if vehicle.configuration == 'Multirotor':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerDescentConstantVyConstantVxEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, mu_air=mu_air, g=g, descent_airspeed=segment.speed, gamma=segment.gamma, fidelity=fidelity),
                                       promotes_inputs=['Weight|takeoff', 'LiftRotor|*'],
                                       promotes_outputs=[('Power|DescentConstantVyConstantVx', f'Power|LiftRotor|segment_{segment.id}'),
                                                         ('LiftRotor|Descent|thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

                elif vehicle.configuration == 'LiftPlusCruise':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerDescentConstantVyConstantVxWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, Cd0=Cd0_propeller, hover_FM=hover_FM_propeller, rho_air=rho_air, mu_air=mu_air, g=g, AoA=AoA, gamma=segment.gamma, descent_airspeed=segment.speed, fidelity=fidelity),
                                       promotes_inputs=['Weight|takeoff', 'Wing|*', 'Propeller|*'],
                                       promotes_outputs=[('Power|DescentConstantVyConstantVx', f'Power|Propeller|segment_{segment.id}'),
                                                         ('Propeller|Descent|thrust', f'Propeller|thrust_each|segment_{segment.id}')])

            if segment.kind in ['NoCreditClimb', 'NoCreditDescent']:
                self.add_subsystem(f'segment_{segment.id}_power',
                                   om.ExecComp(['zero_power = 0.0', 'zero_thrust = 0.0'], zero_power={'units': 'W'}, zero_thrust={'units': 'N'}),
                                   promotes_outputs=[('zero_power', f'Power|LiftRotor|segment_{segment.id}'), ('zero_thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

            if segment.kind == 'CruiseConstantSpeed':
                cruise_segment_id = segment.id
                if vehicle.configuration == 'Multirotor':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerCruiseConstantSpeedEdgewise(vehicle=vehicle, N_rotor=N_lift_rotor, n_blade=n_blade_lift_rotor, Cd0=Cd0_lift_rotor, hover_FM=hover_FM_lift_rotor, rho_air=rho_air, mu_air=mu_air, g=g, fidelity=fidelity),
                                       promotes_inputs=['Weight|*', ('Mission|cruise_speed', f'Mission|segment_{segment.id}|speed'), 'LiftRotor|*'],
                                       promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|LiftRotor|segment_{segment.id}'), 'LiftRotor|Cruise|T_to_P', ('Power|profile_power', f'Power|segment_{segment.id}|profile_power'),
                                                         ('Power|induced_power', f'Power|segment_{segment.id}|induced_power'), ('Power|propulsive_power', f'Power|segment_{segment.id}|propulsive_power'),
                                                         ('LiftRotor|Cruise|thrust', f'LiftRotor|thrust_each|segment_{segment.id}'), 'LiftRotor|Cruise|mu', 'LiftRotor|Cruise|thrust_coefficient',
                                                         'Aero|Cruise|total_drag', 'Aero|Cruise|f_total'])

                elif vehicle.configuration == 'LiftPlusCruise':
                    self.add_subsystem(f'segment_{segment.id}_power',
                                       PowerCruiseConstantSpeedWithWing(vehicle=vehicle, N_propeller=N_propeller, n_blade=n_blade_propeller, rho_air=rho_air, mu_air=mu_air, v_sound=v_sound, Cd0=Cd0_propeller, hover_FM=hover_FM_propeller, g=g, AoA=AoA, fidelity=fidelity),
                                       promotes_inputs=['Weight|*', ('Mission|cruise_speed', f'Mission|segment_{segment.id}|speed'), 'Wing|*', 'Propeller|*'],
                                       promotes_outputs=[('Power|CruiseConstantSpeed', f'Power|Propeller|segment_{segment.id}'), 'Propeller|Cruise|T_to_P', ('Power|profile_power', f'Power|segment_{segment.id}|profile_power'),
                                                         ('Power|induced_power', f'Power|segment_{segment.id}|induced_power'), ('Power|propulsive_power', f'Power|segment_{segment.id}|propulsive_power'),
                                                         'Aero|Cruise|CL', 'Aero|Cruise|CD', 'Aero|Cruise|AoA', 'Propeller|Cruise|thrust_coefficient', 'Propeller|Cruise|J',
                                                         ('Propeller|Cruise|thrust', f'Propeller|thrust_each|segment_{segment.id}'),
                                                         'Aero|Cruise|total_drag', 'Aero|Cruise|f_total'])
            if segment.kind == 'ConstantPower':

                self.add_subsystem(f'segment_{segment.id}_power',
                                   PowerConstantFractionOfMaxPower(percent_max_power=segment.percent_max_power),
                                   promotes_inputs=[('max_power', 'Power|LiftRotor|maximum')],
                                   promotes_outputs=[('fractional_power', f'Power|LiftRotor|segment_{segment.id}'), ('zero_thrust', f'LiftRotor|thrust_each|segment_{segment.id}')])

            if segment.kind == 'ReserveCruise':
                if vehicle.configuration == 'Multirotor':
                    component = 'LiftRotor'
                elif vehicle.configuration == 'LiftPlusCruise':
                    component = 'Propeller'

                self.add_subsystem('reserve_segment_power',
                                   PowerConstantFractionOfMaxPower(percent_max_power=100.0),
                                   promotes_inputs=[('max_power', f'Power|{component}|segment_{cruise_segment_id}')],
                                   promotes_outputs=[('fractional_power', 'Power|reserve_segment')])

        # ------------------------------------------ #
        # ---- Writing maximum thrust equations ---- #
        # ------------------------------------------ #
        # T_max = max(T_segment_1, ..., T_segment_n)

        if vehicle.configuration == 'Multirotor':
            max_thrust_eq = 'T_max = '
            kwargs_T = {'T_max': {'units': 'N'}}
            for ctr, i in enumerate(ids_for_max_p):
                if ctr == len(ids_for_max_p) - 1:
                    max_thrust_eq += f'T_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                else:
                    max_thrust_eq += f'maximum(T_segment_{i}, '
                kwargs_T[f'T_segment_{i}'] = {'units': 'N'}

        elif vehicle.configuration == 'LiftPlusCruise':
            max_thrust_liftrotor_eq = 'T1_max = '
            max_thrust_propeller_eq = 'T2_max = '
            kwargs_T1 = {'T1_max': {'units': 'N'}}
            kwargs_T2 = {'T2_max': {'units': 'N'}}
            for ctr, i in enumerate(ids_for_max_p):
                if ctr == len(ids_for_max_p) - 1:
                    max_thrust_liftrotor_eq += f'T1_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                    max_thrust_propeller_eq += f'T2_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                else:
                    max_thrust_liftrotor_eq += f'maximum(T1_segment_{i}, '
                    max_thrust_propeller_eq += f'maximum(T2_segment_{i}, '
                kwargs_T1[f'T1_segment_{i}'] = {'units': 'N'}
                kwargs_T2[f'T2_segment_{i}'] = {'units': 'N'}

        # ----------------------------------------- #
        # ---- Writing maximum power equations ---- #
        # ----------------------------------------- #
        # p_max = max(p_segment_1, ..., p_segment_n)

        if vehicle.configuration == 'Multirotor':
            max_power_eq = 'p_max = '
            kwargs_p = {'p_max': {'units': 'W'}}
            for ctr, i in enumerate(ids_for_max_p):
                if ctr == len(ids_for_max_p) - 1:
                    max_power_eq += f'p_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                else:
                    max_power_eq += f'maximum(p_segment_{i}, '
                kwargs_p[f'p_segment_{i}'] = {'units': 'W'}

        elif vehicle.configuration == 'LiftPlusCruise':
            max_power_liftrotor_eq = 'p1_max = '
            max_power_propeller_eq = 'p2_max = '
            kwargs_p1 = {'p1_max': {'units': 'W'}}
            kwargs_p2 = {'p2_max': {'units': 'W'}}
            for ctr, i in enumerate(ids_for_max_p):
                if ctr == len(ids_for_max_p) - 1:
                    max_power_liftrotor_eq += f'p1_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                    max_power_propeller_eq += f'p2_segment_{i}' + (len(ids_for_max_p) - 1) * ')'
                else:
                    max_power_liftrotor_eq += f'maximum(p1_segment_{i}, '
                    max_power_propeller_eq += f'maximum(p2_segment_{i}, '
                kwargs_p1[f'p1_segment_{i}'] = {'units': 'W'}
                kwargs_p2[f'p2_segment_{i}'] = {'units': 'W'}

        # ----------------------------------------------------------------------------- #
        # --- Calculate maximum thrust requirement per component for sizing purpose --- #
        # ----------------------------------------------------------------------------- #
        if vehicle.configuration == 'Multirotor':
            max_thrust_comp = om.ExecComp(max_thrust_eq, **kwargs_T)
            self.add_subsystem('max_thrust_req', max_thrust_comp,
                               promotes_outputs=[('T_max', 'LiftRotor|thrust_each|maximum')])

            for ctr, i in enumerate(ids_for_max_p):
                self.connect(f'LiftRotor|thrust_each|segment_{i}', f'max_thrust_req.T_segment_{i}')

        elif vehicle.configuration == 'LiftPlusCruise':
            max_thrust_liftrotor_comp = om.ExecComp(max_thrust_liftrotor_eq, **kwargs_T1)
            max_thrust_propeller_comp = om.ExecComp(max_thrust_propeller_eq, **kwargs_T2)
            self.add_subsystem('max_thrust_liftrotor_req', max_thrust_liftrotor_comp,
                               promotes_outputs=[('T1_max', 'LiftRotor|thrust_each|maximum')])
            self.add_subsystem('max_thrust_propeller_req', max_thrust_propeller_comp,
                               promotes_outputs=[('T2_max', 'Propeller|thrust_each|maximum')])

            for ctr, i in enumerate(ids_for_max_p):
                self.connect(f'LiftRotor|thrust_each|segment_{i}', f'max_thrust_liftrotor_req.T1_segment_{i}')
                self.connect(f'Propeller|thrust_each|segment_{i}', f'max_thrust_propeller_req.T2_segment_{i}')

        # ---------------------------------------------------------------------------- #
        # --- Calculate maximum power requirement per component for sizing purpose --- #
        # ---------------------------------------------------------------------------- #
        if vehicle.configuration == 'Multirotor':
            max_power_comp = om.ExecComp(max_power_eq, **kwargs_p)
            self.add_subsystem('max_power_req', max_power_comp,
                               promotes_outputs=[('p_max', 'Power|LiftRotor|maximum')])

            for ctr, i in enumerate(ids_for_max_p):
                self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_req.p_segment_{i}')

        elif vehicle.configuration == 'LiftPlusCruise':
            max_power_liftrotor_comp = om.ExecComp(max_power_liftrotor_eq, **kwargs_p1)
            max_power_propeller_comp = om.ExecComp(max_power_propeller_eq, **kwargs_p2)
            self.add_subsystem('max_power_liftrotor_req', max_power_liftrotor_comp,
                               promotes_outputs=[('p1_max', 'Power|LiftRotor|maximum')])
            self.add_subsystem('max_power_propeller_req', max_power_propeller_comp,
                               promotes_outputs=[('p2_max', 'Power|Propeller|maximum')])

            for ctr, i in enumerate(ids_for_max_p):
                self.connect(f'Power|LiftRotor|segment_{i}', f'max_power_liftrotor_req.p1_segment_{i}')
                self.connect(f'Power|Propeller|segment_{i}', f'max_power_propeller_req.p2_segment_{i}')

        # ------------------------------------------------------#
        # --- Calculate total power requirement per segment --- #
        # ------------------------------------------------------#
        if vehicle.configuration == 'Multirotor':
            for i in range(1, mission.n_segments + 1):
                kwargs = {f'power_segment_{i}': {'units': 'W'}, f'p_{i}': {'units': 'W'}}
                segment_power_comp = om.ExecComp(f'power_segment_{i} = p_{i}', **kwargs)
                self.add_subsystem(f'power_segment_req_{i}', segment_power_comp,
                                   promotes_outputs=[(f'power_segment_{i}', f'Power|segment_{i}')])
                self.connect(f'Power|LiftRotor|segment_{i}', f'power_segment_req_{i}.p_{i}')

        elif vehicle.configuration == 'LiftPlusCruise':
            for i in range(1, mission.n_segments + 1):
                adder = om.AddSubtractComp()
                adder.add_equation(f'Power|segment_{i}',
                                   input_names=[f'Power|LiftRotor|segment_{i}', f'Power|Propeller|segment_{i}'],
                                   units='W',
                                   scaling_factors=[1., 1.])
                self.add_subsystem(f'p_total_segment_{i}',
                                   adder,
                                   promotes_inputs=[f'Power|LiftRotor|segment_{i}', f'Power|Propeller|segment_{i}'],
                                   promotes_outputs=[f'Power|segment_{i}'])

        # -----------------------------------------------------#
        # --- Calculate disk loading per rotor per segment --- #
        # -----------------------------------------------------#
        if vehicle.configuration == 'Multirotor':
            for i in range(1, mission.n_segments + 1):
                DL_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
                                      disk_loading={'units': 'N/m**2'},
                                      thrust={'units': 'N'},
                                      r={'units': 'm'})
                self.add_subsystem(f'DL_segment_{i}', DL_comp,
                                   promotes_inputs=[('r', 'LiftRotor|radius')],
                                   promotes_outputs=[('disk_loading', f'DiskLoading|LiftRotor|segment_{i}')])
                self.connect(f'LiftRotor|thrust_each|segment_{i}', f'DL_segment_{i}.thrust')

        elif vehicle.configuration == 'LiftPlusCruise':
            for i in range(1, mission.n_segments + 1):
                DL_1_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
                                        disk_loading={'units': 'N/m**2'},
                                        thrust={'units': 'N'},
                                        r={'units': 'm'})
                DL_2_comp = om.ExecComp('disk_loading = thrust / (pi * r**2)',
                                        disk_loading={'units': 'N/m**2'},
                                        thrust={'units': 'N'},
                                        r={'units': 'm'})
                self.add_subsystem(f'DL_1_segment_{i}', DL_1_comp,
                                   promotes_inputs=[('r', 'LiftRotor|radius')],
                                   promotes_outputs=[('disk_loading', f'DiskLoading|LiftRotor|segment_{i}')])
                self.connect(f'LiftRotor|thrust_each|segment_{i}', f'DL_1_segment_{i}.thrust')
                self.add_subsystem(f'DL_2_segment_{i}', DL_2_comp,
                                   promotes_inputs=[('r', 'Propeller|radius')],
                                   promotes_outputs=[('disk_loading', f'DiskLoading|Propeller|segment_{i}')])
                self.connect(f'Propeller|thrust_each|segment_{i}', f'DL_2_segment_{i}.thrust')

        # -----------------------------------------------------#
        # --- Add nonlinear solvers for implicit equations --- #
        # -----------------------------------------------------#

        self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=100, iprint=0, rtol=1e-3)
        self.nonlinear_solver.options['err_on_non_converge'] = False
        self.nonlinear_solver.options['reraise_child_analysiserror'] = False
        self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
        self.nonlinear_solver.linesearch.options['maxiter'] = 10
        self.nonlinear_solver.linesearch.options['iprint'] = 0
        self.linear_solver = om.DirectSolver(assemble_jac=True, rhs_checking=rhs_checking)
