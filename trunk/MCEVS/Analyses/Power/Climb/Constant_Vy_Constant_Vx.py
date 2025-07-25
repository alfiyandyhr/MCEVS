import openmdao.api as om
from MCEVS.Analyses.Stability.Trim import WingedConstantClimbTrimOfLift, WingedConstantClimbTrimOfThrust
from MCEVS.Analyses.Stability.Trim import MultirotorConstantClimbTrim
from MCEVS.Analyses.Aerodynamics.Parasite import ParasiteDragViaComponentBuildUpApproach, BacchiniExperimentalFixedValueForLPC
from MCEVS.Analyses.Aerodynamics.Empirical import WingedParasiteDragViaWeightBasedRegression, MultirotorParasiteDragViaWeightBasedRegression
from MCEVS.Analyses.Aerodynamics.Parabolic import WingedAeroDragViaParabolicDragPolar
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor, ThrustCoefficient, RotorAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import RotorInflow, InducedVelocity
from MCEVS.Analyses.Power.Rotor import RotorProfilePower, InducedPowerFactor, PowerForwardComp


class PowerClimbConstantVyConstantVxEdgewise(om.Group):
    """
    Computes the power required in edgewise climb phase
    Parameters:
            N_rotor			: number or rotors
            n_blade 		: number of blades per rotor
            Cd0 			: rotor's parasite drag coefficient
            hover_FM		: hover figure of merit
            rho_air			: air density [kg/m**3]
            g 				: gravitational acceleration [m/s**2]
            climb_airspeed	: climb air speed [m/s]
            gamma			: flight path angle during climb/descent
    Inputs:
            Weight|takeoff 			: total take-off weight [kg]
            Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
            Rotor|radius			: rotor radius [m]
            Rotor|chord 			: rotor's chord length [m]
            Rotor|mu 	 			: rotor's advance ratio
            Rotor|alpha				: rotor tilt angle [rad]
    Outputs:
            Power|ClimbConstantVyConstantVx 	: required power for climb [W]
            Rotor|thrust						: thrust of a rotor [N]
    """

    def initialize(self):
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('N_rotor', types=int, desc='Number of lifting rotors')
        self.options.declare('n_blade', types=int, desc='Number of blades per rotor')
        self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
        self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
        self.options.declare('rho_air', types=float, desc='Air density')
        self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
        self.options.declare('g', types=float, desc='Gravitational acceleration')
        self.options.declare('climb_airspeed', desc='Climb air speed')
        self.options.declare('gamma', desc='Flight path angle during climb/descent')
        self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

    def setup(self):
        vehicle = self.options['vehicle']
        N_rotor = self.options['N_rotor']
        n_blade = self.options['n_blade']
        Cd0 = self.options['Cd0']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        mu_air = self.options['mu_air']
        g = self.options['g']
        gamma = self.options['gamma']
        climb_airspeed = self.options['climb_airspeed']
        fidelity = self.options['fidelity']

        # Step 1: Calculate the drag for the multirotor in climb
        indep = self.add_subsystem('climb', om.IndepVarComp())
        indep.add_output('climb_airspeed', val=climb_airspeed, units='m/s')
        indep.add_output('gamma', val=gamma, units='rad')

        if fidelity['aerodynamics']['parasite'] == 'WeightBasedRegression':
            self.add_subsystem('parasite_drag',
                               MultirotorParasiteDragViaWeightBasedRegression(N_rotor=N_rotor, rho_air=rho_air),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'climb.climb_airspeed'), ('Rotor|radius', 'LiftRotor|radius')],
                               promotes_outputs=[('Aero|total_drag', 'Aero|Climb|total_drag'), ('Aero|Cd0', 'Aero|Climb|Cd0')])
        elif fidelity['aerodynamics']['parasite'] == 'ComponentBuildUp':
            self.add_subsystem('parasite_drag',
                               ParasiteDragViaComponentBuildUpApproach(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='climb'),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'climb.climb_airspeed'), ('Rotor|radius', 'LiftRotor|radius')],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Climb|Cd0'), ('Aero|parasite_drag', 'Aero|Climb|total_drag')])

        # Step 2: Calculate thrust required for trim and the body tilt angle
        self.add_subsystem('trim',
                           MultirotorConstantClimbTrim(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff', ('Aero|total_drag', 'Aero|Climb|total_drag')],
                           promotes_outputs=[('Thrust', 'Thrust_all_climb'), ('Body|cos_delta', 'Body|Climb|cos_delta')])

        # Step 3: Convert Body|cos_delta into Rotor|alpha
        self.add_subsystem('delta2alpha',
                           om.ExecComp('alpha = pi/2 + gamma - arccos(cos_delta)', alpha={'units': 'rad'}, gamma={'units': 'rad'}),
                           promotes_inputs=[('cos_delta', 'Body|Climb|cos_delta'), ('gamma', 'climb.gamma')],
                           promotes_outputs=[('alpha', 'LiftRotor|Climb|alpha')])

        # Step 4: Calculate the thrust required by each rotor
        self.add_subsystem('thrust_each',
                           ThrustOfEachRotor(N_rotor=N_rotor),
                           promotes_inputs=[('Thrust_all', 'Thrust_all_climb')],
                           promotes_outputs=[('Rotor|thrust', 'LiftRotor|Climb|thrust')])

        # Step 5: Calculate rotor omega and advance ratio from RPM
        self.add_subsystem('rpm2omega',
                           om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units': 'rad/s'}, rpm={'units': 'rpm'}),
                           promotes_inputs=[('rpm', 'LiftRotor|Climb|RPM')],
                           promotes_outputs=[('omega', 'LiftRotor|Climb|omega')])

        self.add_subsystem('mu',
                           RotorAdvanceRatio(),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|alpha', 'LiftRotor|Climb|alpha'),
                                            ('Rotor|omega', 'LiftRotor|Climb|omega'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('Rotor|mu', 'LiftRotor|Climb|mu')])

        # Step 6: Calculate the thrust coefficient Ct
        self.add_subsystem('Ct',
                           ThrustCoefficient(rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Climb|thrust'),
                                            ('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|omega', 'LiftRotor|Climb|omega')],
                           promotes_outputs=[('Rotor|thrust_coefficient', 'LiftRotor|Climb|thrust_coefficient')])

        # Step 7: Calculate profile power
        self.add_subsystem('profile_power',
                           RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|chord', 'LiftRotor|chord'),
                                            ('Rotor|mu', 'LiftRotor|Climb|mu'),
                                            ('Rotor|omega', 'LiftRotor|Climb|omega')],
                           promotes_outputs=[('Rotor|profile_power', 'LiftRotor|Climb|profile_power')])

        # Step 8: Calculate induced power
        self.add_subsystem('rotor_inflow',
                           RotorInflow(),
                           promotes_inputs=[('Rotor|mu', 'LiftRotor|Climb|mu'),
                                            ('Rotor|alpha', 'LiftRotor|Climb|alpha'),
                                            ('Rotor|thrust_coefficient', 'LiftRotor|Climb|thrust_coefficient'),],
                           promotes_outputs=[('Rotor|lambda', 'LiftRotor|Climb|lambda')])
        self.add_subsystem('v_induced',
                           InducedVelocity(),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|alpha', 'LiftRotor|Climb|alpha'),
                                            ('Rotor|omega', 'LiftRotor|Climb|omega'),
                                            ('Rotor|lambda', 'LiftRotor|Climb|lambda'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('v_induced', 'LiftRotor|Climb|v_induced')])
        self.add_subsystem('kappa',
                           InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Climb|thrust'),
                                            ('Rotor|profile_power', 'LiftRotor|Climb|profile_power'),
                                            ('Rotor|radius', 'LiftRotor|radius'),],
                           promotes_outputs=[('Rotor|kappa', 'LiftRotor|Climb|kappa')])

        # Step 9: Calculate total power required
        self.add_subsystem('power_req',
                           PowerForwardComp(N_rotor=N_rotor, g=g),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Climb|thrust'),
                                            ('Rotor|profile_power', 'LiftRotor|Climb|profile_power'),
                                            ('Rotor|alpha', 'LiftRotor|Climb|alpha'),
                                            ('Rotor|kappa', 'LiftRotor|Climb|kappa'),
                                            ('v_induced', 'LiftRotor|Climb|v_induced'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('Power|forward', 'Power|ClimbConstantVyConstantVx'), ('Rotor|T_to_P', 'LiftRotor|Climb|T_to_P')])


class PowerClimbConstantVyConstantVxWithWing(om.Group):
    """
    Computes the power required in winged climb phase
    Parameters:
            N_propeller			: number of propellers
            n_blade 			: number of blades per propeller
            rho_air				: air density [kg/m**3]
            Cd0 				: rotor's parasite drag coefficient
            hover_FM			: hover figure of merit
            g 					: gravitational acceleration [m/s**2]
            AoA 				: aircraft's angle of attack [deg]
            climb_airspeed		: climb air speed [m/s]
            gamma				: flight path angle during climb/descent
    Inputs:
            Weight|takeoff 		: total take-off weight [kg]
            eVTOL|S_wing 		: wing area [m**2]
            eVTOL|AR_wing		: wing aspect ratio
            Rotor|radius		: rotor radius [m]
            Rotor|J				: propeller's advance ratio
            Rotor|alpha 		: rotor tilt angle [rad]
    Outputs:
            Power|CruiseConstantSpeed 	: required power for cruise [W]
            Rotor|thrust				: thrust of a rotor [N]
    """

    def initialize(self):
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('N_propeller', types=int, desc='Number of propellers')
        self.options.declare('n_blade', types=int, desc='Number of blades per propeller')
        self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
        self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
        self.options.declare('rho_air', types=float, desc='Air density')
        self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
        self.options.declare('g', types=float, desc='Gravitational acceleration')
        self.options.declare('AoA', desc='Aircraft angle of attack')
        self.options.declare('climb_airspeed', desc='Climb air speed')
        self.options.declare('gamma', desc='Flight path angle during climb/descent')
        self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

    def setup(self):
        vehicle = self.options['vehicle']
        N_propeller = self.options['N_propeller']
        n_blade = self.options['n_blade']
        Cd0 = self.options['Cd0']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        mu_air = self.options['mu_air']
        g = self.options['g']
        AoA = self.options['AoA']
        gamma = self.options['gamma']
        climb_airspeed = self.options['climb_airspeed']
        fidelity = self.options['fidelity']

        # Step 1: Trim analysis
        self.add_subsystem('trim_lift',
                           WingedConstantClimbTrimOfLift(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=[('Aero|lift', 'Aero|Climb|lift')])

        # Step 2: Calculate drag in climb using simple polar equations
        indep = self.add_subsystem('climb', om.IndepVarComp())
        indep.add_output('climb_airspeed', val=climb_airspeed, units='m/s')

        if fidelity['aerodynamics']['parasite'] == 'WeightBasedRegression':
            self.add_subsystem('parasite_drag',
                               WingedParasiteDragViaWeightBasedRegression(rho_air=rho_air),
                               promotes_inputs=['Weight|takeoff', 'Wing|area', ('Aero|speed', 'climb.climb_airspeed')],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Climb|Cd0'), ('Aero|parasite_drag', 'Aero|Climb|parasite_drag')])

        elif fidelity['aerodynamics']['parasite'] == 'ComponentBuildUp':
            self.add_subsystem('parasite_drag',
                               ParasiteDragViaComponentBuildUpApproach(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='climb'),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'climb.climb_airspeed'), 'Wing|area'],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Climb|Cd0'), ('Aero|parasite_drag', 'Aero|Climb|parasite_drag')])

        elif fidelity['aerodynamics']['parasite'] == 'BacchiniExperimentalFixedValueForLPC':
            self.add_subsystem('parasite_drag',
                               BacchiniExperimentalFixedValueForLPC(rho_air=rho_air),
                               promotes_inputs=[('Aero|speed', 'Mission|cruise_speed'), 'Wing|area'],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Cruise|Cd0'), ('Aero|parasite_drag', 'Aero|Cruise|parasite_drag')])

        self.add_subsystem('total_drag',
                           WingedAeroDragViaParabolicDragPolar(rho_air=rho_air),
                           promotes_inputs=[('Aero|Cd0', 'Aero|Climb|Cd0'), ('Aero|lift', 'Aero|Climb|lift'), ('Aero|speed', 'climb.climb_airspeed'), 'Wing|*'],
                           promotes_outputs=[('Aero|total_drag', 'Aero|Climb|total_drag'), ('Aero|CL', 'Aero|Climb|CL')])

        # Step 3: Calculate thrust required by each propeller after trimming
        self.add_subsystem('trim_thrust',
                           WingedConstantClimbTrimOfThrust(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff', ('Aero|total_drag', 'Aero|Climb|total_drag')],
                           promotes_outputs=['Thrust_all'])
        self.add_subsystem('thrust_each',
                           ThrustOfEachRotor(N_rotor=N_propeller),
                           promotes_inputs=['Thrust_all'],
                           promotes_outputs=[('Rotor|thrust', 'Propeller|Climb|thrust')])

        # Step 4: Calculate propeller omega from RPM
        self.add_subsystem('rpm2omega',
                           om.ExecComp('omega = rpm * 2*pi/60.0', omega={'units': 'rad/s'}, rpm={'units': 'rpm'}),
                           promotes_inputs=[('rpm', 'Propeller|Climb|RPM')],
                           promotes_outputs=[('omega', 'Propeller|Climb|omega')])

        # Step 5: Calculate rotor advance ratio mu and thrust coefficient Ct
        # Treating propeller as a rotor
        self.add_subsystem('mu',
                           RotorAdvanceRatio(),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|alpha', 'Propeller|Climb|alpha'),
                                            ('Rotor|omega', 'Propeller|Climb|omega'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('Rotor|mu', 'Propeller|Climb|mu')])
        self.add_subsystem('Ct',
                           ThrustCoefficient(rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Climb|thrust'),
                                            ('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|omega', 'Propeller|Climb|omega')],
                           promotes_outputs=[('Rotor|thrust_coefficient', 'Propeller|Climb|thrust_coefficient')])

        # Step 6: Calculate profile power of a rotor
        self.add_subsystem('profile_power',
                           RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|chord', 'Propeller|chord'),
                                            ('Rotor|mu', 'Propeller|Climb|mu'),
                                            ('Rotor|omega', 'Propeller|Climb|omega')],
                           promotes_outputs=[('Rotor|profile_power', 'Propeller|Climb|profile_power')])

        # Step 7: Calculate induced power
        self.add_subsystem('rotor_inflow',
                           RotorInflow(),
                           promotes_inputs=[('Rotor|mu', 'Propeller|Climb|mu'),
                                            ('Rotor|alpha', 'Propeller|Climb|alpha'),
                                            ('Rotor|thrust_coefficient', 'Propeller|Climb|thrust_coefficient')],
                           promotes_outputs=[('Rotor|lambda', 'Propeller|Climb|lambda')])

        # Assume the rotor tilt angle is 85, or AoA = 5
        self.set_input_defaults('Propeller|Climb|alpha', val=90.0 - AoA, units='deg')

        self.add_subsystem('v_induced',
                           InducedVelocity(),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|alpha', 'Propeller|Climb|alpha'),
                                            ('Rotor|omega', 'Propeller|Climb|omega'),
                                            ('Rotor|lambda', 'Propeller|Climb|lambda'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('v_induced', 'Propeller|Climb|v_induced')])
        self.add_subsystem('kappa',
                           InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Climb|thrust'),
                                            ('Rotor|profile_power', 'Propeller|Climb|profile_power'),
                                            ('Rotor|radius', 'Propeller|radius'),],
                           promotes_outputs=[('Rotor|kappa', 'Propeller|Climb|kappa')])

        # Step 8: Calculate total power required for winged forward flight
        self.add_subsystem('power_req',
                           PowerForwardComp(N_rotor=N_propeller, g=g),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Climb|thrust'),
                                            ('Rotor|profile_power', 'Propeller|Climb|profile_power'),
                                            ('Rotor|alpha', 'Propeller|Climb|alpha'),
                                            ('Rotor|kappa', 'Propeller|Climb|kappa'),
                                            ('v_induced', 'Propeller|Climb|v_induced'),
                                            ('v_inf', 'climb.climb_airspeed')],
                           promotes_outputs=[('Power|forward', 'Power|ClimbConstantVyConstantVx'), ('Rotor|T_to_P', 'Propeller|Climb|T_to_P')])
