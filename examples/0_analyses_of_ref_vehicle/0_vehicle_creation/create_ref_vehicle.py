from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Applications.OpenVSP.Standard_Vehicles import create_NASA_QuadRotor_vsp3, create_NASA_LiftPlusCruise_vsp3

# Design and operation variables for multirotor
design_var1 = {'r_lift_rotor': 4.20624} # 13.8 ft = 4.20624 m
operation_var1 = {'RPM_lift_rotor': {'hover_climb':None, 'cruise':450.0}}

# Design and operation variables for lift+cruise
design_var2 = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
operation_var2 = {'RPM_lift_rotor':{'hover_climb':None}, 'RPM_propeller': {'cruise':500.0}}

# Vehicle creation for multirotor
vehicle1 = StandardMultirotorEVTOL(design_var1, operation_var1, n_pax=6)

# Vehicle creation for lift+cruise
vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, operation_var2, n_pax=6)

create_NASA_QuadRotor_vsp3('multirotor.vsp3', vehicle1)
create_NASA_LiftPlusCruise_vsp3('liftpluscruise.vsp3', vehicle2)