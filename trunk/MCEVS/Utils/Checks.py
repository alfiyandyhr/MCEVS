def check_fidelity_dict(fidelity: object, vehicle_config: str):

    # Check for solver fidelity
    for drag_type, aero_model in fidelity['aerodynamics'].items():
        if drag_type == 'parasite':
            if vehicle_config == 'Multirotor':
                if aero_model not in ['WeightBasedRegression', 'ComponentBuildUp']:
                    raise ValueError('Parasite drag model should be in ["WeightBasedRegression", "ComponentBuildUp"]')
        elif drag_type == 'parasite':
            if vehicle_config == 'LiftPlusCruise':
                if aero_model not in ['WeightBasedRegression', 'ComponentBuildUp', 'BacchiniExperimentalFixedValueForLPC']:
                    raise ValueError('Parasite drag model should be in ["WeightBasedRegression", "ComponentBuildUp", "BacchiniExperimentalFixedValueForLPC"]')
        if drag_type == 'induced':
            if vehicle_config == 'LiftPlusCruise':
                if aero_model not in ['ParabolicDragPolar']:
                    raise ValueError('Induced drag model should be in ["ParabolicDragPolar"]')
    for segment, power_model in fidelity['power_model'].items():
        if segment == 'hover_climb':
            if power_model not in ['MomentumTheory', 'ModifiedMomentumTheory', 'BladeElementMomentumTheory']:
                raise ValueError('Power model should be in ["MomentumTheory", "ModifiedMomentumTheory", "BladeElementMomentumTheory"]')
    for component, weight_model in fidelity['weight_model'].items():
        if component == 'structure':
            if vehicle_config == 'Multirotor':
                if weight_model not in ['Roskam']:
                    raise ValueError('Weight model should be in ["Roskam"]')
            elif vehicle_config == 'LiftPlusCruise':
                if weight_model not in ['Roskam', 'M4Regression']:
                    raise ValueError('Weight model should be in ["Roskam", "M4Regression"]')
    for stability_type, segment_val in fidelity['stability'].items():
        if stability_type == 'AoA_trim':
            for segment, val in segment_val.items():
                if val not in ['ManualFixedValue', 'Automatic']:
                    raise ValueError('AoA_trim should be in ["ManualFixedValue", "Automatic"]')
