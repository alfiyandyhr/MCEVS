def check_fidelity_dict(fidelity: object, vehicle_config: str, modules_to_check=[]):
    """
    Validate the 'fidelity' dictionary according to model availability and coupling logic.

    Enforced logic summary:
    - Aerodynamics:
        - parasite model options depend on vehicle_config (unchanged from original)
        - induced model must be one of ["ParabolicDragPolar", "VortexLatticeMethod"]
    - Stability (AoA trim @ cruise):
        - Allowed values: ["ManualFixedValue", "Automatic"]
        - If induced == "VortexLatticeMethod": AoA_trim['cruise'] must be "Automatic"
          (this combination implies skipping the manual/trim subsystem path in implementation)
    - Power and weight model checks retained.

    Parameters
    ----------
    fidelity : dict
    vehicle_config : str
    modules_to_check : list[str]
    """

    # If modules_to_check not specified, infer from provided fidelity keys
    if not modules_to_check:
        modules_to_check = [k for k in fidelity.keys()
                            if k in ['aerodynamics', 'power_model', 'weight_model', 'stability']]

    # Aerodynamics checks
    if 'aerodynamics' in modules_to_check:
        aero = fidelity.get('aerodynamics', {})
        # Parasite drag model constraints (retain previous logic)
        if 'parasite' in aero:
            parasite_model = aero['parasite']
            if vehicle_config == 'Multirotor':
                if parasite_model not in ['WeightBasedRegression', 'ComponentBuildUp']:
                    raise ValueError('Parasite drag model should be in ["WeightBasedRegression", "ComponentBuildUp"] for Multirotor')
            elif vehicle_config == 'LiftPlusCruise':
                if parasite_model not in ['WeightBasedRegression', 'ComponentBuildUp', 'BacchiniExperimentalFixedValueForLPC']:
                    raise ValueError('Parasite drag model should be in ["WeightBasedRegression", "ComponentBuildUp", "BacchiniExperimentalFixedValueForLPC"] for LiftPlusCruise')

        # Induced drag model constraints
        induced_model = aero.get('induced', None)
        if induced_model is not None:
            if induced_model not in ['ParabolicDragPolar', 'VortexLatticeMethod']:
                raise ValueError('Induced drag model should be in ["ParabolicDragPolar", "VortexLatticeMethod"]')

    # Power model checks (unchanged)
    if 'power_model' in modules_to_check:
        pwr = fidelity.get('power_model', {})
        for segment, power_model in pwr.items():
            if segment == 'hover_climb':
                if power_model not in ['MomentumTheory', 'ModifiedMomentumTheory', 'BladeElementMomentumTheory']:
                    raise ValueError('Power model should be in ["MomentumTheory", "ModifiedMomentumTheory", "BladeElementMomentumTheory"]')

    # Weight model checks (retain previous logic)
    if 'weight_model' in modules_to_check:
        wgt = fidelity.get('weight_model', {})
        for component, weight_model in wgt.items():
            if component == 'structure':
                if vehicle_config == 'Multirotor':
                    if weight_model not in ['Roskam']:
                        raise ValueError('Weight model should be in ["Roskam"] for Multirotor')
                elif vehicle_config == 'LiftPlusCruise':
                    if weight_model not in ['Roskam', 'M4Regression']:
                        raise ValueError('Weight model should be in ["Roskam", "M4Regression"] for LiftPlusCruise')

    # Stability checks updated per new logic
    if 'stability' in modules_to_check:
        stab = fidelity.get('stability', {})
        aoa_trim = stab.get('AoA_trim', {})

        # Validate allowed values for any provided segments
        for segment, val in aoa_trim.items():
            if val not in ['ManualFixedValue', 'Automatic']:
                raise ValueError('AoA_trim should be in ["ManualFixedValue", "Automatic"]')

        # Coupling rule:
        # If induced == VortexLatticeMethod, then AoA_trim at cruise MUST be Automatic.
        induced_model = fidelity.get('aerodynamics', {}).get('induced', None)
        cruise_trim = aoa_trim.get('cruise', None)

        if induced_model == 'VortexLatticeMethod':
            if cruise_trim is None:
                raise ValueError('AoA_trim at "cruise" must be specified when using "VortexLatticeMethod". Set it to "Automatic".')
            if cruise_trim != 'Automatic':
                raise ValueError('When induced model is "VortexLatticeMethod", AoA_trim at cruise must be "Automatic" (manual fixed AoA is not allowed).')
        # If induced != VLM, both ManualFixedValue and Automatic are acceptable (already validated above).               
