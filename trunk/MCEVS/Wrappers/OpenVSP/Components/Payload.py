import openvsp as vsp
import numpy as np


def Human(N_PAX: int, config: str):

    # Seating according to N_PAX
    if N_PAX == 1:
        if config == 'Multirotor':
            x_pos = np.array([1.30])
            y_pos = np.array([0.00])
            z_pos = np.array([-0.15])
            y_rot = np.array([20.0])
            z_rot = np.array([0.00])
    elif N_PAX == 4:
        if config == 'Multirotor':
            x_pos = np.array([1.90, 1.90, 3.50, 3.50])
            y_pos = np.array([0.40, -0.40, 0.40, -0.40])
            z_pos = np.array([-0.35, -0.35, -0.45, -0.45])
            y_rot = np.array([20.0, 20.0, 20.0, 20.0])
            z_rot = np.array([0.00, 0.00, 0.00, 0.00])
        elif config == 'LiftPlusCruise':
            x_pos = np.array([2.10, 2.10, 3.70, 3.70])
            y_pos = np.array([0.40, -0.40, 0.40, -0.40])
            z_pos = np.array([0.20, 0.20, 0.20, 0.20])
            y_rot = np.array([20.0, 20.0, 20.0, 20.0])
            z_rot = np.array([0.00, 0.00, 0.00, 0.00])
    elif N_PAX == 6:
        if config == 'Multirotor':
            x_pos = np.array([2.20, 2.20, 3.30, 3.30, 4.40, 4.40])
            y_pos = np.array([0.40, -0.40, 0.40, -0.40, 0.40, -0.40])
            z_pos = np.array([-0.45, -0.45, -0.45, -0.45, -0.45, -0.45])
            y_rot = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 20.0])
            z_rot = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
        elif config == 'LiftPlusCruise':
            x_pos = np.array([1.90, 1.90, 3.00, 3.00, 4.10, 4.10])
            y_pos = np.array([0.40, -0.40, 0.40, -0.40, 0.40, -0.40])
            z_pos = np.array([0.20, 0.20, 0.20, 0.20, 0.20, 0.20])
            y_rot = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 20.0])
            z_rot = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])

    human_ids = []
    for i in range(N_PAX):
        human_ids.append(vsp.AddGeom('HUMAN'))
        vsp.SetParmVal(human_ids[i], 'X_Rel_Location', 'XForm', x_pos[i])
        vsp.SetParmVal(human_ids[i], 'Y_Rel_Location', 'XForm', y_pos[i])
        vsp.SetParmVal(human_ids[i], 'Z_Rel_Location', 'XForm', z_pos[i])
        # vsp.SetParmVal(human_id, 'Stature', 'Anthropometric', 1.756)						# each person's height is 1.756 m -> a little bit buggy
        vsp.SetParmVal(human_ids[i], 'Mass', 'Anthropometric', 75.0)  # each person weighs 75 kg (165.347 lbm)
        vsp.SetParmVal(human_ids[i], 'LenUnit', 'Anthropometric', 2)  # in [m]
        vsp.SetParmVal(human_ids[i], 'MassUnit', 'Anthropometric', 1)  # in [kg]
        vsp.SetParmVal(human_ids[i], 'PresetPose', 'Pose', 1)  # pose: sitting
        vsp.SetParmVal(human_ids[i], 'ShoulderIERt', 'Pose', 45.0)
        vsp.SetParmVal(human_ids[i], 'ElbowRt', 'Pose', 80.0)
        vsp.SetParmVal(human_ids[i], 'HipFERt', 'Pose', 80.0)
        vsp.SetParmVal(human_ids[i], 'KneeRt', 'Pose', 80.0)
        vsp.SetParmVal(human_ids[i], 'Y_Rel_Rotation', 'XForm', y_rot[i])  # inclined seating
        vsp.SetParmVal(human_ids[i], 'Z_Rel_Rotation', 'XForm', z_rot[i])  # in which direction they sit

    return human_ids
