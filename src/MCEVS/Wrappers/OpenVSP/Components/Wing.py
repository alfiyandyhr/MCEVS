import MCEVS
import openvsp as vsp
import numpy as np


def NASA_LPC_Wing(area=210.27814, aspect_ratio=12.12761, l_fuse=30.0, fuse_id=None):

    # Baseline params

    Span = [14.745022, 6.626930, 2.083091, 1.809956]
    Root_Chord = [5.4432819, 4.130830, 3.304664, 2.840884]
    Tip_Chord = [4.1308305, 3.304664, 2.840884, 1.043961]
    Sweep = [0.0000000, 0.000000, 3.636364, 8.000000]
    Sweep_Location = [0.7000000, 0.700000, 0.700000, 0.700000]
    Sec_SW_Location = [1.0000000, 1.000000, 1.000000, 1.000000]
    Twist = [-0.5000000, -0.500000, -0.500000, -0.500000]
    Twist_Location = [0.7000000, 0.700000, 0.700000, 0.700000]
    Dihedral = [-2.0000000, -2.000000, -2.000000, -2.000000]

    SectTess_U = [2.0000000, 2.000000, 3.000000, 4.000000]

    InLEMode = [0.0000000, 0.000000, 1.000000, 1.000000]
    InLEStrength = [1.0000000, 1.002239, 1.000000, 1.000000]
    InLESweep = [3.6165960, 6.340037, 16.72000, 70.51000]
    InLEDihedral = [2.1284308, 2.135989, 2.695763, 3.264543]

    OutLEMode = [1.0000000, 1.000000, 1.000000, 0.000000]
    OutLEStrength = [1.0000000, 1.000000, 0.793800, 2.181159]
    OutLESweep = [3.7200000, 5.160000, 16.72336, 69.38534]
    OutLEDihedral = [-2.2667261, -2.583862, -3.114677, -3.157360]

    # Calculating other params
    S = area
    AR = aspect_ratio
    b = np.sqrt(S * AR)  # wing span
    Span_i = []
    Root_Chord_i = []
    Tip_Chord_i = []

    for i in range(len(Span)):

        # Adjusting span
        span_i = Span[i] / 25.26 * b / 2
        Span_i.append(span_i)

        # Adjusting root and tip chords
        area_i = (Root_Chord[i] + Tip_Chord[i]) / 2 * Span[i]
        S_i = area_i / 210.27814 * S
        tr_i = Tip_Chord[i] / Root_Chord[i]
        rc_i = 2.0 * S_i / span_i / (1 + tr_i)
        tc_i = tr_i * rc_i
        Root_Chord_i.append(rc_i)
        Tip_Chord_i.append(tc_i)

    wing_id = vsp.AddGeom('WING', fuse_id)
    vsp.InsertXSec(wing_id, 1, vsp.XS_FILE_AIRFOIL)
    vsp.InsertXSec(wing_id, 1, vsp.XS_FILE_AIRFOIL)
    vsp.InsertXSec(wing_id, 1, vsp.XS_FILE_AIRFOIL)

    vsp.SetParmVal(wing_id, 'X_Rel_Location', 'XForm', 0.0)
    vsp.SetParmVal(wing_id, 'Y_Rel_Location', 'XForm', 0.0)
    vsp.SetParmVal(wing_id, 'Z_Rel_Location', 'XForm', 0.0)
    vsp.SetParmVal(wing_id, 'Sym_Planar_Flag', 'Sym', 2)
    vsp.SetParmVal(wing_id, 'Trans_Attach_Flag', 'Attach', 2.0)
    vsp.SetParmVal(wing_id, 'Rots_Attach_Flag', 'Attach', 1.0)
    vsp.SetParmVal(wing_id, 'U_Attach_Location', 'Attach', 0.39)
    vsp.SetParmVal(wing_id, 'V_Attach_Location', 'Attach', 0.75)
    vsp.SetParmVal(wing_id, 'Tess_W', 'Shape', 13)
    vsp.SetParmVal(wing_id, 'LECluster', 'WingGeom', 1.0)
    vsp.SetParmVal(wing_id, 'TECluster', 'WingGeom', 1.0)

    for i in range(1, 5):
        vsp.SetParmValUpdate(wing_id, 'Span', f'XSec_{i}', Span_i[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Root_Chord', f'XSec_{i}', Root_Chord_i[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Tip_Chord', f'XSec_{i}', Tip_Chord_i[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Sweep', f'XSec_{i}', Sweep[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Sweep_Location', f'XSec_{i}', Sweep_Location[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Sec_Sweep_Location', f'XSec_{i}', Sec_SW_Location[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Twist', f'XSec_{i}', Twist[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Twist_Location', f'XSec_{i}', Twist_Location[i - 1])
        vsp.SetParmValUpdate(wing_id, 'Dihedral', f'XSec_{i}', Dihedral[i - 1])

        vsp.SetParmValUpdate(wing_id, 'SectTess_U', f'XSec_{i}', SectTess_U[i - 1])

        vsp.SetParmValUpdate(wing_id, 'InLEMode', f'XSec_{i}', InLEMode[i - 1])
        vsp.SetParmValUpdate(wing_id, 'InLEStrength', f'XSec_{i}', InLEStrength[i - 1])
        vsp.SetParmValUpdate(wing_id, 'InLESweep', f'XSec_{i}', InLESweep[i - 1])
        vsp.SetParmValUpdate(wing_id, 'InLEDihedral', f'XSec_{i}', InLEDihedral[i - 1])

        vsp.SetParmValUpdate(wing_id, 'OutLEMode', f'XSec_{i}', OutLEMode[i - 1])
        vsp.SetParmValUpdate(wing_id, 'OutLEStrength', f'XSec_{i}', OutLEStrength[i - 1])
        vsp.SetParmValUpdate(wing_id, 'OutLESweep', f'XSec_{i}', OutLESweep[i - 1])
        vsp.SetParmValUpdate(wing_id, 'OutLEDihedral', f'XSec_{i}', OutLEDihedral[i - 1])

    # Change airfoil shape using NASA/LANGLEY LS(1)-0417 (GA(W)-1) AIRFOIL
    airfoil_dir = MCEVS.__file__[:-11] + 'Wrappers/OpenVSP/Components/Airfoils/LS417.dat'
    wing_surf = vsp.GetXSecSurf(wing_id, 0)
    xsec_num = vsp.GetNumXSec(wing_surf)
    for i in range(xsec_num):
        vsp.ChangeXSecShape(wing_surf, i, vsp.XS_FILE_AIRFOIL)
        xsec = vsp.GetXSec(wing_surf, i)
        vsp.ReadFileAirfoil(xsec, airfoil_dir)

    # wing_xsec_surf = vsp.GetXSecSurf(wing_id, 0)
    # for i in range(2):
    # 	wing_xsec = vsp.GetXSec(wing_xsec_surf, i)
    # 	vsp.SetParmVal(vsp.GetXSecParm(wing_xsec, 'ThickChord'), 0.12)
    # 	vsp.SetParmVal(vsp.GetXSecParm(wing_xsec, 'Camber'), 0.02)
    # 	vsp.SetParmVal(vsp.GetXSecParm(wing_xsec, 'CamberLoc'), 0.4)

    return wing_id
