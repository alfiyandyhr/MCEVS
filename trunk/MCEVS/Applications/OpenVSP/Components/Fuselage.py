import openvsp as vsp

def NASA_QR_Fuselage(l_fuse=21.0, d_fuse_max=6.745500):

	# Baseline params
	X  				= [  0.000000,  0.301450,  0.490545,  0.669739,  0.818715,  1.000000 ]
	Y  				= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	Z  				= [ -0.081739, -0.053347, -0.041389, -0.037689, -0.005047,  0.090324 ]
	Height  		= [  0.000000,  6.000000,  6.745500,  6.638240,  5.065584,  0.000000 ]
	Width   		= [  0.000000,  6.259090,  6.429731,  6.259091,  4.373392,  0.000000 ]
	MaxWidthLoc 	= [  0.000000,  0.000000,  0.172727,  0.400000,  0.672727,  0.000000 ]
	TopStr			= [  0.000000,  0.785604,  1.466508,  1.431410,  1.643959,  0.000000 ]
	UpStr			= [  0.000000,  1.347545,  0.955949,  0.738126,  0.164678,  0.000000 ]
	BotStr			= [  0.000000,  1.397455,  1.397455,  1.397455,  1.122952,  0.000000 ]
	LowStr			= [  0.000000,  1.322591,  1.322591,  1.322591,  0.911463,  0.000000 ]
	TopLAngle 		= [  5.869565,  15.65217,  3.913043,  0.000000, -3.913043, -17.60870 ]
	TopLSlew		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	TopLStrength 	= [  0.000000,  1.000000,  1.000000,  1.000000,  1.000000,  0.750000 ]
	TopLCurveSet 	= [  1.000000,  0.000000,  0.000000,  1.000000,  0.000000,  0.000000 ]
	TopLCurve 		= [  2.241795, -14.60444, -0.329035,  0.524525, -0.379533, -0.526023 ]
	RightLAngle 	= [  25.43478,  3.913043,  0.000000, -7.826087, -23.47826, -45.00000 ]
	RightLSlew 		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	RightLStrength 	= [  0.217391,  1.597353,  1.000000,  0.000000,  1.000000,  0.750000 ]
	RightLCurve 	= [  2.337958, -10.13734,  1.345234,  0.262028, -1.915935,  1.279334 ]
	BottomLAngle 	= [  52.82609,  0.000000,  0.000000,  0.000000, -35.21739, -91.95652 ]
	BottomLStrength = [  0.750000,  1.000000,  1.000000,  1.000000,  1.000000,  0.750000 ]
	BottomLSlew		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	BottomLCurve 	= [ -0.983689, -7.061080, -0.282552,  0.559674,  0.490045, -3.960283 ]

	# Create a fuselage
	fuselage_id 	= vsp.AddGeom('FUSELAGE')
	vsp.InsertXSec(fuselage_id, 1, vsp.XS_GENERAL_FUSE)
	fuselage_surf 	= vsp.GetXSecSurf(fuselage_id, 0)
	xsec_num		= vsp.GetNumXSec(fuselage_surf)
	vsp.SetParmVal(fuselage_id, 'Length', 'Design', l_fuse)

	# Change shape of XSecs, and set height + width
	for i in range(1, xsec_num-1):
		vsp.ChangeXSecShape(fuselage_surf, i, vsp.XS_GENERAL_FUSE)
		xsec = vsp.GetXSec(fuselage_surf, i)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'MaxWidthLoc'),		MaxWidthLoc[i]					)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopStr'), 			TopStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'UpStr'), 			UpStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BotStr'), 			BotStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'LowStr'), 			LowStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'Height'), 			Height[i]/6.745500*d_fuse_max	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'Width'), 			Width[i]/6.745500*d_fuse_max	)	

	# Set locations and skinning params
	for i in range(xsec_num):
		xsec = vsp.GetXSec(fuselage_surf, i)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'XLocPercent'),		X[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'YLocPercent'), 		Y[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'ZLocPercent'), 		Z[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TBSym'), 			0.0					)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLAngle'), 		TopLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLSlew'), 			TopLSlew[i]			)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLStrength'), 		TopLStrength[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLCurveSet'), 		TopLCurveSet[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLCurve'), 		TopLCurve[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLAngle'), 		RightLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLSlew'), 		RightLSlew[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLStrength'), 	RightLStrength[i]	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLCurve'), 		RightLCurve[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLAngle'), 		BottomLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLStrength'), 	BottomLStrength[i]	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLSlew'), 		BottomLSlew[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLCurve'), 		BottomLCurve[i]		)

	return fuselage_id

def NASA_LPC_Fuselage(l_fuse=30.0, d_fuse_max=6.150259):

	# Baseline params
	X  				= [0.000000,  0.024145,  0.121627,  0.211428,  0.418359,  0.479564,  0.614653,  0.923427,  1.000000]
	Y  				= [0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000]
	Z  				= [0.000000, -0.002082,  0.025612,  0.043950,  0.043950,  0.053155,  0.098281,  0.116377,  0.118292]
	Height  		= [0.000000,  2.140000,  5.000000,  6.127066,  6.127066,  5.621694,  2.692018,  1.187394,  0.000000]
	Width   		= [0.000000,  2.294849,  4.939956,  5.769831,  6.150259,  5.110106,  3.179449,  0.874706,  0.000000]
	MaxWidthLoc 	= [0.000000,  0.024812, -0.117303, -0.190324, -0.190324, -0.125562, -0.310309, -0.500607,  0.000000]
	TopStr			= [0.000000,  1.109250,  0.342421,  0.651397,  1.001460,  1.250000,  1.542616,  2.000000,  0.000000]
	UpStr			= [0.000000,  0.692274,  1.730201,  1.740909,  2.000000,  1.504718,  0.818164,  1.278041,  0.000000]
	BotStr			= [0.000000,  0.827589,  1.191510,  1.138295,  1.276590,  0.781013,  0.337871,  0.952476,  0.000000]
	LowStr			= [0.000000,  0.814290,  1.270000,  1.422861,  1.422861,  1.500000,  1.068302,  0.678190,  0.000000]
	TopLAngle 		= [80.19936,  43.88675,  32.82609,  5.846639,  5.846639,  0.000000, -3.701413, -3.288315, -90.00000]
	TopLSlew		= [0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000]
	TopLStrength 	= [0.864554,  1.471351,  0.750000,  1.342558,  1.342558,  0.927330,  0.930924,  1.000000,  0.412710]
	RightLAngle 	= [83.83450,  43.16453,  16.00000,  0.469499,  0.469499, -14.29721, -11.79425, -8.309167, -90.00000]
	RightLSlew 		= [0.000000,  0.000000, -15.65217, -3.913043,  0.000000, -19.56522,  0.000000,  0.000000,  0.000000]
	RightLStrength 	= [0.864554,  1.471351,  0.750000,  1.516699,  1.342558,  0.927330,  0.930924,  1.000000,  0.412710]
	BottomLAngle 	= [79.92145,  25.92945,  4.000000,  0.056444,  0.056444, -25.65048, -6.790423, -6.870941, -90.00000]
	BottomLStrength = [0.764942,  1.471351,  0.831583,  1.342558,  1.342558,  0.515420,  2.191863,  1.000000,  0.412710]
	BottomLSlew		= [0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000]

	# Create a fuselage
	fuselage_id 	= vsp.AddGeom('FUSELAGE')
	vsp.InsertXSec(fuselage_id, 1, vsp.XS_GENERAL_FUSE)
	vsp.InsertXSec(fuselage_id, 1, vsp.XS_GENERAL_FUSE)
	vsp.InsertXSec(fuselage_id, 1, vsp.XS_GENERAL_FUSE)
	vsp.InsertXSec(fuselage_id, 1, vsp.XS_GENERAL_FUSE)
	fuselage_surf 	= vsp.GetXSecSurf(fuselage_id, 0)
	xsec_num		= vsp.GetNumXSec(fuselage_surf)
	vsp.SetParmVal(fuselage_id, 'Length', 'Design', l_fuse)

	# Change shape of XSecs, and set height + width
	for i in range(1, xsec_num-1):
		vsp.ChangeXSecShape(fuselage_surf, i, vsp.XS_GENERAL_FUSE)
		xsec = vsp.GetXSec(fuselage_surf, i)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'MaxWidthLoc'),		MaxWidthLoc[i]					)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopStr'), 			TopStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'UpStr'), 			UpStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BotStr'), 			BotStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'LowStr'), 			LowStr[i]						)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'Height'), 			Height[i]/6.150259*d_fuse_max	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'Width'), 			Width[i]/6.150259*d_fuse_max	)

	# Set locations and skinning params
	for i in range(xsec_num):
		xsec = vsp.GetXSec(fuselage_surf, i)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'XLocPercent'),		X[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'YLocPercent'), 		Y[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'ZLocPercent'), 		Z[i]				)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TBSym'), 			0.0					)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLAngle'), 		TopLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLSlew'), 			TopLSlew[i]			)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLStrength'), 		TopLStrength[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLAngle'), 		RightLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLSlew'), 		RightLSlew[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLStrength'), 	RightLStrength[i]	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLAngle'), 		BottomLAngle[i]		)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLStrength'), 	BottomLStrength[i]	)
		vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLSlew'), 		BottomLSlew[i]		)








