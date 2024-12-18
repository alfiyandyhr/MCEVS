import numpy as np

def flat2lla(X_m, Y_m, Z_m, lat_ref_deg, lon_ref_deg, h_ref_m, psi0_deg, R_planet_m=6371000, f=1/298.257223563):
	"""
	Flath planet coordinate system to LLA (Latitude Longitude Altitude) coordinate system
	Inputs:
		X_m			: flat planet coordinate x-axis [m]
		Y_m			: flat planet coordinate y-axis [m]
		Z_m 		: flat planet coordinate z-axis [m] (negative towards the planet's center)
		lat_ref_deg	: latitude of reference location [deg] (flat planet origin)
		lon_ref_deg : longitude of reference location [deg] (flat planet origin)
		h_ref_m 	: height of reference location [m] (distance from the planet's surface to the flat planet origin)
		psi0_deg 	: angular direction/reference track angle [deg] (clockwise from north direction)
		R_planet_m 	: planetary equatorial radius [m] (default for earth = 6371000)
		f 			: flattening of the planet (default for earth = 1/298.257223563)
	Outputs:
		lat_deg 	: latitude [deg]
		lon_deg 	: longitude [deg]
		alt_m		: altitude [m]
	Source:
		https://ww2.mathworks.cn/help/aerotbx/ug/flat2lla.html
	"""
	# Reference lat lon psi
	lat_0 = lat_ref_deg * np.pi/180 	# reference latitude in rad
	lon_0 = lon_ref_deg * np.pi/180 	# reference longitude in rad
	psi_0 = psi0_deg * np.pi/180 		# angular direction in rad

	# North and east directions
	N = X_m * np.cos(psi_0) - Y_m * np.sin(psi_0)
	E = X_m * np.sin(psi_0) + Y_m * np.cos(psi_0)

	# Radius of curvature in prime vertical (R_N) and meridian (R_M)
	R_N = R_planet_m / np.sqrt(1 - (2*f-f**2)*np.sin(lat_0)**2)
	R_M = R_N * (1 - (2*f-f**2)) / (1 - (2*f-f**2)*np.sin(lat_0)**2)

	# Changes in lateral and longitudinal
	d_lat = N * np.arctan2(1,R_M)
	d_lon = E * np.arctan2(1,(R_N*np.cos(lat_0)))

	# Lateral and longitudinal in degrees, altitude in m
	lat_deg = (lat_0 + d_lat) * 180/np.pi
	lon_deg = (lon_0 + d_lon) * 180/np.pi
	alt_m 	= - Z_m - h_ref_m

	return lat_deg, lon_deg, alt_m

def lla2flat(lat_deg, lon_deg, alt_m, lat_ref_deg, lon_ref_deg, h_ref_m, psi0_deg, R_planet_m=6371000, f=1/298.257223563):
	"""
	LLA (Latitude Longitude Altitude) coordinate system to flat planet coordinate system
	Inputs:
		lat_deg 	: latitude [deg]
		lon_deg 	: longitude [deg]
		alt_m		: altitude [m]
		lat_ref_deg	: latitude of reference location [deg] (flat planet origin)
		lon_ref_deg : longitude of reference location [deg] (flat planet origin)
		h_ref_m 	: height of reference location [m] (distance from the planet's surface to the flat planet origin)
		psi0_deg 	: angular direction [deg] (clockwise from north direction)
		R_planet_m 	: planetary equatorial radius [m] (default for earth = 6371000)
		f 			: flattening of the planet (default for earth = 1/298.257223563)
	Outputs:
		X_m			: flat planet coordinate x-axis [m]
		Y_m			: flat planet coordinate y-axis [m]
		Z_m 		: flat planet coordinate z-axis [m] (negative towards the planet's center)
	Source:
		https://ww2.mathworks.cn/help/aerotbx/ug/lla2flat.html
	"""
	# Reference lat lon psi
	lat_0 = lat_ref_deg * np.pi/180 	# reference latitude in rad
	lon_0 = lon_ref_deg * np.pi/180 	# reference longitude in rad
	psi_0 = psi0_deg * np.pi/180 		# angular direction in rad

	# Changes in lateral and longitudinal in radians
	d_lat = (lat_deg - lat_ref_deg) * np.pi/180
	d_lon = (lon_deg - lon_ref_deg) * np.pi/180

	# Radius of curvature in prime vertical (R_N) and meridian (R_M)
	R_N = R_planet_m / np.sqrt(1 - (2*f-f**2)*np.sin(lon_0)**2)
	R_M = R_N * (1 - (2*f-f**2)) / (1 - (2*f-f**2)*np.sin(lon_0)**2)

	# Changes in north and east directions
	dN = d_lat / np.arctan2(1,R_M)
	dE = d_lon / np.arctan2(1,R_N*np.cos(lat_0))

	# X, Y, Z in flat planet in m
	X_m =   dN * np.cos(psi_0) + dE * np.sin(psi_0)
	Y_m = - dN * np.sin(psi_0) + dE * np.cos(psi_0)
	Z_m = - alt_m - h_ref_m

	return X_m, Y_m, Z_m