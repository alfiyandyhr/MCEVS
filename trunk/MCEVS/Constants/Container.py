from MCEVS.Constants.Gravity.Earth import EarthGravityModel
from MCEVS.Constants.Atmosphere.Earth import Constant_Temperature, US_Standard_1976


class EarthGravityAndAtmosphere(object):
    """
    docstring for EarthGravityAndAtmosphere
    It is a wrapper for gravity and atmosphere model
    """

    def __init__(self, atmosphere_model: str):
        super(EarthGravityAndAtmosphere, self).__init__()
        self.atmosphere_model = atmosphere_model

    def compute_constants(self, altitude: float):

        g = EarthGravityModel().compute_gravity(altitude)

        if self.atmosphere_model == 'US_Standard_1976':
            data = US_Standard_1976().compute_constants(altitude=altitude, gravity=g)
        if self.atmosphere_model == 'Constant_Temperature':
            data = Constant_Temperature().compute_constants(altitude=altitude, gravity=g)

        # Add gravity data
        data['g'] = g

        return data
