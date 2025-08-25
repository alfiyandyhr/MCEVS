class Airfoil(object):
    """
    Represents an airfoil with basic aerodynamic properties at Re= 4E6, Mach= 0.2
    Data are derived from XFOIL analysis,
    see "MCEVS/examples/0_analyses_of_ref_vehicle/1_aerodynamic_analysis/liftcruise/airfoil"
    """
    _airfoil_data = {
        'NACA2412': {'CL_alpha': 6.512296194147039, 'CL_0': 0.23977560975609744},
        'NACA0012': {'CL_alpha': 6.5644003948073575, 'CL_0': 0.0},
        'LS417': {'CL_alpha': 6.8183155478056205, 'CL_0': 0.5050512195121949}
    }

    def __init__(self, name):
        self.name = name.upper()

    @property
    def CL_alpha(self):
        try:
            return self._airfoil_data[self.name]['CL_alpha']
        except KeyError:
            raise ValueError(f"Airfoil '{self.name}' not recognized.")

    @property
    def CL_0(self):
        try:
            return self._airfoil_data[self.name]['CL_0']
        except KeyError:
            raise ValueError(f"Airfoil '{self.name}' not recognized.")

    def __repr__(self):
        return f"<Airfoil {self.name}: CL_alpha={self.CL_alpha}, CL_0={self.CL_0}>"
