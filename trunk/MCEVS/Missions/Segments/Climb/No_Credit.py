import numpy as np


class NoCreditClimb():
    """
    docstring for NoCreditClimb
    kwargs:
            distance_Y		: distance of the climb in Y-direction [m]
            distance_X 		: distance of the climb in X-direction [m]
            duration		: total duration of the climb [s]
    """

    def __init__(self, id: int, name: str, kwargs: dict, n_discrete=10):
        super(NoCreditClimb, self).__init__()
        self.id = id 					# segment id
        self.name = name 				# segment name
        self.kind = 'NoCreditClimb'		# segment kind
        self.n_discrete = n_discrete  # mission discretization
        self.kwargs = kwargs

        self.distance_Y = None
        self.distance_X = None
        self.distance = None 			# Euclidean distance
        self.duration = 0.0

    def _initialize(self):
        for item in list(self.kwargs):
            if item == 'distance_Y':
                self.distance_Y = float(self.kwargs[item])
            elif item == 'distance_X':
                self.distance_X = float(self.kwargs[item])

        try:
            self.distance = np.sqrt(self.distance_X**2 + self.distance_Y**2)
        except NameError:
            raise NameError('Need to define two of the followings: distance_X, distance_Y')

    def _calc_time(self, t_list):
        t0 = t_list[-1][-1]
        t_next = t0 * np.ones(self.n_discrete + 1)
        return t_next

    def _calc_position(self, x_list, y_list, t_next):
        x0, y0, _ = x_list[-1][-1], y_list[-1][-1], t_next[0]
        # Position under hypothetical constant Vy and constant Vx
        # print(np.arange(0,10,1))
        x_next = x0 + self.distance_X / self.n_discrete * np.arange(0, self.n_discrete + 1)
        y_next = y0 + self.distance_Y / self.n_discrete * np.arange(0, self.n_discrete + 1)
        x_list.append(x_next)
        y_list.append(y_next)
        return x_list, y_list

    def _calc_velocity(self, vx_list, vy_list, t_next):
        vx0, vy0, t0 = vx_list[-1], vy_list[-1], t_next[0]
        # Velocity under hypothetical zero acceleration
        vx_next = vx0 + 0.0 * (t_next - t0)
        vy_next = vy0 + 0.0 * (t_next - t0)
        vx_list.append(vx_next)
        vy_list.append(vy_next)
        return vx_list, vy_list

    def _calc_acceleration(self, ax_list, ay_list, t_next):
        # Zero acceleration in y and x
        ax_next = np.zeros_like(t_next)
        ay_next = np.zeros_like(t_next)
        ax_list.append(ax_next)
        ay_list.append(ay_next)
        return ax_list, ay_list

    def _info(self):
        info = f'\t\tSegment ID = {self.id}\n'
        info += f'\t\tSegment Name: {self.name}\n'
        info += f'\t\t\tDistance_Y = {self.distance_Y} m\n'
        info += f'\t\t\tDistance_X = {self.distance_X} m\n'
        info += f'\t\t\tEuclidean_distance = {self.distance} m'
        return info

    def print_info(self):
        print(f'Segment ID = {self.id}')
        print(f'Segment Name: {self.name}')
        print(f'\tDistance_Y = {self.distance_Y} m')
        print(f'\tDistance_X = {self.distance_X} m')
        print(f'\tEuclidean_distance = {self.distance} m')
