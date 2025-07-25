import numpy as np


class DescentConstantVyConstantVx():
    """
    docstring for DescentConstantVyConstantVx
    kwargs:
            speed_Y			: speed of descent in Y-direction [m/s]
            distance_Y		: distance of the descent in Y-direction [m]
            speed_X			: speed of cruise in X-direction [m/s]
            distance_X 		: distance of the descent in X-direction [m]
            duration		: total duration of the descent [s]
    """

    def __init__(self, id: int, name: str, kwargs: dict, n_discrete=10):
        super(DescentConstantVyConstantVx, self).__init__()
        self.id = id 								# segment id
        self.name = name 							# segment name
        self.kind = 'DescentConstantVyConstantVx'  # segment kind
        self.n_discrete = n_discrete				# mission discretization
        self.kwargs = kwargs

        self.AoA = None
        self.speed_Y = None
        self.distance_Y = None
        self.speed_X = None
        self.distance_X = None
        self.speed = None
        self.duration = None
        self.gamma = None

        self.distance = None  # Euclidean distance

        # Constants (atmosphere and gravity)
        self.constants = None

    def _initialize(self):
        for item in list(self.kwargs):
            if item == 'speed_Y':
                self.speed_Y = float(self.kwargs[item])
            elif item == 'distance_Y':
                self.distance_Y = float(self.kwargs[item])
            elif item == 'speed_X':
                self.speed_X = float(self.kwargs[item])
            elif item == 'distance_X':
                self.distance_X = float(self.kwargs[item])
            elif item == 'speed':
                self.speed = float(self.kwargs[item])
            elif item == 'duration':
                self.duration = float(self.kwargs[item])

        try:
            if self.duration is None:
                self.duration = self.distance_Y / self.speed_Y
            if self.speed_Y is None:
                self.speed_Y = self.distance_Y / self.duration
            if self.distance_Y is None:
                self.distance_Y = self.speed_Y * self.duration

            if self.distance_X is None and self.speed_X is None:
                self.speed_X = np.sqrt(self.speed**2 - self.speed_Y**2)
                self.distance_X = self.speed_X * self.duration
            if self.distance_X is None and self.speed is None:
                self.distance_X = self.speed_X * self.duration
                self.speed = np.sqrt(self.speed_X**2 + self.speed_Y**2)
            if self.speed_X is None and self.speed is None:
                self.speed_X = self.distance_X / self.duration
                self.speed = np.sqrt(self.speed_X**2 + self.speed_Y**2)

        except NameError:
            raise NameError("Need to define at least two of the followings: speed_Y, distance_Y, duration; Must also define at least one of the following speed_X, distance_X, speed;")

        self.distance = np.sqrt(self.distance_X**2 + self.distance_Y**2)
        self.gamma = np.arctan(self.speed_Y / self.speed_X)

    def _calc_time(self, t_list):
        t0 = t_list[-1][-1]
        t_next = np.linspace(t0, t0 + self.duration, self.n_discrete + 1)
        return t_next

    def _calc_position(self, x_list, y_list, t_next):
        x0, y0, t0 = x_list[-1][-1], y_list[-1][-1], t_next[0]
        # Position under constant Vy and constant Vx
        x_next = x0 + self.speed_X * (t_next - t0)
        y_next = y0 - self.speed_Y * (t_next - t0)
        x_list.append(x_next)
        y_list.append(y_next)
        return x_list, y_list

    def _calc_velocity(self, vx_list, vy_list, t_next):
        vx0, vy0, t0 = self.speed_X, -self.speed_Y, t_next[0]
        # Velocity under zero acceleration
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
        info += f'\t\t\tSpeed_Y = {self.speed_Y} m/s\n'
        info += f'\t\t\tDistance_Y = {self.distance_Y} m\n'
        info += f'\t\t\tSpeed_X = {self.speed_X} m/s\n'
        info += f'\t\t\tDistance_X = {self.distance_X} m\n'
        info += f'\t\t\tDuration = {self.duration} s\n'
        info += f'\t\t\tEuclidean_distance = {self.distance} m'
        return info

    def print_info(self):
        print(f'Segment ID = {self.id}')
        print(f'Segment Name: {self.name}')
        print(f'\tSpeed_Y = {self.speed_Y} m/s')
        print(f'\tDistance_Y = {self.distance_Y} m')
        print(f'\tSpeed_X = {self.speed_X} m/s')
        print(f'\tDistance_X = {self.distance_X} m')
        print(f'\tDuration = {self.duration} s')
        print(f'\tEuclidean_distance = {self.distance} m')
