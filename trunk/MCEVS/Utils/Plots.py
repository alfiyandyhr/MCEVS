import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def plot_mission_parameters(mission: object, print_info=False, save_fig=False):

    if print_info:
        mission.print_info()

    # Preprocessing data
    t = np.array([])
    x, y = np.array([]), np.array([])
    vx, vy = np.array([]), np.array([])
    ax, ay = np.array([]), np.array([])
    for t_segment in mission.t:
        t = np.concatenate((t, t_segment))
    for x_segment in mission.x:
        x = np.concatenate((x, x_segment))
    for y_segment in mission.y:
        y = np.concatenate((y, y_segment))
    for vx_segment in mission.vx:
        vx = np.concatenate((vx, vx_segment))
    for vy_segment in mission.vy:
        vy = np.concatenate((vy, vy_segment))
    for ax_segment in mission.ax:
        ax = np.concatenate((ax, ax_segment))
    for ay_segment in mission.ay:
        ay = np.concatenate((ay, ay_segment))

    # Plot position
    plt.figure(figsize=(6, 3.5))
    plt.plot(x / 1000.0, y, '-o')
    plt.xlabel('Range ($km$) in X-dir')
    plt.ylabel('AGL Altitude ($m$) in Y-dir')
    plt.title('Aircraft position y vs x')
    plt.minorticks_on()
    plt.grid(which='major', linewidth=0.8)
    plt.grid(which='minor', linewidth=0.2)
    if save_fig:
        plt.tight_layout()
        plt.savefig('range_vs_altitude.pdf', format='pdf')
    plt.show()

    # Plot bulk
    fig, ax = plt.subplots(nrows=3, ncols=2, sharex=False, figsize=(12, 9))
    ax[0, 0].plot(t / 60.0, x / 1000.0, '-o')
    ax[0, 0].set_ylabel('Range ($km$) in X-dir')
    ax[0, 0].minorticks_on()
    ax[0, 0].grid(which='major', linewidth=0.8)
    ax[0, 0].grid(which='minor', linewidth=0.2)
    ax[0, 1].plot(t / 60.0, y, '-o')
    ax[0, 1].set_ylabel('AGL Altitude ($m$) in Y-dir')
    ax[0, 1].minorticks_on()
    ax[0, 1].grid(which='major', linewidth=0.8)
    ax[0, 1].grid(which='minor', linewidth=0.2)
    ax[1, 0].plot(t / 60.0, vx * 3.6, '-o')
    ax[1, 0].set_ylabel('Velocity x-dir ($km/h$)')
    ax[1, 0].minorticks_on()
    ax[1, 0].grid(which='major', linewidth=0.8)
    ax[1, 0].grid(which='minor', linewidth=0.2)
    ax[1, 1].plot(t / 60.0, vy, '-o')
    ax[1, 1].set_ylabel('Velocity y-dir ($m/s$)')
    ax[1, 1].minorticks_on()
    ax[1, 1].grid(which='major', linewidth=0.8)
    ax[1, 1].grid(which='minor', linewidth=0.2)
    ax[2, 0].plot(t / 60.0, ax, '-o')
    ax[2, 0].set_ylabel('Acceleration x-dir ($m/s^2$)')
    ax[2, 0].set_xlabel('Time ($min$)')
    ax[2, 0].minorticks_on()
    ax[2, 0].grid(which='major', linewidth=0.8)
    ax[2, 0].grid(which='minor', linewidth=0.2)
    ax[2, 1].plot(t / 60.0, ay, '-o')
    ax[2, 1].set_ylabel('Acceleration y-dir ($m/s^2$)')
    ax[2, 1].set_xlabel('Time ($min$)')
    ax[2, 1].minorticks_on()
    ax[2, 1].grid(which='major', linewidth=0.8)
    ax[2, 1].grid(which='minor', linewidth=0.2)
    if save_fig:
        plt.savefig('flight_params.pdf', format='pdf')
    plt.show()


def plot_performance_by_segments(mission: object, vehicle: object):

    # Constant segments
    constant_segments = ['HoverClimbConstantSpeed', 'ClimbConstantVyConstantVx', 'CruiseConstantSpeed', 'DescentConstantVyConstantVx', 'HoverDescentConstantSpeed', 'HoverStay']

    # Preprocessing data
    t = np.array([])

    for i, segment in enumerate(mission.segments):
        if segment.kind in constant_segments:
            t = np.concatenate((t, mission.t[i]))

    if vehicle.configuration == 'LiftPlusCruise':

        P_propeller, P_lift_rotor = np.array([]), np.array([])
        DL_propeller, DL_lift_rotor = np.array([]), np.array([])

        for P in mission.P['Propeller']:
            P_propeller = np.concatenate((P_propeller, P))
        for P in mission.P['LiftRotor']:
            P_lift_rotor = np.concatenate((P_lift_rotor, P))
        for DL in mission.DL['Propeller']:
            DL_propeller = np.concatenate((DL_propeller, DL))
        for DL in mission.DL['LiftRotor']:
            DL_lift_rotor = np.concatenate((DL_lift_rotor, DL))

        # Plot bulk
        fig, ax = plt.subplots(nrows=2, ncols=2, sharex=False)
        ax[0, 0].plot(t / 60.0, P_lift_rotor / 1000.0, '-o')
        ax[0, 0].set_ylabel('LiftRotor power total ($kW$)')
        ax[0, 0].minorticks_on()
        ax[0, 0].grid(which='major', linewidth=0.8)
        ax[0, 0].grid(which='minor', linewidth=0.2)
        ax[0, 1].plot(t / 60.0, P_propeller / 1000.0, '-o')
        ax[0, 1].set_ylabel('Propeller power total ($kW$)')
        ax[0, 1].minorticks_on()
        ax[0, 1].grid(which='major', linewidth=0.8)
        ax[0, 1].grid(which='minor', linewidth=0.2)
        ax[1, 0].plot(t / 60.0, DL_lift_rotor, '-o')
        ax[1, 0].set_ylabel('LiftRotor disk loading each ($N/m^2$)')
        ax[1, 0].minorticks_on()
        ax[1, 0].grid(which='major', linewidth=0.8)
        ax[1, 0].grid(which='minor', linewidth=0.2)
        ax[1, 0].set_xlabel('Time ($min$)')
        ax[1, 1].plot(t / 60.0, DL_propeller, '-o')
        ax[1, 1].set_ylabel('Propeller disk loading each ($N/m^2$)')
        ax[1, 1].minorticks_on()
        ax[1, 1].grid(which='major', linewidth=0.8)
        ax[1, 1].grid(which='minor', linewidth=0.2)
        ax[1, 1].set_xlabel('Time ($min$)')

        plt.show()

    elif vehicle.configuration == 'Multirotor':

        P_lift_rotor, DL_lift_rotor = np.array([]), np.array([])

        for P in mission.P['LiftRotor']:
            P_lift_rotor = np.concatenate((P_lift_rotor, P))
        for DL in mission.DL['LiftRotor']:
            DL_lift_rotor = np.concatenate((DL_lift_rotor, DL))

        # Plot bulk
        fig, ax = plt.subplots(nrows=1, ncols=2, sharex=False)
        ax[0].plot(t / 60.0, P_lift_rotor / 1000.0, '-o')
        ax[0].set_ylabel('Rotor power total ($kW$)')
        ax[0].minorticks_on()
        ax[0].grid(which='major', linewidth=0.8)
        ax[0].grid(which='minor', linewidth=0.2)
        ax[0].set_xlabel('Time ($min$)')
        ax[1].plot(t / 60.0, DL_lift_rotor, '-o')
        ax[1].set_ylabel('Rotor disk loading each ($N/m^2$)')
        ax[1].minorticks_on()
        ax[1].grid(which='major', linewidth=0.8)
        ax[1].grid(which='minor', linewidth=0.2)
        ax[1].set_xlabel('Time ($min$)')

        plt.show()


def plot_geometries(vehicle_list: list, label_list: list, cruise_speed_list=[None],
                    color_list=['gray', 'blue', 'red', 'green'], alpha_list=[0.5, 0.25, 0.25, 0.25],
                    figname='fig', savefig=False):

    with_operation_var = False if cruise_speed_list[0] is None else True
    if with_operation_var:
        cruise_speed_list = np.round(np.array(cruise_speed_list), 1)

    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(8, 8))

    # Center of coordinate
    center_x = 0.0
    center_y = 0.0

    # Limits for figure
    x_lims = [0, 0]
    y_lims = [0, 0]

    if vehicle_list[0].configuration == 'Multirotor':

        # Iterate the vehicle in the list
        for i, vehicle in enumerate(vehicle_list):

            # Extracting the information
            fuse_length = vehicle.fuselage.length
            fuse_max_D = vehicle.fuselage.max_diameter
            r_lift_rotor = vehicle.lift_rotor.radius
            r_ref = 4.20624
            l_ref_list = [8.2377, 7.3779]
            theta_list = [43 / 180 * np.pi, 47.5 / 180 * np.pi]
            delta_y = (0.54 - 0.50) * fuse_length

            x_lims[0] = min(x_lims[0], - r_lift_rotor / r_ref * l_ref_list[0] * np.sin(theta_list[0]) - r_lift_rotor)
            x_lims[1] = max(x_lims[1], r_lift_rotor / r_ref * l_ref_list[0] * np.sin(theta_list[0]) + r_lift_rotor)

            y_lims[0] = min(y_lims[0], - delta_y - r_lift_rotor / r_ref * l_ref_list[1] * np.cos(theta_list[1]) - r_lift_rotor)
            y_lims[1] = max(y_lims[1], - delta_y + r_lift_rotor / r_ref * l_ref_list[0] * np.cos(theta_list[0]) + r_lift_rotor)

            for j in range(2):

                circle_x1 = center_x + (2 * j - 1) * r_lift_rotor / r_ref * l_ref_list[0] * np.sin(theta_list[0])
                circle_y1 = center_y - delta_y + r_lift_rotor / r_ref * l_ref_list[0] * np.cos(theta_list[0])
                circle_x2 = center_x + (2 * j - 1) * r_lift_rotor / r_ref * l_ref_list[1] * np.sin(theta_list[1])
                circle_y2 = center_y - delta_y - r_lift_rotor / r_ref * l_ref_list[1] * np.cos(theta_list[1])

                # Circle above
                circle_above = patches.Circle((circle_x1, circle_y1), r_lift_rotor, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i], label=label_list[i] if j == 0 else None)
                ax.add_patch(circle_above)

                # Circle below
                circle_below = patches.Circle((circle_x2, circle_y2), r_lift_rotor, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i])
                ax.add_patch(circle_below)

        # --- Operation Variables --- #

        if with_operation_var:

            # Cruise speed
            for k, cruise_speed in enumerate(cruise_speed_list):
                arrow_loc_y = center_y + y_lims[1] + 2.3
                if len(cruise_speed_list) == 1:
                    arrow_loc_x = center_x
                elif len(cruise_speed_list) == 2:
                    arrow_loc_x = center_x + (k - 1 / 2)
                elif len(cruise_speed_list) == 3:
                    arrow_loc_x = center_x + (k / 2 - 1 / 2)
                elif len(cruise_speed_list) == 4:
                    arrow_loc_x = center_x + (k / 2 - 0.75)
                dy = cruise_speed_list[k] / max(cruise_speed_list) * (-3.0)
                ax.arrow(x=arrow_loc_x, y=arrow_loc_y, dx=0, dy=dy, head_width=0.2, head_length=0.2, fc=color_list[k], ec=color_list[k])

        # --- PLOT ONLY ONCE !!! --- #

        # Create the fuselage
        ellipse = patches.Ellipse((center_x, center_y), fuse_max_D, fuse_length, ec='gray', fc='gray', linewidth=1, alpha=0.5)
        ax.add_patch(ellipse)

        # Draw the center line
        ax.plot([0, 0], [0.7 * y_lims[0], 0.7 * y_lims[1]], color="gray", linestyle="--", linewidth=1, alpha=0.4)

        if with_operation_var:

            # --- Cruise Speeds --- #

            ax.text(center_x, center_y + y_lims[1] + 4.0, 'Cruise Speed (km/h)', color='black', fontsize=12, ha='center', va='center')

            # Starting position for the text
            start_y = center_y + y_lims[1] + 3.0
            spacing = 0.4 if len(cruise_speed_list) < 4 else 0.5  # Spacing between characters

            # Calculate total width of the text and starting x position to center the text
            text_length = spacing * (2 + sum(len(str(num)) for num in cruise_speed_list) + len(cruise_speed_list) - 1)  # Brackets + numbers + commas
            start_x = center_x - text_length / 2

            ax.text(start_x, start_y, "[", color="black", fontsize=12, ha="left", va="center")
            start_x += spacing
            for k, num in enumerate(cruise_speed_list):
                num = np.round(num, 1)
                ax.text(start_x, start_y, str(num), color=color_list[k], fontsize=12, ha="left", va="center")
                start_x += len(str(num)) * spacing
                if k < len(cruise_speed_list) - 1:
                    ax.text(start_x, start_y, ",", color="black", fontsize=12, ha="left", va="center")
                    start_x += spacing
                if len(cruise_speed_list) == 1:
                    start_x += 0.25
            ax.text(start_x, start_y, "]", color="black", fontsize=12, ha="left", va="center")

            # --- RPM Rotors at cruise --- #
            RPM_list = np.round([vehicle.lift_rotor.RPM['cruise'] for vehicle in vehicle_list], 1)

            start_x = center_x
            start_y = y_lims[0] - 1.85

            ax.text(start_x, start_y + 0.85, 'RPM rotors', color='black', fontsize=12, ha='left', va='center')

            ax.text(start_x, start_y, "[", color="black", fontsize=12, ha="left", va="center")
            start_x += spacing

            # Loop through the RPM list
            for k, num in enumerate(RPM_list):
                # Check if we need to start a new line
                if k > 0 and k % 3 == 0:  # After every 3 items, go to a new line
                    start_x = center_x  # Reset x to the starting x position
                    start_y -= 1.0      # Move y to the next line

                # Write the RPM number
                ax.text(start_x, start_y, str(num), color=color_list[k], fontsize=12, ha="left", va="center")
                start_x += len(str(num)) * spacing

                # Add a comma unless it's the last item
                if k < len(RPM_list) - 1:
                    ax.text(start_x, start_y, ",", color="black", fontsize=12, ha="left", va="center")
                    start_x += spacing
                if len(RPM_list) == 1:
                    start_x += 0.25

            # Close the list with a bracket
            ax.text(start_x, start_y, "]", color="black", fontsize=12, ha="left", va="center")

        # Set limits and aspect ratio
        ax.set_xlim(x_lims[0] - 1.0, x_lims[1] + 1.0)
        ax.set_ylim(y_lims[0] - 4.0 if len(vehicle_list) < 4 else y_lims[0] - 5.0, y_lims[1] + 5.0)
        ax.set_aspect('equal')
        ax.axis('on')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.legend(loc='lower left')
        plt.savefig(f'{figname}.pdf', format='pdf', dpi=300) if savefig else plt.show()

    if vehicle_list[0].configuration == 'LiftPlusCruise':

        # Iterate the vehicle in the list
        for i, vehicle in enumerate(vehicle_list):

            # Extracting the information
            wing_AR = vehicle.wing.aspect_ratio
            wing_S = vehicle.wing.area
            wing_span = np.sqrt(wing_S * wing_AR)
            wing_chord = wing_span / wing_AR
            r_lift_rotor = vehicle.lift_rotor.radius
            r_propeller = vehicle.propeller.radius
            fuse_length = vehicle.fuselage.length
            fuse_max_D = vehicle.fuselage.max_diameter
            boom_length = vehicle.boom.length
            htail_AR = vehicle.horizontal_tail.aspect_ratio
            htail_S = vehicle.horizontal_tail.area
            htail_span = np.sqrt(htail_S * htail_AR)
            htail_chord = htail_span / htail_AR

            # Coordinates
            wing_x = center_x - wing_span / 2
            wing_y = center_y - wing_chord / 2
            htail_x = center_x - htail_span / 2
            htail_y = center_y - fuse_length * 0.68
            fuse_x = center_x
            fuse_y = center_y - fuse_length * 1 / 5

            # Limits for figure
            if x_lims[1] < wing_span / 2 + r_lift_rotor:
                x_lims = [-wing_span / 2 - r_lift_rotor, wing_span / 2 + r_lift_rotor]

            # Create the wing box (centered at (center_x, center_y))
            rectangle1 = patches.Rectangle((wing_x, wing_y), wing_span, wing_chord, ec=color_list[i], fc=color_list[i], linewidth=1, alpha=alpha_list[i], label=label_list[i])
            ax.add_patch(rectangle1)

            # Draw the propeller
            ax.plot([-r_propeller, r_propeller], [-fuse_length * 0.72 - i * 0.1, -fuse_length * 0.72 - i * 0.1], color=color_list[i], linewidth=1.5, alpha=alpha_list[i])

            # Define the circle properties
            circle_radius = r_lift_rotor
            # circle_spacing = wing_span / 4  # Spaced evenly along the rectangle's length

            hor_rect_length = wing_span
            # hor_rect_width = wing_chord
            # vert_rect_length = fuse_length

            # Calculate horizontal circle positions (4 above and 4 below the horizontal rectangle)
            # hor_circle_spacing = hor_rect_length / 4
            vert_circle_offset = boom_length / 2  # Offset from the rectangle

            # Draw the outer lift rotors
            for j in range(2):
                # Circle positions along the x-axis
                circle_x = center_x + (j - 1 / 2) * hor_rect_length

                # Circle above the horizontal rectangle
                circle_above = patches.Circle((circle_x, center_y + vert_circle_offset), circle_radius, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i])
                ax.add_patch(circle_above)

                # Circle below the horizontal rectangle
                circle_below = patches.Circle((circle_x, center_y - vert_circle_offset), circle_radius, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i])
                ax.add_patch(circle_below)

                # Draw the booms
                ax.plot([circle_x, circle_x], [-vert_circle_offset, vert_circle_offset], color=color_list[i], linewidth=2, alpha=0.5)

            # Draw the inner lift rotors
            for j in range(2):
                # Circle positions along the x-axis
                circle_x = center_x + (2 * j - 1) * ((fuse_max_D + wing_span) / 4 - 0.5 * r_lift_rotor)

                # Circle above the horizontal rectangle
                circle_above = patches.Circle((circle_x, center_y + vert_circle_offset), circle_radius, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i])
                ax.add_patch(circle_above)

                # Circle below the horizontal rectangle
                circle_below = patches.Circle((circle_x, center_y - vert_circle_offset), circle_radius, ec=color_list[i], fc='none', linewidth=1.5, alpha=alpha_list[i])
                ax.add_patch(circle_below)

                # Draw the booms
                ax.plot([circle_x, circle_x], [-vert_circle_offset, vert_circle_offset], color=color_list[i], linewidth=2, alpha=0.5)

                # --------------------------------------------------------------------------------------------------------------------------- #

            # --- Frontal View --- #

            # Draw wing
            ax.plot([-wing_span / 2, wing_span / 2], [-fuse_length + 0.75 * fuse_max_D / 2 - 1.0, -fuse_length + 0.75 * fuse_max_D / 2 - 1.0], color=color_list[i], linewidth=6, alpha=alpha_list[i])

            # Draw propeller
            black_circle = patches.Circle((center_x, -fuse_length - 1.0), r_propeller, color=color_list[i], fill=False, linewidth=1.5, alpha=alpha_list[i])
            ax.add_patch(black_circle)

            # --- Operation Variables --- #

            # Cruise speed
            if with_operation_var:
                arrow_loc_y = center_y + fuse_length * 3 / 4
                if len(cruise_speed_list) == 1:
                    arrow_loc_x = center_x
                elif len(cruise_speed_list) == 2:
                    arrow_loc_x = center_x + (i - 1 / 2)
                elif len(cruise_speed_list) == 3:
                    arrow_loc_x = center_x + (i / 2 - 1 / 2)
                dy = cruise_speed_list[i] / max(cruise_speed_list) * (-2.5)
                ax.arrow(x=arrow_loc_x, y=arrow_loc_y, dx=0, dy=dy, head_width=0.2, head_length=0.2, fc=color_list[i], ec=color_list[i])

        # --- PLOT ONLY ONCE !!! --- #

        # Create the fuselage
        ellipse = patches.Ellipse((fuse_x, fuse_y), fuse_max_D, fuse_length, ec='gray', fc='gray', linewidth=1, alpha=0.5)
        ax.add_patch(ellipse)
        circle = patches.Circle((center_x, -fuse_length - 1.0), fuse_max_D / 2, ec='gray', fc='gray', linewidth=1, alpha=0.5)
        ax.add_patch(circle)

        # Create the empennage
        rectangle2 = patches.Rectangle((htail_x, htail_y), htail_span, htail_chord, ec='gray', fc='gray', linewidth=1, alpha=0.5)
        ax.add_patch(rectangle2)

        # Draw the center line
        ax.plot([0, 0], [-1.4 * fuse_length, fuse_length * 0.4], color="gray", linestyle="--", linewidth=1, alpha=0.4)

        # Draw the separation line
        ax.plot([x_lims[0], x_lims[1]], [-0.8 * fuse_length, -0.8 * fuse_length], color="black", linestyle="--", linewidth=1, alpha=0.4)

        # Texts
        ax.text(x_lims[0], -0.75 * fuse_length, 'top view', color='black', fontsize=12, ha='left', va='center')
        ax.text(x_lims[0], -0.86 * fuse_length, 'front view', color='black', fontsize=12, ha='left', va='center')

        if with_operation_var:

            # --- Cruise Speeds --- #

            ax.text(center_x, center_y + fuse_length * 3 / 4 + 1.5, 'Cruise Speed (km/h)', color='black', fontsize=12, ha='center', va='center')

            # Starting position for the text
            start_y = center_y + fuse_length * 3 / 4 + 0.65
            spacing = 0.4  # Spacing between characters

            # Calculate total width of the text and starting x position to center the text
            text_length = spacing * (2 + sum(len(str(num)) for num in cruise_speed_list) + len(cruise_speed_list) - 1)  # Brackets + numbers + commas
            start_x = center_x - text_length / 2

            ax.text(start_x, start_y, "[", color="black", fontsize=12, ha="left", va="center")
            start_x += spacing
            for k, num in enumerate(cruise_speed_list):
                num = np.round(num, 1)
                ax.text(start_x, start_y, str(num), color=color_list[k], fontsize=12, ha="left", va="center")
                start_x += len(str(num)) * spacing
                if k < len(cruise_speed_list) - 1:
                    ax.text(start_x, start_y, ",", color="black", fontsize=12, ha="left", va="center")
                    start_x += spacing
            ax.text(start_x, start_y, "]", color="black", fontsize=12, ha="left", va="center")

            # --- RPM Propellers at cruise --- #
            RPM_list = np.round([vehicle.propeller.RPM['cruise'] for vehicle in vehicle_list], 1)

            # Starting position for the text
            start_x = center_x + 3.0
            start_y = center_y - fuse_length - 2.5

            ax.text(start_x, start_y + 0.85, 'RPM prop', color='black', fontsize=12, ha='left', va='center')

            ax.text(start_x, start_y, "[", color="black", fontsize=12, ha="left", va="center")
            start_x += spacing
            for k, num in enumerate(RPM_list):
                ax.text(start_x, start_y, str(num), color=color_list[k], fontsize=12, ha="left", va="center")
                start_x += len(str(num)) * spacing
                if k < len(RPM_list) - 1:
                    ax.text(start_x, start_y, ",", color="black", fontsize=12, ha="left", va="center")
                    start_x += spacing
            ax.text(start_x, start_y, "]", color="black", fontsize=12, ha="left", va="center")

        ax.set_xlim(x_lims[0] - 1.0, x_lims[1] + 1.0)
        ax.set_ylim(center_y - fuse_length - 5.0, center_y + fuse_length * 3 / 4 + 2.5)
        ax.set_aspect('equal')
        ax.axis('on')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.legend(loc='lower left')
        plt.savefig(f'{figname}.pdf', format='pdf', dpi=300) if savefig else plt.show()


def plot_weight_breakdown(data_list: list, label_list: list = None, color_list=['gray', 'blue', 'red', 'green'], figname='fig', figtitle='Component Weight Breakdown', savefig=False):
    
    if not isinstance(data_list, list):
        return ValueError('Please provide a list of pd.DataFrame!')

    if label_list is None:
        label_list = [f'Design {i+1}' for i in range(len(data_list))]

    components = ['MTOW', 'Battery', 'Structure', 'Propulsion', 'Equipment']

    fig, ax = plt.subplots(figsize=(max(6, len(components) * 2.0), 6))

    x = np.arange(len(components))
    width = 0.8 / len(data_list)  # total width = 0.8, divided among designs

    for i, data in enumerate(data_list):

        data_i = data[['Weight|takeoff', 'Weight|battery', 'Weight|structure', 'Weight|propulsion', 'Weight|equipment']]

        offset = (i - (len(data_list) - 1) / 2) * width
        bars = ax.bar(x + offset, data_i.to_numpy()[0], width, label=label_list[i], color=color_list[i], alpha=0.5)
        
        # Add value labels on top of bars
        for bar in bars:
            height = np.round(bar.get_height(), 1)
            ax.annotate(f'{height}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=8)

    ax.set_ylabel(r'Weight $[kg]$', fontsize=12)
    ax.set_title(f'{figtitle}', fontsize=16)
    ax.set_xticks(x)
    ax.set_xticklabels(components)
    ax.legend()
    plt.tight_layout()
    plt.savefig(f'{figname}.pdf', format='pdf', dpi=300) if savefig else plt.show()


def plot_power_energy_breakdown(data_list: list, label_list: list = None, color_list=['gray', 'blue', 'red', 'green'],
                                figname='fig', figtitle='Power Energy Breakdown', savefig=False,
                                xticks_labels=[r'HoverClimb $[kW]$', r'Cruise $[kW]$', r'HoverDescent $[kW]$', r'Energy Capacity $[kWh]$'],
                                indexes=['Power|segment_1', 'Power|segment_2', 'Power|segment_3', 'Energy|entire_mission']):
    
    if not isinstance(data_list, list):
        return ValueError('Please provide a list of pd.DataFrame!')

    if label_list is None:
        label_list = [f'Design {i+1}' for i in range(len(data_list))]

    fig, ax = plt.subplots(figsize=(max(6, len(xticks_labels) * 2.0), 6))

    x = np.arange(len(xticks_labels))
    width = 0.8 / len(data_list)  # total width = 0.8, divided among designs

    for i, data in enumerate(data_list):

        data_i = data[indexes]

        offset = (i - (len(data_list) - 1) / 2) * width
        bars = ax.bar(x + offset, data_i.to_numpy()[0], width, label=label_list[i], color=color_list[i], alpha=0.5)
        
        # Add value labels on top of bars
        for bar in bars:
            height = np.round(bar.get_height(), 1)
            ax.annotate(f'{height}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=8)

    ax.set_title(f'{figtitle}', fontsize=16)
    ax.set_xticks(x)
    ax.set_xticklabels(xticks_labels)
    ax.legend()
    plt.tight_layout()
    plt.savefig(f'{figname}.pdf', format='pdf', dpi=300) if savefig else plt.show()


def plot_weight_breakdown_pie_chart(data_list: list, label_list: list = None, color_list=['gray', 'blue', 'red', 'green', 'purple'],
                                    figname='fig', figtitle='Component Weight Breakdown For Varying Battery Levels', savefig=False):

    if not isinstance(data_list, list):
        return ValueError('Please provide a list of pd.DataFrame!')

    if label_list is None:
        label_list = [f'Battery {i+1}' for i in range(len(data_list))]

    # Component labels
    components = ['Payload', 'Battery', 'Structure', 'Propulsion', 'Equipment']

    # Function to display both percentage and value
    def autopct_format(values):
        def inner_autopct(pct):
            total = sum(values)
            val = int(round(pct * total / 100.0))
            return '{:.1f}%\n({:d})'.format(pct, val)
        return inner_autopct

    fig, axs = plt.subplots(1, 3, figsize=(14, 6))

    # Store wedges just once for global legend
    data_0 = data_list[0][['Weight|payload', 'Weight|battery', 'Weight|structure', 'Weight|propulsion', 'Weight|equipment']].to_numpy()[0]
    wedges, _, _ = axs[0].pie(
        data_0,
        autopct=autopct_format(data_0),
        startangle=90,
        textprops={'fontsize': 12},
        colors=color_list,
        wedgeprops={'edgecolor': 'white', 'linewidth': 2, 'alpha': 0.5}
    )
    axs[0].set_title(label_list[0])

    for i, data in enumerate(data_list):

        data_i = data[['Weight|payload', 'Weight|battery', 'Weight|structure', 'Weight|propulsion', 'Weight|equipment']].to_numpy()[0]

        axs[i].pie(
            data_i,
            autopct=autopct_format(data_i),
            startangle=90,
            textprops={'fontsize': 12},
            colors=color_list,
            wedgeprops={'edgecolor': 'white', 'linewidth': 2, 'alpha': 0.5 if i > 0 else 0.0},
        )
        axs[i].set_title(label_list[i], loc='center', y=0.95, fontsize=12)

    # Add a single global legend at the top center
    fig.legend(wedges, components, loc='upper center', ncol=len(components), fontsize=12, title_fontsize=13, bbox_to_anchor=(0.5, 0.89))
    fig.suptitle(f'{figtitle}', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.87])
    plt.subplots_adjust(wspace=-0.3)
    plt.savefig(f'{figname}.pdf', format='pdf', dpi=300) if savefig else plt.show()
