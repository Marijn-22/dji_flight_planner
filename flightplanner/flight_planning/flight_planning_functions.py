import numpy as np
# from scipy.spatial.transform import Rotation as R
import shapely as sh
from shapely.geometry import Polygon, MultiLineString, MultiPoint, LineString

# Functions used for flightplanning for the DJI Matrice 300 RTK

# def find_max_waypointTurnDampingDist(points: list):
#     '''
#     Function finds max waypointTurnDampingDist. First checks length of wayline
#     of previous and next point in the flightplan. So the distance
#     between the point of interest and the previous and next coordinate.

#     Args:
#         points: List containing first the previous point, then the point
#             of interest and third the next point. These points must be
#             given in an square coordinate system in meters.
#    
#     Returns:
#         (float): Max damping distance allowed for this point in meters
#     '''
    

#     pass

def make_multi_linestring(xmin, ymin, xmax, ymax, offset, distance_between_lines):
    amount_of_lines_float = abs(xmax - xmin)/distance_between_lines
    amount_of_lines_roundedup = int(amount_of_lines_float) + (distance_between_lines % abs(xmax - xmin) > 0)
    coords = []
    for i in range(amount_of_lines_roundedup):
        xline = xmin + i*distance_between_lines + offset*distance_between_lines
        coords.append(((xline,ymin),(xline,ymax)))
    return MultiLineString(coords)


def flightcoordinates(polygon_coords: np.array, angle: float, offset: float, buffer: float, distance_between_flight_lines: float):
    ''' This function calculates the waypoints that are required to survay the whole polygon that is given as input. '''
    # Add buffer to the drawn polygon
    sh_poly = Polygon(polygon_coords).buffer(buffer)
    xmin_no_rotation, ymin_no_rotation, xmax_no_rotation, ymax_no_rotation = sh_poly.bounds

    # Rotate the polygon to be able to make rotated flightlines
    sh_poly_negative_rotation = sh.affinity.rotate(sh_poly, -angle, origin=(xmin_no_rotation,ymin_no_rotation))
    
    sh_poly_negative_rotation_bounds = sh_poly_negative_rotation.bounds
    xmin, ymin, xmax, ymax = sh_poly_negative_rotation_bounds
    
    #Make linestrings vertical to be used as flightlines
    multi_line_string_flightdirection = make_multi_linestring(xmin, ymin, xmax, ymax, offset, distance_between_flight_lines)

    #Find intersection flightlines with polygon
    intersection = multi_line_string_flightdirection.intersection(sh_poly_negative_rotation)
    
    # Rotate intersection linestrings back around the same point
    sh_multi_rotated_back = sh.affinity.rotate(intersection, angle, origin=(xmin_no_rotation,ymin_no_rotation))

    # Save flight lines in geojson dict
    overlapping_lines = sh_multi_rotated_back

    # Save all points of the flightplan in flight order and make this a linestring.
    sh_linestrings = list(sh_multi_rotated_back.geoms)
    points_list = []
    for i in range(len(sh_linestrings)):
        line_coords = np.array(sh_linestrings[i].coords,dtype=object)
        if i % 2 == 0: #check for even number of linestring
            points_list.append(line_coords[0])
            points_list.append(line_coords[1])
        else:
            points_list.append(line_coords[1])
            points_list.append(line_coords[0])
    
    return points_list, overlapping_lines

def find_distance_flightlines(overlap: float, flying_height: float, view_angle: float = 70*np.pi/180):
    ''' 
    Function calculates the distance between flightlines basd on the required overlap, flying height and
    viewing angle.
    
    Args:
        overlap: Float between 0 and 1.
        flying_height: Float in meters.
        view_angle: Complete view angle of the instrument in radials.
        
    Returns:
        (float): Distance flightlines in meters, with 2 decimals.'''
    
    swath_width = 2*np.tan(view_angle/2)*flying_height
    distance_flightlines = swath_width * (1-overlap)
    return np.round(distance_flightlines,2)

def find_estimated_point_density(flying_height: float, ground_speed: float, view_angle: float = 70*np.pi/180, points_per_second: float = 240000):
    '''
    Function estimates the point density of the LiDAR acquisition. It is calculated for a single
    flightline, estimates uniform distribution and only single returns.
    
    Args:
        flying_height: Float in meters.
        ground_speed: Ground speed in meters/second.
        view_angle: Complete view angle of the instrument in radials.
        Points per second: Amount of points per second the LiDAR scans (single return).

    Returns:
        (float): Estimated point density [points/m2] rounded to integer.
    '''
    swath_width = 2*np.tan(view_angle/2)*flying_height
    covered_area_in_second = swath_width * ground_speed * 1
    point_density = points_per_second/covered_area_in_second
    return int(point_density)

def find_estimated_mission_duration_and_distance(points_list: list, ground_speed: float):
    ''' 
    Function estimates the mission duration and total distance that is flown.
    
    Args:
        point_list: Contains the coordinates of all the waypoints of the mission in a 
            coordinate system in meters. This is the format that is given as output 
            from the flightcoordinates() function. 
        ground_speed: Ground speed in meters/second.
        
    Returns:
        (float) Estimated duration in minutes, rounded to integer.
        (float) Estimated distance in meters, rounded to integer.
    '''
    point_array = np.asarray(points_list)
    distances_p2 = np.diff(point_array, axis = 0)**2
    L2_distances = np.sum(distances_p2, axis = 1)**0.5
    total_distance = np.sum(L2_distances)
    total_time_s = total_distance/ground_speed
    total_time_min = total_time_s/60
    return int(total_time_min), int(total_distance)


def find_all_max_waypointTurnDampingDists(points: np.array, max_setting: float | None = None):
    '''
    Function finds max allowed waypointTurnDampingDist. First checks length of wayline
    of previous and next point in the flightplan. So the distance
    between the point of interest and the previous and next coordinate. The waypointTurnDampingDist
    is not allowed to be larger than half of the wayline distance of contiguous points.

    Args:
        points: Numpy array containing all points in the flightplan, it calculates the max
            damping distance for all these points in meters. These points
            must be given in an square coordinate system in meters. Must
            be X by 2 for X points.
    
    Returns:
        (float): Max damping distance allowed for these points in meters
    '''
    diff_xy = np.diff(points, axis = 0)
    # print('shape', diff_xy.shape)
    # print('test\n',np.sqrt(np.sum(diff_xy**2, axis = 1)))
    lengths = (np.sum(diff_xy**2, axis = 1))**0.5
    lengths_before = lengths[:-1]
    lengths_after = lengths[1:]
    max_allowed_lengths = np.zeros(len(points))
    minimum_wayline_lengths = np.minimum(lengths_before,lengths_after)
    # Minimum length of the drone on the wayline is set to 1 meter (0.5 on both sides)
    # With larger values this results in error in the DJI Pilot 2 app. 
    maximum_allowed_lengths_by_DJI_pilot2 = ((minimum_wayline_lengths)-0.5)/2 
    max_allowed_lengths[1:-1] = maximum_allowed_lengths_by_DJI_pilot2

    if max_setting != None:
        if max_setting >= 0.2:
            max_allowed_lengths[0] = 0.2
            max_allowed_lengths[-1] = 0.2
            max_chosen = np.ones(len(points))*max_setting
            max_chosen_and_allowed = np.minimum(max_allowed_lengths, max_chosen)
        
        else:
            raise ValueError(f'Max_setting must be larger than 0.2 m or None, but {max_setting} was given.')
    else:
        max_chosen_and_allowed = max_allowed_lengths
    return max_chosen_and_allowed

def rotation_matrix(rot_x_axis, rot_y_axis, rot_z_axis):
    omg_x = rot_x_axis 
    omg_y = rot_y_axis
    omg_z = rot_z_axis
    
    R1 = np.array(  [[1, 0, 0],
                    [0, np.cos(omg_x), -np.sin(omg_x)],
                    [0, np.sin(omg_x), np.cos(omg_x)]])
    R2 = np.array(  [[np.cos(omg_y), 0, np.sin(omg_y)],
                    [0, 1, 0],
                    [-np.sin(omg_y), 0, np.cos(omg_y)]])
    R3 = np.array(  [[np.cos(omg_z), -np.sin(omg_z), 0],
                    [np.sin(omg_z), np.cos(omg_z), 0],
                    [0, 0, 1]])
    rotation_matrix_3d = R3 @ R2 @ R1
    return rotation_matrix_3d

def similarity_transformation3d(source_coordinates, translate_coordinates, rot_x_axis, rot_y_axis, rot_z_axis, scale = 1):
    '''source_coordinates 3x1 matrix
    '''
    rotation_matrix_3d = rotation_matrix(rot_x_axis, rot_y_axis, rot_z_axis)
    transformed_coordinates = scale * rotation_matrix_3d @ source_coordinates + translate_coordinates
    return transformed_coordinates

def coordinated_turn_corners(x, y, damping_distances, z = None, amount = 2):
    ''' Calculates corner points for the coordinated turns option in the dji_kmz_creator to visualise the turn.'''
    # print('d',len(damping_distances))
    index_zero_angle = []
    index_zero_angle_coord = []
    # print(index_zero_angle)

    points = np.zeros((3,len(x)))
    points[0,:] = x
    points[1,:] = y
    points[2,:] = z

    # Translate all vectors to a value closer to zero
    translate_vector = np.array([
        [x[0]],
        [y[0]],
        [z[0]],
    ]
    )
    points = points - translate_vector

    # Vectors in direction from point
    L1 = np.fliplr(np.diff(np.fliplr(points)))[:,:-1]
    L2 = np.diff(points, axis=1)[:,1:]

    # unit vectors in direction from point
    L1_u = L1/np.linalg.norm(L1, axis = 0) 
    L2_u = L2/np.linalg.norm(L2, axis = 0)

    # plane vectors middle point
    plane_vectors = np.cross(L1_u, L2_u, axis=0)

    # rotations to new coordinate system in plane vector direction to z-direction original coordinate system
    # yz_vectors = plane_vectors[1:,:]
    # yz_vectors_norm = np.linalg.norm(yz_vectors, axis=0)
    # dot_product = plane_vectors[-1,:]
    # rots_x_axis2 = np.arccos(dot_product/(1*yz_vectors_norm))

    # Find rotation angles to new coordinate system
    rots_x_axis = np.arctan2(plane_vectors[1,:],plane_vectors[2,:])
    rots_y_axis = np.zeros(len(rots_x_axis))
    for i,rot_x_axis in enumerate(rots_x_axis):
        inbetween_vector = (rotation_matrix(rot_x_axis, 0, 0)@plane_vectors[:,i])[np.newaxis].T
        rots_y_axis[i] = -np.arctan2(inbetween_vector[0,0], inbetween_vector[2,0])

    turn_points_old_coords = np.zeros((3,amount+2,L1_u.shape[1]))

    # Rotate the 3d vectors in direction of 2 neigbouring waypoints to 2d vector in this plane. 
    for i in range(L1_u.shape[1]):
        rotation_matrix_used = rotation_matrix(rots_x_axis[i], rots_y_axis[i], 0)
        L1_u_new3d = rotation_matrix_used@(L1_u[:,i][np.newaxis].T)
        L2_u_new3d = rotation_matrix_used@(L2_u[:,i][np.newaxis].T)
        point_new3d = rotation_matrix_used@(points[:,i+1][np.newaxis].T)
        
        L1_u_new2d = L1_u_new3d[:2]
        L2_u_new2d = L2_u_new3d[:2]
        point_new2d = point_new3d[:2]

        dot_product = L1_u_new2d.T @ L2_u_new2d
        abs_total_angle = np.arccos(dot_product)

        # Find the coordinates of the middle point of the turn.
        middle_line_length = damping_distances[i+1]/np.cos(abs_total_angle/2)
        middle_line_direction = L1_u_new2d + L2_u_new2d
        # Check if direction to and from the waypoint are in the same direction or oposite
        # This will result in division by zero in these cases. (so angle 0 rad or pi rad)
        if np.linalg.norm(middle_line_direction) > 1e-14:
            middle_line_unit = middle_line_direction/np.linalg.norm(middle_line_direction) 
            vector_to_middle_point = middle_line_length*middle_line_unit
            middle_point_coord = point_new2d+vector_to_middle_point

            # Find radius of the turn
            turn_radius = np.tan(abs_total_angle/2)*damping_distances[i+1]

            # Find start and end angle of turn
            # First find start vector
            first_waypoint_coord = (point_new2d+L1_u_new2d*damping_distances[i+1])
            last_waypoint_coord = (point_new2d+L2_u_new2d*damping_distances[i+1])
            start_vector = first_waypoint_coord-middle_point_coord 
            end_vector = last_waypoint_coord-middle_point_coord 
            start_angle = np.arctan2(start_vector[1],start_vector[0])
            end_angle = np.arctan2(end_vector[1],end_vector[0])
            
            # Get points in 3d vector
            turn_points_new_coords = np.zeros((3,amount+2))
            turn_points_new_coords[:2,0] = first_waypoint_coord[:,0]
            turn_points_new_coords[:2,-1] = last_waypoint_coord[:,0]

            if amount != 0:
                # Total turn angle increments. Should always be smaller than pi rad.
                total_angle = -(start_angle-end_angle)
                if abs(total_angle) > np.pi:
                    total_angle = (2*np.pi - abs(total_angle))*-(total_angle/abs(total_angle)) #keeps the sign
                
                # Angle increments based on amount 
                turn_angle_inc = total_angle/amount
                for j in range(amount):
                    turn_points_new_coords[0,j+1] = np.cos((turn_angle_inc*j)+start_angle)*turn_radius+middle_point_coord[0,0]
                    turn_points_new_coords[1,j+1] = np.sin((turn_angle_inc*j)+start_angle)*turn_radius+middle_point_coord[1,0]

            # Get points in old coordinate system
            for k in range(turn_points_new_coords.shape[1]):
                # print(rotation_matrix_used.T@(turn_points_new_coords[:,k][np.newaxis].T)[:,0])
                turn_points_old_coords[:,k,i] = rotation_matrix_used.T@(turn_points_new_coords[:,k][np.newaxis].T)[:,0] #+translate_vector[:,0]

        else:
            # Add indexes where this is middle line has 0 length
            index_zero_angle.append(i)
            # Find start and end coordinate in case of 0 or 180 degree turn
            start_coord = points[:,i][np.newaxis].T + L1_u[:,i][np.newaxis].T*damping_distances[i+1]
            end_coord = points[:,i][np.newaxis].T + L2_u[:,i][np.newaxis].T*damping_distances[i+1]
            start_end_coord_tuple = (start_coord,end_coord)
            index_zero_angle_coord.append(start_end_coord_tuple)


    new_x = [points[0,0]]
    new_y = [points[1,0]]
    new_z = [points[2,0]]
    
    # Add turn waypoints to the original waypoints at the start and end
    for i in range(L1_u.shape[1]):
        new_x.extend(turn_points_old_coords[0,:,i])
        new_y.extend(turn_points_old_coords[1,:,i])
        new_z.extend(turn_points_old_coords[2,:,i])

    # Add start and end coordinates for 0 or 180 degree turn
    if len(index_zero_angle) != 0:
        for i in range(len(index_zero_angle)):
            # work backwards
            i = len(index_zero_angle)-i-1
            index = 1 + ((amount+2)*index_zero_angle[i]) 

            # Remove empty coordinates
            del new_x[index:index+(amount+2)]
            del new_y[index:index+(amount+2)]
            del new_z[index:index+(amount+2)]
            # Insert first coordinate
            new_x.insert(index,index_zero_angle_coord[i][0][0,0])
            new_y.insert(index,index_zero_angle_coord[i][0][1,0])
            new_z.insert(index,index_zero_angle_coord[i][0][2,0])            
            # Insert second coordinate
            new_x.insert(index+1,index_zero_angle_coord[i][1][0,0])
            new_y.insert(index+1,index_zero_angle_coord[i][1][1,0])
            new_z.insert(index+1,index_zero_angle_coord[i][1][2,0])

    new_x.append(points[0,-1])
    new_y.append(points[1,-1])
    new_z.append(points[2,-1])

    # print(new_x)
    # translate the coordinates back
    new_x = np.asarray(new_x) + translate_vector[0]
    new_y = np.asarray(new_y) + translate_vector[1]
    new_z = np.asarray(new_z) + translate_vector[2]

    return new_x, new_y, new_z
        # print('d', abs_total_angle, middle_point_coord)


    # rots_y_axis = np.arctan2(inbetween_vectors[0,:],inbetween_vectors[2,:])
    # test_vec2 = (rotation_matrix(0, -rots_y_axis, 0)@inbetween_vectors)
    # print('testing', test_vec2)
    # xz_vectors = plane_vectors[::2,:]
    # xz_vectors_norm = np.linalg.norm(xz_vectors, axis=0)
    # dot_product = plane_vectors[-1,:]
    # rots_y_axis2 = np.arccos(dot_product/(1*xz_vectors_norm))
    # rots_y_axis = np.arctan2(xz_vectors[0,:],xz_vectors[1,:])

    # translate_array = np.array([[0],[0],[0]])
    # # Rotate vectors to new coordinate system
    # for i,(rot_x_axis, rot_y_axis) in enumerate(zip(rots_x_axis, rots_y_axis)):
    #     point = points[:,i][np.newaxis].T

    #     test_vec = (rotation_matrix(rot_x_axis, 0, 0)@plane_vectors[:,i][np.newaxis].T)
    #     rot_y_axis = np.arctan2(test_vec[0,0],test_vec[2,0])
    #     test_vec2 = (rotation_matrix(0, -rot_y_axis, 0)@test_vec)
    #     print('ok__________________\n', test_vec2)

        # print('should only contain z\n', rotation_matrix(0, rot_y_axis, 0)@plane_vectors[:,i][np.newaxis].T)
        # r = R.from_euler('xy', [0,-rot_y_axis])#
        # r2 = R.from_euler('xy', [rot_x_axis,0])
        # tt = r.apply(plane_vectors[:,i])
        # tt2 = r2.apply(plane_vectors[:,i])
        # print('kkklll',tt,tt2)
        # print('test',point,test_point)


        

    # print(x)



def add_imu_movement_yellowscan(middle_coordinate:np.array, rotation):
    middle_coordinate
    rotation

    pass

if __name__ == "__main__":
    points = [
        [0,2],
        [1,4],
        [2,6],
        [6,10],
        [7,15],
    ]

    max_lengths = find_all_max_waypointTurnDampingDists(points)
    # print(max_lengths)

    # x = np.array((2,3,4,6,7,5,3,2))
    # y = np.array((2,3,7,6,9,5,4,4))
    # z = np.array((6,1,9,4,7,3,2,4))
    # d = np.array((0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5))
    # x = np.array((0,0,4,4))
    # y = np.array((-2,1,1,3))
    x = np.array((2,2,2,2))
    y = np.array((0,1,2,-5))

    z = np.array((0,0,0,0))
    d = np.array((0.5,0.5,0.5,0.5))
    nx,ny,nz = coordinated_turn_corners(x, y, d, z = z, amount=2)
    print('ny',ny)
    # to check norm np.sqrt((new_x[1]-x[1])**2+(new_y[1]-y[1])**2+(new_z[1]-z[1])**2)


