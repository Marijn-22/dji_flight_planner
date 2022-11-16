import numpy as np

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
    
#     Returns:
#         (float): Max damping distance allowed for this point in meters
#     '''
    

#     pass

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
    lengths = np.sqrt(np.sum(diff_xy**2, axis = 1))
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
                    [0, np.cos(omg_x), np.sin(omg_x)],
                    [0, -np.sin(omg_x), np.cos(omg_x)]])
    R2 = np.array(  [[np.cos(omg_y), 0, -np.sin(omg_y)],
                    [0, 1, 0],
                    [np.sin(omg_y), 0, np.cos(omg_y)]])
    R3 = np.array(  [[np.cos(omg_z), np.sin(omg_z), 0],
                    [-np.sin(omg_z), np.cos(omg_z), 0],
                    [0, 0, 1]])
    rotation_matrix_3d = R1 @ R2 @ R3
    return rotation_matrix_3d

def similarity_transformation3d(source_coordinates, translate_coordinates, rot_xaxis, rot_y_axis, rot_z_axis, scale = 1):
    '''source_coordinates 3x1 matrix
    '''
    rotation_matrix_3d = rotation_matrix(rot_xaxis, rot_y_axis, rot_z_axis)
    transformed_coordinates = scale * rotation_matrix_3d @ source_coordinates + translate_coordinates
    return transformed_coordinates

def coordinated_turn_corner(x, y, damping_distance, z = None):
    ''' Calculates corner points for the coordinated turns option in the dji_kmz_creator to visualise the turn.'''
    points = np.zeros((3,len(x)))
    points[0,:] = x
    points[1,:] = y
    points[2,:] = z

    # Vectors in direction from point
    L1 = np.flip(np.diff(np.flip(points, axis = 1), axis=1))[:,:-1]
    L2 = np.diff(points, axis=1)[:,1:]

    # unit vectors in direction from point
    L1_u = L1/np.linalg.norm(L1, axis = 0) 
    L2_u = L2/np.linalg.norm(L2, axis = 0)

    # plane vectors middle point
    plane_vectors = np.cross(L1_u, L2_u, axis=0)

    # rotations to new coordinate system in plane vector direction to z-direction original coordinate system
    yz_vectors = plane_vectors[1:,:]
    yz_vectors_norm = np.linalg.norm(yz_vectors, axis=0)
    dot_product = plane_vectors[-1,:]
    rots_x_axis = -np.arccos(dot_product/(1*yz_vectors_norm))

    xz_vectors = plane_vectors[::2,:]
    xz_vectors_norm = np.linalg.norm(xz_vectors, axis=0)
    dot_product = plane_vectors[-1,:]
    rots_y_axis = -np.arccos(dot_product/(1*xz_vectors_norm))

    for rot_x_axis, rot_y_axis in zip(rots_x_axis,rots_y_axis):
        pass

    print(x)

def coordinated_turn_corners():
    pass

def add_imu_movement():
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
    print(max_lengths)

    x = np.array((2,3,4,6,7,5,3,2))
    y = np.array((2,3,7,6,9,5,4,4))
    z = np.array((6,1,9,4,7,3,2,4))
    d = np.array((0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5))
    coordinated_turn_corner(x, y, d, z = z)


