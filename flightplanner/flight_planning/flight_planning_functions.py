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
    between the point of interest and the previous and next coordinate.

    Args:
        points: Numpy array containing all points in the flightplan, it calculates the max
            damping distance for all these points in meters. These points
            must be given in an square coordinate system in meters. Must
            be X by 2 for X points.
    
    Returns:
        (float): Max damping distance allowed for these points in meters
    '''
    diff_xy = np.diff(points, axis = 0)
    lengths = np.sum(diff_xy**2, axis = 1)
    lengths_before = lengths[:-1]
    lengths_after = lengths[1:]
    max_allowed_lengths = np.zeros(len(points))
    max_allowed_lengths[1:-1] = np.minimum(lengths_before,lengths_after)

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