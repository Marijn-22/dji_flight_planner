
# setting path for flightplanner
import sys
sys.path.append('./flightplanner')
import dji_kmz_creator

import unittest

class TestSum(unittest.TestCase):

    def test_1(self):
        point1 = dji_kmz_creator.dji_waypoint_mission(10, 4.233, 52.00)
        

if __name__ == '__main__':
    unittest.main()


