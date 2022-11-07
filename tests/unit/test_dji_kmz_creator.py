
# setting path for flightplanner
import sys
sys.path.append('./flightplanner')
import dji_kmz_creator as kmz

import unittest
import xml.etree.ElementTree as ET

class TestDJIKMZCreator(unittest.TestCase):

    def test_Template_kml_file_only_coordinates(self):
        # Setup variables
        basic_point1 = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        basic_point1_xml = basic_point1.build_waypoint_kml()
        basic_point2 = kmz.dji_waypoint_mission(1, 4.233, 52.00)
        basic_point2_xml = basic_point2.build_waypoint_kml()
        points = [basic_point1_xml, basic_point2_xml] 
        
        kmz_creator = kmz.dji_kmz(
            points,
            80,
            5,
            80,
        )
        kml_element = kmz_creator.build_xml()

        # self.assertEqual


if __name__ == '__main__':
    unittest.main()


    # point1 = dji_waypoint_mission(10, 4.233, 52.00)
    # point1.add_hover_action(5)
    # point1.add_yaw_action(-20)
    # point1_xml = point1.build_waypoint_kml()
    # point2 = dji_waypoint_mission(10, 4.233, 52.00)
    # point2.add_hover_action(22)
    # point2.add_yaw_action(-5)
    # point2_xml = point2.build_waypoint_kml()

    # test = dji_kmz(
    #     [point1_xml, point2_xml],
    #     80,
    #     5,
    #     80,
    # )

    # kml_element = test.build_kml()
    # waylines_wpml_element = test.build_waylines_wpml()
    # test.build_kmz("yess.kmz")
    # print('hoi')
    
    # string_xml = return_string(point1_xml)
    # print(string_xml)