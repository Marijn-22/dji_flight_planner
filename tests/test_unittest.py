
# setting path for flightplanner
import sys
sys.path.append('./flightplanner')
import dji_kmz_creator as kmz

import unittest
import xml.etree.ElementTree as ET

class TestDjiKmzCreator(unittest.TestCase):

    def setUp(self):
        # Set up the most basic kmz_creator (only coordinates)
        basic_point1 = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        basic_point1_xml = basic_point1.build_waypoint_xml()
        basic_point2 = kmz.dji_waypoint_mission(1, 4.233, 52.00)
        basic_point2_xml = basic_point2.build_waypoint_xml()
        points = [basic_point1_xml, basic_point2_xml] 
        
        self.basic_kmz_creator = kmz.dji_kmz(
            points,
            80,
            5,
            80,
        )
        self.kml_element = self.basic_kmz_creator.build_kml()
        self.waylines_wpml = self.basic_kmz_creator.build_waylines_wpml()


        ### Options for waylineCoordinateSysParam ###
        self.options_coordinateMode = [
            'WGS84',
        ]
        self.options_heightMode = [
            'EGM96',
            'relativeToStartPoint',
            'aboveGroundLevel',
        ]


    def test_Template_kml_basic_outer(self):
        ''' Test the outer xml elements of the template kml file, only things that are changed by this function are checked.'''
        kml_element = self.kml_element
        self.assertEqual(kml_element.tag,'kml')
        
        # Check if correct keys are in the outer xml and order is correct
        required_kml_keys = ['xmlns', 'xmlns:wpml']
        required_kml_attribs = [
            "http://www.opengis.net/kml/2.2",
            "http://www.dji.com/wpmz/1.0.0",
        ]
        kml_keys = kml_element.keys()
        self.assertEqual(len(kml_keys), 2)
        for kml_key, required_kml_attrib in zip(kml_keys, required_kml_attribs):
            self.assertIn(kml_key, required_kml_keys)
            kml_attrib = kml_element.get(kml_key)
            self.assertEqual(kml_attrib,required_kml_attrib)

        # Check if the second element is only "Document"
        document = kml_element[0]
        self.assertEqual(len(list(kml_element)),1) # So 1 child
        self.assertEqual(document.tag,"Document")

        # Check if Document contains wpml:author, wpml:createTime, wpml:updateTime,wpml:missionConfig,Folder 
        # and in this order.
        required_elements_in_Document = [
            'wpml:author',
            'wpml:createTime',
            'wpml:updateTime',
            'wpml:missionConfig',
            'Folder',
        ]
        for element, required_element_tag in zip(document, required_elements_in_Document):
            self.assertEqual(element.tag, required_element_tag)

        # Check file creation information
        self.assertEqual(type(document[0].text), str)
        self.assertEqual(type(int(document[1].text)), int)
        self.assertEqual(type(int(document[2].text)), int)

        # Mission config is added by other function

        # Check folder and template
        folder = document.find('Folder')
        required_waypoint_template_elements = [
            'wpml:templateType',
            'wpml:templateId',
            'wpml:autoFlightSpeed',
            'wpml:waylineCoordinateSysParam',
        ]
        # Check if it contains all the required params
        for i, element in enumerate(required_waypoint_template_elements):
            self.assertEqual(element, required_waypoint_template_elements[i])
        # Check if the required params are in the correct format
        option_templateType = folder.find('wpml:templateType').text
        self.assertEqual(option_templateType, 'waypoint')
        option_templateId = folder.find('wpml:templateId')
        option_templateId = int(option_templateId.text)
        self.assertTrue((0 <= option_templateId) and (option_templateId <= 65535))
        option_autoFlightSpeed = folder.find('wpml:autoFlightSpeed')
        option_autoFlightSpeed = float(option_autoFlightSpeed.text)
        self.assertTrue(option_autoFlightSpeed > 0)

        # check waylineCoordinateSysParam
        # Check if it contains all the required params
        waylineCoordinateSysParam = folder.find('wpml:waylineCoordinateSysParam')
        required_waylineCoordinateSysParam_elements = [
            'wpml:coordinateMode',
            'wpml:heightMode',
        ]
        # Check if all elements are required
        for element in waylineCoordinateSysParam:
            self.assertIn(element.tag, required_waylineCoordinateSysParam_elements)

        # Check if waylineCoordinateSysParam options are valid
        option_coordinateMode = waylineCoordinateSysParam.find('wpml:coordinateMode').text
        option_heightMode = waylineCoordinateSysParam.find('wpml:heightMode').text
        self.assertIn(option_coordinateMode, self.options_coordinateMode)
        self.assertIn(option_heightMode, self.options_heightMode)


    def test_Template_kml_basic_missionConfig(self):
        ''' Test the mission config element from the basic Template kml'''
        
        options_flyToWaylineMode = [
            'safely',
            'pointToPoint',
        ]
        options_finishAction = [
            'goHome',
            'autoLand',
            'gotoFirstWaypoint',
        ]
        options_exitOnRCLost = [
            'goContinue',
            'executeLostAction',            
        ]

        options_executeRCLostAction = [
            'goBack',
            'landing',
            'hover',
        ]

        options_droneEnumValue = [
            60,
            67,
            77,
        ]

        options_droneSubEnumValue = [
            0,
            1,
        ]

        ######## ADD globalTransitionalSpeed must be larger than 1
        mission_config = self.kml_element[0].find('wpml:missionConfig')
        # Check mission_config tag
        self.assertEqual(mission_config.tag, 'wpml:missionConfig')

        # Check if all required elements are there, the order is correct and not more elements are there. 
        required_mission_config_elements = [
            'wpml:flyToWaylineMode',
            'wpml:finishAction',
            'wpml:exitOnRCLost',
            'wpml:executeRCLostAction',
            'wpml:takeOffSecurityHeight',
            'wpml:globalTransitionalSpeed',
            'wpml:droneInfo',
        ]

        for element, required_element in zip(mission_config, required_mission_config_elements):
            self.assertEqual(element.tag, required_element)
        
        # Check if the parameters of the config_elements are valid
        option_flyToWaylineMode = mission_config.find('wpml:flyToWaylineMode').text
        self.assertIn(option_flyToWaylineMode, options_flyToWaylineMode)
        option_finishAction = mission_config.find('wpml:finishAction').text
        self.assertIn(option_finishAction, options_finishAction)
        option_exitOnRCLost = mission_config.find('wpml:exitOnRCLost').text
        self.assertIn(option_exitOnRCLost, options_exitOnRCLost)
        option_executeRCLostAction = mission_config.find('wpml:executeRCLostAction').text
        self.assertIn(option_executeRCLostAction, options_executeRCLostAction)
        # Must be between [1.5, 1500]
        option_takeOffSecurityHeight = mission_config.find('wpml:takeOffSecurityHeight').text
        option_takeOffSecurityHeight = float(option_takeOffSecurityHeight)
        self.assertTrue((option_takeOffSecurityHeight >= 1.5) and (option_takeOffSecurityHeight <= 1500))
        # Must be larger than 0
        option_globalTransitionalSpeed = mission_config.find('wpml:globalTransitionalSpeed').text
        self.assertTrue(float(option_globalTransitionalSpeed)>0)

        # Check droneInfo
        droneEnumValue = mission_config.find('wpml:droneInfo')[0]
        option_droneEnumValue = int(droneEnumValue.text)
        if len(list(droneEnumValue)) == 1:
            # if droneEnumValue is 67 the param droneSubEnumValue is required
            self.assertFalse(option_droneEnumValue, 67)
            self.assertIn(option_droneEnumValue, options_droneEnumValue)

        if len(list(droneEnumValue)) == 2:
            droneSubEnumValue = mission_config.find('wpml:droneInfo')[1]
            option_droneSubEnumValue = int(droneSubEnumValue)
            self.assertIn(option_droneEnumValue, options_droneEnumValue)
            self.assertIn(option_droneSubEnumValue, options_droneSubEnumValue)

    def test_build_waylines_wpml(self):
        ''' Tests the build_waylines_wpml function'''
        waylines_wpml = self.waylines_wpml
        self.assertEqual(waylines_wpml.tag,'kml')
        # Check waylines_wpml has 1 child
        self.assertEqual(len(waylines_wpml),1)
        
        # Check if correct keys are in the outer xml and order is correct
        required_kml_keys = ['xmlns', 'xmlns:wpml']
        required_kml_attribs = [
            "http://www.opengis.net/kml/2.2",
            "http://www.dji.com/wpmz/1.0.0",
        ]
        kml_keys = waylines_wpml.keys()
        self.assertEqual(len(kml_keys), 2)
        for kml_key, required_kml_attrib in zip(kml_keys, required_kml_attribs):
            self.assertIn(kml_key, required_kml_keys)
            kml_attrib = waylines_wpml.get(kml_key)
            self.assertEqual(kml_attrib,required_kml_attrib)
        
        # Check document has 1 child and is called Document itself
        document = waylines_wpml[0]
        self.assertEqual(len(document),1)
        self.assertEqual(document.tag,"Document")


if __name__ == '__main__':
    unittest.main()
