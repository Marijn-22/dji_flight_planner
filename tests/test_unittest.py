# This file performs unittests for the dji_kmz_creator_functions
# setting path for flightplanner
import sys
sys.path.append('./flightplanner')
import dji_kmz_creator as kmz
import unittest
import xml.etree.ElementTree as ET

#######possible to check still
# - input types  __init__
# - do keywords work
# - check rules when valid function
# - integrated test
# - check templateId is the same for kml and wpml

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
        self.list_of_points = self.basic_kmz_creator.waypoints_elements

        ### Options for waylineCoordinateSysParam ###
        self.options_coordinateMode = [
            'WGS84',
        ]
        self.options_heightMode = [
            'EGM96',
            'relativeToStartPoint',
            'aboveGroundLevel',
        ]


    def test_build_kml(self):
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
        for i in range(len(required_waypoint_template_elements)):
            self.assertEqual(folder[i].tag, required_waypoint_template_elements[i])

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


    def test_build_mission_config(self):
        ''' Test the build_mission_config function from the dji_kmz class. This
        function is both used by the build_kml and build_waylines function'''
        
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
        mission_config = self.basic_kmz_creator.build_mission_config()
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

    def test_build_waypoint_template(self):
        ''' Tests the function build_waypoint_template'''
        waypoint_template_list = self.basic_kmz_creator.build_waypoint_template()
        # Check if output is indeed a list
        self.assertEqual(type(waypoint_template_list), list)

        # The amount of placemarks depends on the amount of waypoints in the data.
        # Do not move this parameter to global as values can be removed.
        required_list_of_elements = [
            'wpml:globalWaypointTurnMode',
            'wpml:globalUseStraightLine',
            'wpml:gimbalPitchMode',
            'wpml:ellipsoidHeight',
            'wpml:height',
            'wpml:globalWaypointHeadingParam',
            'Placemark',
        ]
        # Check if wpml:globalUseStraightLine is required and used
        if (waypoint_template_list[0].text != 'toPointAndStopWithContinuityCurvature') or (waypoint_template_list[0].text != 'toPointAndPassWithContinuityCurvature'):
            if waypoint_template_list[1].tag != 'wpml:globalUseStraightLine':
                # In this line the amount of placemarks is checked
                self.assertEqual(len(waypoint_template_list), len(self.list_of_points)+6)
                # In this line the not required and used element is removed from the list
                required_list_of_elements.pop(1)
            else:
                # In this line the amount of placemarks is checked
                self.assertEqual(len(waypoint_template_list), len(self.list_of_points)+6)
        else:
            # In this line the amount of placemarks is checked
            self.assertEqual(len(waypoint_template_list), len(self.list_of_points)+6)

        # Check all required elements in the list.
        for i in range(len(required_list_of_elements)):
            self.assertEqual(waypoint_template_list[i].tag, required_list_of_elements[i])

        # Check if length of the placemarks is correct.


    def test_build_globalWaypointHeadingParam(self):
        options_waypointHeadingMode = [
            'followWayline',
            'manually',
            'fixed',
            'smoothTransition'
        ]
        options_waypointHeadingYawPathMode = [
            'clockwise',
            'counterClockwise',
            'followBadArc',
        ]
        # Do not move this parameter to global self element in this class as values can be removed.
        required_globalWaypointHeadingParam_elements = [
            'wpml:waypointHeadingMode',
            'wpml:waypointHeadingAngle',
            'wpml:waypointHeadingYawPathMode',
        ]


        globalWaypointHeadingParam = self.basic_kmz_creator.build_globalWaypointHeadingParam()

        # Check if wpml:waypointHeadingAngle is required and used
        if (globalWaypointHeadingParam[0].text !='smoothTransition'): 
            if globalWaypointHeadingParam[1].tag != 'wpml:waypointHeadingAngle':
                required_globalWaypointHeadingParam_elements.pop(1)

        # Check if tags are valid
        for i in range(len(globalWaypointHeadingParam)):
            self.assertEqual(globalWaypointHeadingParam[i].tag, required_globalWaypointHeadingParam_elements[i])

        # Check if values are valid
        option_waypointHeadingMode = globalWaypointHeadingParam.find('wpml:waypointHeadingMode').text
        self.assertIn(option_waypointHeadingMode, options_waypointHeadingMode)
        option_waypointHeadingYawPathMode = globalWaypointHeadingParam.find('wpml:waypointHeadingYawPathMode').text
        self.assertIn(option_waypointHeadingYawPathMode, options_waypointHeadingYawPathMode)
        # Check if input is integer
        if globalWaypointHeadingParam[1].tag == 'wpml:waypointHeadingAngle':
            value = int(globalWaypointHeadingParam[1].text)
            self.assertEqual(type(value), int)


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
        
        # Check document has 2 childs and is called Document itself
        document = waylines_wpml[0]
        self.assertEqual(len(document),2)
        self.assertEqual(document.tag,"Document")

        # Check if Document has all the required elements in the correct order
        required_elements_in_Document = [
            'wpml:missionConfig',
            'Folder',
        ]
        # Check if it contains all the required tags
        for element, req_element_txt in zip(document, required_elements_in_Document):
            self.assertEqual(element.tag, req_element_txt)

        # Mission config is added by other function that is already checked. 

        # Check Waylines Information in Folder element
        required_elements_in_Folder = [
            'wpml:templateId',
            'wpml:waylineId',
            'wpml:autoFlightSpeed',
            'wpml:executeHeightMode',
            'Placemark',
        ]

        # Check if the tags are correct
        folder = document.find('Folder')
        for i in range(len(required_elements_in_Folder)):
            self.assertEqual(folder[i].tag, required_elements_in_Folder[i])
        # Check if all excess elements are placemarks
        placemarks_elements = folder[4:]
        for placemark_element in placemarks_elements:
            self.assertEqual(placemark_element.tag, 'Placemark')

        # Check length of placemarks is correct
        self.assertEqual(len(folder[4:]),len(self.list_of_points))

        # Check if the parameters of the tags are correct
        # Check type and value of templateId
        templateId_element = folder.find('wpml:templateId')
        templateId = int(templateId_element.text)
        self.assertEqual(templateId,self.basic_kmz_creator.templateId)
        
        waylineId_element = folder.find('wpml:waylineId')
        waylineId = int(waylineId_element.text)
        self.assertTrue(waylineId >= 0)
        self.assertTrue(waylineId <= 65535)
        self.assertEqual(type(waylineId),int)

        # Is float that must be larger than 0 
        autoFlightSpeed_element = folder.find('wpml:autoFlightSpeed')
        autoFlightSpeed = float(autoFlightSpeed_element.text)
        self.assertTrue(autoFlightSpeed >=0)

        options_executeHeightMode = [
            'WGS84',
            'relativeToStartPoint',
        ]
        executeHeightMode_element = folder.find('wpml:executeHeightMode')
        option_executeHeightMode = executeHeightMode_element.text
        self.assertIn(option_executeHeightMode, options_executeHeightMode)


class TestDjiWaypointMission(unittest.TestCase):
    def setUp(self):
        # Set up a DjiWaypointMission with no actions
        self.basic_point1 = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        self.basic_point1_xml_no_action = self.basic_point1.build_waypoint_xml()
        # Set up a DjiWaypointMission with one action
        self.basic_point2 = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        self.basic_point2.add_yaw_action(-20)
        self.basic_point2_xml_one_action = self.basic_point2.build_waypoint_xml()
        # Set up a DjiWaypointMission with three actions
        self.basic_point3 = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        self.basic_point3.add_hover_action(5)
        self.basic_point3.add_yaw_action(-180)
        self.basic_point3.add_yaw_action(180)
        self.basic_point3_xml_actions = self.basic_point3.build_waypoint_xml()

    def test_add_yaw_action(self):
        basic_point = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        basic_point.add_yaw_action(120, aircraftPathMode = 'counterClockwise')
        output_list = basic_point.action_param_list
        action1 = output_list[0]
        self.assertEqual(action1[0], 'rotateYaw')
        # Check id
        self.assertEqual(action1[1], 0)
        # Check aircraftHeading
        self.assertEqual(action1[2], 120)
        # Check aircraftPathMode
        self.assertEqual(action1[3], 'counterClockwise')

    def test_add_hover_action(self):
        basic_point = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        basic_point.add_hover_action(7)
        output_list = basic_point.action_param_list
        action1 = output_list[0]
        self.assertEqual(action1[0], 'hover')
        # Check id
        self.assertEqual(action1[1], 0)
        # Check hovertime
        self.assertEqual(action1[2], 7)

    def test_kml_actions(self):
        # Test kml_actions function. It tests the test_yaw_action_kml and hover_action_xml directly as well.
        basic_point = kmz.dji_waypoint_mission(0, 4.233, 52.00)
        basic_point.action_param_list = [['hover', 0, 5], ['rotateYaw', 1, -180, 'clockwise'], ['rotateYaw', 2, 180, 'clockwise']]
        test_kml_actions_output = basic_point.kml_actions()

        self.assertEqual(len(test_kml_actions_output), len(basic_point.action_param_list))

        options_actionActuatorFunc = [
            'hover',
            'rotateYaw',
        ]
        options_aircraftPathMode = [
            'clockwise',
            'counterClockwise',
        ]

        for i, action in enumerate(test_kml_actions_output):
            self.assertEqual(action.tag, 'wpml:action')
            # Test actionId
            self.assertEqual(action[0].tag, 'wpml:actionId')
            actionId = int(action[0].text)
            self.assertEqual(actionId, i)
            # Test actionActuatorFunc
            self.assertEqual(action[1].tag, 'wpml:actionActuatorFunc')
            self.assertIn(action[1].text, options_actionActuatorFunc)
            # Test actionActuatorFuncParam
            self.assertEqual(action[2].tag, 'wpml:actionActuatorFuncParam')
            params = action[2]
            if action[1].tag == 'hover':
                self.assertEqual(len(params), 1)
                self.assertEqual(params[0].tag, 'wpml:hoverTime')
                self.assertEqual(type(float(params[0].text)), float)
            elif action[1].tag == 'rotateYaw':
                self.assertEqual(len(params), 2)
                # Check first parameter
                self.assertEqual(params[0].tag, 'wpml:aircraftHeading')
                self.assertEqual(type(float(params[0].text)), float)
                self.assertTrue(float(params[0].text) >= -180)
                self.assertTrue(float(params[0].text) <= 180)
                # Check second parameter
                self.assertEqual(params[1].tag, 'wpml:aircraftPathMode')
                self.assertIn(params[1].text, options_aircraftPathMode)

    def test_build_waypointHeadingParam(self):
        # Get the output of the function to test.
        local_waypointHeadingParam = self.basic_point1.build_waypointHeadingParam() 
        # Check if the tag is as expected.
        self.assertEqual(local_waypointHeadingParam.tag, 'wpml:waypointHeadingParam')
        
        required_local_waypointHeadingParam_elements = [
            'wpml:waypointHeadingMode',
            'wpml:waypointHeadingAngle',
            'wpml:waypointHeadingYawPathMode',
        ]

        options_waypointHeadingMode = [
            'followWayline',
            'manually',
            'fixed',
            'smoothTransition'
        ]

        options_waypointHeadingYawPathMode = [
            'clockwise',
            'counterClockwise',
            'followBadArc',
        ]

        # Check if wpml:waypointHeadingAngle is required and used
        if (local_waypointHeadingParam[0].text !='smoothTransition'): 
            if local_waypointHeadingParam[1].tag != 'wpml:waypointHeadingAngle':
                required_local_waypointHeadingParam_elements.pop(1)

        # Check if tags are valid
        for i in range(len(local_waypointHeadingParam)):
            self.assertEqual(local_waypointHeadingParam[i].tag, required_local_waypointHeadingParam_elements[i])

        # Check if values are valid
        option_waypointHeadingMode = local_waypointHeadingParam.find('wpml:waypointHeadingMode').text
        self.assertIn(option_waypointHeadingMode, options_waypointHeadingMode)
        option_waypointHeadingYawPathMode = local_waypointHeadingParam.find('wpml:waypointHeadingYawPathMode').text
        self.assertIn(option_waypointHeadingYawPathMode, options_waypointHeadingYawPathMode)
        # Check if input is integer
        if local_waypointHeadingParam[1].tag == 'wpml:waypointHeadingAngle':
            value = int(local_waypointHeadingParam[1].text)
            self.assertEqual(type(value), int)

    def test_build_waypointTurnParam(self):
        # It was chosen to always have a damping distance of a very 
        # small value of standard 0.1 meter. Therefore it is considered
        # as always required. When used by the code the distance between 
        # waypoints should then be larger than 0.1 meters. Check the 
        # documentation when the parameter is actually used by the drone 
        # and how it should be used.
        tags_waypointTurnParam = [
            'wpml:waypointTurnMode',
            'wpml:waypointTurnDampingDist',
        ]
        # Get the output of the function that is tested.
        output_waypointTurnParam = self.basic_point1.build_waypointTurnParam()

        # Check if the tag of the xml element is as expected
        self.assertEqual(output_waypointTurnParam.tag, 'wpml:waypointTurnParam')

        # Check if the elements in the output are as expected
        for element, req_element in zip(output_waypointTurnParam, tags_waypointTurnParam):
            self.assertEqual(element.tag, req_element)

        options_waypointTurnMode = [
            'coordinateTurn',
            'toPointAndStopWithDiscontinuityCurvature',
            'toPointAndStopWithContinuityCurvature',
            'toPointAndPassWithContinuityCurvature',
        ]
        # Check if waypointTurnMode is given a valid option
        turn_mode_txt = output_waypointTurnParam.find('wpml:waypointTurnMode').text
        self.assertIn(turn_mode_txt, options_waypointTurnMode)

        # Check if wpml:waypointTurnDampingDist is a float
        waypointTurnDampingDist_float = float(output_waypointTurnParam.find('wpml:waypointTurnDampingDist').text)
        self.assertEqual(type(waypointTurnDampingDist_float), float)

    def test_build_action_group_with_action(self):
        # check build_action_group for one action.
        required_elements_action_group = [
            'wpml:actionGroupId',
            'wpml:actionGroupStartIndex',
            'wpml:actionGroupEndIndex',
            'wpml:actionGroupMode',
            'wpml:actionTrigger',
            'wpml:action',
        ]

        test_one_action_group = self.basic_point2.build_action_group()
        # Check the tag of the parameter
        self.assertEqual(test_one_action_group.tag, 'wpml:actionGroup')

        # Test the elements tags in the output of the function
        for element, req_element in zip(test_one_action_group,required_elements_action_group):
            self.assertEqual(element.tag, req_element)

        # Test the value types of the first three parameters (should all be of the same type)
        for i in range(3):
            value = int(test_one_action_group[i].text)
            self.assertEqual(type(value), int)
            self.assertTrue(value >= 0)
            self.assertTrue(value <+65535)

        # Test actionGroupMode parameter
        options_actionGroupMode = [
            'sequence'
        ]
        test_actionGroupMode = test_one_action_group.find('wpml:actionGroupMode')
        self.assertIn(test_actionGroupMode.text, options_actionGroupMode)

        # Test actionTrigger
        test_actionTrigger = test_one_action_group.find('wpml:actionTrigger') #Tag is checked directly
        required_elements_action_trigger = [
            'wpml:actionTriggerType',
        ]
        # Check the elements tags in actionTrigger
        for element, req_element in zip(test_actionTrigger, required_elements_action_trigger):
            self.assertEqual(element.tag, req_element)
        # Test actionTriggerType for only option supported by this program yet.
        options_actionTriggerType = [
            'reachPoint'
        ]
        test_actionTriggerType = test_actionTrigger.find('wpml:actionTriggerType')
        self.assertIn(test_actionTriggerType.text, options_actionTriggerType)

    def test_build_action_group_with_actions(self):
        # check build_action_group for 3 actions.
        required_elements_action_group = [
            'wpml:actionGroupId',
            'wpml:actionGroupStartIndex',
            'wpml:actionGroupEndIndex',
            'wpml:actionGroupMode',
            'wpml:actionTrigger',
            'wpml:action',
            'wpml:action',
            'wpml:action',
        ]

        test_actions_group = self.basic_point3.build_action_group()
        # Check the tag of the parameter
        self.assertEqual(test_actions_group.tag, 'wpml:actionGroup')

        # Test the elements tags in the output of the function
        for element, req_element in zip(test_actions_group,required_elements_action_group):
            self.assertEqual(element.tag, req_element)

        # Test the value types of the first three parameters (should all be of the same type)
        for i in range(3):
            value = int(test_actions_group[i].text)
            self.assertEqual(type(value), int)
            self.assertTrue(value >= 0)
            self.assertTrue(value <+65535)

        # Test actionGroupMode parameter
        options_actionGroupMode = [
            'sequence'
        ]
        test_actionGroupMode = test_actions_group.find('wpml:actionGroupMode')
        self.assertIn(test_actionGroupMode.text, options_actionGroupMode)

        # Test actionTrigger
        test_actionTrigger = test_actions_group.find('wpml:actionTrigger') #Tag is checked directly
        required_elements_action_trigger = [
            'wpml:actionTriggerType',
        ]
        # Check the elements tags in actionTrigger
        for element, req_element in zip(test_actionTrigger, required_elements_action_trigger):
            self.assertEqual(element.tag, req_element)
        # Test actionTriggerType for only option supported by this program yet.
        options_actionTriggerType = [
            'reachPoint'
        ]
        test_actionTriggerType = test_actionTrigger.find('wpml:actionTriggerType')
        self.assertIn(test_actionTriggerType.text, options_actionTriggerType)

    def test_build_waypoint_xml_no_action(self):
        # Check the build_waypoint_xml function for no actions
        basic_p_no_action = self.basic_point1_xml_no_action
        # Check tag
        self.assertEqual(basic_p_no_action.tag,'Placemark')

        # Check point Information in Placemark element
        required_elements_in_Placemark = [ 
            'Point',
            'wpml:index',
            'wpml:useGlobalHeight',
            'wpml:ellipsoidHeight',
            'wpml:height',
            'wpml:executeHeight',
            'wpml:useGlobalSpeed',
            'wpml:waypointSpeed',
            'wpml:useGlobalHeadingParam',
            'wpml:waypointHeadingParam',
            'wpml:useGlobalTurnParam',
            'wpml:waypointTurnParam',
            'wpml:useStraightLine',
            'wpml:gimbalPitchAngle',
        ]
        # Check if the amount of elements is correct
        self.assertEqual(len(basic_p_no_action),len(required_elements_in_Placemark))
        # Check if all required elements are there
        for i in range(len(required_elements_in_Placemark)):
            self.assertEqual(basic_p_no_action[i].tag, required_elements_in_Placemark[i])
        
        # Check all values
        check_point = basic_p_no_action.find('Point')
        option_coordinates = check_point[0]
        self.assertEqual(option_coordinates.tag, 'coordinates')
        option_long, option_lat = option_coordinates.text.split(',')
        self.assertEqual(type(float(option_long)), float)
        self.assertEqual(type(float(option_lat)), float)

        check_index = basic_p_no_action.find('wpml:index')
        self.assertEqual(type(int(check_index.text)), int)
        self.assertTrue(int(check_index.text) >= 0)

        options_useGlobalHeight = [
            0,
            1,
        ]
        option_useGlobalHeight = basic_p_no_action.find('wpml:useGlobalHeight')
        self.assertIn(int(option_useGlobalHeight.text), options_useGlobalHeight)

        option_ellipsoidHeight = basic_p_no_action.find('wpml:ellipsoidHeight')
        self.assertEqual(type(float(option_ellipsoidHeight.text)), float)
        
        option_height = basic_p_no_action.find('wpml:height')
        self.assertEqual(type(float(option_height.text)), float)

        options_useGlobalSpeed = [
            0,
            1,
        ]
        option_useGlobalSpeed = basic_p_no_action.find('wpml:useGlobalSpeed')
        self.assertIn(int(option_useGlobalSpeed.text), options_useGlobalSpeed)

        option_waypointSpeed = basic_p_no_action.find('wpml:waypointSpeed')
        self.assertEqual(type(float(option_waypointSpeed.text)), float)

        options_useGlobalHeadingParam = [
            0,
            1,
        ]
        option_useGlobalHeadingParam = basic_p_no_action.find('wpml:useGlobalHeadingParam')
        self.assertIn(int(option_useGlobalHeadingParam.text), options_useGlobalHeadingParam)

        option_waypointHeadingParam = basic_p_no_action.find('wpml:waypointHeadingParam')
        self.assertEqual(type(option_waypointHeadingParam[0]), ET.Element)

        options_useGlobalTurnParam = [
            0,
            1,
        ]
        option_GlobalTurnParam = basic_p_no_action.find('wpml:useGlobalTurnParam')
        self.assertIn(int(option_GlobalTurnParam.text), options_useGlobalTurnParam)

        option_waypointTurnParam = basic_p_no_action.find('wpml:waypointTurnParam')
        self.assertEqual(type(option_waypointTurnParam[0]), ET.Element)

        options_useStraightLine = [
            0,
            1,
        ]
        option_useStraightLine = basic_p_no_action.find('wpml:useStraightLine')
        self.assertIn(int(option_useStraightLine.text), options_useStraightLine)

        option_gimbalPitchAngle = basic_p_no_action.find('wpml:gimbalPitchAngle')
        self.assertEqual(type(float(option_gimbalPitchAngle.text)), float)

    def test_build_waypoint_xml_with_action(self):
        # Check the build_waypoint_xml function for one action
        basic_one_action = self.basic_point2_xml_one_action
        # Check tag
        self.assertEqual(basic_one_action.tag,'Placemark')

        # Check point Information in Placemark element
        required_elements_in_Placemark = [
            'Point',
            'wpml:index',
            'wpml:useGlobalHeight',
            'wpml:ellipsoidHeight',
            'wpml:height',
            'wpml:executeHeight',
            'wpml:useGlobalSpeed',
            'wpml:waypointSpeed',
            'wpml:useGlobalHeadingParam',
            'wpml:waypointHeadingParam',
            'wpml:useGlobalTurnParam',
            'wpml:waypointTurnParam',
            'wpml:useStraightLine',
            'wpml:gimbalPitchAngle',
            'wpml:actionGroup',
        ]
        # Check if the amount of elements is correct
        self.assertEqual(len(basic_one_action),len(required_elements_in_Placemark))
        # Check if all required elements are there
        for i in range(len(required_elements_in_Placemark)):
            self.assertEqual(basic_one_action[i].tag, required_elements_in_Placemark[i])

        # Check all values
        check_point = basic_one_action.find('Point')
        option_coordinates = check_point[0]
        self.assertEqual(option_coordinates.tag, 'coordinates')
        option_long, option_lat = option_coordinates.text.split(',')
        self.assertEqual(type(float(option_long)), float)
        self.assertEqual(type(float(option_lat)), float)

        check_index = basic_one_action.find('wpml:index')
        self.assertEqual(type(int(check_index.text)), int)
        self.assertTrue(int(check_index.text) >= 0)

        options_useGlobalHeight = [
            0,
            1,
        ]
        option_useGlobalHeight = basic_one_action.find('wpml:useGlobalHeight')
        self.assertIn(int(option_useGlobalHeight.text), options_useGlobalHeight)

        option_ellipsoidHeight = basic_one_action.find('wpml:ellipsoidHeight')
        self.assertEqual(type(float(option_ellipsoidHeight.text)), float)
        
        option_height = basic_one_action.find('wpml:height')
        self.assertEqual(type(float(option_height.text)), float)

        options_useGlobalSpeed = [
            0,
            1,
        ]
        option_useGlobalSpeed = basic_one_action.find('wpml:useGlobalSpeed')
        self.assertIn(int(option_useGlobalSpeed.text), options_useGlobalSpeed)

        option_waypointSpeed = basic_one_action.find('wpml:waypointSpeed')
        self.assertEqual(type(float(option_waypointSpeed.text)), float)

        options_useGlobalHeadingParam = [
            0,
            1,
        ]
        option_useGlobalHeadingParam = basic_one_action.find('wpml:useGlobalHeadingParam')
        self.assertIn(int(option_useGlobalHeadingParam.text), options_useGlobalHeadingParam)

        option_waypointHeadingParam = basic_one_action.find('wpml:waypointHeadingParam')
        self.assertEqual(type(option_waypointHeadingParam[0]), ET.Element)

        options_useGlobalTurnParam = [
            0,
            1,
        ]
        option_GlobalTurnParam = basic_one_action.find('wpml:useGlobalTurnParam')
        self.assertIn(int(option_GlobalTurnParam.text), options_useGlobalTurnParam)

        option_waypointTurnParam = basic_one_action.find('wpml:waypointTurnParam')
        self.assertEqual(type(option_waypointTurnParam[0]), ET.Element)

        options_useStraightLine = [
            0,
            1,
        ]
        option_useStraightLine = basic_one_action.find('wpml:useStraightLine')
        self.assertIn(int(option_useStraightLine.text), options_useStraightLine)

        option_gimbalPitchAngle = basic_one_action.find('wpml:gimbalPitchAngle')
        self.assertEqual(type(float(option_gimbalPitchAngle.text)), float)
            
    def test_build_waypoint_xml_with_actions(self):
        # Check the build_waypoint_xml function for multiple (3 are tested) actions
        basic_actions = self.basic_point3_xml_actions
        # Check tag
        self.assertEqual(basic_actions.tag,'Placemark')

        # Check point Information in Placemark element
        required_elements_in_Placemark = [
            'Point',
            'wpml:index',
            'wpml:useGlobalHeight',
            'wpml:ellipsoidHeight',
            'wpml:height',
            'wpml:executeHeight',
            'wpml:useGlobalSpeed',
            'wpml:waypointSpeed',
            'wpml:useGlobalHeadingParam',
            'wpml:waypointHeadingParam',
            'wpml:useGlobalTurnParam',
            'wpml:waypointTurnParam',
            'wpml:useStraightLine',
            'wpml:gimbalPitchAngle',
            'wpml:actionGroup',
        ]
        # Check if the amount of elements is correct
        self.assertEqual(len(basic_actions),len(required_elements_in_Placemark))
        # Check if all required elements are there
        for i in range(len(required_elements_in_Placemark)):
            self.assertEqual(basic_actions[i].tag, required_elements_in_Placemark[i])

        # Check all values
        check_point = basic_actions.find('Point')
        option_coordinates = check_point[0]
        self.assertEqual(option_coordinates.tag, 'coordinates')
        option_long, option_lat = option_coordinates.text.split(',')
        self.assertEqual(type(float(option_long)), float)
        self.assertEqual(type(float(option_lat)), float)

        check_index = basic_actions.find('wpml:index')
        self.assertEqual(type(int(check_index.text)), int)
        self.assertTrue(int(check_index.text) >= 0)

        options_useGlobalHeight = [
            0,
            1,
        ]
        option_useGlobalHeight = basic_actions.find('wpml:useGlobalHeight')
        self.assertIn(int(option_useGlobalHeight.text), options_useGlobalHeight)

        option_ellipsoidHeight = basic_actions.find('wpml:ellipsoidHeight')
        self.assertEqual(type(float(option_ellipsoidHeight.text)), float)
        
        option_height = basic_actions.find('wpml:height')
        self.assertEqual(type(float(option_height.text)), float)

        options_useGlobalSpeed = [
            0,
            1,
        ]
        option_useGlobalSpeed = basic_actions.find('wpml:useGlobalSpeed')
        self.assertIn(int(option_useGlobalSpeed.text), options_useGlobalSpeed)

        option_waypointSpeed = basic_actions.find('wpml:waypointSpeed')
        self.assertEqual(type(float(option_waypointSpeed.text)), float)

        options_useGlobalHeadingParam = [
            0,
            1,
        ]
        option_useGlobalHeadingParam = basic_actions.find('wpml:useGlobalHeadingParam')
        self.assertIn(int(option_useGlobalHeadingParam.text), options_useGlobalHeadingParam)

        option_waypointHeadingParam = basic_actions.find('wpml:waypointHeadingParam')
        self.assertEqual(type(option_waypointHeadingParam[0]), ET.Element)

        options_useGlobalTurnParam = [
            0,
            1,
        ]
        option_GlobalTurnParam = basic_actions.find('wpml:useGlobalTurnParam')
        self.assertIn(int(option_GlobalTurnParam.text), options_useGlobalTurnParam)

        option_waypointTurnParam = basic_actions.find('wpml:waypointTurnParam')
        self.assertEqual(type(option_waypointTurnParam[0]), ET.Element)

        options_useStraightLine = [
            0,
            1,
        ]
        option_useStraightLine = basic_actions.find('wpml:useStraightLine')
        self.assertIn(int(option_useStraightLine.text), options_useStraightLine)

        option_gimbalPitchAngle = basic_actions.find('wpml:gimbalPitchAngle')
        self.assertEqual(type(float(option_gimbalPitchAngle.text)), float)


if __name__ == '__main__':
    unittest.main()

