# Code to create a waypoint mission for the DJI Matrice 300 RTK. It can be used to create an
# KMZ file, which can be loaded into the DJI Pilot 2 application.
#
# The documentation that was used can be found at the following website:
# https://developer.dji.com/doc/cloud-api-tutorial/en/specification/dji-wpml/template-kml.html

import time
import xml.etree.ElementTree as ET
import shutil
import os
import tempfile
import numpy as np
import copy

from pyproj import Transformer

def export_formated_xml(ET_xml: ET.Element, filename = 'test.xml'):
    ''' Export the input ET_xml parameter to a xml file. The name of the file 
    is given by the filename.'''
    tree = ET.ElementTree(ET_xml)
    ET.indent(tree, space="\t", level=0)
    tree.write(filename, encoding="utf-8", xml_declaration=True)


def return_string(ET_element: ET.Element, encoding = 'unicode'):
    ''' Return a string of the ET_element parameter.'''
    # encoding can also be 'utf8'
    ET_string = ET.tostring(ET_element, encoding=encoding , method='xml')
    return ET_string


class dji_waypoint_mission():
    ''' For now only hover and yaw actions are supported.

    First the mission should be initialised by giving the __init__ equations. For each point the 
    'add_yaw_action' and the 'add_hover_action' can be used to perform actions at a certain 
    waypoint. The actions will be performed in the order the actions are added. The self.action_param_list
    parameter can be used as quicker method to give the actions that should be performed. 

    The build_action_group can be used to return a action group xml element. This requires added 
    actions.

    The build_waypoint_xml can be used to return a waypoint xml element with included action group element
    '''
    def __init__(
        self, 
        point_id: int, 
        longitude: float, 
        latitude: float, 
        height: float = 100, 
        executeHeight: float = 100,
        useGlobalHeight: int = 1,
        ellipsoidHeight: float = 100,
        useGlobalSpeed: int = 1, 
        waypointSpeed: float = 1, 
        useGlobalHeadingParam: int = 1, 
        useGlobalTurnParam: int = 1, 
        useStraightLine: int = 1, 
        waypointHeadingMode: str = 'followWayline',
        waypointHeadingAngle: int | None = None,
        waypointHeadingYawPathMode: str = 'followBadArc',
        waypointTurnMode: str = 'toPointAndPassWithContinuityCurvature',
        waypointTurnDampingDist: float = 0.2,
        gimbalPitchAngle: float = 0,
        ):
        '''
        Args:
            point_id: Waypoint index, this must be unique for a route. It must be 
                a sequence number that monotonously and continuously increases 
                from 0. Range [0, 65535]
            longitude: Longitude value.
            latitude: Latitude value.
            altitude: Wayline height in EGM96 or relative to take of height.
            useGlobalHeight: If the global height is used. Must be 0 or 1. False 
                or True respectively.
            executeHeight: Execution altitude of waypoint. The reference is declared 
                in wpml:executeHeightMode.
            useGlobalSpeed: If the global speed is used. Must be 0 or 1. False 
                or True respectively.
            waypointSpeed: Waypoint flight speed in m/s. Overruled by useGlobalSpeed.
            useGlobalHeadingParam: Whether to use the global yaw mode parameter. Must 
                be 0 or 1. False or True respectively.
            useGlobalTurnParam: If the global waypoint type should be used. 0 is do not
                use. 1 indicates use it.
            useStraightLine
            gimbalPitchAngle
            waypointTurnMode: Can be: 'coordinateTurn', 'toPointAndStopWithDiscontinuityCurvature',
                'toPointAndStopWithContinuityCurvature', 'toPointAndPassWithContinuityCurvature'.
            waypointTurnDampingDist: WARNING: When this parameter is used by the code it 
                should be smaller than the wayline length (distance between consecutive
                waypoints). So if 0.1 m is set the minimum distance between waypoints 
                should be 0.1 meters. This is only the case.


        '''
        self.longitude = longitude
        self.latitude = latitude
        self.point_id = point_id
        self.height = height
        self.useGlobalHeight = useGlobalHeight
        self.useGlobalSpeed = useGlobalSpeed
        self.waypointSpeed = waypointSpeed
        self.useGlobalHeadingParam = useGlobalHeadingParam
        self.useGlobalTurnParam = useGlobalTurnParam
        self.useStraightLine = useStraightLine
        self.gimbalPitchAngle = gimbalPitchAngle
        self.ellipsoidHeight = ellipsoidHeight
        self.executeHeight = executeHeight
        self.waypointHeadingMode = waypointHeadingMode
        self.waypointHeadingAngle = waypointHeadingAngle
        self.waypointHeadingYawPathMode = waypointHeadingYawPathMode
        self.waypointTurnMode = waypointTurnMode
        self.waypointTurnDampingDist = waypointTurnDampingDist

        #Inititise parameters
        self.actionId = 0
        self.action_param_list = []

    # def new_method(self, gimbalPitchAngle):
    #     self.gimbalPitchAngle = gimbalPitchAngle

    def add_yaw_action(self, aircraftHeading: float, aircraftPathMode: str = 'clockwise'):
        '''
        This function adds a yaw action to a waypoint.

        Args:
            aircraftHeading: heading to north in degrees. [-180,180]
            aircraftPathMode: 'clockwise' or 'counterClockwise'
            
        '''
        #Get correct action id
        currentId = len(self.action_param_list)

        # Add parameters to the action parameter list
        params = ['rotateYaw', currentId, aircraftHeading, aircraftPathMode]
        self.action_param_list.append(params)

    def add_hover_action(self, hoverTime: float):
        ''' 
        This function adds a hover action to a waypoint

        Args:
            hoverTime: Time to hover in seconds.'''

        #Get correct action id
        currentId = len(self.action_param_list)

        # Add parameters to the action parameter list
        params = ['hover', currentId, hoverTime]
        self.action_param_list.append(params)

    def yaw_action_xml(self, aircraftHeading: float, aircraftPathMode: str):
        heading = ET.Element('wpml:aircraftHeading')
        heading.text = str(aircraftHeading)
        pathmode = ET.Element('wpml:aircraftPathMode')
        pathmode.text = str(aircraftPathMode)
        return (heading,pathmode)

    def hover_action_xml(self, hoverTime: float):
        hovertime_xml = ET.Element('wpml:hoverTime')
        hovertime_xml.text = str(hoverTime)
        return hovertime_xml

    def kml_actions(self):
        '''
        actionActuatorFunc can be:
            rotateYaw
            hover
        '''
        actions = []
        for params in self.action_param_list:
            actionId = params[1]
            actionActuatorFunc = params[0]
            if actionActuatorFunc == 'hover':
                actionActuatorFuncParam_kml_xml = self.hover_action_xml(params[2])
            elif actionActuatorFunc == 'rotateYaw':
                actionActuatorFuncParam_kml_xml = self.yaw_action_xml(params[2], params[3])
            else:
                raise ValueError('The actionActuatorFunc {} was provided but does not exist. Pleas choose from ["hover","rotateYaw"].'.format(actionActuatorFunc))
            
            action_xml = ET.Element('wpml:action')
            actionid = ET.SubElement(action_xml, 'wpml:actionId')
            actionid.text = str(actionId)
            actionac = ET.SubElement(action_xml, 'wpml:actionActuatorFunc')
            actionac.text = str(actionActuatorFunc)
            actionacparam = ET.SubElement(action_xml, 'wpml:actionActuatorFuncParam')
            
            if len(actionActuatorFuncParam_kml_xml) == 0:
                actionacparam.append(actionActuatorFuncParam_kml_xml)
            else:
                for element in actionActuatorFuncParam_kml_xml:
                    actionacparam.append(element)

            actions.append(action_xml)
        return actions

    def build_waypointHeadingParam(self):
        waypoint_heading_param = ET.Element('wpml:waypointHeadingParam')
        mode = ET.SubElement(waypoint_heading_param, 'wpml:waypointHeadingMode')
        mode.text = str(self.waypointHeadingMode)
        if self.waypointHeadingAngle != None:
            angle = ET.SubElement(waypoint_heading_param, 'wpml:waypointHeadingAngle')
            angle.text = str(self.waypointHeadingAngle)
        path = ET.SubElement(waypoint_heading_param, 'wpml:waypointHeadingYawPathMode')
        path.text = str(self.waypointHeadingYawPathMode)
        return waypoint_heading_param

    def build_waypointTurnParam(self):
        waypoint_turn_param = ET.Element('wpml:waypointTurnParam')
        turn_mode = ET.SubElement(waypoint_turn_param, 'wpml:waypointTurnMode')
        turn_mode.text = str(self.waypointTurnMode)
        # It was chosen to always have a damping distance. It is set as a small value as it must 
        # be larger than 0 but smaller than the maximum length of wayline segment.
        waypoint_damping_distance = ET.SubElement(waypoint_turn_param, 'wpml:waypointTurnDampingDist')
        waypoint_damping_distance.text = str(self.waypointTurnDampingDist)
        return waypoint_turn_param

    def build_action_group(self):
        ''' Returns an action group KML.
        '''
        actionGroupId = self.point_id
        actionGroupStartIndex = self.point_id
        actionGroupEndIndex = self.point_id
        actionGroupMode = 'sequence' # only option yet
        actionTriggerType = 'reachPoint' # other options not yet supported.
        actions = self.kml_actions()

        action_group_xml = ET.Element('wpml:actionGroup')
        action_group_id = ET.SubElement(action_group_xml, 'wpml:actionGroupId')
        action_group_id.text = str(actionGroupId)
        action_group_st = ET.SubElement(action_group_xml, 'wpml:actionGroupStartIndex')
        action_group_st.text = str(actionGroupStartIndex)
        action_group_ed = ET.SubElement(action_group_xml, 'wpml:actionGroupEndIndex')
        action_group_ed.text = str(actionGroupEndIndex)
        action_group_id = ET.SubElement(action_group_xml, 'wpml:actionGroupMode')
        action_group_id.text = str(actionGroupMode)            
        action_group_t = ET.SubElement(action_group_xml, 'wpml:actionTrigger')
        action_group_tt = ET.SubElement(action_group_t, 'wpml:actionTriggerType')
        action_group_tt.text = str(actionTriggerType) 

        for action in actions:
            action_group_xml.append(action)
        
        return action_group_xml


    def build_waypoint_xml(self):
        
        location = str(self.longitude) + ', ' + str(self.latitude)

        waypoint_xml = ET.Element('Placemark')
        
        point = ET.SubElement(waypoint_xml, 'Point')
        coords = ET.SubElement(point, 'coordinates')
        coords.text = str(location)

        index_xml = ET.SubElement(waypoint_xml, 'wpml:index')
        index_xml.text = str(self.point_id)
        use_global_height = ET.SubElement(waypoint_xml, 'wpml:useGlobalHeight')
        use_global_height.text = str(self.useGlobalHeight)
        ellip_h = ET.SubElement(waypoint_xml, 'wpml:ellipsoidHeight')
        ellip_h.text = str(self.ellipsoidHeight)
        height_xml = ET.SubElement(waypoint_xml, 'wpml:height') 
        height_xml.text = str(self.height)
        execute_height = ET.SubElement(waypoint_xml, 'wpml:executeHeight')
        execute_height.text = str(self.executeHeight)
        gl_sp_xml = ET.SubElement(waypoint_xml, 'wpml:useGlobalSpeed')
        gl_sp_xml.text = str(self.useGlobalSpeed)
        waypoint_speed = ET.SubElement(waypoint_xml, 'wpml:waypointSpeed')
        waypoint_speed.text = str(self.waypointSpeed)        
        gl_hprm_xml = ET.SubElement(waypoint_xml, 'wpml:useGlobalHeadingParam')
        gl_hprm_xml.text = str(self.useGlobalHeadingParam)
        # needs to be created with function build_waypointHeadingParam.
        waypointHeadingParam = self.build_waypointHeadingParam()
        waypoint_xml.append(waypointHeadingParam)
        gl_tprm_xml = ET.SubElement(waypoint_xml, 'wpml:useGlobalTurnParam')
        gl_tprm_xml.text = str(self.useGlobalTurnParam)
        # waypointTurnParam needs to be created with a function
        waypointTurnParam = self.build_waypointTurnParam()
        waypoint_xml.append(waypointTurnParam)
        use_straigt_line = ET.SubElement(waypoint_xml, 'wpml:useStraightLine')
        use_straigt_line.text = str(self.useStraightLine)        
        gpa_xml = ET.SubElement(waypoint_xml, 'wpml:gimbalPitchAngle')
        gpa_xml.text = str(self.gimbalPitchAngle)

        # Only add actions if there is a filled action list.
        if self.action_param_list != []:
            action_group_element = self.build_action_group()
            waypoint_xml.append(action_group_element)

        return waypoint_xml


class dji_kmz():
    def __init__(self, 
                    waypoints_elements: list,
                    takeOffSecurityHeight: float,
                    autoFlightSpeed: float,
                    globalHeight: float,
                    globalTransitionalSpeed: float = 10,
                    flyToWaylineMode: str = 'safely',
                    finishAction: str = 'goHome', 
                    exitOnRCLost: str = 'executeLostAction',
                    executeRCLostAction: str = 'goBack',
                    takeOffRefPoint = None,
                    droneEnumValue: int = 60,
                    droneSubEnumValue: int | None = None,
                    coordinateMode: str = 'WGS84',
                    heightMode: str = 'relativeToStartPoint',
                    templateId: int = 0,
                    globalWaypointTurnMode: str = 'toPointAndPassWithContinuityCurvature',
                    globalUseStraightLine: int | None = 1,
                    waypointTurnDampingDist: float | None = 1,
                    gimbalPitchMode: str = 'usePointSetting',
                    ellipsoidHeight: float = 0,
                    waypointHeadingMode: str = 'followWayline',
                    waypointHeadingAngle: int | None = None,
                    waypointHeadingYawPathMode: str = 'followBadArc',
                    executeHeightMode: str = 'relativeToStartPoint',
                    nameAutor: str = 'dji_kml_creator', ):
        '''
        Drone payloads are not yet supported. Keyarguments are set for M300RTK.
        So please double check for other drones.

        (gimbalPitchMode is not added as gimbals are not yet supported by the code.)

        Args:
            takeOffSecurityHeight: Altitude that the UAV climbs to after 
                taking off. This value is relative to the ground level. 
                Range [1.5, 1500].
            autoFlightSpeed: Global flight speed in m/s. Range [0, max target flight speed].
            height: Global route height. Can be EGM96 altitude, relative take-off point height or AGL relative ground height. Standard is relative to take-off point. This can be changed in the heightMode parameter. This Element is used in conjunction with "wpml:ellipsoidHeight".
            templateType: For now only the 'waypoint' template is supported.
            templateId: Id of the template. Range [0, 65535].
            globalTransitionalSpeed: Speed in m/s the aircraft flight to and from 
                the first and last waypoint respectively. Also the speed
                used when the mission is interupted. Must be larger than 0.
            flyToWaylineMode: Can be 'safely' or 'pointToPoint'. 
            finishAction: Can be 'goHome', 'autoLand' or 'goToFirstWaypoint'.
            exitOnRCLost: Can be 'goContinue' or 'executeLostAction'
            executeRCLostAction: Can be 'goBack', 'landing' or 'hover'. It 
                is overruled by the the exitOnRCLost parameter.
            takeOffRefPoint: In the form x,y,z. If None is provided it is 
                not used. 
            droneEnumValue: Drone type. 60 for M300RTK and 67 for M30 series.
            droneSubEnumValue: drone sub type. Is not required for M300RTK.
            coordinateMode: Currently only WGS84 is supported
            heightMode: Can be 'EGM96', 'relativeToStartPoint' or 'aboveGroundLevel'.
            globalWaypointTurnMode: Can be 'coordinateTurn', 'toPointAndStopWithDiscontinuityCurvature','toPointAndStopWithDiscontinuityCurvature','toPointAndPassWithContinuityCurvature'
            globalUseStraightLine: Boolean 0 or 1.
            ellipsoidHeight: Gives the ellipsoidheight in meters. Is used in combination with "wpml:height", which are expressions of different elevation reference planes at the same location.
            waypointHeadingMode: Can be 'followWayline', 'manually', 'fixed'.
            waypointHeadingAngle: Required if "wpml:waypointHeadingMode" is "smoothTransition". Is given in degrees.
            waypointHeadingYawPathMode: Can be 'clockwise', 'counterClockwise' or 'followBadArc' (shortest rotation).
            executeHeightMode: Can be 'WGS84' or 'relativeToStartPoint'.
            nameAutor: Name of the Autor in the created KML file.
        
        '''
        # self.payloadInfo = payloadInfo
        templateType = 'waypoint'

        self.waypoints_elements = waypoints_elements
        self.globalHeight = globalHeight
        self.takeOffSecurityHeight = takeOffSecurityHeight
        self.autoFlightSpeed = autoFlightSpeed
        self.executeRCLostAction = executeRCLostAction
        self.takeOffRefPoint = takeOffRefPoint
        self.nameAutor = nameAutor
        self.finishAction = finishAction
        self.exitOnRCLost = exitOnRCLost
        self.flyToWaylineMode = flyToWaylineMode
        self.globalTransitionalSpeed = globalTransitionalSpeed
        self.droneEnumValue = droneEnumValue
        self.droneSubEnumValue = droneSubEnumValue
        self.templateType = templateType
        self.templateId = templateId
        self.globalWaypointTurnMode = globalWaypointTurnMode
        self.coordinateMode = coordinateMode
        self.heightMode = heightMode
        self.globalUseStraightLine = globalUseStraightLine
        self.ellipsoidHeight = ellipsoidHeight
        self.waypointHeadingMode = waypointHeadingMode
        self.waypointHeadingAngle = waypointHeadingAngle
        self.waypointHeadingYawPathMode = waypointHeadingYawPathMode
        self.executeHeightMode = executeHeightMode
        self.waylineId = 0
        self.waypointTurnDampingDist = waypointTurnDampingDist
        self.gimbalPitchMode = gimbalPitchMode
        self.__check_input_types__()
        self.__check_required_rules__()

    def __check_input_types__(self):
        options = []
        ## Options for Template.kml settings
        # Options missionConfig
        options.append(
            (
            self.flyToWaylineMode,
            'enum',
            'safely',
            'pointToPoint',
        ))
        options.append((
            self.finishAction,
            'enum',
            'goHome',
            'autoLand',
            'gotoFirstWaypoint',
        ))
        options.append((
            self.exitOnRCLost,
            'enum',
            'goContinue',
            'executeLostAction',
        ))
        options.append((
            self.executeRCLostAction,
            'enum',
            'goBack',
            'landing',
            'hover',
        ))
        options.append((
            self.takeOffSecurityHeight,
            'range',
            1.5,
            1500,
        ))
        options.append((
            self.globalTransitionalSpeed,
            'range_not_equal_to_borders',
            0,
            99999999, # Must be larger than 0
        ))

        # Options Template information
        options.append((
            self.templateType,
            'enum',
            'waypoint',
            'mapping2d',
            'mapping3d',
            'mappingStrip',
        ))
        options.append((
            self.templateId,
            'range',
            0,
            65535,
        ))
        options.append((
            self.autoFlightSpeed,
            'range_not_equal_to_borders',
            0,
            50, # max speed of the aircraft can be set higher when required.
        ))
        # Options Waypoint Template 
        options.append((
            self.globalWaypointTurnMode, 
            'enum',
            'coordinateTurn',
            'toPointAndStopWithDiscontinuityCurvature',
            'toPointAndStopWithContinuityCurvature',
            'toPointAndPassWithContinuityCurvature',
        ))
        options.append((
            self.globalUseStraightLine,
            'enum',
            0,
            1,
        ))
        options.append((
            self.gimbalPitchMode,
            'enum'
            'manual',
            'usePointSetting'
        ))

        ## Options for Waylines.wpml settings
        # Mission Informationalready covered

        # Waylines Information
        options.append((
            self.executeHeightMode,
            'enum',
            'WGS84',
            'relativeToStartPoint',
        ))
        options.append((
            self.waylineId,
            'range',
            0,
            65535,
        ))    

        ## Options for Common Elements settings
        # droneInfo
        options.append((
            self.droneEnumValue,
            'enum',
            60,
            67,
            77,
        ))
        options.append((
            self.droneSubEnumValue,
            'enum',
            0,
            1,
            None,
        ))
        # As payload is not supported by the current code, payloadInfo is also not checked
        #waylineCoordinateSysParam
        options.append((
            self.coordinateMode,
            'enum'
            'WGS84',
        ))
        options.append((
            self.heightMode,
            'enum',
            'EGM96',
            'relativeToStartPoint',
            'aboveGroundLevel',
            'realTimeFollowSurface', # only supported by Mavic 3 Enterprise series.
        ))
        # options.append(options_positioningType = ( #Only used to mark the positioningtype. Does not effect route execution.
        #     self.positioningType,
        #     'enum',
        #     'GPS',
        #     'RTKBaseStation',
        #     'QianXun',
        #     'Custom',
        # ))
        # globalShootHeight, surfaceFollowModeEnable, surfaceRelativeHeight only available for template types mapping2d, mapping3d, mappingStrip
        # <wpml:payloadParam> is not supported by this code.
        # <wpml:overlap> is not yet supported by this code.
        # <wpml:waypointHeadingParam> & <wpml:globalWaypointHeadingParam>
        options.append((
            self.waypointHeadingMode,
            'enum',
            'followWayline',
            'manually',
            'fixed',
            'smoothTransition', # The target yaw angle for a waypoint is given by "wpml:waypointHeadingAngle" and transitions evenly to the target yaw angle of the next waypoint during the flight segment.
        ))
        if self.waypointHeadingAngle != None:
            options.append(( # this is not set in the documentation. But good to check to be sure.
                self.waypointHeadingAngle,
                'range',
                -180,
                180,
            ))
        options.append((
            self.waypointHeadingYawPathMode,
            'enum'
            'clockwise',
            'counterClockwise',
            'followBadArc',
        ))

        # <wpml:waypointTurnParam> should be given as input if required
        # options.append(options_waypointTurnMode = (
        #     self.waypointTurnMode,
        #     'enum',
        #     'coordinateTurn',
        #     'toPointAndStopWithDiscontinuityCurvature',
        #     'toPointAndStopWithContinuityCurvature',
        #     'toPointAndPassWithContinuityCurvature',
        # ))
        # <wpml:Point> is given as input.
        # <wpml:actionGroup> is given as input
        # <wpml:actionTrigger> is given as input
        # <wpml:action> is given as input
        # <wpml:actionActuatorFuncParam> is not supported yet
        # startRecord is not supported yet
        # stopRecord is not supported yet
        # focus is not supported yet
        # zoom is not supported yet
        # customDirName is not supported yet
        # gimbalRotate is not supported yet
        # rotateYaw is given as input
        # hover is given as input

        # to add globalTransitionalSpeed and globalSpeed larger than 0

        # Test if the correct parameters have been given as input.
        for options_param in options:
            if options_param[1] == 'enum':
                if options_param[0] not in options_param[2:]:
                    raise ValueError(f'Not a possible value. Parameter was {options_param[0]}, but can only be one of the following arguments: {options_param[2:]}')
            elif options_param[1] == 'range':
                if not (options_param[0] >= options_param[2]) & (options_param[0] <= options_param[3]): 
                    raise ValueError(f'Not a possible value. Parameter was {options_param[0]}, but can only be between: {options_param[2:]}')
            elif options_param[1] == 'range_not_equal_to_borders':
                if not (options_param[0] > options_param[2]) & (options_param[0] < options_param[3]): 
                    raise ValueError(f'Not a possible value. Parameter was {options_param[0]}, but can only be between: {options_param[2:]}')

    def __check_required_rules__(self):
        # globalUseStraightLine
        if (self.globalWaypointTurnMode == "toPointAndStopWithContinuityCurvature") | (self.globalWaypointTurnMode == "toPointAndPassWithContinuityCurvature"):
            if (self.globalUseStraightLine != 0) and (self.globalUseStraightLine != 1) :
                raise ValueError('globalUseStraightLine cannot be None if "wpml:globalWaypointTurnMode" is set to "toPointAndStopWithContinuityCurvature" or "toPointAndPassWithContinuityCurvature".')

        if self.droneEnumValue == 67:
            if self.droneSubEnumValue == None:
                raise ValueError('This element is required when droneEnumValue is 67(M30 Series) and connot be None.')
        
        if self.waypointHeadingMode == "smoothTransition":
            if self.waypointHeadingAngle == None:
                raise ValueError('Required if "wpml:waypointHeadingMode" is "smoothTransition" and cannot be set to None in this case.')

        if self.globalWaypointTurnMode == "coordinateTurn":
            if self.waypointTurnDampingDist == None:
                raise ValueError('waypointTurnDampingDist is required when waypointTurnMode" is "coordinateTurn", "wpml:waypointTurnMode" is "toPointAndPassWithContinuityCurvature" and "wpml:useStraightLine" is 1')
        if self.globalUseStraightLine == 1:
            if self.waypointTurnDampingDist == None:
                raise ValueError('waypointTurnDampingDist is required when waypointTurnMode" is "coordinateTurn", "wpml:waypointTurnMode" is "toPointAndPassWithContinuityCurvature" and "wpml:useStraightLine" is 1')
        if self.globalWaypointTurnMode == "toPointAndPassWithContinuityCurvature":
            if self.waypointTurnDampingDist == None:
                raise ValueError('waypointTurnDampingDist is required when waypointTurnMode" is "coordinateTurn", "wpml:waypointTurnMode" is "toPointAndPassWithContinuityCurvature" and "wpml:useStraightLine" is 1')
        # check if WaypointTurnMode in point 

    def build_globalWaypointHeadingParam(self):
        global_waypoint_heading = ET.Element('wpml:globalWaypointHeadingParam')
        mode = ET.SubElement(global_waypoint_heading, 'wpml:waypointHeadingMode')
        mode.text = str(self.waypointHeadingMode)
        if self.waypointHeadingAngle != None:
            angle = ET.SubElement(global_waypoint_heading, 'wpml:waypointHeadingAngle')
            angle.text = str(self.waypointHeadingAngle)
        path = ET.SubElement(global_waypoint_heading, 'wpml:waypointHeadingYawPathMode')
        path.text = str(self.waypointHeadingYawPathMode)
        return global_waypoint_heading

    def build_waypoint_template(self):
        ''' This function adds all waypoint elements to 1 list.'''
        xml_template_list = []
        global_waypoint_turn_mode = ET.Element('wpml:globalWaypointTurnMode')
        global_waypoint_turn_mode.text = str(self.globalWaypointTurnMode)
        xml_template_list.append(global_waypoint_turn_mode)
        # global_waypoint_straightline is only added as it is marked as required.
        if self.globalUseStraightLine != None:
            global_waypoint_straightline = ET.Element('wpml:globalUseStraightLine')
            global_waypoint_straightline.text = str(self.globalUseStraightLine)
            xml_template_list.append(global_waypoint_straightline)
        gimbal_pitch_mode = ET.Element('wpml:gimbalPitchMode')
        gimbal_pitch_mode.text = str(self.gimbalPitchMode)
        xml_template_list.append(gimbal_pitch_mode)    
        ellipsoid_height = ET.Element('wpml:ellipsoidHeight')
        ellipsoid_height.text = str(self.ellipsoidHeight)
        xml_template_list.append(ellipsoid_height) 
        # Changed to globalHeight instead of height after not loading in the DJI pilot 2 app
        global_height = ET.Element('wpml:globalHeight')
        global_height.text = str(self.globalHeight)
        xml_template_list.append(global_height) 
        global_waypoint_heading_params = self.build_globalWaypointHeadingParam()  
        xml_template_list.append(global_waypoint_heading_params) 

        # Remove wpml:executeHeight from waypoint.kml file
        waypoints_elements_kml = []
        elements = copy.deepcopy(self.waypoints_elements)#[:]
        for element in elements:
                wpml_executeHeight = element.find('wpml:executeHeight')
                if wpml_executeHeight != None: 
                    element.remove(wpml_executeHeight)
                waypoints_elements_kml.append(element)

        # Add waypoints
        waypoints_xml_template_list = xml_template_list + waypoints_elements_kml

        return waypoints_xml_template_list 

    def build_mission_config(self):
        mission_config = ET.Element('wpml:missionConfig')
        ftwlm = ET.SubElement(mission_config, 'wpml:flyToWaylineMode')
        ftwlm.text = str(self.flyToWaylineMode)
        finish_ac = ET.SubElement(mission_config, 'wpml:finishAction')
        finish_ac.text = str(self.finishAction)
        exit_rc_lost = ET.SubElement(mission_config, 'wpml:exitOnRCLost')
        exit_rc_lost.text = str(self.exitOnRCLost)
        ex_rc_lost_action = ET.SubElement(mission_config, 'wpml:executeRCLostAction')
        ex_rc_lost_action.text = str(self.executeRCLostAction)       
        tofs = ET.SubElement(mission_config, 'wpml:takeOffSecurityHeight')
        tofs.text = str(self.takeOffSecurityHeight)
        gts = ET.SubElement(mission_config, 'wpml:globalTransitionalSpeed')
        gts.text = str(self.globalTransitionalSpeed)
        # Drone info
        drone_info = ET.SubElement(mission_config, 'wpml:droneInfo')
        drone_enum = ET.SubElement(drone_info, 'wpml:droneEnumValue')
        drone_enum.text = str(self.droneEnumValue)
        if self.droneSubEnumValue != None:
            drone_sub_enum = ET.SubElement(drone_info, 'wpml:droneSubEnumValue')
            drone_sub_enum.text = str(self.droneSubEnumValue)
        # Payload info
        ## is for now skipped as Yellowscan payload is not supported. 
        return mission_config

    def build_kml(self):
        kml_xml = ET.Element('kml')
        kml_xml.set('xmlns', "http://www.opengis.net/kml/2.2")
        kml_xml.set('xmlns:wpml', "http://www.dji.com/wpmz/1.0.0")
        document_xml = ET.SubElement(kml_xml, 'Document')

        # Add file creation information
        author = ET.SubElement(document_xml, 'wpml:author')
        author.text = str(self.nameAutor)
        createtime = ET.SubElement(document_xml, 'wpml:createTime')
        createtime.text = str(int( time.time_ns() / 1000 ))
        updatetime = ET.SubElement(document_xml, 'wpml:updateTime')
        updatetime.text = str(int( time.time_ns() / 1000 ))

        # Setup mission configuration, payload info is not added yet
        document_xml.append(self.build_mission_config())

        # Setup folder and template
        folder = ET.SubElement(document_xml, 'Folder')
        templatetype = ET.SubElement(folder, 'wpml:templateType')
        templatetype.text = str(self.templateType)
        template_id = ET.SubElement(folder, 'wpml:templateId')
        template_id.text = str(self.templateId)

        # Set the autoFlightSpeed
        global_speed = ET.SubElement(folder, 'wpml:autoFlightSpeed')
        global_speed.text = str(self.autoFlightSpeed)

        # Wayline coordinate system parameters
        coords_sys_params = ET.SubElement(folder, 'wpml:waylineCoordinateSysParam')
        coords_sys = ET.SubElement(coords_sys_params, 'wpml:coordinateMode')
        coords_sys.text = self.coordinateMode
        height_mode = ET.SubElement(coords_sys_params, 'wpml:heightMode')
        height_mode.text = self.heightMode
        # surfaceFollowModeEnable, globalShootHeight, surfaceRelativeHeight not 
        # added yet as this cannot be used for waypoint missions.

        if self.templateType == 'waypoint':
            waypoint_xml_template_list = self.build_waypoint_template()
            for element in waypoint_xml_template_list:
                folder.append(element)
        return kml_xml 

    def build_waylines_wpml(self):
        waylines_kml = ET.Element('kml')
        waylines_kml.set('xmlns',"http://www.opengis.net/kml/2.2")
        waylines_kml.set('xmlns:wpml',"http://www.dji.com/wpmz/1.0.0")
        document_xml = ET.SubElement(waylines_kml, 'Document')

        # Setup mission configuration, payload info is not added yet
        document_xml.append(self.build_mission_config())

        # Setup Waylines Information
        folder = ET.SubElement(document_xml, 'Folder')
        template_id = ET.SubElement(folder, 'wpml:templateId')
        template_id.text = str(self.templateId) 
        # Create an increasing waylineID    
        wayline_id = ET.SubElement(folder, 'wpml:waylineId')
        wayline_id.text = str(self.waylineId) 
        self.waylineId +=1
  
        auto_flight_speed = ET.SubElement(folder, 'wpml:autoFlightSpeed')
        auto_flight_speed.text = str(self.autoFlightSpeed)
        execute_height_mode = ET.SubElement(folder, 'wpml:executeHeightMode')
        execute_height_mode.text = str(self.executeHeightMode)

        # Add placemark information.
        if self.templateType == 'waypoint':
            # Required to have different wayline and kml placemarks (self keeps these linked otherwise)
            wayline_elements = copy.deepcopy(self.waypoints_elements)
            for element in wayline_elements:
                # # The following elements should not be in the waylines.wpml file
                e = element
                wpml_height = e.find('wpml:height')
                wpml_Useglobalheight = e.find('wpml:useGlobalHeight')
                wpml_Ellispoidheight = e.find('wpml:ellipsoidHeight')
                wpml_Useglobalspeed = e.find('wpml:useGlobalSpeed')
                wpml_UseGlobalHeading = e.find('wpml:useGlobalHeading')
                wpml_useGlobalHeadingParam = e.find('wpml:useGlobalHeadingParam')
                # The following is required to be able to run this function
                # multiple times without producing errors.
                if wpml_height != None: 
                    e.remove(wpml_height)
                if wpml_Useglobalheight != None: 
                    e.remove(wpml_Useglobalheight)
                if wpml_Ellispoidheight != None: 
                    e.remove(wpml_Ellispoidheight)
                if wpml_Useglobalspeed != None: 
                    e.remove(wpml_Useglobalspeed)
                if wpml_UseGlobalHeading != None: 
                    e.remove(wpml_UseGlobalHeading)
                if wpml_useGlobalHeadingParam != None: 
                    e.remove(wpml_useGlobalHeadingParam)
                # e.append(ET.Element('yo'))
                folder.append(e)

        # wpml:startActionGroup was not added as the documentation was not 
        # clear about this parameter. And is not usable for the Matrice 300 RTK

        return waylines_kml

    def build_kmz(self, file =  'test.kmz'):
        # Make sure that a path can be provided.
        splitted_path = os.path.split(file)
        if splitted_path[0] == '':
            filename = file
        else:
            path_dir, filename = splitted_path
 
        kml_element = self.build_kml()
        waylines_wpml_element = self.build_waylines_wpml()

        # Make the kmz file in a temporary directory as files need to be
        # temporary created to make a zip file. This zipfile is then
        # changed to the kmz file
        with tempfile.TemporaryDirectory(prefix = 'temp_kmz_1') as tmpdir_root:
            zip_path = os.path.join(tmpdir_root, 'zip_folder')
            wpmz_path = os.path.join(zip_path, 'wpmz')
            kml_path = os.path.join(wpmz_path, 'template.kml')
            wpml_path = os.path.join(wpmz_path, 'waylines.wpml')
            res_path = os.path.join(wpmz_path, 'res')
            
            # Create required folders
            os.mkdir(zip_path)
            os.mkdir(wpmz_path)
            os.mkdir(res_path)

            # Export xml files
            export_formated_xml(kml_element, kml_path)
            export_formated_xml(waylines_wpml_element, wpml_path)

            # Zip the file and move it to the final directory
            path_zip = os.path.join(tmpdir_root,filename) + '.zip'
            shutil.make_archive(
                os.path.join(tmpdir_root,filename),
                'zip',
                zip_path,
                )

            shutil.move(path_zip, file)

def imu_callibration_j_turn(start_coordinate_epsg_local, epsg_local, epsg_kml, rotation, start_point_index, height, max_speed = 10, length = 30, turn_radius = 5):
    '''
    angle = angle with north-south line
    '''
    transformer = Transformer.from_crs(epsg_local,
                                        epsg_kml)
    x = np.zeros(6)
    y = np.zeros(6)
    
    x[0] = start_coordinate_epsg_local[0]
    x[1] = start_coordinate_epsg_local[0] + (length/np.sin(rotation))
    x[2] = start_coordinate_epsg_local[0]
    x[3] = start_coordinate_epsg_local[0] + (length/np.sin(rotation))
    x[4] = start_coordinate_epsg_local[0] + ((length+turn_radius)/np.sin(rotation))
    x[5] = start_coordinate_epsg_local[0] + ((length+turn_radius)/np.sin(rotation))

    y[0] = start_coordinate_epsg_local[0]
    y[1] = start_coordinate_epsg_local[0] + (length/np.cos(rotation))
    y[2] = start_coordinate_epsg_local[0]
    y[3] = start_coordinate_epsg_local[0] + (length/np.cos(rotation))
    y[4] = start_coordinate_epsg_local[0] + ((length+turn_radius)/np.cos(rotation))
    y[5] = start_coordinate_epsg_local[0] + ((length+turn_radius)/np.cos(rotation))

    # Transorm to kml crs
    transformed_lat, transformed_lon = transformer.transform(x, y)
    
    point0 = dji_waypoint_mission(
                start_point_index, 
                transformed_lon, 
                transformed_lat, 
                height = height,
                useGlobalHeight = 1,
                useGlobalSpeed = 1,
                useGlobalTurnParam = 0,
                waypointTurnMode = 'toPointAndStopWithDiscontinuityCurvature',
                useStraightLine = 1,
                # waypointTurnDampingDist = checked_waypointTurnDampingDist,
                gimbalPitchAngle = 0,
            )



if __name__ == '__main__':
    # Example code
    point1 = dji_waypoint_mission(10, 4.233, 52.00)
    point1.add_hover_action(5)
    point1.add_yaw_action(-20)
    point1_xml = point1.build_waypoint_xml()
    point2 = dji_waypoint_mission(10, 4.233, 52.00)
    point2.add_hover_action(22)
    point2.add_yaw_action(-5)
    point2_xml = point2.build_waypoint_xml()

    test = dji_kmz(
        [point1_xml, point2_xml],
        80,
        5,
        80,
    )

    kml_element = test.build_kml()
    waylines_wpml_element = test.build_waylines_wpml()
    test.build_kmz("data/test_dji_kmz_creator.kmz")
    
    string_xml = return_string(point1_xml)
    print(string_xml)
    
    print('############### KML ####################')
    string_xml = return_string(kml_element)
    print(string_xml)


    print('############### WAYLINES ####################')
    string_xml = return_string(waylines_wpml_element)
    print(string_xml)

    
