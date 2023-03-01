# import dash_html_components as html
import dash_leaflet as dl
from dash import Dash, html, dcc, no_update
import dash
from dash.dependencies import Output, Input
from dash.exceptions import PreventUpdate
from dash_extensions.javascript import assign
# from dash_extensions.javascript import Namespace
import dash_bootstrap_components as dbc
import numpy as np
from shapely.geometry import Polygon, MultiLineString, MultiPoint, LineString
from shapely.ops import transform as TransForm
import json 
import pyproj
import shapely as sh
import time
from pyproj import Transformer
# import simplekml as skml
import xml.etree.ElementTree as ET
import os

# import own functions
import dji_kmz_creator as dji_kml
import flight_planning as fp


# Set the correct working directory. The dash_extensions.javascript import assign
# does not work otherwise with the point_to_layer variable.
current_working_dir = os.getcwd()
if os.path.split(current_working_dir)[1] != 'flightplanner':
    new_working_dir = os.path.join(os.getcwd(), 'flightplanner')
    os.chdir(new_working_dir)

external_stylesheets = [dbc.themes.BOOTSTRAP]
# # How to render geojson.
point_to_layer = assign("""function(feature, latlng, context){
    const p = feature.properties;
    if(p.type === 'circlemarker'){return L.circleMarker(latlng, radius=p._radius)}
    if(p.type === 'circle'){return L.circle(latlng, radius=p._mRadius)}
    return L.marker(latlng);
}""")

# Create example app.

edit_drawtoolbar = {
    'polyline' : False,
    'marker' : False,
    'circlemarker': False,
    'circle': False,
    'rectangle': False,
}

app = Dash(__name__, external_stylesheets=external_stylesheets)

########################## Body ##########################


# Inputs
body_controls1 = dbc.Card(
    [
        html.Div([
            dbc.Label("Draw your area to survey, make sure it is correct as you cannot go back.")
            ]),

        
        html.Div([
            dbc.Button('Finished', id='polygon_drawing_button', outline=True, color="success", n_clicks = 0)
            ]),
    ], 
    body = True
)


body_controls_mode = dbc.Card(
    [
        html.Div(html.H4("Set Mode")
        ),
        dbc.RadioItems(
            options=[
                {"label": "Standard", "value": 1},
                {"label": "Yellowscan Mapper +", "value": 2},
            ],
            value=2,
            id="input_mode",
        ),
        # html.Div('Angle: 0 deg', id = 'angle_text'),
        # html.Div([
        #     dcc.Slider(id="angle_slider", min=0, max=360, step=1, value=0)
        # ]),

    ], 
    body = True
)

body_controls2 = dbc.Card(
    [
        html.Div(html.H4("Plan your flight")
        ),

        html.Div('Angle: 0 deg', id = 'angle_text'),
        html.Div([
            dcc.Slider(id="angle_slider", marks=None, min=0, max=360, step=1, value=0)
        ]),

        html.Div('Offset: 0', id = 'offset_text'),
        html.Div([
            dcc.Slider(id="offset_slider", marks=None, min=0.001, max=0.5, step=0.001, value=0.5)
        ]),

        html.Div('Buffer: 0 meter(s)', id = 'buffer_text'),
        html.Div([
            dcc.Slider(id="buffer_slider", marks=None, min=0, max=20, step=0.1, value=5)
        ]),

        html.Div('Damping: 0.2 meter(s)', id = 'damping_text'),
        html.Div([
            dcc.Slider(id="damping_slider", marks=None, min=0.2, max=50, step=0.1, value=0.2)
        ]),

        # html.Div('Distance flight lines: 40 meter(s)', id = 'flight_lines_dist_text'),
        # html.Div([
        #     dcc.Slider(id="flight_lines_dist_slider", marks=None, min=0, max=120, step=0.5, value=40)
        # ]),

        dbc.Collapse(
            dbc.Col([
                html.Div('Distance flight lines: 40 meter(s)', id = 'flight_lines_dist_text'),
            html.Div([
                dcc.Slider(id="flight_lines_dist_slider", marks=None, min=0, max=120, step=0.5, value=40)
                ]),
            ],
            align = 'center'
            ),
            id='standard_mode_dist',
            is_open = False,
        ),
        
        html.Div('Global height: 50 meter(s)', id = 'global_height_text'),
        html.Div([
            dcc.Slider(id="global_height_slider", marks=None, min=0, max=120, step=0.1, value=50)
        ]),

        html.Div('Auto flight ground speed: 5 m/s', id = 'autoflightspeed_text'),
        html.Div([
            dcc.Slider(id="autoflightspeed_slider", marks=None, min=0, max=15, step=0.1, value=5)
        ]),

        dbc.Collapse(
            dbc.Col([
                html.Div('Flight lines overlap: 0.3 ', id = 'overlap_text'),
                html.Div([
                    dcc.Slider(id="yellowscan_mode_overlap_slider", marks=None, min=0, max=0.99, step=0.1, value=0.3)
                ]),
            ],
            align = 'center'
            ),
            id='yellow_scan_mode1',
            is_open = True, #standard true as the yellowscan mode is selected first standard.
        ),
        
        html.Div(html.H4('Estimated parameters', id = 'title_estimated_parameters_text')),

        # html.Div('Flight lines overlap: 0.3 ', id = 'yellowscan_mode_overlap_text'),
        # html.Div([
        #     dcc.Slider(id="yellowscan_mode_overlap_slider", marks=None, min=0, max=0.99, step=0.1, value=0.3)
        # ]),
        dbc.Collapse(
            dbc.Col([
                html.Div('Distance between flight lines: ', id = 'yellowscan_mode_dist_flightlines_text'),
            ],
            align = 'center'
            ),
            id='yellowscan_mode_dist_flightlines_visible',
            is_open = True,
        ),

        dbc.Collapse(
            dbc.Col([
                html.Div('Overlap flight lines: ', id = 'standard_mode_overlap_flightlines_text'),
            ],
            align = 'center'
            ),
            id='standard_mode_overlap_flightlines_text_visible',
            is_open = False,
        ),

        html.Div([
            dbc.Button('Download KMZ', id='download_kml_btn', outline=True, color="success", n_clicks = 0)
        ]),
    ], 
    body = True
)


body_output_drawing = html.Div([
        dl.Map(
            center=[56, 10],
            zoom=4,
            children = [
            dl.LayersControl([
                dl.BaseLayer(
                    dl.TileLayer(),
                    name="OpenStreetMaps",
                    checked = True,
                ),
                dl.BaseLayer(
                    dl.TileLayer(
                        url="https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                        attribution="Google Satellite",                        
                    ),
                    name="Google Satellite",
                    checked = False,
                ),
            ]),
            dl.FeatureGroup([dl.EditControl(id="edit_control", draw = edit_drawtoolbar)]),
        ],
        style={'width': '100%', 'height': '87vh', 'margin': "auto", "display": "inline-block"},id="map"),
    ])


body_output_flightplan = html.Div([
        dl.Map(center=[56, 10], zoom=4, children=[
        dl.LayersControl([
            dl.BaseLayer(
                dl.TileLayer(),
                name="OpenStreetMaps",
                checked = True,
            ),
            dl.BaseLayer(
                dl.TileLayer(
                    url="https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    attribution="Google Satellite",                        
                ),
                name="Google Satellite",
                checked = False,
            ),
        ]),
        dl.GeoJSON(id="geojson", options=dict(pointToLayer=point_to_layer), zoomToBounds=True),
        dl.GeoJSON(id ="flight_lines", options=dict(pointToLayer=point_to_layer)),
        dl.GeoJSON(id ="waypoints_multipoint", options=dict(pointToLayer=point_to_layer)),
        ], style={'width': '100%', 'height': '87vh', 'margin': "auto", "display": "inline-block"}, id="mirror"),
    ])


app.layout = dbc.Container(
    [
        html.H1("DJI flight planner", id="nav-pills"),
        html.Hr(),
        html.Div([
            dbc.Collapse(
                dbc.Row([
                    dbc.Col(body_controls1, width = 2),
                    dbc.Col(body_output_drawing, width = 10),
                ],
                align = 'center'
                ),
                id='drawing_body',
                is_open = True,
            ),
            dbc.Collapse(
                dbc.Row([
                    dbc.Col([dbc.Row([body_controls_mode]),
                             dbc.Row([body_controls2])], width = 2),
                    dbc.Col(body_output_flightplan, width = 10),
                ],
                align = 'center'
                ),
                id='flight_plan_body',
                is_open = True,
            ),
        ]),
        dcc.Store(id = 'polygon_step1'),
        dcc.Store(id = 'polygon_update'),
        dcc.Store(id = 'waypoints'),
        dcc.Store(id = 'kml_clicks', data = "0"),
        dcc.Download(id="download_kml"),
    ],
    fluid=True,
)


@app.callback(
    Output('drawing_body', 'is_open'),
    Output('flight_plan_body','is_open'),
    Output("geojson", "data"),
    Output('polygon_step1', "data"),
    Input('polygon_drawing_button', 'n_clicks'),
    Input("edit_control", "geojson"),
    Input('polygon_step1', "data"),
    )
def hide_drawing_options(n_clicks, geojson, saved_polygon):
    if type(saved_polygon) != type(None):
        raise PreventUpdate
    output1 = True
    output2 = False
    if n_clicks == 1:
        output1 = False
        output2 = True
        data = geojson
        return output1, output2, data, data
    else: 
        return no_update, output2, no_update, no_update


@app.callback(
    Output('polygon_update', "data"),
    Input('polygon_step1', "data"),
    Input('drawing_body', 'is_open'),
)
def polygon_reproject(geojson_dict, boolean_screen_1_open):
    # Only run this callback function when screen 1 is not open.
    if boolean_screen_1_open:
        raise PreventUpdate

    # Get the coordinates in a numpy array.
    coordinate_list_to_many_dims = geojson_dict['features'][0]['geometry']['coordinates']
    coordinate_list = np.squeeze(coordinate_list_to_many_dims)
    
    epsg_leaflet = 4326
    epsg_rd_new = 28992

    # Transform coordinates to rd new
    transformer5 = Transformer.from_crs(epsg_leaflet,
                                        epsg_rd_new)
    lat = coordinate_list[:,1]
    lon = coordinate_list[:,0]
    transformed_x, transformed_y = transformer5.transform(lat, lon)

    # Put the coordinates first in a list to be able to make a json. This json is made as only strings can be stored between functions.
    transformed_coords = json.dumps({"x": transformed_x.tolist() , "y" : transformed_y.tolist(), "epsg": epsg_rd_new})

    return transformed_coords



@app.callback(
    Output("waypoints","data"),
    Output("flight_lines","data"),
    Output("waypoints_multipoint","data"),
    Output('angle_text','children'),
    Output('offset_text','children'),
    Output('buffer_text','children'),
    Output('damping_text','children'),
    Output('flight_lines_dist_text', 'children'),
    Output('global_height_text', 'children'),
    Output('autoflightspeed_text', 'children'),
    Output('overlap_text', 'children'),
    Output('flight_lines_dist_slider', 'value'),
    Output('yellow_scan_mode1', 'is_open'), # if flightlines overlap slider should be displayed.
    Output('yellowscan_mode_dist_flightlines_visible', 'is_open'),
    Output('standard_mode_dist', 'is_open'), # if flight lines distance slider should be displayed.
    Output('standard_mode_overlap_flightlines_text_visible', 'is_open'),
    Output('yellowscan_mode_dist_flightlines_text','children'), 
    Output('standard_mode_overlap_flightlines_text','children'),
    Input('polygon_update', "data"),
    Input('angle_slider','value'),
    Input('offset_slider','value'),
    Input('buffer_slider','value'),
    Input('damping_slider','value'),
    Input('flight_lines_dist_slider', 'value'),
    Input('global_height_slider', 'value'),
    Input('autoflightspeed_slider','value'),
    # Extra added
    Input('input_mode','value'),
    Input('yellowscan_mode_overlap_slider', 'value'),
)
def update_flightplan(polygon_coords_json, angle, offset, buffer, damping, flight_lines_dist, global_height, autoflightspeed, mode: int, overlap: float):
    ''''
    This function updates the flightplan based on the callback variables that can be seen above.

    Args:
        mode: Determines the mode of the program. Now supported are custom (value 1) and yellowscan mapper plus (value 2). 
        overlap: The overlap the flightlines should have. This is a value between 0 and 1.
    '''
    # Make sure this function only executes when there is a polygon_coords_json file.
    if type(polygon_coords_json) == type(None):
        raise PreventUpdate

    # Load coordinates in numpy array
    polygon_coords_dict = json.loads(polygon_coords_json)
    xy_coords = np.array([polygon_coords_dict['x'],polygon_coords_dict['y']]).T # N by 2 
    
    # Set epsg codes
    epsg_local = polygon_coords_dict['epsg']
    epsg_leaflet = 4326

    # Use the selected program mode
    if mode == 1: # Use the custom mode
        points_coords, sh_overlapping_lines = fp.flightcoordinates(xy_coords , angle, offset, buffer, flight_lines_dist)
        visible_yellowscan_mode_flight_lines_overlap = False
        yellowscan_mode_dist_flightlines_visible = False
        visible_standard_mode_distance_flightlines = True
        standard_mode_overlap_flightlines_text_visible = True

    elif mode == 2: # Use the Yellowscan Mapper plus mode
        flight_lines_dist = fp.find_distance_flightlines(overlap, global_height, view_angle = 70*np.pi/180)
        points_coords, sh_overlapping_lines = fp.flightcoordinates(xy_coords , angle, offset, buffer, flight_lines_dist)
        visible_yellowscan_mode_flight_lines_overlap = True
        yellowscan_mode_dist_flightlines_visible = True
        visible_standard_mode_distance_flightlines = False
        standard_mode_overlap_flightlines_text_visible = False
    else:
        raise ValueError(f'Selected mode should be 0 or 1 but is {mode}')
    # Find additional_coords


    array_flight_points_coords = np.asarray(points_coords)


    points_coords_x = array_flight_points_coords[:,0]
    points_coords_y = array_flight_points_coords[:,1]

    #### Calculate added point coordinates for smooth corners
    # Find max allowed waypointTurnDampingDists for each point
    # print('points_coords', array_flight_points_coords)
    # print('ttestt',np.sqrt(np.sum(array_flight_points_coords**2, axis = 1)))
    checked_waypointTurnDampingDists = fp.find_all_max_waypointTurnDampingDists(array_flight_points_coords, max_setting = float(damping))
    # checked_waypointTurnDampingDists  =5
    # print('damp',len(checked_waypointTurnDampingDists))
    # print('x',len(points_coords_x))
    new_x, new_y, new_z = fp.coordinated_turn_corners(points_coords_x,points_coords_y, checked_waypointTurnDampingDists, z = np.zeros(len(points_coords_y)), amount = 5)
    new_points_coords = []
    for i in range(len(new_x)):
        new_points_coords.append(np.array((new_x[i],new_y[i])))

    # print('new points coords', new_points_coords)

    #### Visualize the flight plan
    # sh_linestring_flight_plan = LineString(points_coords) # working
    sh_linestring_flight_plan = LineString(new_points_coords) 
    sh_waypoints_flight_plan = MultiPoint(points_coords)
    # sh_linestring_flight_plan = LineString(new_points_coords)

    # Transform the coordinates
    transformer2 = Transformer.from_crs(epsg_local,
                                        epsg_leaflet,
                                        always_xy=True,  # to make sure xy coords to lat/lon goes correctly
                                        ).transform
    sh_linestring_flight_plan_crs_leaflet = TransForm(transformer2, sh_linestring_flight_plan)
    sh_waypoints_flight_plan_crs_leaflet = TransForm(transformer2, sh_waypoints_flight_plan)

    # Get the coordinates in readable geojson format for dash_leaflet library
    sh_linestring_flight_plan_crs_leaflet_geojson_step1 = sh.geometry.mapping(sh_linestring_flight_plan_crs_leaflet)
    sh_linestring_flight_plan_crs_leaflet_geojson = {'type': 'GeometryCollection', 'features':[sh_linestring_flight_plan_crs_leaflet_geojson_step1]}
    
    sh_waypoints_flight_plan_crs_leaflet_geojson_step1 = sh.geometry.mapping(sh_waypoints_flight_plan_crs_leaflet)
    sh_waypoints_flight_plan_crs_leaflet_geojson = {'type': 'GeometryCollection', 'features':[sh_waypoints_flight_plan_crs_leaflet_geojson_step1]}

    # Get coordinates in string format to store in dcc.Store for cross function use
    np_array_points_coords = np.squeeze(points_coords)
    dcc_local_crs_waypoints = json.dumps({"x": np_array_points_coords[:,0].tolist() , "y" : np_array_points_coords[:,1].tolist() , "epsg": epsg_local})

    # Get damping parameters in string format to store in dcc.Store for cross function use
    np_array_points_coords = np.squeeze(points_coords)
    dcc_local_crs_waypoints = json.dumps({"x": np_array_points_coords[:,0].tolist() , "y" : np_array_points_coords[:,1].tolist() , "epsg": epsg_local})

    # output strings to update settings
    string_offset = f'Offset: {offset}'
    string_angle = f"Angle: {angle} degrees"
    string_buffer = f"Buffer: {buffer} meter(s)"
    string_damping = f"Damping: {damping} meter(s)"
    string_flight_lines_distance = f"Distance flight lines: {flight_lines_dist} meter(s)"
    string_global_height = f"Global height: {global_height} meter(s)"
    string_autoflightspeed = f"Auto flight ground speed: {autoflightspeed} m/s"
    string_overlap = f"Flight lines overlap: {overlap}"
    string_estimated_density = r"Estimated point density: {} points/$m^2$".format(density)


    return dcc_local_crs_waypoints, sh_linestring_flight_plan_crs_leaflet_geojson, sh_waypoints_flight_plan_crs_leaflet_geojson, string_angle, string_offset, string_buffer, string_damping, string_flight_lines_distance, string_global_height, string_autoflightspeed, string_overlap, flight_lines_dist, visible_yellowscan_mode_flight_lines_overlap, yellowscan_mode_dist_flightlines_visible, visible_standard_mode_distance_flightlines, standard_mode_overlap_flightlines_text_visible, string_flight_lines_distance, string_overlap

# The callback and function below make sure the kmz data can be downloaded.
@app.callback(
    Output("download_kml", "data"),
    Output("kml_clicks", "data"),
    Input("waypoints","data"),
    Input("download_kml_btn", "n_clicks"),
    Input('kml_clicks','data'),
    Input('damping_slider','value'),
    Input('global_height_slider','value'),
    Input('autoflightspeed_slider','value'),
    prevent_initial_call=True,
)
def download_kml(waypoints_dict, n_clicks, kml_clicks, damping_slider_value, global_height, autoflightspeed):
    kml_clicks = int(kml_clicks)
    if n_clicks == kml_clicks:
        raise PreventUpdate
    # Load coordinates in numpy array
    waypoints_dict = json.loads(waypoints_dict)
    xy_coords = np.array([waypoints_dict['x'], waypoints_dict['y']]).T

    # Find max allowed waypointTurnDampingDists for each point
    checked_waypointTurnDampingDists = fp.find_all_max_waypointTurnDampingDists(xy_coords, max_setting = float(damping_slider_value))

    # Specify crs
    epsg_local = waypoints_dict['epsg']
    epsg_kml = 4326

    # Transorm to kml crs
    transformer5 = Transformer.from_crs(epsg_local,
                                        epsg_kml)
    x = xy_coords[:,0]
    y = xy_coords[:,1]
    transformed_lat, transformed_lon = transformer5.transform(x, y)
    
    heights = np.zeros(len(transformed_lon))
    
    points = []
    for i, (lat, lon, checked_waypointTurnDampingDist) in enumerate(zip(transformed_lat, transformed_lon, checked_waypointTurnDampingDists)):
        point = dji_kml.dji_waypoint_mission(
            i, 
            lon, 
            lat, 
            useGlobalHeight = 1,
            useGlobalSpeed = 1,
            useGlobalTurnParam = 0,
            waypointTurnMode = 'coordinateTurn',
            useStraightLine = 1,
            waypointTurnDampingDist = checked_waypointTurnDampingDist,
            gimbalPitchAngle = 0,
            # Following added and makes the DJI app read the data better.
            executeHeight = global_height,
            waypointSpeed = autoflightspeed,
            ellipsoidHeight = global_height,
            height = global_height,
        )
        # point.add_hover_action(5)
        # point.add_yaw_action(-20)

        point_xml = point.build_waypoint_xml()
        points.append(point_xml)

    dji_mission = dji_kml.dji_kmz(
        points,             #point elements
        80,                 #takeOffSecurityHeight
        autoflightspeed,                  #autoFlightSpeed
        global_height,                 #globalHeight
        coordinateMode = 'WGS84',
        heightMode = 'relativeToStartPoint',
    )


    dji_mission.build_kmz('test.kmz')

    fh = open('test.kmz', 'rb')

    kmz_bytearray = bytes(fh.read())

    kmz_dict_download = dcc.send_bytes(kmz_bytearray, 'finaly.kmz')

    return kmz_dict_download, str(n_clicks) #dict(content=python_string, filename="flightplan_new_method.kml")

# Add as variable
#   flight lines distance
#
# function
#   plots waypoints
#   plots flightlines
# 
# function
#   add 
#   
# 
# 
# 


if __name__ == '__main__':
    app.run_server(port=7781, debug=True)
