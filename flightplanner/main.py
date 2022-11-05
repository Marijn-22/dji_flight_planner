# import dash_html_components as html
import dash_leaflet as dl
from dash import Dash, html, dcc, no_update
from dash.dependencies import Output, Input
from dash.exceptions import PreventUpdate
from dash_extensions.javascript import assign
import dash_bootstrap_components as dbc
import numpy as np
from shapely.geometry import Polygon, MultiLineString, MultiPoint, LineString
from shapely.ops import transform as TransForm
import json 
import pyproj
import shapely as sh
import time
from pyproj import Transformer
import simplekml as skml
import dji_kmz_creator as dji_kml
import xml.etree.ElementTree as ET
import io

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

body_controls2 = dbc.Card(
    [
        html.Div(html.H4("Plan your flight")
        ),

        html.Div('Angle: 0 deg', id = 'angle_text'),
        html.Div([
            dcc.Slider(id="angle_slider", min=0, max=360, step=1, value=0)
        ]),

        html.Div('Offset: 0', id = 'offset_text'),
        html.Div([
            dcc.Slider(id="offset_slider", min=0.001, max=0.5, step=0.001, value=0.5)
        ]),

        html.Div('Buffer: 0 meter(s)', id = 'buffer_text'),
        html.Div([
            dcc.Slider(id="buffer_slider", min=0, max=20, step=0.1, value=5)
        ]),
        
        html.Div([
            dbc.Button('Download KMZ', id='download_kml_btn', outline=True, color="success", n_clicks = 0)
        ]),
    ], 
    body = True
)


body_output_drawing = html.Div([
        dl.Map(center=[56, 10], zoom=4, children=[
            dl.TileLayer(),
            dl.FeatureGroup([dl.EditControl(id="edit_control", draw = edit_drawtoolbar)]),
        ], style={'width': '100%', 'height': '87vh', 'margin': "auto", "display": "inline-block"},id="map"),
    ])

body_output_flightplan = html.Div([
        dl.Map(center=[56, 10], zoom=4, children=[
        dl.TileLayer(), 
        dl.GeoJSON(id="geojson", options=dict(pointToLayer=point_to_layer), zoomToBounds=True),
        dl.GeoJSON(id ="flight_lines", options=dict(pointToLayer=point_to_layer)),
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
                    dbc.Col(body_controls2, width = 2),
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

def make_multi_linestring(xmin, ymin, xmax, ymax, offset, distance_between_lines):
    amount_of_lines_float = abs(xmax - xmin)/distance_between_lines
    amount_of_lines_roundedup = int(amount_of_lines_float) + (distance_between_lines % abs(xmax - xmin) > 0)
    coords = []
    for i in range(amount_of_lines_roundedup):
        xline = xmin + i*distance_between_lines + offset*distance_between_lines
        coords.append(((xline,ymin),(xline,ymax)))
    return MultiLineString(coords)


def flightcoordinates(polygon_coords: np.array, angle: float, offset: float, buffer: float, distance_between_flight_lines: float):
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

@app.callback(
    Output("waypoints","data"),
    Output("flight_lines","data"),
    Output('angle_text','children'),
    Output('offset_text','children'),
    Output('buffer_text','children'),
    Input('polygon_update', "data"),
    Input('angle_slider','value'),
    Input('offset_slider','value'),
    Input('buffer_slider','value'),
)
def update_flightplan(polygon_coords_json, angle, offset, buffer):
    # Make sure this function only executes when there is a polygon_coords_json file.
    if type(polygon_coords_json) == type(None):
        raise PreventUpdate

    # Load coordinates in numpy array
    polygon_coords_dict = json.loads(polygon_coords_json)
    xy_coords = np.array([polygon_coords_dict['x'],polygon_coords_dict['y']]).T # N by 2 
    
    # Set epsg codes
    epsg_local = polygon_coords_dict['epsg']
    epsg_leaflet = 4326
    
    points_coords, sh_overlapping_lines = flightcoordinates(xy_coords , angle, offset, buffer, 40)

    #### Visualize the flight plan
    sh_linestring_flight_plan = LineString(points_coords)
    # Transform the coordinates
    transformer2 = Transformer.from_crs(epsg_local,
                                        epsg_leaflet,
                                        always_xy=True,  # to make sure xy coords to lat/lon goes correctly
                                        ).transform
    sh_linestring_flight_plan_crs_leaflet = TransForm(transformer2, sh_linestring_flight_plan)

    # Get the coordinates in readable geojson format for dash_leaflet library
    sh_linestring_flight_plan_crs_leaflet_geojson_step1 = sh.geometry.mapping(sh_linestring_flight_plan_crs_leaflet)
    sh_linestring_flight_plan_crs_leaflet_geojson = {'type': 'GeometryCollection', 'features':[sh_linestring_flight_plan_crs_leaflet_geojson_step1]}
    
    # Get coordinates in string format to store in dcc.Store for cross function use
    np_array_points_coords = np.squeeze(points_coords)
    dcc_local_crs_waypoints = json.dumps({"x": np_array_points_coords[:,0].tolist() , "y" : np_array_points_coords[:,1].tolist() , "epsg": epsg_local})

    # output strings to update settings
    string_offset = f'Offset: {offset}'
    string_angle = f"Angle: {angle} degrees"
    string_buffer = f"Buffer: {buffer} meter(s)"

    return dcc_local_crs_waypoints, sh_linestring_flight_plan_crs_leaflet_geojson, string_angle, string_offset, string_buffer


@app.callback(
    Output("download_kml", "data"),
    Output("kml_clicks", "data"),
    Input("waypoints","data"),
    Input("download_kml_btn", "n_clicks"),
    Input('kml_clicks','data'),
    prevent_initial_call=True,
)
def download_kml(waypoints_dict, n_clicks, kml_clicks):
    kml_clicks = int(kml_clicks)
    if n_clicks == kml_clicks:
        raise PreventUpdate
    # Load coordinates in numpy array
    waypoints_dict = json.loads(waypoints_dict)
    xy_coords = np.array([waypoints_dict['x'], waypoints_dict['y']]).T
    
    # Specify crs
    epsg_local = waypoints_dict['epsg']
    epsg_kml = 4326

    # Transorm to kml crs
    transformer5 = Transformer.from_crs(epsg_local,
                                        epsg_kml)
    x = xy_coords[:,0]
    y = xy_coords[:,1]
    transformed_lat, transformed_lon = transformer5.transform(x, y)
    
    # print('lat',transformed_lat ,'lon',transformed_lon )



    # kml = skml.Kml()
    # for i in range(len(transformed_lat)):
    #     pnt = kml.newpoint(name=f"waypoint {i}",
    #                coords=[(transformed_lon[i],transformed_lat[i],0)])
    # # lin = kml.newlinestring(name="Pathway", description="A pathway in Kirstenbosch",
    # #                     coords=[(18.43312,-33.98924), (18.43224,-33.98914),
    # #                             (18.43144,-33.98911), (18.43095,-33.98904)])
    # kml_string = kml.kml()
    heights = np.zeros(len(transformed_lon))
    # kml_string = to_dji_kml(transformed_lat, transformed_lon, heights)
    
    points = []
    for i, (lat, lon, height) in enumerate(zip(transformed_lat, transformed_lon, heights)):
        point = dji_kml.dji_waypoint_mission(i, lon, lat)
        point.add_hover_action(2)
        point.add_yaw_action(-5)
        test = point.build_waypoint_kml()
        points.append(test)

    dji_mission = dji_kml.dji_kml(points, 80, 5, 80)
    dji_mission.build_kmz('test.kmz')

    fh = open('test.kmz', 'rb')

    kmz_bytearray = bytes(fh.read())

    kmz_dict_download = dcc.send_bytes(kmz_bytearray, 'finaly.kmz')
    

    # point1 = dji_waypoint_mission(10, 4.233, 52.00)

    # point1 = point1.build_waypoint_kml()
    # point2 = dji_waypoint_mission(10, 4.233, 52.00)
    # point2.add_hover_action(22)
    # point2.add_yaw_action(-5)
    # point2 = point2.build_waypoint_kml()

    # test = dji_kml(
    #                     [point1, point2],
    #                     80,
    #                     5,
    #                     80,
    # )

    return kmz_dict_download, str(n_clicks) #dict(content=python_string, filename="flightplan_new_method.kml")

## Function 1
# Find leaflet crs polygon coordinates
# then reproject polygon to utrs or rd and store this. proj

## Function 3
# Reproject back to leaflet crs.

## Function 2
# then find the boundingbox in leaflet crs with specified angle
# Find intersection: lines with angle (param) distance_between_flight_lines (param) and polygon
# should be possible with shapely
# Give buffer length (param) to these waypoints.
# Should be able to update live
# function 3 to reproject back to leaflet crs
# plot flightplan

## Function 4
# Download polygon
# Download waypoint kml
if __name__ == '__main__':
    app.run_server(port=7781, debug=True)