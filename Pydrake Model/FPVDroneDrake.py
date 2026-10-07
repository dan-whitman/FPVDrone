##### CONFIG
# must run the following in terminal to establish pydrake environment variables:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''
# will work out an automatica way to establish this later

##### PYDRAKE IMPORTS
# from pydrake.all import 

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.sensors import CameraConfig, ApplyCameraConfig, CameraInfo
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, Propeller, PropellerInfo, CoulombFriction
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration
from pydrake.geometry import StartMeshcat, HalfSpace, ProximityProperties, AddContactMaterial

##### OTHER IMPORTS
from pathlib import Path
import argparse
import numpy as np
import matplotlib.pyplot as plt
import cv2

##### SELF-DEFINED IMPORTS
from utils.xacro import XacroToURDF
from utils.camera import CameraMeasure
from utils.figure import PlotData
from utils.video import WriteVideoText, WriteVideoPoint

##### USER INPUTS
user_name = 'DAN'
camera_width = 640 # (px)
camera_height = 480 # (px)
camera_fps = 15 # (fps)
camera_fx = 500 # (px)
camera_fy = 500 # (px)

simulation_duration = 5.0 # (s)
drone_initial_position = [0.0, 0.0, 0.5] # (m)
drone_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
target_initial_position = [1.0, 3.0, 0.0] # (m)

vid_file_name = 'fpv_camera_' + user_name + '.avi'
heading_file_name = 'camera_heading_' + user_name
pixel_file_name = 'pixel_measurement_' + user_name
data_file_name = 'model_data_' + user_name

##### PARSING ARGUMENTS
argparser = argparse.ArgumentParser(description='Perception Midterm Quaddrone Model') # creating parser

argparser.add_argument('--frames', type=int, required=False, default=1, help='(1) for model frame visibility, (!=1) for no frames')
argparser.add_argument('--video', type=int, required=False, default=1, help='(1) for simulation video output, (!=1) for no video')
argparser.add_argument('--figures', type=int, required=False, default=1, help='(1) for figure outputs, (!=1) for no figures')
argparser.add_argument('--data', type=int, required=False, default=1, help='(1) for data output, (!=1) for no data')

args = argparser.parse_args() # getting the arguments

##### MAKING DIRECTORIES
MOD_DIR = Path('models')
VID_DIR = Path('vids')
FIG_DIR = Path('figs')
DAT_DIR = Path('data')
UTI_DIR = Path('utils')

MOD_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
VID_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
FIG_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
DAT_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
UTI_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

##### DRAKE MODEL
builder = DiagramBuilder() # initiating the builder

##### MATHEMATICAL MODEL
print(f'\n=====CREATING PLANT=====')
plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=0.0) # creating the plant and scene graph

parser = Parser(plant) # initialize parser
inspector = scene_graph.model_inspector() # initialize inspector

# extracting constants
gravity = plant.gravity_field().gravity_vector()
g_mag = np.linalg.norm(gravity)

# adding floor to world
X_WG = RigidTransform.Identity() # identity transform

proximity_properties = ProximityProperties()
surface_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5) # floor friction
AddContactMaterial(friction=surface_friction, properties=proximity_properties)

plant.RegisterCollisionGeometry(
    body=plant.world_body(), X_BG=X_WG, shape=HalfSpace(), name='ground_collision', properties=proximity_properties
)

# loading all models to parser
Quaddrone = XacroToURDF(str(MOD_DIR / 'FPVDrone.urdf.xacro'))
Target = XacroToURDF(str(MOD_DIR / 'Target.urdf.xacro'))

parser.AddModelsFromString(Quaddrone, 'urdf')
parser.AddModelsFromString(Target, 'urdf')

##### CREATING PROPELLERS
# finding prop bodies and frames
drone_prop_names = ['drone_prop_1', 'drone_prop_2', 'drone_prop_3', 'drone_prop_4'] # link names
drone_prop_bodies = [plant.GetBodyByName(drone_prop_name) for drone_prop_name in drone_prop_names] # model bodies

# adding the propellers
drone_prop_info = []
for i, drone_prop_body in enumerate(drone_prop_bodies):
    thrust_ratio = 1.0 # thrust ratio for prop
    moment_ratio = 0.1 * (-1)**i # moment ratio for prop (pos and negative to signify direction of prop spin)

    X_DP = RigidTransform.Identity() # identity transform

    drone_prop_info.append(PropellerInfo(drone_prop_body.index(), X_BP=X_DP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for arm

##### CREATING CAMERA
camera_body = plant.GetBodyByName('drone_camera')

# creating camera configuration
config = CameraConfig()
config.name = 'drone_camera'
config.width = camera_width
config.height = camera_height
config.fps = camera_fps
config.focal = CameraConfig.FocalLength(x=camera_fx, y=camera_fy)
config.X_PB.base_frame = 'drone::drone_camera'
config.rgb = True 
config.depth = False
config.label = False

camera_info = CameraInfo(
    config.width, config.height, config.focal_x(), config.focal_y(), *config.principal_point()
)

ApplyCameraConfig(config=config, builder=builder, scene_graph=scene_graph)

##### ADDING VISUAL FRAMES
if args.frames: # if adding visual frames
    print(f'\n=====ADDING FRAMES=====')
    for i in range(plant.num_model_instances()): # for each model
        model_instance = ModelInstanceIndex(i) # get the model
        body_indices = plant.GetBodyIndices(model_instance) # get each body

        for body_index in body_indices: # for each body
            body = plant.get_body(body_index) 

            # add frame
            AddFrameTriadIllustration(
                scene_graph=scene_graph, plant=plant, body=body, length=0.15, radius=0.005
            )

##### BUILDING MODEL
print(f'\n=====BUILDING MODEL=====')
plant.Finalize() # finalize the plant

# connecting props to model
propellers = builder.AddSystem(Propeller(drone_prop_info)) # creating the propellers

builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

builder.ExportInput(propellers.get_command_input_port(), 'propeller_thrusts')
# builder.ExportOutput(plant.get_state_output_port(), 'plant_state')

##### VISUALIZING MODEL
print(f'\n=====STARTING VISUALIZATION=====')
meshcat = StartMeshcat() # initialize meshcat
# print(f'Open this URL in your browser: {meshcat.web_url()}')

AddDefaultVisualization(builder=builder, meshcat=meshcat)

##### FINALIZING BUILD
diagram = builder.Build() # building final diagram

##### PLACING MODELS IN SCENE
context = diagram.CreateDefaultContext() # creating numerical context
plant_context = plant.GetMyMutableContextFromRoot(context)

drone_instance = plant.GetModelInstanceByName('drone') # drone instance
target_instance = plant.GetModelInstanceByName('target') # target instance

# placing the drone
drone_body = plant.GetBodyByName('drone_base')
X_WD = RigidTransform(drone_initial_rpy, drone_initial_position) # target transform
plant.SetFreeBodyPose(context=plant_context, body=drone_body, X_JpJc=X_WD)

# placing the target
target_body = plant.GetBodyByName('target_base')
X_WT = RigidTransform(RollPitchYaw(0.0, 0.0, 0.0), target_initial_position) # target transform
plant.SetFreeBodyPose(context=plant_context, body=target_body, X_JpJc=X_WT)

##### INITIAL CONDITIONS
q_num = plant.num_positions() # number of q coords
v_num = plant.num_velocities() # number of velocities
u_num = propellers.get_command_input_port().size() # number of control inputs

# defining initial propeller thrusts
drone_mass = plant.CalcTotalMass(plant_context, [drone_instance])
prop_thrust = 1.00 * g_mag * drone_mass / u_num # splitting thurst over all props
u_zero = prop_thrust * np.ones(u_num) # setting control

u_zero[0] = 0.998 * u_zero[0]
u_zero[2] = 0.998 * u_zero[2]
u_zero[1] = 1.002 * u_zero[1]
u_zero[3] = 1.002 * u_zero[3]

diagram_input_port = diagram.get_input_port(0)
diagram_input_port.FixValue(context, u_zero)

##### RUNNING SIMULATION
print(f'\n=====BEGINNING SIMULATION=====')
meshcat.StartRecording(set_visualizations_while_recording=True) # begin recording

simulator = Simulator(system=diagram, context=context)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)

frame_times = np.arange(0.0, simulation_duration, 1.0 / camera_fps)
# num_frames = len(frame_times)

camera = diagram.GetSubsystemByName('rgbd_sensor_drone_camera') # getting camera

# making video output
if args.video == 1: # if outputting video
    video = cv2.VideoWriter(
        filename=str(VID_DIR / vid_file_name), apiPreference=0, fourcc=cv2.VideoWriter_fourcc(*'MJPG'), fps=camera_fps, frameSize=(camera_width, camera_height)
    )

# data values
times = []
camera_headings = []

drone_positions = []
drone_rpys = []
drone_velocities = []
drone_w_velocities = []

us = []
vs = []

# step through simulation
for t in frame_times:
    # t = frame / camera_fps # simulation time
    simulator.AdvanceTo(t) # advance simulation
    camera_sim_context = camera.GetMyContextFromRoot(simulator.get_context()) # get camera context at t
    plant_sim_context = plant.GetMyContextFromRoot(simulator.get_context()) # get plant context at t

    # finding data of interest
    X_WC = camera.body_pose_in_world_output_port().Eval(camera_sim_context) # finding camera pose
    X_CW = X_WC.inverse() # finding world in camera
    R_WC = X_WC.rotation().matrix() # camera rotation (in world frame)
    heading_WC = R_WC @ np.array([0.0, 0.0, 1.0]) # extracting heading

    X_WD = plant.EvalBodyPoseInWorld(plant_sim_context, drone_body) # finding drone pose
    V_WD = plant.EvalBodySpatialVelocityInWorld(plant_sim_context, drone_body) # finding drone velocoties
    p_WD = X_WD.translation() # drone position (in world frame)
    R_WD = X_WD.rotation().matrix() # drone rotation (in world frame)
    rpy = RollPitchYaw(R_WD) # finding rpy
    rpy = [rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()] # finding rpy
    v_WD = V_WD.translational() # drone linear velocity (in world frame)
    w_WD = V_WD.rotational() # drone rotational velocoty (in world frame)

    X_WT = plant.EvalBodyPoseInWorld(plant_sim_context, target_body) # finding target pose
    x_T = X_WT.translation() # target position (in world frame)

    u, v = CameraMeasure(config, X_CW, x_T) # finding target measurement in camera

    # appending to data variables
    times.append(t)
    camera_headings.append(heading_WC.copy())

    drone_positions.append(p_WD.copy())
    drone_rpys.append(rpy.copy())
    drone_velocities.append(v_WD.copy())
    drone_w_velocities.append((R_WD.T @ np.array(w_WD)).copy())

    us.append(u)
    vs.append(v)

    if args.video == 1: # if outputting video
        image = camera.color_image_output_port().Eval(camera_sim_context) # getting color image 
        rgba = image.data # extracting data
        rgba = rgba.copy() # creating copy for manipulation

        rgba = WriteVideoText(rgba=rgba, textflag=u == None, t=t) # add text to image
        rgba = WriteVideoPoint(rgba=rgba, coords=(u, v)) # add point to image

        rgb = rgba[:,:,:3] # removing apha channel
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # converting to bgr for opencv
        video.write(bgr) # add frame to video

# convert data and parse
times = np.array(times)
camera_headings = np.array(camera_headings)

drone_positions = np.array(drone_positions)
drone_rpys = np.array(drone_rpys)
drone_velocities = np.array(drone_velocities)
drone_w_velocities = np.array(drone_w_velocities)

us = np.array(us)
vs = np.array(vs)

# ending simulation
meshcat.StopRecording()
meshcat.PublishRecording() # watch simulation

##### OUTPUTTING DATA
print(f'\n=====SAVING RESULTS=====')

# exporting video
if args.video == 1: # if outputting video
    video.release() # save video
    print('Video saved!')

# exporting data
if args.data == 1: # if outputting data
    np.savez(str(DAT_DIR / data_file_name), 
        times=times, drone_positions=drone_positions, drone_rpys=drone_rpys, drone_velocities=drone_velocities, drone_w_velocities=drone_w_velocities
    )
    print('Data saved!')

# plotting results
if args.figures == 1: # if outputting figures
    plotdata = [times, camera_headings, us, vs, camera_width, camera_height] # combining data to pass to function (bad way to do this, rethink later)
    figheadings, figtargetpixels = PlotData(plotdata=plotdata) # plotting figures

    figheadings.savefig(str(FIG_DIR / heading_file_name), dpi=300, bbox_inches='tight')
    figtargetpixels.savefig(str(FIG_DIR / pixel_file_name), dpi=300, bbox_inches='tight')

    print('Figures saved!')

##### PRINTING INFO
print(f'\n=====CAMERA INFO=====')
print('fx:', config.focal_x())
print('fy:', config.focal_y())
print('ux:', config.principal_point()[0])
print('uy:', config.principal_point()[1])
print('Horizontal FOV:', np.degrees(camera_info.fov_x()), 'deg')
print('Vertical FOV:', np.degrees(camera_info.fov_y()), 'deg')

input(f'\nKeeping Meshcat alive!')