##### PYDRAKE IMPORTS
# from pydrake.all import 

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.sensors import CameraConfig, ApplyCameraConfig, CameraInfo
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, Propeller, PropellerInfo, CoulombFriction
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration
from pydrake.geometry import StartMeshcat, HalfSpace, ProximityProperties, AddContactMaterial

##### OTHER IMPORTS
import argparse
import numpy as np
import matplotlib.pyplot as plt
# from PIL import Image
import cv2
from pathlib import Path

##### SELF-DEFINED IMPORTS
from utils.XacroToURDF import XacroToURDF

##### PARSING ARGUMENTS
argparser = argparse.ArgumentParser(description='Perception Midterm Quadcopter Model') # creating parser

argparser.add_argument('--frames', type=int, required=False, default=1, help='(1) for model frames, (!=1) for no frames')

args = argparser.parse_args() # getting the arguments

##### MAKING DIRECTORIES
VID_DIR = Path('vids')
FIG_DIR = Path('figs')

VID_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
FIG_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

##### USER INPUTS
user_name = 'DAN'
camera_height = 480 # (px)
camera_width = 480 # (px)
camera_fps = 15 # (fps)
simulation_duration = 5.0
vid_file_name = 'fpv_camera_' + user_name + '.avi'
heading_file_name = 'camera_heading_' + user_name
orient_file_name = 'drone_orient_' + user_name

##### DRAKE MODEL
builder = DiagramBuilder() # initiating the builder

##### MATHEMATICAL MODEL
plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=0.0) # creating the plant and scene graph

parser = Parser(plant) # initialize parser
inspector = scene_graph.model_inspector() # initialize inspector

# adding floor to world
X_BG = RigidTransform.Identity() # identity transform

proximity_properties = ProximityProperties()
surface_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5) # floor friction
AddContactMaterial(friction=surface_friction, properties=proximity_properties)

plant.RegisterCollisionGeometry(
    plant.world_body(), X_BG, HalfSpace(), 'ground_collision', proximity_properties
)

# loading all models to parser
Quadcopter = XacroToURDF('models/FPVDrone.urdf.xacro')
Target = XacroToURDF('models/Target.urdf.xacro')

parser.AddModelsFromString(Quadcopter, 'urdf')
parser.AddModelsFromString(Target, 'urdf')

##### CREATING PROPELLERS
# finding prop bodies and frames
copter_prop_names = ['copter_prop_1', 'copter_prop_2', 'copter_prop_3', 'copter_prop_4'] # link names
copter_prop_bodies = [plant.GetBodyByName(copter_prop_name) for copter_prop_name in copter_prop_names] # model bodies

# adding the propellers
copter_prop_info = []
for i, copter_prop_body in enumerate(copter_prop_bodies):
    thrust_ratio = 1.0 # thrust ratio for prop
    moment_ratio = 0.1 * (-1)**i # moment ratio for prop (pos and negative to signify direction of prop spin)

    X_BP = RigidTransform.Identity() # identity transform

    copter_prop_info.append(PropellerInfo(copter_prop_body.index(), X_BP=X_BP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for split

    if args.frames == 1: # if adding frames to visualization
        AddFrameTriadIllustration(
            scene_graph=scene_graph, plant=plant, body=copter_prop_body, length=0.15, radius=0.005
        )

##### CREATING CAMERA
camera_body = plant.GetBodyByName('copter_camera')

config = CameraConfig() # creating camera configuration
config.name = 'copter_camera'
config.width = camera_width
config.height = camera_height
config.fps = camera_fps
config.X_PB.base_frame = 'copter::copter_camera'
config.rgb = True 
config.depth = False
config.label = False

camera_info = CameraInfo(
    config.width,
    config.height,
    config.focal_x(),
    config.focal_y(),
    *config.principal_point()
)

ApplyCameraConfig(config=config, builder=builder, scene_graph=scene_graph)

if args.frames == 1: # if adding frames to visualization
    AddFrameTriadIllustration(
        scene_graph=scene_graph, plant=plant, body=camera_body, length=0.15, radius=0.005
    )

##### BUILDING MODEL
plant.Finalize() # finalize the plant

# connecting props to model
propellers = builder.AddSystem(Propeller(copter_prop_info)) # creating the propellers

builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

builder.ExportInput(propellers.get_command_input_port(), 'propeller_thrusts')
# builder.ExportOutput(plant.get_state_output_port(), 'plant_state')

##### VISUAL MODEL
print(f'\n=====STARTING VISUALIZATION=====')
meshcat = StartMeshcat() # initialize meshcat
# print(f'Open this URL in your browser: {meshcat.web_url()}')

AddDefaultVisualization(builder=builder, meshcat=meshcat)

##### FINALIZING BUILD
diagram = builder.Build() # building final diagram

##### CONFIGURATION
gravity = plant.gravity_field().gravity_vector()
g_mag = np.linalg.norm(gravity)

##### PLACING MODELS IN SCENE
context = diagram.CreateDefaultContext() # creating numerical context
plant_context = plant.GetMyMutableContextFromRoot(context)

copter_instance = plant.GetModelInstanceByName('copter') # copter instance
target_instance = plant.GetModelInstanceByName('target') # target instance

# placing the copter
copter_body = plant.GetBodyByName('copter_base')
X_WC = RigidTransform(RollPitchYaw(0.0, 0.0, 0.0), [0.0, 0.0, 1.0]) # target transform
plant.SetFreeBodyPose(context=plant_context, body=copter_body, X_JpJc=X_WC)

# placing the target
target_body = plant.GetBodyByName('target_base')
X_WT = RigidTransform(RollPitchYaw(0.0, 0.0, 0.0), [1.0, 3.0, 0.0]) # target transform
plant.SetFreeBodyPose(context=plant_context, body=target_body, X_JpJc=X_WT)

##### INITIAL CONDITIONS
q_num = plant.num_positions() # number of q coords
v_num = plant.num_velocities() # number of velocities
u_num = propellers.get_command_input_port().size() # number of control inputs

# defining initial propeller thrusts
copter_mass = plant.CalcTotalMass(plant_context, [copter_instance])
prop_thrust = 1.00 * g_mag * copter_mass / u_num # splitting thurst over all props
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

simulator = Simulator(system=diagram, context=context) # starting simulation (for visualization)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)

##### RUNNING SIMULATION
# num_frames = int(simulation_duration * camera_fps)
frame_times = np.arange(0.0, simulation_duration, 1.0 / camera_fps)

camera = diagram.GetSubsystemByName('rgbd_sensor_copter_camera') # getting camera

# making video output
video = cv2.VideoWriter(
    str(VID_DIR / vid_file_name),
    cv2.VideoWriter_fourcc(*'MJPG'),
    camera_fps,
    (camera_width, camera_height),
)
# print('Video writer opened:', video.isOpened())

# data values
times = []
camera_positions = []
camera_headings = []

drone_rpys = []

# step through simulation
for t in frame_times:
    # t = frame / camera_fps # simulation time
    simulator.AdvanceTo(t) # advance simulation
    camera_sim_context = camera.GetMyContextFromRoot(simulator.get_context()) # get camera context at t
    plant_sim_context = plant.GetMyContextFromRoot(simulator.get_context()) # get plant context at t
    
    color_image = camera.color_image_output_port().Eval(camera_sim_context) # getting color image 
    rgba = color_image.data # extracting data
    rgb = rgba[:, :, :3] # removing apha channel

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # converting to bgr for opencv
    video.write(bgr) # add frame to video

    # finding data of interest
    X_WC = camera.body_pose_in_world_output_port().Eval(camera_sim_context) # finding camera in world
    p_WC = X_WC.translation() # camera position
    R_WC = X_WC.rotation().matrix() # camera rotation
    heading_W = R_WC @ np.array([0.0, 0.0, 1.0]) # extracting heading

    X_WB = plant.EvalBodyPoseInWorld(plant_context, copter_body) # finding drone pose
    rpy = RollPitchYaw(X_WB.rotation()) # drone rotation
    rpy = [rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()] # finding rpy

    # appending to data variables
    times.append(t)
    camera_positions.append(p_WC.copy())
    camera_headings.append(heading_W.copy())

    drone_rpys.append(rpy.copy())

print(f'\n=====SAVING RESULTS=====')
video.release() # save video
print('Video saved!')

# convert data to numpy
times = np.array(times)
camera_positions = np.array(camera_positions)
camera_headings = np.array(camera_headings)

drone_rpys = np.array(drone_rpys)

# plotting results!
plt.figure()
plt.plot(times, camera_headings[:, 0], label='x')
plt.plot(times, camera_headings[:, 1], label='y')
plt.plot(times, camera_headings[:, 2], label='z')
plt.xlabel(r'Time ($s$)')
plt.ylabel('Camera heading component')
plt.title('Camera Heading in World Frame')
plt.legend()
plt.grid()
plt.savefig(str(FIG_DIR / heading_file_name), dpi=300, bbox_inches='tight')
# plt.show()

plt.figure()
plt.plot(times, drone_rpys[:, 0], label=r'$\phi$')
plt.plot(times, drone_rpys[:, 1], label=r'$\theta$')
plt.plot(times, drone_rpys[:, 2], label=r'$\psi$')
plt.xlabel(r'Time ($s$)')
plt.ylabel('Drone orientation component')
plt.title('Drone Orientation in World Frame')
plt.legend()
plt.grid()
plt.savefig(str(FIG_DIR / orient_file_name), dpi=300, bbox_inches='tight')
# plt.show()

print('Figures saved!')

meshcat.StopRecording()
meshcat.PublishRecording() # watch simulation

##### PRINTING INFO
print(f'\n=====CAMERA INFO=====')
print('fx:', config.focal_x())
print('fy:', config.focal_y())
print('Horizontal FOV:', np.degrees(camera_info.fov_x()), 'deg')
print('Vertical FOV:', np.degrees(camera_info.fov_y()), 'deg')
print('')

input('Keeping Meshcat alive!')