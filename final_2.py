from robotic import ry
import numpy as np
import time
import matplotlib.pyplot as plt
import quaternion
from utils.robotUtils import mapTOWorldSpace
from utils.cv_func import  arucoDetection
import math
from svgpathtools import svg2paths2, real, imag

RobotGlobal = ry.Config()
# C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
RobotGlobal.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
cameraType = 'cameraWrist'
# cameraType = 'camera'
SIMULATION_ANGLE = False
RealRObot = True
HEIGHT_OFFSET = 0.028
ROT_OFFSET = 0
TABLE_0POS = 0.605

DEBUG = False
SIMULATION_ANGLE_OF_OBJECT = 190

EXT_CAM = [0,0.93,0]



#joint_limit_offset = [0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0]
JOINT_LIMIT_OFFSET = [-0.1]

SPEED_MULTIPLIER = 0.8
SECS_TO_HIDE = 1.4
PAINTING_SPEED = 0.35
MOVING_SPEED = 0.5
MOVE_ADD_TIME = 0.0
HIDING_ANLGE = 25
RESULTION = 50
PICTURE_SIZE = 0.8 #MAX 0.6 with full size circle
Z_OFFSET_CENTER = -0.1
Y_OFFSET = -0.15 # offset in y direction for the whole drawing
CLOSE_DISTANCE = 0.01 # distance in meter where tunring light away is deemed unnecessary
Y_RANGE = -0.3 # range in y direction for stroke width levels, set to 0 to disable depth
DEPTH_CONNECT_TOLERANCE = 0.08 # tolerance for connecting paths in depth, set to 0 to disable connecting

MIN_CONST_VELOCITY_STEPS = 4

OPTIMIZE_TRAJECTORY_ORDER = True
CONTINUOUS_DRAWING = True
FOCUS_ON_CAM = False
KOMO_VIEW = False
REAL_ROBOT = RealRObot
FILENAME = 'cube_depth_big.svg'


resultion = RESULTION


cameraFrame = RobotGlobal.getFrame(cameraType)
# cameraFrame.setPosition([0,-0.18,1.2])
# cameraFrame.setPose('t(.2 0.3 1.8) d(45 0 1 0)')


camerMarker = RobotGlobal.addFrame('cameraMarker', cameraType)
camerMarker.setShape(ry.marker, size=[.2])
camerMarker.setRelativePosition([0,0,0.0])

boxSize = 0.07
heightFactor = 0.7
lengthFactor = 2.5

obj = RobotGlobal.addFrame('obj')
obj.setPose('t(0. 0.5 0.8)')
obj.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
obj.setColor([1,.0,0])
# obj.setMass(.1)
obj.setContact(False)

obj.setPose(f't(-0.5 0.0 0.8) d({SIMULATION_ANGLE_OF_OBJECT} 0 0 1)')


# create a 5 by grid of black and white boxes that lie on the surface on the "obj" box and all together are the size of the box surface. The boxes are all quadratic and flat. 
# The color is given by this 2D array
color = [[0,0,0,0,0,0],[0,0,0,1,1,0],[0,0,0,1,1,0],[0,0,0,1,0,0],[0,1,1,0,1,0],[0,0,0,0,0,0]]
#create rotated color array
color = np.rot90(color, k=1, axes=(1, 0))

for i in range(6):
    for j in range(6):
        box = RobotGlobal.addFrame('box_'+str(i)+'_'+str(j), 'obj')
        #box.setShape(ry.ST.ssBox, size=[.01,.01,.001,.00])
        box.setShape(ry.ST.ssBox, size=[boxSize/6.0, boxSize/6.0, 0.001, 0.00])
        box.setColor([color[i][j],color[i][j],color[i][j]])
        # box.setMass(0)
        box.setContact(False)
        #box.setRelativePose('t('+str(-0.02+0.01*i)+' '+str(-0.02+0.01*j)+' 0.025)')
        box.setRelativePose('t('+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*i)+' '+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*j)+' ' + str(heightFactor*boxSize/2.0) +  ')')


# set initial pose of the robot
objWaypoint = RobotGlobal.addFrame('objWaypoint', 'obj')
objWaypoint.setRelativePosition([0,0,0.1 ])
objWaypoint.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
objWaypoint.setColor([1,.0,0,0.2])

if SIMULATION_ANGLE:
    objWaypoint.setRelativePose('t(0 0 0.5) d(10 0 0 1) d(10 0 1 0)')
RobotGlobal.view()


input("Press Enter to move home..")

bot = ry.BotOp(RobotGlobal, RealRObot)
bot.home(RobotGlobal)
while bot.getTimeToEnd()>0:
    bot.sync(RobotGlobal, .1)

# komo problem for initial pose
komo = ry.KOMO()
komo.setConfig(RobotGlobal, True)
komo.setTiming(2., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.poseDiff, ['l_gripper', 'objWaypoint'], ry.OT.eq, [1e1]);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
print(ret)

# komo.view(True, "waypoints solution")
path = komo.getPath()



input("Press Enter to move to torch initial location...")

bot.move(path, [3])
while bot.getTimeToEnd()>0:
    bot.sync(RobotGlobal, .1)

input("press ENTER to open gripper")
bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(RobotGlobal, .1)



boxMarker = RobotGlobal.addFrame('boxMarker', 'obj')
boxMarker.setShape(ry.marker, size=[.2])
boxMarker.setRelativePosition([0,0,0.0])

glob = RobotGlobal.addFrame('glob')
glob.setShape(ry.marker, size=[.5])
glob.setPose('t(0. 0.0 0.0)')





#cameraFrame = C.addFrame("myCamera")
#cameraFrame.setShape(ry.ST.marker, [0.3])
#cameraFrame.setPosition([0,0,2.0])
#cameraFrame.setPosition([0,1.0,2.0])
#cameraFrame.setQuaternion([1,-0.5,0,1])

RobotGlobal.view()







rgb, depth = bot.getImageAndDepth(cameraType)

fxypxy = bot.getCameraFxypxy(cameraType)
print(fxypxy)
depth.shape
cameraFrame = RobotGlobal.getFrame(cameraType)

# fig = plt.figure(figsize=(10,5))
# axs = fig.subplots(1, 2)
# axs[0].imshow(rgb)
# axs[1].matshow(depth)
# plt.show()
import time
time.sleep(0.5)

#include opencv
import cv2

# open webcam
# cap = rgb
x = 0
y = 0

ids = None

while ids is None:
    time.sleep(1)
    print('looking for aruco marker')
    rgb, depth = bot.getImageAndDepth(cameraType)
    frame,ids,corners = arucoDetection(rgb)
    # time.sleep(0.5)
    


# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
# aruco_params =  cv2.aruco.DetectorParameters()
# detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


# # loop through frames
# #while True:
# # read frame from webcam
# frame = rgb
# # convert frame to grayscale
# gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
# # detect aruco markers
# corners, ids, rejected = detector.detectMarkers(gray)
# #print(ids)
# # draw markers on frame
# frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)




def mapTOWorldSpace_old(point,z,fxypxy):
    '''
        Maps a point from pixel space to world space
        :param point: The point to be mapped    
        :param z: The depth of the point
        :param fxypxy: The camera matrix and distortion coefficients
        :return: The point in world space
    '''
    fx, fy = fxypxy[0], fxypxy[1]
    px, py = fxypxy[2], fxypxy[3]
    x = (point[0] - px) * z / fx
    y = (point[1] - py) * -z / fy
    z = -z
    point = np.array([x, y, z])

    return point


RobotGlobal.view()
#get center of marker
print(ids)
if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0])/4)
        y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1])/4)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
        centerOfAruco = (y,x)
        #calculate width of the marker in pixel space
        w = int(c[2][0] - c[0][0])
        #calculate height in pixel space
        h = int(c[2][1] - c[0][1])
        # print out all elements of c
        print(c)
# get center from 2 markers
if ids is not None:
    if len(ids) == 2:
        c1 = corners[0][0]
        c2 = corners[1][0]
        x = int((c1[0][0] + c1[1][0] + c1[2][0] + c1[3][0] + c2[0][0] + c2[1][0] + c2[2][0] + c2[3][0])/8)
        y = int((c1[0][1] + c1[1][1] + c1[2][1] + c1[3][1] + c2[0][1] + c2[1][1] + c2[2][1] + c2[3][1])/8)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

# for each marker get the center of the edge of all 4 sides
if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        # top left
        x1 = int((c[0][0] + c[1][0])/2)
        y1 = int((c[0][1] + c[1][1])/2)
        cv2.circle(frame, (x1, y1), 4, (255, 0, 0), -1)
        # top right
        x2 = int((c[1][0] + c[2][0])/2)
        y2 = int((c[1][1] + c[2][1])/2)
        cv2.circle(frame, (x2, y2), 4, (0, 255, 0), -1)
        # bottom right
        x3 = int((c[2][0] + c[3][0])/2)
        y3 = int((c[2][1] + c[3][1])/2)
        cv2.circle(frame, (x3, y3), 4, (0, 0, 255), -1)
        # bottom left
        x4 = int((c[3][0] + c[0][0])/2)
        y4 = int((c[3][1] + c[0][1])/2)
        cv2.circle(frame, (x4, y4), 4, (255, 255, 0), -1)
       





# display frame using matplotlib
plt.imshow(frame)
plt.show()

     

RedPoint1 = mapTOWorldSpace((y1,x1),depth[y1,x1],fxypxy)
BluePoint2 = mapTOWorldSpace((y2,x2),depth[y2,x2],fxypxy)
YellowPoint3 = mapTOWorldSpace((y3,x3),depth[y3,x3],fxypxy)
GreenPoint4 = mapTOWorldSpace((y4,x4),depth[y4,x4],fxypxy)
# centerPoint = mapTOWorldSpace((cy,cx),depth[cy,cx],fxypxy
centerPoint = mapTOWorldSpace(centerOfAruco,depth[centerOfAruco[0],centerOfAruco[1]],fxypxy)



RedFrame = RobotGlobal.addFrame("RedFramePoint",cameraType)
RedFrame.setRelativePosition(RedPoint1)
print('\nRedPoint1: ',RedPoint1)
RedFrame.setShape(ry.ST.sphere, [0.019])
RedFrame.setColor([1,0,0])

BlueFrame = RobotGlobal.addFrame("BlueFramePoint",cameraType)
BlueFrame.setRelativePosition(BluePoint2)
BlueFrame.setShape(ry.ST.sphere, [0.019])
BlueFrame.setColor([0,0,1])
print('\nBluePoint2: ',BlueFrame.getPosition())

YelloFrame = RobotGlobal.addFrame("YelloFramePoint",cameraType)
YelloFrame.setRelativePosition(YellowPoint3)
YelloFrame.setShape(ry.ST.sphere, [0.019])
YelloFrame.setColor([1,1,0])
print('\nYellowPoint3: ',YellowPoint3)

GreenFrame = RobotGlobal.addFrame("GreenFramePoint",cameraType)
GreenFrame.setRelativePosition(GreenPoint4)
GreenFrame.setShape(ry.ST.sphere, [0.019])
GreenFrame.setColor([0,1,0])
print('\nGreenPoint4: ',GreenPoint4)

CenterFrame = RobotGlobal.addFrame("CenterFramePoint",cameraType)
CenterFrame.setRelativePosition(centerPoint)
CenterFrame.setShape(ry.ST.sphere, [0.019])
CenterFrame.setColor([1,0,1])
print('\nCenterPoint: ',centerPoint)
obj.setColor([1,0,1,0.3])
if DEBUG:
    input("Press Enter to view the 4 points...")
RobotGlobal.view()







# find normalised vector of plane
# find vector between two points
#convert world points to numpy array
RedPoint1 = np.array(RedPoint1)
BluePoint2 = np.array(BluePoint2)
YellowPoint3 = np.array(YellowPoint3)
GreenPoint4 = np.array(GreenPoint4)


v1 = RedPoint1 - BluePoint2
v2 = RedPoint1 - YellowPoint3

lengthAxis = RedPoint1 - BluePoint2
widthAxis = GreenPoint4 - YellowPoint3

# convert 2 points to global space
RedPoint1Global = RobotGlobal.addFrame("RedPoint1Global")
RedPoint1Global.setPosition(RedFrame.getPosition())
RedPoint1Global.setQuaternion(RedFrame.getQuaternion())
RedPoint1Global.setShape(ry.ST.ssBox, [.03,.03,0.03,.005])
RedPoint1Global.setColor([1,1,1])



GreenPoint2Global = RobotGlobal.addFrame("GreenPoint2Global")

GreenPoint2Global.setPosition(GreenFrame.getPosition())
GreenPoint2Global.setQuaternion(GreenFrame.getQuaternion())
GreenPoint2Global.setShape(ry.ST.ssBox,  [.03,.03,0.03,.005])
RobotGlobal.view()


YellowPoint2Global = RobotGlobal.addFrame("YellowPoint2Global")

YellowPoint2Global.setPosition(YelloFrame.getPosition())
YellowPoint2Global.setQuaternion(YelloFrame.getQuaternion())
YellowPoint2Global.setShape(ry.ST.ssBox,  [.03,.03,0.03,.005])
RobotGlobal.view()
# find cross product
# heightAxis = np.cross(lengthAxis,widthAxis)

# normalise
# heightAxis = heightAxis/np.linalg.norm(heightAxis)
lengthAxis = lengthAxis/np.linalg.norm(lengthAxis)
widthAxis = widthAxis/np.linalg.norm(widthAxis)
heightAxis = np.cross(lengthAxis,widthAxis)
heightAxis = heightAxis/np.linalg.norm(heightAxis)

# length axis in global space
lenghtAxisGlobal = RedPoint1Global.getPosition() - YellowPoint2Global.getPosition()

import math
rot = math.atan2(lenghtAxisGlobal[1],lenghtAxisGlobal[0])
rot = np.degrees(rot)
# lenghtAxisGlobal = np.array([lenghtAxisGlobal[0], lenghtAxisGlobal[1], 0])/np.linalg.norm(np.array([lenghtAxisGlobal[0], lenghtAxisGlobal[1], 0]))
# find angle between 
# round to 3 decimal places
# normalisero

# lenghtAxisGlobal = lenghtAxisGlobal/np.linalg.norm(lenghtAxisGlobal)
# rot = np.arccos(-(lenghtAxisGlobal[1]/lenghtAxisGlobal[0]))
# rot = np.degrees(rot)

# x = [lenghtAxisGlobal[1], 0, 0]/np.linalg.norm([lenghtAxisGlobal[1], 0, 0]) 
# y = [0, lenghtAxisGlobal[0], 0]/np.linalg.norm([0, lenghtAxisGlobal[0], 0])
# rot = np.arccos(np.dot(lenghtAxisGlobal, np.array([1,0,0])))
# # rot = np.clip(rot, -np.pi/2, np.pi/2)
# # limit precision

# # covert to degrees
# rot = np.degrees(rot)


# convert to rotation matrix
# add vectors columnwise to matrix
rotationMatrix = np.column_stack((widthAxis,lengthAxis,heightAxis))

# boardQuarternion = quaternion.from_rotation_matrix(rotationMatrix)




# centerPoint = [0.,0.,0.]
torch = RobotGlobal.addFrame("torch",cameraType)
torch.setRelativePosition(centerPoint)
# torch.setRelativeQuaternion(quaternion.as_float_array(boardQuarternion))
torch.setShape(ry.marker, [.3])
torch.setColor([1,0,1])
if DEBUG:
    input("Press Enter to view the object marker...")

RobotGlobal.view()
torchReal = RobotGlobal.addFrame("torchReal",'torch')
string = f't(0 0 0)d(-90 0 0 1))'

torchReal.setRelativePose(string)
if DEBUG:
    input("Press Enter to rotate torch...")
torchReal.setShape(ry.marker, [.3])
torchReal.setColor([1,0,1,0.1])
torch.setShape(ry.marker, [.01])
RobotGlobal.view()


# # find distance from origin
# d = np.dot(normal,worldPoint1)
# # find plane
# plane = np.append(normal,d)
if DEBUG:
    input("set object on location")

# torchPyhsical = RobotGlobal.addFrame("torchPyhsical",'torchReal')
# torchPyhsical.setRelativePose('t(0 0 0)')
# torchPyhsical.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
# torchPyhsical.setColor([1,.0,0])
# torchPyhsical.setMass(.1)
# torchPyhsical.setContact(True)
# display frame
#cv2.imshow('frame', frame)
# check if user pressed 'q'
#cv2.waitKey(0)
print(x,y)
print(w,h)

RobotGlobal.view()
if DEBUG:

    input("way point")

# add gripping way point

# birdview = RobotGlobal.addFrame("birdview",'torchReal')
# birdview.setRelativePose('t(0 0 0.1)')
# birdview.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
# birdview.setColor([1,0,1,0.3])
# RobotGlobal.view()




# map to global coordinates
globalObjectWay1 =  RobotGlobal.addFrame("globalObjectWay1")
globalObjectWay1.setPosition(torchReal.getPosition())
positon = torchReal.getPosition()
# globalObjectWay1.setQuaternion(torchReal.getQuaternion())
string = f't({positon[0]} {positon[1]} {positon[2]-HEIGHT_OFFSET}) d({rot-90} 0 0 1)'

globalObjectWay1.setPose(string)
globalObjectWay1.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
globalObjectWay1.setColor([0,1,0])

globalObjectWay1_marker =  RobotGlobal.addFrame("globalObjectWay1_marker",'globalObjectWay1')
globalObjectWay1_marker.setShape(ry.marker, [.5])
globalObjectWay1_marker.setRelativePosition([0,0,0])

rotationMatrix = torchReal.getRotationMatrix()
vectorAlign = [rotationMatrix[0][1],rotationMatrix[1][1],0.0]

# globalObjectWay1.setMass(.1)
# globalObjectWay1.setContact(True)
RobotGlobal.view()

globalObjectWay0 =  RobotGlobal.addFrame("globalObjectWay0","globalObjectWay1")
globalObjectWay0.setRelativePose('t(0 0 0.1)')
globalObjectWay0.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
globalObjectWay0.setColor([1,0,1,0.3])
RobotGlobal.view()



input("Press Enter to continue...")

# define komo problem
komo = ry.KOMO()
komo.setConfig(RobotGlobal, True)
komo.setTiming(2., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([1.], ry.FS.poseDiff, ['l_gripper', 'globalObjectWay0'], ry.OT.eq, [1e1]);
komo.addObjective([2.], ry.FS.poseDiff, ['l_gripper', 'globalObjectWay1'], ry.OT.eq, [1e1]);
# komo.addObjective([2.], ry.FS.vector, ['l_gripper', 'globalObjectWay1'], ry.OT.eq, [1e1]);
# komo.addObjective([], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],vectorAlign)
# komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1],[0,0,1])





ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
print(ret)

komo.view(True, "waypoints solution")



# komo.view_close()
path = komo.getPath()
path_to_grip = path

# bot = ry.BotOp(C, False)

input("press ENTER to move to Gripping position")

# bot.home(RobotGlobal)
bot.move(path, [5])
while bot.getTimeToEnd()>0:
    bot.sync(RobotGlobal, .1)


input("press ENTER to close gripper")
bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(RobotGlobal, .1)
    
    
bot.home(RobotGlobal)
while bot.getTimeToEnd()>0:
    bot.sync(RobotGlobal, .1)

del bot
del RobotGlobal


# translated from c++ function "Quaternion:setDiff" in file "Quaternion.cpp"
def getQuat(from_vector, to_vector):
    offsetNorm = initial_position #/ np.linalg.norm(to_vector)
    offset = np.array([0,offsetNorm[1],0]) * np.linalg.norm(to_vector)
    #offset = np.array([0,0.6,0])
    a = from_vector / np.linalg.norm(from_vector)
    b = (to_vector - offset) / np.linalg.norm(to_vector)
    scalarProduct = np.dot(a, b)
    phi = np.arccos(scalarProduct)
    if phi == 0:
        return np.array([0, 0, 0, 0])
    axis = np.cross(a, b)
    if np.linalg.norm(axis) < 1e-10:
        axis = np.cross(np.array([1, 0, 0]), b)
        if np.linalg.norm(axis) < 1e-10:
            axis = np.cross(np.array([0, 1, 0]), b)
    return np.array([np.cos(phi / 2), axis[0] * np.sin(phi / 2), axis[1] * np.sin(phi / 2), axis[2] * np.sin(phi / 2)])


def getRotatedVectorFrameFromPoint(point):
    externalCamera = C.getFrame('externalCamera')
    diffVector = externalCamera.getPosition() - point # order matters here!
    ext = externalCamera.getPosition()

    quat = getQuat(np.array([0, 1, 0]), diffVector)
    
    # create unique frame for point
    frameName = 'rotatedVectorFrame' + str(time.time())
    rotatedVectorFrame = C.addFrame(frameName)
    rotatedVectorFrame.setShape(ry.ST.marker, size=[.1]) # this can be commented out to hide the marker
    rotatedVectorFrame.setPosition(point)
    rotatedVectorFrame.setQuaternion(quat)
    return frameName

def getDepthInterpolation(paths, path_attributes, svg_attributes):
    depth_interpolation = []
    start_adjacancies = []
    end_adjacancies = []
    for i, path in enumerate(paths):
        # getSVGParams
        _, resize_factor, _ = getSvgPathParams(path, svg_attributes, [])
        for j, comparePath in enumerate(paths):
            if path != comparePath:
                path_start = np.array([real(path.start), 0, imag(path.start)])
                path_end = np.array([real(path.end), 0, imag(path.end)])
                comparePath_start = np.array([real(comparePath.start), 0, imag(comparePath.start)])
                comparePath_end = np.array([real(comparePath.end), 0, imag(comparePath.end)])
                tolerance = [DEPTH_CONNECT_TOLERANCE / resize_factor, 0, DEPTH_CONNECT_TOLERANCE / resize_factor]
                if np.allclose(path_start, comparePath_start, atol=tolerance) or np.allclose(path_start, comparePath_end, atol=tolerance):
                    start_adjacancies.append(j)
                if np.allclose(path_end, comparePath_start, atol=tolerance) or np.allclose(path_end, comparePath_end, atol=tolerance):
                    end_adjacancies.append(j)
        if len(start_adjacancies) == 0 or len(end_adjacancies) == 0:
            depth_interpolation.append([getStrokeWidth(path_attributes[i])])
        else:
            stroke_widths_start = []
            stroke_widths_end = []
            for start_adjacency in start_adjacancies:
                stroke_widths_start.append(getStrokeWidth(path_attributes[start_adjacency]))
            for end_adjacency in end_adjacancies:
                stroke_widths_end.append(getStrokeWidth(path_attributes[end_adjacency]))
            
            for width in stroke_widths_start:
                if width not in stroke_widths_end:
                    # check that two widths are actually at least 5 percent apart
                    if width * 1.05 < stroke_widths_end[0] or width * 0.95 > stroke_widths_end[0]:
                        depth_interpolation.append((width, stroke_widths_end[0]))
                        break
            if len(depth_interpolation) == i:
                for width in stroke_widths_end:
                    if width not in stroke_widths_start:
                        # check that two widths are actually at least 5 percent apart
                        if width * 1.05 < stroke_widths_start[0] or width * 0.95 > stroke_widths_start[0]:
                            depth_interpolation.append((stroke_widths_start[0], width))
                            break
            if len(depth_interpolation) == i:
                depth_interpolation.append([getStrokeWidth(path_attributes[i])])
        start_adjacancies = []
        end_adjacancies = []
    return depth_interpolation

def getSvgPathParams(path, svg_attributes, depth_interpolation = [], stroke_width_normalize_factor = 0):
    length = path.length(error=1e-4)
    height = int(svg_attributes['height'].replace('px',''))
    width = int(svg_attributes['width'].replace('px',''))

    # rescale canvas size depending on highest viewbox parameter
    viewBox = svg_attributes['viewBox'].split(' ')
    viewBox = [float(i) for i in viewBox]
    max_viewbox = max(viewBox)

    height = int(height * (max_viewbox/height))
    width = int(width * (max_viewbox/width))

    #Normalize the length
    if height > width:
        longest_side = height
    else:
        longest_side = width
    length = (length/longest_side)*PICTURE_SIZE
    resize_factor = (1/longest_side)*PICTURE_SIZE
    resultion=RESULTION*PICTURE_SIZE

    # adjust length to DEPTH
    if (len(depth_interpolation) > 1):
        a = (depth_interpolation[1]-depth_interpolation[0])
        if stroke_width_normalize_factor  > 0:
            a = a / stroke_width_normalize_factor
        else:
            a = 0
        a = a * Y_RANGE
        b = length
        length = math.sqrt(a*a + b*b)

    return length, resize_factor, resultion

def getXYZ(path, path_attributes, svg_attributes, depth_interpolation, stroke_width_normalize_factor, stroke_width_offset):
    length, resize_factor, resultion = getSvgPathParams(path, svg_attributes, depth_interpolation, stroke_width_normalize_factor)
    x = []
    y = []
    z = []
    scalar_product = []
    
    t_dec = HIDING_ANLGE/90*SECS_TO_HIDE
    secs_per_step = (1/resultion)/PAINTING_SPEED
    steps_to_turn = math.ceil(t_dec/secs_per_step)
    t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

    a = PAINTING_SPEED/t_dec
    
    s_dec = 0.5*a*t_dec**2

    angle_percent = 1
    
    
    if s_dec*2+MIN_CONST_VELOCITY_STEPS/RESULTION > length: #Path is to short to hide
        angle_percent = s_dec*2/(length+MIN_CONST_VELOCITY_STEPS/RESULTION) #Reduce the turning toward camera if path is to short
        s_dec = length/2 #new shorter acceleration distance
        t_dec = math.sqrt(2*s_dec/a) #new time for 
        steps_to_turn = math.ceil(t_dec/secs_per_step)
        t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps
        a = 2*s_dec/t_dec**2 #Adapt to fixed timesteps

        steps_constant = 0  
    else:
        # Calculate the number of steps for the constant speed part 
        offset_constant = s_dec/length
        #steps_constant = int((length - 2 * s_dec) * resultion)
        steps_constant = math.ceil((length - 2 * s_dec) * resultion)
        #progress_per_step = (1/resultion)/length
        progress_per_step = ((length - 2 * s_dec) / float(steps_constant)) / length

    duration = t_dec * 2 +steps_constant*secs_per_step

    # calculate depth profile
    stroke_width_start = 0
    stroke_width_end = 0
    if (len(depth_interpolation) == 1):
        stroke_width_start = depth_interpolation[0]
        stroke_width_end = depth_interpolation[0]
    else:
        stroke_width_start = depth_interpolation[0]
        stroke_width_end = depth_interpolation[1]

    stroke_width_start = stroke_width_start - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width_start = stroke_width_start / stroke_width_normalize_factor
    else:
        stroke_width_start = 0

    stroke_width_end = stroke_width_end - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width_end = stroke_width_end / stroke_width_normalize_factor
    else:
        stroke_width_end = 0

    for i in range(0,steps_to_turn+1):
        progress = (0.5*a*(i*secs_per_step)**2)/length
         #progress at time t
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (HIDING_ANLGE-i/steps_to_turn*HIDING_ANLGE*angle_percent)*math.pi/180
        scalar_product.append(math.cos(angle))
    
    for i in range(1,steps_constant+1):
        
        progress = offset_constant + progress_per_step*i
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        scalar_product.append(1)
    
    for i in range(1,steps_to_turn+1):

        progress = 1 - (0.5*a*((steps_to_turn-i)*secs_per_step)**2)/length #progress at time t
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (HIDING_ANLGE*(1-angle_percent)+(i/steps_to_turn*HIDING_ANLGE*angle_percent))*math.pi/180
        scalar_product.append( math.cos(angle))
    
    return x,y,z,scalar_product, length, duration

def getStrokeWidth(path_attributes):
    stroke_width = 0
    if "style" in path_attributes:
        style_attr = path_attributes["style"].split(";")
        # search through separated style attr to find stroke_width
        
        for style in style_attr:
            if "stroke-width" in style:
                stroke_width =  style.split(":")[1]
                return float(stroke_width.replace('px',''))
    elif "stroke_width" in path_attributes:
        return float(stroke_width.replace('px',''))
    else: 
        return 0
    
        
def getYPosition(stroke_width, stroke_width_offset, stroke_width_normalize_factor):
    y = stroke_width - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width = stroke_width / stroke_width_normalize_factor
    else:
        stroke_width = 0
    return y
    

def greedyTrajectoryOptimizer(svg_paths, path_attributes, svg_attributes, depth_interpolation, stroke_width_offset, stroke_width_normalize_factor):
    svg_paths_ord = []
    path_attributes_ord = []
    depth_interpolation_ord = []

    last_trajectory_end_point = [10,10,10]
    number_of_trajectories = len(svg_paths)
    # GREEDY ALGORITHM TO FIND CLOSEST NEXT TRAJECTORY
    for i in range(number_of_trajectories):
        first_or_second = 0
        trajectory = 0
        closesDistance = 999
        predicted_last_trajectory_end_point = np.array([])
        for j in range(len(svg_paths)):
            # figure out y coordinates
            yStart = 0
            yEnd = 0
            if (len(depth_interpolation[j]) == 1):
                yStart = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
            else:
                yStart = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation[j][1], stroke_width_offset, stroke_width_normalize_factor)
            # check if first point of trajectory is close to last point of current trajectory
            startPoint = np.array([real(svg_paths[j].start), yStart * Y_RANGE, imag(svg_paths[j].start)])
            endPoint = np.array([real(svg_paths[j].end), yEnd * Y_RANGE , imag(svg_paths[j].end)])

            distanceFirst = np.linalg.norm(startPoint - last_trajectory_end_point)
            if distanceFirst < closesDistance:
                closesDistance = distanceFirst
                trajectory = j
                first_or_second = 0
                predicted_last_trajectory_end_point = endPoint
            # check if second point of trajectory is close to last point of current trajectory
            distanceSecond = np.linalg.norm(endPoint - last_trajectory_end_point)
            if distanceSecond < closesDistance:
                closesDistance = distanceSecond
                trajectory = j
                first_or_second = 1
                predicted_last_trajectory_end_point = startPoint

        last_trajectory_end_point = predicted_last_trajectory_end_point
        if first_or_second == 0:
            svg_paths_ord.append(svg_paths[trajectory])
            path_attributes_ord.append(path_attributes[trajectory])
            depth_interpolation_ord.append(depth_interpolation[trajectory])
        else:
            svg_paths_ord.append(svg_paths[trajectory].reversed())
            path_attributes_ord.append(path_attributes[trajectory])
            if(type(depth_interpolation[trajectory]) == float):
                depth_interpolation_ord.append(depth_interpolation[trajectory])
            else:
                depth_interpolation_ord.append(depth_interpolation[trajectory][::-1])
            

        # remove trajectory from list of possible trajectories
        svg_paths.pop(trajectory)
        path_attributes.pop(trajectory)
        depth_interpolation.pop(trajectory)

        # create an array that contains whether the following trajectory is so close that the light should not be turned away
        next_trajectory_really_close = [False] # when starting always have to turn towards
        for j in range(len(svg_paths_ord)-1):
            # figure out y coordinates
            yStart = 0
            yEnd = 0
            if (len(depth_interpolation_ord[j]) == 1):
                yStart = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
            else:
                yStart = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation_ord[j][1], stroke_width_offset, stroke_width_normalize_factor)
            # check if first point of trajectory is close to last point of current trajectory
            endPoint = np.array([real(svg_paths_ord[j].end), yStart * Y_RANGE, imag(svg_paths_ord[j].end)])
            startPoint = np.array([real(svg_paths_ord[j+1].start), yEnd * Y_RANGE, imag(svg_paths_ord[j+1].start)])

            # convert points to world scaling
            _, resize_factor, _ = getSvgPathParams(svg_paths_ord[j], svg_attributes, [])

            endPoint = endPoint * resize_factor
            startPoint = startPoint * resize_factor

            distance = np.linalg.norm(startPoint - endPoint)
            if distance < CLOSE_DISTANCE:
                next_trajectory_really_close.append(True)
            else:
                next_trajectory_really_close.append(False)
            
        next_trajectory_really_close.append(False) #  when finishing always has to turn away

    return svg_paths_ord, path_attributes_ord, depth_interpolation_ord, next_trajectory_really_close 

def waypoints_from_svg(filepath, center_position):
    svg_paths, path_attributes, svg_attributes = svg2paths2(filepath)
    waypoint_paths = []
    scalar_product_paths = []
    lengths = []
    durations = []
    temp = []
    new_path_attributes = []
    for svg_path, attr in zip(svg_paths,path_attributes):
        svg_path = svg_path.continuous_subpaths()
        for sub_svg_path in svg_path:
            new_path_attributes.append(attr)
            temp.append(sub_svg_path.reversed())
    svg_paths = temp
    path_attributes = new_path_attributes
    # calculate normalizing factor for stroke-width
    sum = 0
    minWidth = 999
    maxWidth = 0
    for attr in path_attributes:
        stroke_width = getStrokeWidth(attr)
        if stroke_width != None:
            if stroke_width < minWidth: minWidth = stroke_width
            if stroke_width > maxWidth: maxWidth = stroke_width
    stroke_width_normalize_factor = maxWidth - minWidth
    stroke_width_offset = minWidth

    # caclulate depth interpolation
    depth_interpolation = getDepthInterpolation(svg_paths, path_attributes, svg_attributes)

    next_trajectory_really_close = []
    # OPTIMIZE TRAJECTORY ORDER if enabled
    if(OPTIMIZE_TRAJECTORY_ORDER):
        svg_paths, path_attributes, depth_interpolation, next_trajectory_really_close = greedyTrajectoryOptimizer(svg_paths, path_attributes, svg_attributes, depth_interpolation, stroke_width_offset, stroke_width_normalize_factor)
    if (not CONTINUOUS_DRAWING or not OPTIMIZE_TRAJECTORY_ORDER): # if not enabled, just set all connectivity to false
        next_trajectory_really_close = []
        for i in range(len(svg_paths)+1):
            next_trajectory_really_close.append(False)
    
    for i in range(0, len(svg_paths)):
        svg_path = svg_paths[i]
        connected_to_previous = next_trajectory_really_close[i]
        connected_to_next = next_trajectory_really_close[i+1]
        waypoints = []
        print("Path: ", svg_path)
        x_vec, y_vec, z_vec, scalar_product_vec, length, duration = getXYZ(svg_path, path_attributes[i], svg_attributes, depth_interpolation[i], stroke_width_normalize_factor, stroke_width_offset)
        if len(x_vec) > 0: # check if path actually has points
            for i in range(len(x_vec)):
                waypoints.append(center_position- [x_vec[i],y_vec[i]*Y_RANGE,z_vec[i]] +[PICTURE_SIZE/2,0,PICTURE_SIZE/2] + [0,Y_OFFSET,Z_OFFSET_CENTER] )
            waypoint_paths.append(waypoints)
            lengths.append(length)
            durations.append(duration)
            scalar_product_paths.append(scalar_product_vec)
    return waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close


#Get motions from svg
def waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths , durations, next_trajectory_really_close, start_pose):
    paths = []
    times = []

    for i in range(len(waypoint_paths)):
        waypoint_path = waypoint_paths[i]
        scalar_product_path = scalar_product_paths[i]
        length = lengths[i]
        duration = durations[i]
        connected_to_previous = next_trajectory_really_close[i]
        connected_to_next = next_trajectory_really_close[i+1]

        #Move to start position
        if paths == []:
            C.setJointState(start_pose)
        else:
            C.setJointState(paths[-1][-1])

        current_position = C.getFrame('l_gripper').getPosition() 
        start_position = waypoint_path[0] # set start position

        dis_to_start = np.linalg.norm(start_position-current_position)

        secs_to_move = dis_to_start*1/MOVING_SPEED
        
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_move, 2)
        komo.addControlObjective([], 0, 1e-0)
        
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
        if connected_to_previous:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(HIDING_ANLGE*math.pi/180)])
            komo.addControlObjective([], 2, 1e+1)
            # secs_to_move += (CLOSE_DISTANCE / moving_speed) * 2.0
        else:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
            komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_move+MOVE_ADD_TIME)
        #print(ret)

        if KOMO_VIEW:
            komo.view(True, "Move to start")
            #komo.view_play(0.6)

        

        if not connected_to_previous:
            #Turn light towards camera
            secs_to_hide_angle = SECS_TO_HIDE*(90-HIDING_ANLGE)/90

            C.setJointState(paths[-1][-1])

            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide_angle, 2)
            komo.addControlObjective([], 0, 1e-0)
            komo.addControlObjective([], 2, 1e+1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
            komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(HIDING_ANLGE*math.pi/180)])
            komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

            ret = ry.NLP_Solver() \
                .setProblem(komo.nlp()) \
                .setOptions( stopTolerance=1e-2, verbose=4 ) \
                .solve()
            paths.append(komo.getPath())
            times.append(secs_to_hide_angle+MOVE_ADD_TIME)

            if KOMO_VIEW:
                komo.view(True, "Turn towards camera")
                komo.view_play(0.6)

        #Move to draw path

        #Move from start to finish with interpolation
        #Calculate time to paint with accerlation
        t_dec = HIDING_ANLGE/90*SECS_TO_HIDE
        secs_per_step = (1/resultion)/PAINTING_SPEED
        steps_to_turn = math.ceil(t_dec/secs_per_step)
        t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

        a = PAINTING_SPEED/t_dec
	    
        s_dec = 0.5*a*t_dec**2
        steps_per_phase = len(waypoint_path)

        # TODO: adjust secs_to_paint to account for connectivity
        #secs_to_paint = length - 2 * s_dec
        #if connected_to_previous: 
        #    secs_to_paint += s_dec
        #if connected_to_next:
        #    secs_to_paint += s_dec
        #secs_to_paint = secs_to_paint / painting_speed + t_dec*2
        #if connected_to_previous: 
        #    secs_to_paint -= t_dec
        #if connected_to_next:
        #    secs_to_paint -= t_dec
        secs_to_paint = duration



        # create array of pointing frames
        camera_pointing_frames = []
        for i in range(0,len(waypoint_path)):
            camera_pointing_frames.append(getRotatedVectorFrameFromPoint(waypoint_path[i]))


        komo = ry.KOMO()
        C.setJointState(paths[-1][-1])
        
        komo.setConfig(C, True)
        komo.setTiming(1, steps_per_phase, secs_to_paint, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);

        gripper_position = C.getFrame('l_gripper').getRotationMatrix()
        
        # adjust starting pos according to connectivity
        starting_point = 1
        if connected_to_previous:
            starting_point = 0
        
        for i in range(starting_point,len(waypoint_path)): # skip first point
            komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint_path[i]);
            if (FOCUS_ON_CAM):
                komo.addObjective([i*1/steps_per_phase], ry.FS.scalarProductYY, ['l_gripper',camera_pointing_frames[i]], ry.OT.eq, [1e1],[scalar_product_path[i]])
            else:
                komo.addObjective([i*1/steps_per_phase], ry.FS.scalarProductYY, ['l_gripper',camera_pointing_frames[i]], ry.OT.eq, [1e1],[scalar_product_path[i]])
        C.view(False, "waypoints")

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        
        times.append(secs_to_paint+MOVE_ADD_TIME)

        print(ret)

        if KOMO_VIEW:
            komo.view(True, "path to draw")
            komo.view_play(0.6)
    
        if not connected_to_next:
            #Rotate torchlight away from camera
            secs_to_hide_angle = SECS_TO_HIDE*(90-HIDING_ANLGE)/90
            end_position = waypoint_path[-1] # set end position

            C.setJointState(paths[-1][-1])
            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide_angle, 2)
            komo.addControlObjective([], 0, 1e0)
            komo.addControlObjective([], 2, 1e+1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
            komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
            komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1e1],target=end_position);

            ret = ry.NLP_Solver() \
                .setProblem(komo.nlp()) \
                .setOptions( stopTolerance=1e-2, verbose=4 ) \
                .solve()
            paths.append( komo.getPath())
            times.append(secs_to_hide_angle+MOVE_ADD_TIME)

    return paths, times




C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)





bot = ry.BotOp(C, REAL_ROBOT)
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
home_position = C.getFrame('l_gripper').getPosition()

# define external camera frame
# y has to be at least 1.0 to not cause undesired swirling around the external camera
cam = C.addFrame( "externalCamera")
cam.setShape(ry.ST.ssBox, size=[.1,.1,.1,.005])
cam.setColor([1,0,0,1])
cam.setPose("t(" + str(home_position[0]+EXT_CAM[0]) + " " + str(home_position[1]+EXT_CAM[1]) + " " + str(home_position[2]+EXT_CAM[2]) + ")")
# cam.setPose("t(0 1.0 1.2)")


#Rotate torchlight away from camera
komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(1., 1, SECS_TO_HIDE, 2)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 2, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=home_position);
komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
point_away_1 = komo.getPath()
print(ret)
#input("Press Enter to rotate torchlight")
bot.move(point_away_1,[2])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


#Calculate path
initial_position = C.getFrame('l_gripper').getPosition()
waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close = waypoints_from_svg(FILENAME,C.getFrame('l_gripper').getPosition())

motions, times = waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close, C.getJointState())


#for motion, move_time in zip(motions, times):
#    move_time = move_time/speed_multiplier
#    bot.move(motion,[move_time])
#    



#bot.move(motions,times)
mall = [] 
tall = []
total_time = 0
for i,motion in enumerate(motions):
    time_i = float(times[i]/len(motion))/SPEED_MULTIPLIER
    for m in motion:
        mall.append(m)
        total_time += time_i
        tall.append(total_time)

print(f"Image will take {tall[-1]} s")         
input("Press Enter to start") 
                
bot.move(mall,tall)
    
while bot.getTimeToEnd()>0:
        bot.sync(C, .1)
        
input("press ENTER to move to Gripping position")
# bot.home(RobotGlobal)
bot.move(path, [5])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


input("press ENTER to open gripper")
bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

input("Press Enter to home")
#Move back to home
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)



input("Press Enter to close script")
del bot
del C
