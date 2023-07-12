#import sys, os
#sys.path.append(os.path.expanduser('~/git/botop/build'))
#import libry as ry
from robotic import ry
import numpy as np
import time
import matplotlib.pyplot as plt
import quaternion
from utils.robotUtils import mapTOWorldSpace

C = ry.Config()
# C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
cameraType = 'cameraWrist'



cameraFrame = C.getFrame(cameraType)
# cameraFrame.setPosition([0,-0.18,1.2])

camerMarker = C.addFrame('cameraMarker', cameraType)
camerMarker.setShape(ry.marker, size=[.5])
camerMarker.setRelativePosition([0,0,0.0])

boxSize = 0.07
heightFactor = 0.7
lengthFactor = 2.5

obj = C.addFrame('obj')
obj.setPose('t(0. 0.5 0.8)')
obj.setShape(ry.ST.ssBox, size=[boxSize+0.03,boxSize*lengthFactor,boxSize*heightFactor,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)

boxMarker = C.addFrame('boxMarker', 'obj')
boxMarker.setShape(ry.marker, size=[.2])
boxMarker.setRelativePosition([0,0,0.0])

glob = C.addFrame('glob')
glob.setShape(ry.marker, size=[.5])
glob.setPose('t(0. 0.0 0.0)')



# create a 5 by grid of black and white boxes that lie on the surface on the "obj" box and all together are the size of the box surface. The boxes are all quadratic and flat. 
# The color is given by this 2D array
color = [[0,0,0,0,0,0],[0,0,0,1,1,0],[0,0,0,1,1,0],[0,0,0,1,0,0],[0,1,1,0,1,0],[0,0,0,0,0,0]]
#create rotated color array
color = np.rot90(color, k=1, axes=(1, 0))

for i in range(6):
    for j in range(6):
        box = C.addFrame('box_'+str(i)+'_'+str(j), 'obj')
        #box.setShape(ry.ST.ssBox, size=[.01,.01,.001,.00])
        box.setShape(ry.ST.ssBox, size=[boxSize/6.0, boxSize/6.0, 0.001, 0.00])
        box.setColor([color[i][j],color[i][j],color[i][j]])
        box.setMass(0)
        box.setContact(False)
        #box.setRelativePose('t('+str(-0.02+0.01*i)+' '+str(-0.02+0.01*j)+' 0.025)')
        box.setRelativePose('t('+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*i)+' '+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*j)+' ' + str(heightFactor*boxSize/2.0) +  ')')


#cameraFrame = C.addFrame("myCamera")
#cameraFrame.setShape(ry.ST.marker, [0.3])
#cameraFrame.setPosition([0,0,2.0])
#cameraFrame.setPosition([0,1.0,2.0])
#cameraFrame.setQuaternion([1,-0.5,0,1])

C.view()

input("Press Enter to continue...")

bot = ry.BotOp(C, False)
bot.home(C)

rgb, depth = bot.getImageAndDepth(cameraType)

input("Press Enter to continue...")

rgb, depth = bot.getImageAndDepth(cameraType)

input("Press Enter to continue...")

rgb, depth = bot.getImageAndDepth(cameraType)

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()
import time
time.sleep(0.5)

input("Press Enter to continue...")

fxypxy = bot.getCameraFxypxy(cameraType)
print(fxypxy)
depth.shape
cameraFrame = C.getFrame(cameraType)

#include opencv
import cv2

# open webcam
cap = rgb
x = 0
y = 0

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


# loop through frames
#while True:
# read frame from webcam
frame = rgb
# convert frame to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
# detect aruco markers
corners, ids, rejected = detector.detectMarkers(gray)
#print(ids)
# draw markers on frame
frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
# get camera pose

# define dummy values for camera matrix and distortion coefficients
cameraMatrix = np.array([[fxypxy[0], 0, fxypxy[2]], [0, fxypxy[1], fxypxy[3]], [0, 0, 1]])

# define a wrogn camera matrix with nonsense values
distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, boxSize, cameraMatrix,
                                                                           distCoeffs)
(rvec - tvec).any()  # get rid of that nasty numpy value array error
cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.01)  # Draw Axis

cameraDistanceFactor = -1.0
print('t(' + str(tvec[0][0][0]) + ' ' + str(tvec[0][0][1]) + ' ' + str(tvec[0][0][2]) + ') r(' + str(rvec[0][0][0]) + ' ' + str(rvec[0][0][1]) + ' ' + str(rvec[0][0][2]) + ')')

arucoPoint = [cameraDistanceFactor*tvec[0][0][0], cameraDistanceFactor*tvec[0][0][1], cameraDistanceFactor*tvec[0][0][2]]

q = quaternion.from_rotation_vector(rvec)

q = quaternion.as_euler_angles(q)
#q[0][0][2] = -q[0][0][2]


q = quaternion.from_euler_angles(q)

b = quaternion.as_float_array([q])

arucoQuaternion = b[0]

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


C.view()
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

       
def colourDetection(frame,colour):
    #convrt to opencv bgr
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #define range of colour in hsv
    if colour == "red":
        lower = np.array([0,50,50])
        upper = np.array([10,255,255])
    elif colour == "green":
        lower = np.array([50,100,100])
        upper = np.array([70,255,255])
    elif colour == "blue":
        lower = np.array([110,50,50])
        upper = np.array([130,255,255])
    #threshold hsv image to get only colour
    mask = cv2.inRange(hsv, lower, upper)
    #bitwise and mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #display frame
    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    #check if user pressed 'q'
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return res

def colourSeperation(frame,mask):
    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #bitwise and mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #display frame
    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    #check if user pressed 'q'
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return res

def contourDetection(frame):
    # copy frame
    original = frame.copy()
    # convert to grayscale
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw all contours
    cv2.drawContours(original, contours, -1, (0, 255, 0), 3)
    # find center of contour
    M = cv2.moments(contours[0])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # draw center of contour
    cv2.circle(original, (cx, cy), 4, (0, 0, 255), -1)
    return cx,cy,original


# redColour = colourDetection(frame,'red')
# cx,cy,redColour = contourDetection(redColour)
# plt.imshow(redColour)
# plt.show()




RedPoint1 = mapTOWorldSpace((y1,x1),depth[y1,x1],fxypxy)
BluePoint2 = mapTOWorldSpace((y2,x2),depth[y2,x2],fxypxy)
YellowPoint3 = mapTOWorldSpace((y3,x3),depth[y3,x3],fxypxy)
GreenPoint4 = mapTOWorldSpace((y4,x4),depth[y4,x4],fxypxy)
# centerPoint = mapTOWorldSpace((cy,cx),depth[cy,cx],fxypxy
centerPoint = mapTOWorldSpace(centerOfAruco,depth[centerOfAruco[0],centerOfAruco[1]],fxypxy)

RedFrame = C.addFrame("RedFramePoint",cameraType)
RedFrame.setRelativePosition(RedPoint1)
RedFrame.setShape(ry.ST.sphere, [0.029])
RedFrame.setColor([1,0,0])

BlueFrame = C.addFrame("BlueFramePoint",cameraType)
BlueFrame.setRelativePosition(BluePoint2)
BlueFrame.setShape(ry.ST.sphere, [0.029])
BlueFrame.setColor([0,1,0])

YelloFrame = C.addFrame("YelloFramePoint",cameraType)
YelloFrame.setRelativePosition(YellowPoint3)
YelloFrame.setShape(ry.ST.sphere, [0.029])
YelloFrame.setColor([0,0,1])

GreenFrame = C.addFrame("GreenFramePoint",cameraType)
GreenFrame.setRelativePosition(GreenPoint4)
GreenFrame.setShape(ry.ST.sphere, [0.029])
GreenFrame.setColor([1,1,0])

CenterFrame = C.addFrame("CenterFramePoint",cameraType)
CenterFrame.setRelativePosition(centerPoint)
CenterFrame.setShape(ry.ST.sphere, [0.029])
CenterFrame.setColor([1,0,1])




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
# find cross product
heightAxis = np.cross(lengthAxis,widthAxis)

# normalise
heightAxis = heightAxis/np.linalg.norm(heightAxis)
lengthAxis = lengthAxis/np.linalg.norm(lengthAxis)
widthAxis = widthAxis/np.linalg.norm(widthAxis)
# convert to rotation matrix
# add vectors columnwise to matrix
rotationMatrix = np.column_stack((widthAxis,lengthAxis,heightAxis))

boardQuarternion = quaternion.from_rotation_matrix(rotationMatrix)
# find cross product
normal = np.cross(v1,v2)
# normalise
normal = normal/np.linalg.norm(normal)
# find angle between normal and x axis
anglex = np.arccos(np.dot(normal,[1,0,0])/(np.linalg.norm(normal)*np.linalg.norm([1,0,0])))
# convert to degrees
anglex = np.degrees(anglex)
# Find angle between normal and z axis
anglez = np.arccos(np.dot(normal,[0,0,1])/(np.linalg.norm(normal)*np.linalg.norm([0,0,1])))
# convert to degrees
anglez = np.degrees(anglez)
# Find angle between normal and y axis
angley = np.arccos(np.dot(normal,[0,1,0])/(np.linalg.norm(normal)*np.linalg.norm([0,1,0])))
# convert to degrees
angley = np.degrees(angley)


#find angle between vector and x axis
angle = np.arccos(np.dot(v1,[1,0,0])/(np.linalg.norm(v1)*np.linalg.norm([1,0,0])))
# convert to degrees
angle = np.degrees(angle)


# v2 = worldPoint1 - worldPoint3
# find cross product
# normal = np.cross(v1,v2)
# normalise
# normal = normal/np.linalg.norm(normal)

# centerPoint = [0.,0.,0.]
torch = C.addFrame("torch",cameraType)
torch.setRelativePosition(centerPoint)
# string = f't({centerPoint[0]} {centerPoint[1]} {centerPoint[2]})d({anglex} 1 0 0)  d({angley} 0 1 0) d({anglez} 0 0 1)'
# torch.setRelativePose(string)
# string = f't({centerPoint[0]} {centerPoint[1]} {centerPoint[2]})d({45} 0 1 0)'
# torch.setRelativePose(string)
# string = f't({centerPoint[0]} {centerPoint[1]} {centerPoint[2]})d({45} 0 0 0)'
# torch.setRelativePose(string)
# set quaternion
torch.setRelativeQuaternion(quaternion.as_float_array(boardQuarternion))
torch.setShape(ry.marker, [.6])
torch.setColor([1,0,1])


# # find distance from origin
# d = np.dot(normal,worldPoint1)
# # find plane
# plane = np.append(normal,d)


# display frame
#cv2.imshow('frame', frame)
# check if user pressed 'q'
#cv2.waitKey(0)
print(x,y)
print(w,h)

#cv2.destroyAllWindows()
#del bot
#del C

input("Press Enter to continue...")

fx, fy = fxypxy[0], fxypxy[1]
px, py = fxypxy[2], fxypxy[3]
R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()
H, W = depth.shape

    
# get depth at pixel
Z = depth[y, x]

point = [1, 2, 3]
Z += (h/ fx)*0.5

## depth is sign-fliped, j: right, i: down
point[0] = Z * (x - px) / fx;
point[1] = -Z * (y - py) / fy;
point[2] = -Z

h = Z * (h);
w = Z * w

tmp = C.addFrame( "center of red", cameraType)
tmp.setShape(ry.ST.ssBox, size=[(w/ fx),(lengthFactor*h/ fy),(heightFactor*w/ fx),.005])
# tmp.setColor([1,0,0,.5])
# set blue colour
tmp.setColor([0,0,1,.5])

tmp.setRelativePosition(arucoPoint)


tmp.setRelativeQuaternion(arucoQuaternion)
C.view()

input("Press Enter to continue...")

# manually define frames as an endeff waypoints, relative to box:
way0 = C.addFrame('way0', 'center of red')
way1 = C.addFrame('way1', 'center of red')
way2 = C.addFrame('way2', 'center of red')
way2.setShape(ry.ST.marker, size=[.5])
way2.setRelativePose('t(0 0 .3) d(180 1 0 0)')

way0.setShape(ry.ST.marker, size=[.1])
way0.setRelativePose('t(0 0 -.2)  d(180 1 0 0)')
#way0.setRelativePose('t(0 0 .1)')

way1.setShape(ry.ST.marker, size=[.1])
way1.setRelativePose(' d(180 1 0 0)')

C.view()
print("dome")

komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(2., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
#komo.addObjective([1.], ry.FS.scalarProductXZ, ['l_gripper','way0'], ry.OT.eq, [1e1]);
komo.addObjective([1.], ry.FS.poseDiff, ['l_gripper', 'way0'], ry.OT.eq, [1e1]);
komo.addObjective([2.], ry.FS.poseDiff, ['l_gripper', 'way1'], ry.OT.eq, [1e1]);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
print(ret)

komo.view(True, "waypoints solution")

# komo.view_close()
path = komo.getPath()

bot = ry.BotOp(C, False)

input("press ENTER to move to home position")

bot.home(C)
    

bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)
    
input("press ENTER to move to gripping pose")
bot.move(path, [10])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
    
input("press ENTER to close gripper")
bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)
    
input("press ENTER to move to home position")
bot.home(C)

input("press ENTER to open gripper")
bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)