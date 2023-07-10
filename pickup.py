from robotic import ry
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2


# Initialize the robot
C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
#C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

boxSize = 0.10

obj = C.addFrame('obj')
obj.setPose('t(0. 0.1 0.8)')
obj.setShape(ry.ST.ssBox, size=[boxSize,boxSize,boxSize,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)

# create a 5 by grid of black and white boxes that lie on the surface on the "obj" box and all together are the size of the box surface. The boxes are all quadratic and flat. 
# The color is given by this 2D array
color = [[0,0,0,0,0,0],[0,0,0,1,1,0],[0,0,0,1,1,0],[0,0,0,1,0,0],[0,1,1,0,1,0],[0,0,0,0,0,0]]

for i in range(6):
    for j in range(6):
        box = C.addFrame('box_'+str(i)+'_'+str(j), 'obj')
        #box.setShape(ry.ST.ssBox, size=[.01,.01,.001,.00])
        box.setShape(ry.ST.ssBox, size=[boxSize/6.0, boxSize/6.0, 0.001, 0.00])
        box.setColor([color[i][j],color[i][j],color[i][j]])
        box.setMass(0)
        box.setContact(False)
        #box.setRelativePose('t('+str(-0.02+0.01*i)+' '+str(-0.02+0.01*j)+' 0.025)')
        box.setRelativePose('t('+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*i)+' '+str(-boxSize/2.0+boxSize/12.0+boxSize/6.0*j)+' ' + str(boxSize/2.0) +  ')')


#cameraFrame = C.addFrame("myCamera")
#cameraFrame.setShape(ry.ST.marker, [0.3])
#cameraFrame.setPosition([0,0,2.0])
#cameraFrame.setPosition([0,1.0,2.0])
#cameraFrame.setQuaternion([1,-0.5,0,1])
C.view()
# pause until the user presses 'Enter
x = input("Press Enter to continue...")

bot = ry.BotOp(C, False)
bot.home(C)

rgb, depth = bot.getImageAndDepth("camera")

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()


#camera parameters
fxypxy = bot.getCameraFxypxy("camera")
print(fxypxy)
depth.shape
cameraFrame = C.getFrame("camera")

# detect aruco markers
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
#rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)
#get center of marker
print(ids)
if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0])/4)
        y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1])/4)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
        #calculate width of the marker in pixel space
        w = int(c[2][0] - c[0][0])
        #calculate height in pixel space
        h = int(c[0][1] - c[2][1])
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
        

# display frame
cv2.imshow('frame', frame)
# check if user pressed 'q'

print(x,y)
print(w,h)