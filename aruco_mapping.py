from robotic import ry
import numpy as np
from utils.cv_func import colourDetection, contourDetection
from utils.robotUtils import mapTOWorldSpace
import matplotlib.pyplot as plt

REAL_ROBOT = False

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
# C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
# cameraType = 'cameraWrist'
cameraType = 'camera'

cameraFrame = C.getFrame(cameraType)

boxSize = 0.07
heightFactor = 0.7
lengthFactor = 2.5

obj = C.addFrame('obj')
obj.setPose('t(0. 0.5 0.8)')
obj.setShape(ry.ST.ssBox, size=[boxSize,boxSize*lengthFactor,boxSize*heightFactor,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)

C.view()
# input("Press Enter to continue...")

bot = ry.BotOp(C, REAL_ROBOT)
bot.home(C)

rgb, depth = bot.getImageAndDepth(cameraType)

fxypxy = bot.getCameraFxypxy(cameraType)
print(fxypxy)
depth.shape
cameraFrame = C.getFrame(cameraType)

redColour = colourDetection(rgb,"red")
plt.imshow(redColour)
plt.show()


cx,cy,frame = contourDetection(redColour,rgb)
plt.imshow(frame)
plt.show()

worldPoint = mapTOWorldSpace([cy,cx],depth[cy,cx],fxypxy)

virtualObject = C.addFrame('virtualObject',cameraType)
virtualObject.setRelativePosition(worldPoint)
virtualObject.setShape(ry.ST.sphere, [.2])
virtualObject.setColor([0,1,0])

C.view()
input("Press Enter to continue...")