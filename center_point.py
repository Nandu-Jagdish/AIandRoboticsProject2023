from robotic import ry
import numpy as np
import time

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
# C.view(True)
center = C.addFrame("CameraCenter","table")
center.setPose('t(2 0 1)')
center.setColor([1,0,0])
center.setShape(ry.ST.sphere, [.05])
C.view(True)

input("Press Enter to home")
bot = ry.BotOp(C, False)
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)




input("Press Enter to move to start postion")
