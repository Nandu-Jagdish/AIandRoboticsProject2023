from robotic import ry
REAL_ROBOT = True

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))


bot = ry.BotOp(C, REAL_ROBOT)
input("Press Enter to open/close gripper")
bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

del bot
del C
