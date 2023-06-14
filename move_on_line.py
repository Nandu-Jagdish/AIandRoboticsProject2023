from robotic import ry
import numpy as np
import time

ry.params_add({'Franka/Kp_freq': [30., 30., 30., 20., 10., 15., 10.],
               'Franka/friction': np.array([0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4]),
               'Franka/Kd_ratio': np.array([0.6, 0.6, 0.3, 0.3, 0.3, 0.3, 0.4])})


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

input("Press Enter to home")
bot = ry.BotOp(C, True)
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

#Starting pose
home_pose = C.getFrame('l_gripper').getPosition()
start_pose = home_pose +[0.3,0,0]
goal_pose = home_pose +[-0.3,0,0]
komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(2., 1, 1, 1)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])
komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=home_pose);
komo.addObjective([2.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_pose);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
    
print("optimization result:", ret)
komo.view(True, "optimization result")
#while(komo.view_play(True, .5)):
#	print("hi")

path_to_start = komo.getPath() 

#Move from home to start
input("Press Enter to move to start postion")
bot.move(path_to_start,[10])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

#Move from start to finish with interpolation
steps = 20
komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(steps, 1, 1., 2)
komo.addControlObjective([], 1, 1e0)
komo.addControlObjective([], 2, 1e0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])

for i in range(steps):
    i=i+1
    waypoint = start_pose + (goal_pose-start_pose)*i/steps
    komo.addObjective([i], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
print("optimization result:", ret)
komo.view(True, "optimization result")
#while(komo.view_play(True, .5)):
#	print("hi")

path = komo.getPath()

input("Press Enter to move to goal postion with interpolation")
bot.setControllerWriteData(1)
bot.move(path,[10])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
bot.setControllerWriteData(0)

input("Press Enter to home")
#Move back to home
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

input("Press Enter to close script")
del bot
del C
