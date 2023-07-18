from robotic import ry
import numpy as np
import time
from svgpathtools import svg2paths2, real, imag

def getXZ(path, svg_attributes, resultion=50):
    length = path.length(error=1e-4)
    height = int(svg_attributes['height'].replace('px',''))
    width = int(svg_attributes['width'].replace('px',''))
    x = []
    z = []

    steps = int(length/width*resultion)
    for i in range(steps+1):
        x.append(1-real(path.point(i/steps))/height)
        z.append(1-imag(path.point(i/steps))/width)
    #weighted length
    length = length/width
    
    return x,z,length



svg_paths, attributes, svg_attributes = svg2paths2('smiley.svg')

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

input("Press Enter to home")
bot = ry.BotOp(C, False)
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
home_pose = C.getFrame('l_gripper').getPosition()


for svg_path in svg_paths:

    x,z,length = getXZ(svg_path, svg_attributes)
    #Go to start position
    current_pose = C.getFrame('l_gripper').getPosition() # get curent position
    start_pose = home_pose +[-0.5,0,-0.5] + [x[0],0,z[0]] # set start position
    print(start_pose)

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1, 1)
    komo.addControlObjective([], 0, 1e-0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])
    komo.addObjective([0.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=home_pose);
    komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_pose);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    path_to_start = komo.getPath()
    print(ret)

    #Move from home to start
    input("Press Enter to move to start postion")
    bot.move(path_to_start,[5])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)



    #Move from start to finish with interpolation
    steps_per_phase = len(x)
    secs_per_phase = length*5
    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1, steps_per_phase, secs_per_phase, 2)
    komo.addControlObjective([], 2, 1e-0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])


    for x,z,i in zip(x,z,range(len(x))):
        waypoint = home_pose +[-0.5,0,-0.5] + [x,0,z]
        komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    path = komo.getPath() 
    print(f"Time for step: {secs_per_phase}")
    input("Press Enter to move to goal postion with interpolation")
    bot.move(path,[secs_per_phase*2])
    while bot.getTimeToEnd()>0:
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
