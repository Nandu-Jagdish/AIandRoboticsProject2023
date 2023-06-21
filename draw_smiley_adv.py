from robotic import ry
import numpy as np
import time
from svgpathtools import svg2paths2, real, imag

speed_multiplier = 1
secs_to_hide = 1.0
painting_speed = 0.3
moving_speed = 0.5
move_add_time = 0.5

secs_to_hide = secs_to_hide/ speed_multiplier
painting_speed = painting_speed* speed_multiplier
moving_speed = moving_speed* speed_multiplier


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
home_position = C.getFrame('l_gripper').getPosition()

#Close gripper to grasp torchlight
input("Press Enter to close gripper")
bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

#Rotate torchlight away from camera
komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(1., 1, secs_to_hide, 2)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 2, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=home_position);
komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
point_away_1 = komo.getPath()
print(ret)
input("Press Enter to rotate torchlight")
bot.move(point_away_1,[2])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

input("Press Enter to start")


for svg_path in svg_paths:

    x_vec,z_vec,length = getXZ(svg_path, svg_attributes)
    #Go to start position
    
    current_joint_state = C.getJointState()
    current_position = C.getFrame('l_gripper').getPosition() 
    start_position = home_position +[-0.5,0,-0.5] + [x_vec[0],0,z_vec[0]] # set start position
    print(start_position)

    dis_to_start = np.linalg.norm(start_position-current_position)
    print(f"Distance to start {dis_to_start}")

    secs_to_move = dis_to_start*1/moving_speed
    #Move to start position
    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, secs_to_move, 2)
    komo.addControlObjective([], 0, 1e-0)
    komo.addControlObjective([], 2, 1e-0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
    #komo.addObjective([0.], ry.FS.qItself, [], ry.OT.eq,scale=[1],target=current_joint_state);
    komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    path_to_start = komo.getPath()
    #print(ret)

    

    #Move from home to start
    #input("Press Enter to move to start postion")
    bot.move(path_to_start,[secs_to_move+move_add_time])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)


    #Turn light towards camera
    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, secs_to_hide, 2)
    komo.addControlObjective([], 0, 1e-1)
    komo.addControlObjective([], 2, 1e-0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([1.], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])
    #komo.addObjective([0.], ry.FS.qItself,[], ry.OT.eq,scale=[1],target=path_to_start[-1]);
    komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    point_to_camera = komo.getPath()
    #print(ret)

    #komo.view(True, "path to rotate")
    #komo.view_play(0.3)


    #Move from start to finish with interpolation
    steps_per_phase = len(x_vec)
    secs_to_paint = length*(1/painting_speed)
    komo = ry.KOMO()
    C.setJointState(point_to_camera[-1])
    komo.setConfig(C, True)
    komo.setTiming(1, steps_per_phase, secs_to_paint, 2)
    komo.addControlObjective([], 2, 1e-0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])
    komo.addObjective([0.], ry.FS.qItself,[], ry.OT.eq,scale=[1],target=point_to_camera[-1])


    for x,z,i in zip(x_vec[1:],z_vec[1:],range(1,len(x_vec))): # skip first point
        waypoint = home_position +[-0.5,0,-0.5] + [x,0,z]
        komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    path = komo.getPath()
    #print(ret)


    #Rotate torchlight away from camera
    end_position = home_position +[-0.5,0,-0.5] + [x_vec[-1],0,z_vec[-1]] # set end position

    print(start_position)
    print(end_position)
    C.setJointState(path[-1])
    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, secs_to_hide, 2)
    #komo.addControlObjective([], 0, 1e-1)
    komo.addControlObjective([], 2, 1e+0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([0.], ry.FS.qItself,[], ry.OT.eq,scale=[1e-1],target=path[-1])
    komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
    komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1e1],target=end_position);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=4 ) \
        .solve()
    point_away = komo.getPath()
    #print(ret)

    #input("Press Enter to move to goal postion with interpolation")
    bot.move(point_to_camera,[secs_to_hide+move_add_time])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.move(path,[secs_to_paint+move_add_time])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.move(point_away,[secs_to_hide+move_add_time])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

input("Press Enter to home")
#Move back to home
bot.home(C)
#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


#Close gripper to grasp torchlight
input("Press Enter to open gripper")
bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

input("Press Enter to close script")
del bot
del C
