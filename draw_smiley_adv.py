from robotic import ry
import numpy as np
import time
from svgpathtools import svg2paths2, real, imag

#joint_limit_offset = [0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0]
joint_limit_offset = [-0.1]

speed_multiplier = 1.1
secs_to_hide = 1.0
painting_speed = 0.3
moving_speed = 0.5
move_add_time = 0.4

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

def waypoints_from_svg(filepath, center_position):
    svg_paths, attributes, svg_attributes = svg2paths2(filepath)
    waypoint_paths = []
    lengths = []
    for svg_path in svg_paths:
        waypoints = []
        x_vec,z_vec,length = getXZ(svg_path, svg_attributes)
        for i in range(len(x_vec)):
            waypoints.append(center_position+ [-0.5,0,-0.5] + [x_vec[i],0,z_vec[i]])
        waypoint_paths.append(waypoints)
        lengths.append(length)
    return waypoint_paths, lengths


#Get motions from svg
def waypoints2motion(C,waypoint_paths, lengths ,start_pose):
    paths = []
    times = []

    for waypoint_path, length in zip(waypoint_paths,lengths):
        #Move to start position
        if paths == []:
            C.setJointState(start_pose)
        else:
            C.setJointState(paths[-1][-1])

        current_position = C.getFrame('l_gripper').getPosition() 
        start_position = waypoint_path[0] # set start position

        dis_to_start = np.linalg.norm(start_position-current_position)

        secs_to_move = dis_to_start*1/moving_speed
        
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_move, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_move+move_add_time)
        #print(ret)

        C.setJointState(paths[-1][-1])

        #Turn light towards camera
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_hide, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        komo.addObjective([1.], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_hide+move_add_time)

        #Move to draw path

        #Move from start to finish with interpolation
        steps_per_phase = len(waypoint_path)
        secs_to_paint = length*(1/painting_speed)
        komo = ry.KOMO()
        C.setJointState(paths[-1][-1])
        komo.setConfig(C, True)
        komo.setTiming(1, steps_per_phase, secs_to_paint, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        komo.addObjective([], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])

        for waypoint, i in zip(waypoint_path, range(1,len(waypoint_path))): # skip first point
            komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint);
scalarproductx
fs.vectorY,


        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_paint+move_add_time)
    
        #Rotate torchlight away from camera
        end_position = waypoint_path[-1] # set end position

        C.setJointState(paths[-1][-1])
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_hide, 2)
        komo.addControlObjective([], 0, 1e0)
        komo.addControlObjective([], 2, 1e+0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1e1],target=end_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append( komo.getPath())
        times.append(secs_to_hide+move_add_time)

    return paths, times




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
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
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


#Calculate path
waypoint_paths, lengths = waypoints_from_svg('smiley.svg',C.getFrame('l_gripper').getPosition())

motions, times = waypoints2motion(C,waypoint_paths, lengths, C.getJointState())
input("Press Enter to start")

for motion, time in zip(motions, times):
    bot.move(motion,[time])
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
