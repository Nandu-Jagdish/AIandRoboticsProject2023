from robotic import ry
import numpy as np
import time
import math
from svgpathtools import svg2paths2, real, imag

#joint_limit_offset = [0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0]
joint_limit_offset = [-0.1]

speed_multiplier = 1.0
secs_to_hide = 1.4
painting_speed = 0.3
moving_speed = 0.3
move_add_time = 0.0
hiding_angle = 45
RESULTION = 40
PICTURE_SIZE = 0.8 #MAX 0.6 with full size circle
Z_OFFSET_CENTER = -0.1

KOMO_VIEW = False
REAL_ROBOT = False
FILENAME = 'smiley.svg'

DOUBLE_LINE = False

def getXZ(path, svg_attributes, resultion=RESULTION):
    length = path.length(error=1e-4)
    height = int(svg_attributes['height'].replace('px',''))
    width = int(svg_attributes['width'].replace('px',''))
    #Normalize the length
    if height > width:
        longest_side = height
    else:
        longest_side = width
    length = length/longest_side*PICTURE_SIZE
    x = []
    z = []
    scalar_product = []
    resize_factor = 1/longest_side*PICTURE_SIZE
    resultion= resultion*PICTURE_SIZE

    
    
    
    t_dec = hiding_angle/90*secs_to_hide
    a = painting_speed/t_dec
    secs_per_step = (1/resultion)/painting_speed
    steps_to_turn = math.ceil(t_dec/secs_per_step)
    s_dec = 0.5*a*t_dec**2
    
    if s_dec > length/2: #Path is to short to hide
        print("Warning: Path to short to point away from the camera")
        return()
    
    steps_constant = int((length - 2*s_dec) * resultion)
    progress_per_step = (1/resultion)/length
    

    for i in range(0,steps_to_turn+1):
        progress = (0.5*a*(i*secs_per_step)**2)/length
         #progress at time t
        if DOUBLE_LINE:
            progress /=2
        x.append(real(path.point(progress))*resize_factor)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (hiding_angle-i/steps_to_turn*hiding_angle)*math.pi/180
        scalar_product.append(math.cos(angle))
    
    for i in range(1,steps_constant+1):
        
        progress = s_dec/length + progress_per_step*i
        if DOUBLE_LINE:
            progress /=2
        x.append(real(path.point(progress))*resize_factor)
        z.append(imag(path.point(progress))*resize_factor)
        scalar_product.append(1)
    
    for i in range(1,steps_to_turn+1):

        progress = 1 - (0.5*a*((steps_to_turn-i)*secs_per_step)**2)/length #progress at time t
        if DOUBLE_LINE:
            progress /=2
        x.append(real(path.point(progress))*resize_factor)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (i/steps_to_turn*hiding_angle)*math.pi/180
        scalar_product.append( math.cos(angle))

    return x,z,scalar_product, length
        




def waypoints_from_svg(filepath, center_position):
    svg_paths, attributes, svg_attributes = svg2paths2(filepath)
    waypoint_paths = []
    scalar_product_paths = []
    lengths = []
    for svg_path in svg_paths:
        waypoints = []
        x_vec,z_vec, scalar_product_vec, length = getXZ(svg_path, svg_attributes)
        for i in range(len(x_vec)):
            waypoints.append(center_position- [x_vec[i],0,z_vec[i]] +[PICTURE_SIZE/2,0,PICTURE_SIZE/2] + [0,0,Z_OFFSET_CENTER] )
        waypoint_paths.append(waypoints)
        lengths.append(length)
        scalar_product_paths.append(scalar_product_vec)
    return waypoint_paths, scalar_product_paths, lengths


#Get motions from svg
def waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths ,start_pose):
    paths = []
    times = []

    for waypoint_path, scalar_product_path, length in zip(waypoint_paths, scalar_product_paths, lengths):
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

        if KOMO_VIEW:
            komo.view(True, "Move to start")
            #komo.view_play(0.6)



        C.setJointState(paths[-1][-1])

        #Turn light towards camera
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_hide, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(hiding_angle*math.pi/180)])
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_hide*(90-hiding_angle)/90+move_add_time)

        if KOMO_VIEW:
            komo.view(True, "Turn towards camera")
            komo.view_play(0.6)

        #Move to draw path

        #Move from start to finish with interpolation
        #Calculate time to paint with accerlation
        t_dec = hiding_angle/90*secs_to_hide
        a = painting_speed/t_dec
        s_dec = 0.5*a*t_dec**2
        steps_per_phase = len(waypoint_path)
        secs_to_paint = (length-2*s_dec)/painting_speed+t_dec*2
        komo = ry.KOMO()
        C.setJointState(paths[-1][-1])
        
        komo.setConfig(C, True)
        komo.setTiming(1, steps_per_phase, secs_to_paint, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);

        gripper_position = C.getFrame('l_gripper').getRotationMatrix()
        

        for i in range(1,len(waypoint_path)): # skip first point
            C.addFrame('waypoint_'+str(i)).setPosition(waypoint_path[i]).setShape(ry.ST.marker, size=[.1])
            komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint_path[i]);
            komo.addObjective([i*1/steps_per_phase], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[scalar_product_path[i]])
        C.view(False, "waypoints")

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        
        times.append(secs_to_paint+move_add_time)

        print(ret)

        if KOMO_VIEW:
            komo.view(True, "path to draw")
            komo.view_play(0.6)
    
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
bot = ry.BotOp(C, REAL_ROBOT)
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
waypoint_paths, scalar_product_paths, lengths = waypoints_from_svg(FILENAME,C.getFrame('l_gripper').getPosition())

motions, times = waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths, C.getJointState())
input("Press Enter to start")

#for motion, move_time in zip(motions, times):
#    move_time = move_time/speed_multiplier
#    bot.move(motion,[move_time])
#    



#bot.move(motions,times)
mall = [] 
tall = []
for i,motion in enumerate(motions):
	time_i = float(times[i]/len(motions))
	for m in motion:
		mall.append(m)
		tall.append(time_i)


bot.move(mall,tall)
    


    

    
print("move command sends")
    

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
