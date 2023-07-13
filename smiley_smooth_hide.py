from robotic import ry
import numpy as np
import time
import math
from svgpathtools import svg2paths2, real, imag
import warnings



#joint_limit_offset = [0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0]
joint_limit_offset = [-0.1]

speed_multiplier = 1.0
secs_to_hide = 0.9
painting_speed = 0.4
moving_speed = 0.5
move_add_time = 0.0
hiding_angle = 45
RESULTION = 40
PICTURE_SIZE = 1.1 #MAX 0.6 with full size circle
Z_OFFSET_CENTER = -0.1
CLOSE_DISTANCE = 0.08 # distance in meter where tunring light away is deemed unnecessary

OPTIMIZE_TRAJECTORY_ORDER = False
CONTINUOUS_DRAWING = False
FOCUS_ON_CAM = False
KOMO_VIEW = False
REAL_ROBOT = False
FILENAME = 'cube.svg'

DOUBLE_LINE = False

resultion = RESULTION


# translated from c++ function "Quaternion:setDiff" in file "Quaternion.cpp"
def getQuat(from_vector, to_vector):
    offsetNorm = initial_position #/ np.linalg.norm(to_vector)
    offset = np.array([0,offsetNorm[1],0]) * np.linalg.norm(to_vector)
    #offset = np.array([0,0.6,0])
    a = from_vector / np.linalg.norm(from_vector)
    b = (to_vector - offset) / np.linalg.norm(to_vector)
    scalarProduct = np.dot(a, b)
    phi = np.arccos(scalarProduct)
    if phi == 0:
        return np.array([0, 0, 0, 0])
    axis = np.cross(a, b)
    if np.linalg.norm(axis) < 1e-10:
        axis = np.cross(np.array([1, 0, 0]), b)
        if np.linalg.norm(axis) < 1e-10:
            axis = np.cross(np.array([0, 1, 0]), b)
    return np.array([np.cos(phi / 2), axis[0] * np.sin(phi / 2), axis[1] * np.sin(phi / 2), axis[2] * np.sin(phi / 2)])


def getRotatedVectorFrameFromPoint(point):
    externalCamera = C.getFrame('externalCamera')
    diffVector = externalCamera.getPosition() - point # order matters here!
    ext = externalCamera.getPosition()

    quat = getQuat(np.array([0, 1, 0]), diffVector)
    
    # create unique frame for point
    frameName = 'rotatedVectorFrame' + str(time.time())
    rotatedVectorFrame = C.addFrame(frameName)
    rotatedVectorFrame.setShape(ry.ST.marker, size=[.1]) # this can be commented out to hide the marker
    rotatedVectorFrame.setPosition(point)
    rotatedVectorFrame.setQuaternion(quat)
    return frameName

def getSvgPathParams(path, svg_attributes):
    length = path.length(error=1e-4)
    height = int(svg_attributes['height'].replace('px',''))
    width = int(svg_attributes['width'].replace('px',''))

    # rescale canvas size depending on highest viewbox parameter
    viewBox = svg_attributes['viewBox'].split(' ')
    viewBox = [float(i) for i in viewBox]
    max_viewbox = max(viewBox)

    height = int(height * (max_viewbox/height))
    width = int(width * (max_viewbox/width))

    #Normalize the length
    if height > width:
        longest_side = height
    else:
        longest_side = width
    length = (length/longest_side)*PICTURE_SIZE
    resize_factor = (1/longest_side)*PICTURE_SIZE
    resultion=RESULTION*PICTURE_SIZE

    return length, resize_factor, resultion

def getXZ(path, svg_attributes, connected_to_previous, connected_to_next):
    length, resize_factor, resultion = getSvgPathParams(path, svg_attributes)
    connected_to_previous = False # this is set to False to keept getXZ as is and not change depending on connectivity
    connected_to_next = False # in future ths this could be deleted to make the continous drawing even faster
    x = []
    z = []
    scalar_product = []
    
    t_dec = hiding_angle/90*secs_to_hide
    secs_per_step = (1/resultion)/painting_speed
    steps_to_turn = math.ceil(t_dec/secs_per_step)
    t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

    a = painting_speed/t_dec
    
    s_dec = 0.5*a*t_dec**2
    
    if s_dec > length/2: #Path is to short to hide
        warnings.warn("Warning: Path to short to point away from the camera")
        return([],[],[],0)
    

    # Calculate the number of steps for the constant speed part depending on connectivity
    steps_constant = length - 2 * s_dec
    start_constant = 1
    offset_constant = s_dec/length
    if connected_to_previous: 
        steps_constant += s_dec
        start_constant = 0
        offset_constant = 0
    if connected_to_next:
        steps_constant += s_dec
    steps_constant = int(steps_constant * resultion)

    progress_per_step = (1/resultion)/length
    
    if not connected_to_next:
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
    
    for i in range(start_constant,steps_constant+1):
        
        progress = offset_constant + progress_per_step*i
        if DOUBLE_LINE:
            progress /=2
        x.append(real(path.point(progress))*resize_factor)
        z.append(imag(path.point(progress))*resize_factor)
        scalar_product.append(1)
    
    if not connected_to_previous:
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
        

def greedyTrajectoryOptimizer(svg_paths, svg_attributes):
    svg_paths_ord = []

    last_trajectory_end_point = [10,10,10]
    number_of_trajectories = len(svg_paths)
    # GREEDY ALGORITHM TO FIND CLOSEST NEXT TRAJECTORY
    for i in range(number_of_trajectories):
        first_or_second = 0
        trajectory = 0
        closesDistance = 999
        predicted_last_trajectory_end_point = np.array([])
        for j in range(len(svg_paths)):
            # check if first point of trajectory is close to last point of current trajectory
            startPoint = np.array([real(svg_paths[j].start), 0, imag(svg_paths[j].start)])
            endPoint = np.array([real(svg_paths[j].end), 0, imag(svg_paths[j].end)])

            distanceFirst = np.linalg.norm(startPoint - last_trajectory_end_point)
            if distanceFirst < closesDistance:
                closesDistance = distanceFirst
                trajectory = j
                first_or_second = 0
                predicted_last_trajectory_end_point = endPoint
            # check if second point of trajectory is close to last point of current trajectory
            distanceSecond = np.linalg.norm(endPoint - last_trajectory_end_point)
            if distanceSecond < closesDistance:
                closesDistance = distanceSecond
                trajectory = j
                first_or_second = 1
                predicted_last_trajectory_end_point = startPoint

        last_trajectory_end_point = predicted_last_trajectory_end_point
        if first_or_second == 0:
            svg_paths_ord.append(svg_paths[trajectory])
        else:
            svg_paths_ord.append(svg_paths[trajectory].reversed())
            

        # remove trajectory from list of possible trajectories
        svg_paths.pop(trajectory)

        # create an array that contains whether the following trajectory is so close that the light should not be turned away
        next_trajectory_really_close = [False] # when starting always have to turn towards
        for j in range(len(svg_paths_ord)-1):
            endPoint = np.array([real(svg_paths_ord[j].end), 0, imag(svg_paths_ord[j].end)])
            startPoint = np.array([real(svg_paths_ord[j+1].start), 0, imag(svg_paths_ord[j+1].start)])

            # convert points to world scaling
            _, resize_factor, _ = getSvgPathParams(svg_paths_ord[j], svg_attributes)

            endPoint = endPoint * resize_factor
            startPoint = startPoint * resize_factor

            distance = np.linalg.norm(startPoint - endPoint)
            if distance < CLOSE_DISTANCE:
                next_trajectory_really_close.append(True)
            else:
                next_trajectory_really_close.append(False)
            
        next_trajectory_really_close.append(False) #  when finishing always has to turn away

    return svg_paths_ord, next_trajectory_really_close


def waypoints_from_svg(filepath, center_position):
    svg_paths, attributes, svg_attributes = svg2paths2(filepath)
    waypoint_paths = []
    scalar_product_paths = []
    lengths = []
    temp = []
    for svg_path in svg_paths:
        temp.append(svg_path.reversed())
    svg_paths = temp
    next_trajectory_really_close = []
    # OPTIMIZE TRAJECTORY ORDER if enabled
    if(OPTIMIZE_TRAJECTORY_ORDER):
        svg_paths, next_trajectory_really_close = greedyTrajectoryOptimizer(svg_paths, svg_attributes)
    if (not CONTINUOUS_DRAWING or not OPTIMIZE_TRAJECTORY_ORDER): # if not enabled, just set all connectivity to false
        for i in range(len(svg_paths)+1):
            next_trajectory_really_close.append(False)
    
    for i in range(0, len(svg_paths)):
        svg_path = svg_paths[i]
        connected_to_previous = next_trajectory_really_close[i]
        connected_to_next = next_trajectory_really_close[i+1]
        waypoints = []
        print("Path: ", svg_path)
        x_vec,z_vec, scalar_product_vec, length = getXZ(svg_path, svg_attributes, connected_to_previous, connected_to_next)
        if len(x_vec) > 0: # check if path actually has points
            for i in range(len(x_vec)):
                waypoints.append(center_position- [x_vec[i],0,z_vec[i]] +[PICTURE_SIZE/2,0,PICTURE_SIZE/2] + [0,0,Z_OFFSET_CENTER] )
            waypoint_paths.append(waypoints)
            lengths.append(length)
            scalar_product_paths.append(scalar_product_vec)
    return waypoint_paths, scalar_product_paths, lengths, next_trajectory_really_close


#Get motions from svg
def waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths , next_trajectory_really_close, start_pose):
    paths = []
    times = []

    for i in range(len(waypoint_paths)):
        waypoint_path = waypoint_paths[i]
        scalar_product_path = scalar_product_paths[i]
        length = lengths[i]
        connected_to_previous = next_trajectory_really_close[i]
        connected_to_next = next_trajectory_really_close[i+1]

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
        
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
        if connected_to_next:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(hiding_angle*math.pi/180)])
            komo.addControlObjective([], 2, 1e+1)
            secs_to_move += (CLOSE_DISTANCE / moving_speed) * 2.0
        else:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
            komo.addControlObjective([], 2, 1e-0)
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

        

        if not connected_to_previous:
            #Turn light towards camera
            secs_to_hide*(90-hiding_angle)/90

            C.setJointState(paths[-1][-1])

            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide, 2)
            komo.addControlObjective([], 0, 1e-0)
            komo.addControlObjective([], 2, 1e+1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);
            komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(hiding_angle*math.pi/180)])
            komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

            ret = ry.NLP_Solver() \
                .setProblem(komo.nlp()) \
                .setOptions( stopTolerance=1e-2, verbose=4 ) \
                .solve()
            paths.append(komo.getPath())
            times.append(secs_to_hide+move_add_time)

            if KOMO_VIEW:
                komo.view(True, "Turn towards camera")
                komo.view_play(0.6)

        #Move to draw path

        #Move from start to finish with interpolation
        #Calculate time to paint with accerlation
        t_dec = hiding_angle/90*secs_to_hide
        secs_per_step = (1/resultion)/painting_speed
        steps_to_turn = math.ceil(t_dec/secs_per_step)
        t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

        a = painting_speed/t_dec
	    
        s_dec = 0.5*a*t_dec**2
        steps_per_phase = len(waypoint_path)

        # TODO: adjust secs_to_paint to account for connectivity
        #secs_to_paint = length - 2 * s_dec
        #if connected_to_previous: 
        #    secs_to_paint += s_dec
        #if connected_to_next:
        #    secs_to_paint += s_dec
        #secs_to_paint = secs_to_paint / painting_speed + t_dec*2
        #if connected_to_previous: 
        #    secs_to_paint -= t_dec
        #if connected_to_next:
        #    secs_to_paint -= t_dec
        secs_to_paint = (length-2*s_dec)/painting_speed+t_dec*2



        # create array of pointing frames
        camera_pointing_frames = []
        for i in range(0,len(waypoint_path)):
            camera_pointing_frames.append(getRotatedVectorFrameFromPoint(waypoint_path[i]))


        komo = ry.KOMO()
        C.setJointState(paths[-1][-1])
        
        komo.setConfig(C, True)
        komo.setTiming(1, steps_per_phase, secs_to_paint, 2)
        komo.addControlObjective([], 0, 1e-0)
        komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],joint_limit_offset);

        gripper_position = C.getFrame('l_gripper').getRotationMatrix()
        
        # adjust starting pos according to connectivity
        starting_point = 1
        if connected_to_previous:
            starting_point = 0
        
        for i in range(starting_point,len(waypoint_path)): # skip first point
            komo.addObjective([i*1/steps_per_phase], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=waypoint_path[i]);
            if (FOCUS_ON_CAM):
                komo.addObjective([i*1/steps_per_phase], ry.FS.scalarProductYY, ['l_gripper',camera_pointing_frames[i]], ry.OT.eq, [1e1],[scalar_product_path[i]])
            else:
                komo.addObjective([i*1/steps_per_phase], ry.FS.scalarProductYY, ['l_gripper',camera_pointing_frames[i]], ry.OT.eq, [1e1],[scalar_product_path[i]])
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
    
        if not connected_to_next:
            #Rotate torchlight away from camera
            end_position = waypoint_path[-1] # set end position

            C.setJointState(paths[-1][-1])
            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide, 2)
            komo.addControlObjective([], 0, 1e0)
            komo.addControlObjective([], 2, 1e+1)
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

# define external camera frame
# y has to be at least 1.0 to not cause undesired swirling around the external camera
cam = C.addFrame( "externalCamera")
cam.setShape(ry.ST.ssBox, size=[.1,.1,.1,.005])
cam.setColor([1,0,0,1])
cam.setPose("t(0 1.0 1.2)")

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
initial_position = C.getFrame('l_gripper').getPosition()
waypoint_paths, scalar_product_paths, lengths, next_trajectory_really_close = waypoints_from_svg(FILENAME,C.getFrame('l_gripper').getPosition())

motions, times = waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths, next_trajectory_really_close, C.getJointState())


#for motion, move_time in zip(motions, times):
#    move_time = move_time/speed_multiplier
#    bot.move(motion,[move_time])
#    



#bot.move(motions,times)
mall = [] 
tall = []
total_time = 0
for i,motion in enumerate(motions):
    time_i = float(times[i]/len(motion))/speed_multiplier
    for m in motion:
        mall.append(m)
        total_time += time_i
        tall.append(total_time)

print(f"Image will take {tall[-1]} s")         
input("Press Enter to start") 
                
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
