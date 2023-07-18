from robotic import ry
import numpy as np
import time
import math
from svgpathtools import svg2paths2, real, imag
import warnings



#joint_limit_offset = [0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0]
JOINT_LIMIT_OFFSET = [-0.1]

SPEED_MULTIPLIER = 0.3
SECS_TO_HIDE = 1.0
PAINTING_SPEED = 0.4
MOVING_SPEED = 0.5
MOVE_ADD_TIME = 0.0
HIDING_ANLGE = 45
RESULTION = 40
PICTURE_SIZE = 0.5 #MAX 0.6 with full size circle
Z_OFFSET_CENTER = -0.1
Y_OFFSET = -0.15 # offset in y direction for the whole drawing
CLOSE_DISTANCE = 0.08 # distance in meter where tunring light away is deemed unnecessary
Y_RANGE = -0.3 # range in y direction for stroke width levels, set to 0 to disable depth
DEPTH_CONNECT_TOLERANCE = 0.05 # tolerance for connecting paths in depth, set to 0 to disable connecting

MIN_CONST_VELOCITY_STEPS = 4

OPTIMIZE_TRAJECTORY_ORDER = True
CONTINUOUS_DRAWING = True
FOCUS_ON_CAM = False
KOMO_VIEW = False
REAL_ROBOT = False
FILENAME = 'smiley.svg'


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

def getDepthInterpolation(paths, path_attributes, svg_attributes):
    depth_interpolation = []
    start_adjacancies = []
    end_adjacancies = []
    for i, path in enumerate(paths):
        # getSVGParams
        _, resize_factor, _ = getSvgPathParams(path, svg_attributes, [])
        for j, comparePath in enumerate(paths):
            if path != comparePath:
                path_start = np.array([real(path.start), 0, imag(path.start)])
                path_end = np.array([real(path.end), 0, imag(path.end)])
                comparePath_start = np.array([real(comparePath.start), 0, imag(comparePath.start)])
                comparePath_end = np.array([real(comparePath.end), 0, imag(comparePath.end)])
                tolerance = [DEPTH_CONNECT_TOLERANCE / resize_factor, 0, DEPTH_CONNECT_TOLERANCE / resize_factor]
                if np.allclose(path_start, comparePath_start, atol=tolerance) or np.allclose(path_start, comparePath_end, atol=tolerance):
                    start_adjacancies.append(j)
                if np.allclose(path_end, comparePath_start, atol=tolerance) or np.allclose(path_end, comparePath_end, atol=tolerance):
                    end_adjacancies.append(j)
        if len(start_adjacancies) == 0 or len(end_adjacancies) == 0:
            depth_interpolation.append([getStrokeWidth(path_attributes[i])])
        else:
            stroke_widths_start = []
            stroke_widths_end = []
            for start_adjacency in start_adjacancies:
                stroke_widths_start.append(getStrokeWidth(path_attributes[start_adjacency]))
            for end_adjacency in end_adjacancies:
                stroke_widths_end.append(getStrokeWidth(path_attributes[end_adjacency]))
            
            for width in stroke_widths_start:
                if width not in stroke_widths_end:
                    # check that two widths are actually at least 5 percent apart
                    if width * 1.05 < stroke_widths_end[0] or width * 0.95 > stroke_widths_end[0]:
                        depth_interpolation.append((width, stroke_widths_end[0]))
                        break
            if len(depth_interpolation) == i:
                for width in stroke_widths_end:
                    if width not in stroke_widths_start:
                        # check that two widths are actually at least 5 percent apart
                        if width * 1.05 < stroke_widths_start[0] or width * 0.95 > stroke_widths_start[0]:
                            depth_interpolation.append((stroke_widths_start[0], width))
                            break
            if len(depth_interpolation) == i:
                depth_interpolation.append([getStrokeWidth(path_attributes[i])])
        start_adjacancies = []
        end_adjacancies = []
    return depth_interpolation

def getSvgPathParams(path, svg_attributes, depth_interpolation = [], stroke_width_normalize_factor = 0):
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

    # adjust length to DEPTH
    if (len(depth_interpolation) > 1):
        a = (depth_interpolation[1]-depth_interpolation[0])
        if stroke_width_normalize_factor  > 0:
            a = a / stroke_width_normalize_factor
        else:
            a = 0
        a = a * Y_RANGE
        b = length
        length = math.sqrt(a*a + b*b)

    return length, resize_factor, resultion

def getXYZ(path, path_attributes, svg_attributes, depth_interpolation, stroke_width_normalize_factor, stroke_width_offset):
    length, resize_factor, resultion = getSvgPathParams(path, svg_attributes, depth_interpolation, stroke_width_normalize_factor)
    x = []
    y = []
    z = []
    scalar_product = []
    
    t_dec = HIDING_ANLGE/90*SECS_TO_HIDE
    secs_per_step = (1/resultion)/PAINTING_SPEED
    steps_to_turn = math.ceil(t_dec/secs_per_step)
    t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

    a = PAINTING_SPEED/t_dec
    
    s_dec = 0.5*a*t_dec**2

    angle_percent = 1
    
    
    if s_dec*2+MIN_CONST_VELOCITY_STEPS/RESULTION > length: #Path is to short to hide
        warnings.warn("Warning: Path to short to point away from the camera")
        angle_percent = s_dec*2/(length+MIN_CONST_VELOCITY_STEPS/RESULTION) #Reduce the turning toward camera if path is to short
        s_dec = length/2 #new shorter acceleration distance
        t_dec = math.sqrt(2*s_dec/a) #new time for 
        steps_to_turn = math.ceil(t_dec/secs_per_step)
        t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

        steps_constant = 0  
    else:
        # Calculate the number of steps for the constant speed part 
        offset_constant = s_dec/length
        #steps_constant = int((length - 2 * s_dec) * resultion)
        steps_constant = math.ceil((length - 2 * s_dec) * resultion)
        #progress_per_step = (1/resultion)/length
        progress_per_step = ((length - 2 * s_dec) / float(steps_constant)) / length

    duration = t_dec * 2 +steps_constant*secs_per_step

    # calculate depth profile
    stroke_width_start = 0
    stroke_width_end = 0
    if (len(depth_interpolation) == 1):
        stroke_width_start = depth_interpolation[0]
        stroke_width_end = depth_interpolation[0]
    else:
        stroke_width_start = depth_interpolation[0]
        stroke_width_end = depth_interpolation[1]

    stroke_width_start = stroke_width_start - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width_start = stroke_width_start / stroke_width_normalize_factor
    else:
        stroke_width_start = 0

    stroke_width_end = stroke_width_end - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width_end = stroke_width_end / stroke_width_normalize_factor
    else:
        stroke_width_end = 0

    for i in range(0,steps_to_turn+1):
        progress = (0.5*a*(i*secs_per_step)**2)/length
         #progress at time t
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (HIDING_ANLGE-i/steps_to_turn*HIDING_ANLGE*angle_percent)*math.pi/180
        scalar_product.append(math.cos(angle))
    
    for i in range(1,steps_constant+1):
        
        progress = offset_constant + progress_per_step*i
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        scalar_product.append(1)
    
    for i in range(1,steps_to_turn+1):

        progress = 1 - (0.5*a*((steps_to_turn-i)*secs_per_step)**2)/length #progress at time t
        x.append(real(path.point(progress))*resize_factor)
        y.append(stroke_width_start + (stroke_width_end - stroke_width_start) * progress)
        z.append(imag(path.point(progress))*resize_factor)
        #angle in radians
        angle = (HIDING_ANLGE*(1-angle_percent)+(i/steps_to_turn*HIDING_ANLGE*angle_percent))*math.pi/180
        scalar_product.append( math.cos(angle))
    
    return x,y,z,scalar_product, length, duration

def getStrokeWidth(path_attributes):
    style_attr = path_attributes["style"].split(";")
    # search through separated style attr to find stroke_width
    stroke_width = 0
    for style in style_attr:
        if "stroke-width" in style:
            stroke_width =  style.split(":")[1]
            return float(stroke_width.replace('px',''))
        
def getYPosition(stroke_width, stroke_width_offset, stroke_width_normalize_factor):
    y = stroke_width - stroke_width_offset
    if stroke_width_normalize_factor  > 0:
        stroke_width = stroke_width / stroke_width_normalize_factor
    else:
        stroke_width = 0
    return y
    

def greedyTrajectoryOptimizer(svg_paths, path_attributes, svg_attributes, depth_interpolation, stroke_width_offset, stroke_width_normalize_factor):
    svg_paths_ord = []
    path_attributes_ord = []
    depth_interpolation_ord = []

    last_trajectory_end_point = [10,10,10]
    number_of_trajectories = len(svg_paths)
    # GREEDY ALGORITHM TO FIND CLOSEST NEXT TRAJECTORY
    for i in range(number_of_trajectories):
        first_or_second = 0
        trajectory = 0
        closesDistance = 999
        predicted_last_trajectory_end_point = np.array([])
        for j in range(len(svg_paths)):
            # figure out y coordinates
            yStart = 0
            yEnd = 0
            if (len(depth_interpolation[j]) == 1):
                yStart = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
            else:
                yStart = getYPosition(depth_interpolation[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation[j][1], stroke_width_offset, stroke_width_normalize_factor)
            # check if first point of trajectory is close to last point of current trajectory
            startPoint = np.array([real(svg_paths[j].start), yStart * Y_RANGE, imag(svg_paths[j].start)])
            endPoint = np.array([real(svg_paths[j].end), yEnd * Y_RANGE , imag(svg_paths[j].end)])

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
            path_attributes_ord.append(path_attributes[trajectory])
            depth_interpolation_ord.append(depth_interpolation[trajectory])
        else:
            svg_paths_ord.append(svg_paths[trajectory].reversed())
            path_attributes_ord.append(path_attributes[trajectory])
            if(type(depth_interpolation[trajectory]) == float):
                depth_interpolation_ord.append(depth_interpolation[trajectory])
            else:
                depth_interpolation_ord.append(depth_interpolation[trajectory][::-1])
            

        # remove trajectory from list of possible trajectories
        svg_paths.pop(trajectory)
        path_attributes.pop(trajectory)
        depth_interpolation.pop(trajectory)

        # create an array that contains whether the following trajectory is so close that the light should not be turned away
        next_trajectory_really_close = [False] # when starting always have to turn towards
        for j in range(len(svg_paths_ord)-1):
            # figure out y coordinates
            yStart = 0
            yEnd = 0
            if (len(depth_interpolation_ord[j]) == 1):
                yStart = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
            else:
                yStart = getYPosition(depth_interpolation_ord[j][0], stroke_width_offset, stroke_width_normalize_factor)
                yEnd = getYPosition(depth_interpolation_ord[j][1], stroke_width_offset, stroke_width_normalize_factor)
            # check if first point of trajectory is close to last point of current trajectory
            endPoint = np.array([real(svg_paths_ord[j].end), yStart * Y_RANGE, imag(svg_paths_ord[j].end)])
            startPoint = np.array([real(svg_paths_ord[j+1].start), yEnd * Y_RANGE, imag(svg_paths_ord[j+1].start)])

            # convert points to world scaling
            _, resize_factor, _ = getSvgPathParams(svg_paths_ord[j], svg_attributes, [])

            endPoint = endPoint * resize_factor
            startPoint = startPoint * resize_factor

            distance = np.linalg.norm(startPoint - endPoint)
            if distance < CLOSE_DISTANCE:
                next_trajectory_really_close.append(True)
            else:
                next_trajectory_really_close.append(False)
            
        next_trajectory_really_close.append(False) #  when finishing always has to turn away

    return svg_paths_ord, path_attributes_ord, depth_interpolation_ord, next_trajectory_really_close 

def waypoints_from_svg(filepath, center_position):
    svg_paths, path_attributes, svg_attributes = svg2paths2(filepath)
    waypoint_paths = []
    scalar_product_paths = []
    lengths = []
    durations = []
    temp = []
    for svg_path in svg_paths:
        temp.append(svg_path.reversed())
    svg_paths = temp
    
    # calculate normalizing factor for stroke-width
    sum = 0
    minWidth = 999
    maxWidth = 0
    for attr in path_attributes:
        stroke_width = getStrokeWidth(attr)
        if stroke_width != None:
            if stroke_width < minWidth: minWidth = stroke_width
            if stroke_width > maxWidth: maxWidth = stroke_width
    stroke_width_normalize_factor = maxWidth - minWidth
    stroke_width_offset = minWidth

    # caclulate depth interpolation
    depth_interpolation = getDepthInterpolation(svg_paths, path_attributes, svg_attributes)

    next_trajectory_really_close = []
    # OPTIMIZE TRAJECTORY ORDER if enabled
    if(OPTIMIZE_TRAJECTORY_ORDER):
        svg_paths, path_attributes, depth_interpolation, next_trajectory_really_close = greedyTrajectoryOptimizer(svg_paths, path_attributes, svg_attributes, depth_interpolation, stroke_width_offset, stroke_width_normalize_factor)
    if (not CONTINUOUS_DRAWING or not OPTIMIZE_TRAJECTORY_ORDER): # if not enabled, just set all connectivity to false
        next_trajectory_really_close = []
        for i in range(len(svg_paths)+1):
            next_trajectory_really_close.append(False)
    
    for i in range(0, len(svg_paths)):
        svg_path = svg_paths[i]
        connected_to_previous = next_trajectory_really_close[i]
        connected_to_next = next_trajectory_really_close[i+1]
        waypoints = []
        print("Path: ", svg_path)
        x_vec, y_vec, z_vec, scalar_product_vec, length, duration = getXYZ(svg_path, path_attributes[i], svg_attributes, depth_interpolation[i], stroke_width_normalize_factor, stroke_width_offset)
        if len(x_vec) > 0: # check if path actually has points
            for i in range(len(x_vec)):
                waypoints.append(center_position- [x_vec[i],y_vec[i]*Y_RANGE,z_vec[i]] +[PICTURE_SIZE/2,0,PICTURE_SIZE/2] + [0,Y_OFFSET,Z_OFFSET_CENTER] )
            waypoint_paths.append(waypoints)
            lengths.append(length)
            durations.append(duration)
            scalar_product_paths.append(scalar_product_vec)
    return waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close


#Get motions from svg
def waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths , durations, next_trajectory_really_close, start_pose):
    paths = []
    times = []

    for i in range(len(waypoint_paths)):
        waypoint_path = waypoint_paths[i]
        scalar_product_path = scalar_product_paths[i]
        length = lengths[i]
        duration = durations[i]
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

        secs_to_move = dis_to_start*1/MOVING_SPEED
        
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, secs_to_move, 2)
        komo.addControlObjective([], 0, 1e-0)
        
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
        if connected_to_previous:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(HIDING_ANLGE*math.pi/180)])
            komo.addControlObjective([], 2, 1e+1)
            # secs_to_move += (CLOSE_DISTANCE / moving_speed) * 2.0
        else:
            komo.addObjective([], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
            komo.addControlObjective([], 2, 1e-0)
        komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions( stopTolerance=1e-2, verbose=4 ) \
            .solve()
        paths.append(komo.getPath())
        times.append(secs_to_move+MOVE_ADD_TIME)
        #print(ret)

        if KOMO_VIEW:
            komo.view(True, "Move to start")
            #komo.view_play(0.6)

        

        if not connected_to_previous:
            #Turn light towards camera
            secs_to_hide_angle = SECS_TO_HIDE*(90-HIDING_ANLGE)/90

            C.setJointState(paths[-1][-1])

            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide_angle, 2)
            komo.addControlObjective([], 0, 1e-0)
            komo.addControlObjective([], 2, 1e+1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
            komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e+0],[math.cos(HIDING_ANLGE*math.pi/180)])
            komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1,1,1],target=start_position);

            ret = ry.NLP_Solver() \
                .setProblem(komo.nlp()) \
                .setOptions( stopTolerance=1e-2, verbose=4 ) \
                .solve()
            paths.append(komo.getPath())
            times.append(secs_to_hide_angle+MOVE_ADD_TIME)

            if KOMO_VIEW:
                komo.view(True, "Turn towards camera")
                komo.view_play(0.6)

        #Move to draw path

        #Move from start to finish with interpolation
        #Calculate time to paint with accerlation
        t_dec = HIDING_ANLGE/90*SECS_TO_HIDE
        secs_per_step = (1/resultion)/PAINTING_SPEED
        steps_to_turn = math.ceil(t_dec/secs_per_step)
        t_dec = secs_per_step*steps_to_turn #Adapt to fixed timesteps

        a = PAINTING_SPEED/t_dec
	    
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
        secs_to_paint = duration



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
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);

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
        
        times.append(secs_to_paint+MOVE_ADD_TIME)

        print(ret)

        if KOMO_VIEW:
            komo.view(True, "path to draw")
            komo.view_play(0.6)
    
        if not connected_to_next:
            #Rotate torchlight away from camera
            secs_to_hide_angle = SECS_TO_HIDE*(90-HIDING_ANLGE)/90
            end_position = waypoint_path[-1] # set end position

            C.setJointState(paths[-1][-1])
            komo = ry.KOMO()
            komo.setConfig(C, True)
            komo.setTiming(1., 1, secs_to_hide_angle, 2)
            komo.addControlObjective([], 0, 1e0)
            komo.addControlObjective([], 2, 1e+1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
            komo.addObjective([1.], ry.FS.scalarProductYY, ['l_gripper','world'], ry.OT.eq, [1e1],[0])
            komo.addObjective([1.], ry.FS.position,['l_gripper'], ry.OT.eq,scale=[1e1],target=end_position);

            ret = ry.NLP_Solver() \
                .setProblem(komo.nlp()) \
                .setOptions( stopTolerance=1e-2, verbose=4 ) \
                .solve()
            paths.append( komo.getPath())
            times.append(secs_to_hide_angle+MOVE_ADD_TIME)

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
komo.setTiming(1., 1, SECS_TO_HIDE, 2)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 2, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1],JOINT_LIMIT_OFFSET);
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
waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close = waypoints_from_svg(FILENAME,C.getFrame('l_gripper').getPosition())

motions, times = waypoints2motion(C,waypoint_paths, scalar_product_paths, lengths, durations, next_trajectory_really_close, C.getJointState())


#for motion, move_time in zip(motions, times):
#    move_time = move_time/speed_multiplier
#    bot.move(motion,[move_time])
#    



#bot.move(motions,times)
mall = [] 
tall = []
total_time = 0
for i,motion in enumerate(motions):
    time_i = float(times[i]/len(motion))/SPEED_MULTIPLIER
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
