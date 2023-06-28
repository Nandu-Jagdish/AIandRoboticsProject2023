from robotic import ry
import numpy as np
import time

ry.params_add({'Franka/Kp_freq': [30., 30., 30., 20., 10., 15., 10.],
               'Franka/friction': np.array([0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4]),
               'Franka/Kd_ratio': np.array([0.6, 0.6, 0.3, 0.3, 0.3, 0.3, 0.4])})


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

# define external camera frame
cam = C.addFrame( "externalCamera")
cam.setShape(ry.ST.ssBox, size=[.1,.1,.1,.005])
cam.setColor([1,0,0,1])
cam.setPose("t(0 1.0 1.2)")

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

C.view()

input("Press Enter to home")
bot = ry.BotOp(C, False)
bot.home(C)

#Wait for the robot to finish homing
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

#quatCam = C.getFrame('externalCamera').getQuaternion()
#rotMatCam = quaternion_rotation_matrix(quatCam)

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
komo.addObjective([], ry.FS.positionRel, ['externalCamera','l_gripper'], ry.OT.ineq, scale=[[0,1,0]],target=[0,0,0]);
komo.addObjective([], ry.FS.positionRel, ['externalCamera','l_gripper'], ry.OT.eq, scale=[[1,0,0],[0,0,0],[0,0,1]],target=[0,0,0]);
#komo.addObjective([], ry.FS.positionRel, ['l_gripper', 'externalCamera'], ry.OT.eq, scale=[[1,0,0],[0,0,1]],target=[0,0,0]);
#komo.addObjective([], ry.FS.quaternionDiff, ['l_gripper', 'externalCamera'], ry.OT.eq, [1e1],target=ry.FS.vectorZ)
#komo.addObjective([], ry.FS.gazeAt, ['l_gripper', 'externalCamera'], ry.OT.eq, [1e1],target=[-0,-1,0])
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
#komo.addObjective([], ry.FS.quaternionDiff, ['l_gripper', 'externalCamera'], ry.OT.eq, [1e1],target=ry.FS.vectorZ)
komo.addObjective([], ry.FS.positionRel, ['externalCamera','l_gripper'], ry.OT.ineq, scale=[[0,1,0]],target=[0,0,0]);
komo.addObjective([], ry.FS.positionRel, ['externalCamera','l_gripper'], ry.OT.eq, scale=[[1,0,0],[0,0,0],[0,0,1]],target=[0,0,0]);
#komo.addObjective([], ry.FS.gazeAt, ['l_gripper', 'externalCamera'], ry.OT.eq, [1e1],target=[0,0])
#komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1],[0,-1,0])

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
