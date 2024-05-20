clc
close
clear

RDK = Robolink; % Generate a Robolink object RDK. This object interfaces with RoboDK.
robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT); % Select robot
if robot.Valid() == 0
    error('No robot selected'); % Missing robot
end
ref = robot.Parent();

UR5; % Create object

targetname = sprintf('example');
robot.setJoints([-30 -30 -30 -30 -30 -30])
target = RDK.AddTarget(targetname, ref, robot);

P = pi/180 * RDK.Item('example').Joints();

[TBW, T06] = UR5.forwardKinematics(P, 1, 6); % Use forward kinematics to acquire transform matrices
pose = RDK.Item('example').Pose();

disp("Forward Kinematics -> Transform Matrix")
disp(pose)
disp(TBW)

disp("Transform Matrix -> XYZ angle-set: ")
DOF = UR5.sixDOF(TBW);
disp(DOF)
disp(Pose_2_Fanuc(TBW).')

disp("XYZ angle-set -> Transform Matrix")
disp(UR5.transformMatrix(DOF))
disp(Fanuc_2_Pose(DOF))
