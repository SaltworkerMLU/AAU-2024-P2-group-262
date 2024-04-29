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

Ps = pi/180 * RDK.Item('start').Joints();
Pe = pi/180 * RDK.Item('end').Joints();
%P0 = pi/180 * [0 ,-90, -90, 0, 90, 0];

[TBW, T06] = UR5.forwardKinematics(Pe, 1, 6); % Use forward kinematics to acquire transform matrices

disp(TBW)

solution = UR5.inverseKinematics(T06);

% Use 8 IK solution where joints are within range [-180, 180]
% Add said moveJ() commands to RoboDk
for i = 1:8 %size(solution,1)
    programname = sprintf('Start->end%i',i);
    progJoint = RDK.AddProgram(programname);

    timer(i) = UR5.moveJtf(Ps, solution(i,:)*pi/180);
    robot.setJoints(solution(i,:)); % via. point in RoboDK

    targetname = sprintf('Target%i',i);
    target = RDK.AddTarget(targetname, ref, robot);
    target.setJoints(solution(i,:));

    progJoint.addMoveJtf(target);
end

% Add moveL() to RoboDK
programname = sprintf('moveL_Start->end');
progJoint = RDK.AddProgram(programname);
progJoint.addMoveL(target);

UR5.moveJ(Ps, Pe);
UR5.moveL(Ps, Pe);
