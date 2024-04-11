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
% P4 = RDK.Item('Target 3').Joints();
%P0 = pi/180 * [0 ,-90, -90, 0, 90, 0];
%Pf = pi/180 * [-59.24 ,-124.95, -78.80, 25.82, 89.62, 107.79];
%P2 = pi/180 * [0 ,0, 0, 0, 90, 0];


%[TBW, T06] = UR5.forwardKinematics(P0, 1, 6); % Use forward kinematics to acquire transform matrices
[TBW, T06] = UR5.forwardKinematics(Ps, 1, 6); % Use forward kinematics to acquire transform matrices

solution = UR5.inverseKinematics(T06);

IK = robot.SolveIK_All(TBW); % Provides all solution in JOINTxSOLUTION matrix
IK(8,:) = []; % Remove 8th "joint" (hint: It shouldn't exist)
IK(7,:) = []; % Remove 7th "joint" (hint: It shouldn't exist)
disp("!Position has: " + length(IK) + " solutions")
disp(IK.'); % Transpose to get SOLUTIONxJOINT matrix

disp("MatLab solution:")
disp(solution)

progJoint = RDK.AddProgram('IK config');

for i = 1:length(solution)
    robot.setJoints(solution(i,:)); % via. point in RoboDK
    targetname = sprintf('TargetL%i',i);
    target = RDK.AddTarget(targetname,ref,robot);
    progJoint.addMoveJ(target);
end

UR5.moveL(Ps, Pe, robot);
UR5.moveJ(Ps, Pe);
