clc
close
clear

UR5; % Create object

v_linear = 100;
a_linear = 100;
v_angular = 11.5;
a_angular = 20;

RDK = Robolink; % Generate a Robolink object RDK. This object interfaces with RoboDK.
robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT); % Select robot
if robot.Valid() == 0
    error('No robot selected'); % Missing robot
end
ref = robot.Parent();

Ps = pi/180 * RDK.Item('start').Joints();
Pe = pi/180 * RDK.Item('end').Joints();

[TBW, T06] = UR5.forwardKinematics(Pe, 1, 6);

solution = UR5.inverseKinematics(T06);

IK = robot.SolveIK_All(TBW); % Provides all solution in JOINTxSOLUTION matrix
IK(8,:) = []; % Remove 8th "joint" (hint: It shouldn't exist)
IK(7,:) = []; % Remove 7th "joint" (hint: It shouldn't exist)
disp("Total RoboDK solutions: " + length(IK.'))
disp(IK.'); % Transpose to get SOLUTIONxJOINT matrix

disp("Total MatLab solutions: " + length(solution))
disp(solution)

UR5.spot(Ps)

% Use (up to) 8 IK solution where joints are within range [-180, 180]
% Add said moveJ() commands to RoboDk
for i = 1:1:size(solution,1)/64
    programname = sprintf('Start->end%i',i);
    progJoint = RDK.AddProgram(programname);
    robot.setJoints(solution(i,:)); % via. point in RoboDK

    targetname = sprintf('Target%i',i);
    target = RDK.AddTarget(targetname, ref, robot);
    target.setJoints(solution(i,:));

    progJoint.addMoveJ(target);

    UR5.moveJ(Ps, solution(i,:)*pi/180, v_angular, a_angular); % Matlab tf only
end

% Add moveL() to RoboDK
programname = sprintf('moveL_Start->end');
progJoint = RDK.AddProgram(programname);
progJoint.addMoveL(target);

UR5.moveL(Ps, Pe, v_linear, a_linear);