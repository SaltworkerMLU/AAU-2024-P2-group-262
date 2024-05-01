clc
close
clear

UR5; % Create object

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

disp("Total MatLab solutions: " + length(solution))
disp(solution)

UR5.spot(Ps) % Spot UR5 at position Ps

% Use 8 IK solution where joints are within range [-180, 180]
for i = 1:8 %size(solution,1)
    UR5.moveJ(Ps, solution(i,:)*pi/180, 11.5, 20); % Matlab tf only
end

UR5.moveL(Ps, Pe, 100, 100);