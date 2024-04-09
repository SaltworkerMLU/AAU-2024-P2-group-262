clc
close
clear

RDK = Robolink; % Generate a Robolink object RDK. This object interfaces with RoboDK.
robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT); % Select robot
if robot.Valid() == 0
    error('No robot selected'); % Missing robot
end

UR5; % Create object

P0 = pi/180 * [0 ,-90, -90, 0, 90, 0];
Pf = pi/180 * [-59.24 ,-124.95, -78.80, 25.82, 89.62, 107.79];
P2 = pi/180 * [0 ,0, 0, 0, 90, 0];

[TBW, T06] = UR5.forwardKinematics(Pf, 1, 6); % Use forward kinematics to acquire transform matrices
%[TBW, T06] = UR5.forwardKinematics(P2, 1, 6); % Use forward kinematics to acquire transform matrices

disp(T06)

solution = UR5.inverseKinematics(T06);

disp(solution)

UR5.moveL(P0, Pf, robot);
UR5.moveJ(P0, Pf, robot);
