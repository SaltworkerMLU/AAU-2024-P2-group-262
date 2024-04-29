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

%solution = UR5.inverseKinematics(T06);
solution = UR5.inverseKinematics(T06);


%IK = robot.SolveIK_All(TBW); % Provides all solution in JOINTxSOLUTION matrix
%IK(8,:) = []; % Remove 8th "joint" (hint: It shouldn't exist)
%IK(7,:) = []; % Remove 7th "joint" (hint: It shouldn't exist)
%disp("!Position has: " + length(IK) + " solutions")
%disp(IK.'); % Transpose to get SOLUTIONxJOINT matrix

%disp(length(solution))

%disp("MatLab solution:")
%disp(solution)

%shortest = solution(1,:);
farthest = solution(1,:);

%timer = zeros(size(solution,1),1);

%progJoint = RDK.AddProgram('Start->end');

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


maxim = find(timer == max(timer));
minim = find(timer == min(timer));
%disp(Ps*180/pi)

disp(max(timer))
disp(solution(maxim(1),:))
disp(min(timer))
disp(solution(minim,:))

UR5.moveJ(Ps, Pe);
UR5.moveL(Ps, Pe);

%UR5.moveJ(Ps,solution(minim(1),:)*pi/180)

%UR5.moveJ(Ps, solution(minim(1,:),:)*pi/180);
%disp(find(min(timer)))
%disp(solution(),:))

%disp("Shortest path:")
%disp(shortest)

%disp("Farthest path:")
%disp(farthest)

%progJoint = RDK.AddProgram('IK config');

%for i = 1:length(solution)
%    robot.setJoints(solution(i,:)); % via. point in RoboDK
    %targetname = sprintf('TargetL%i',i);
    %target = RDK.AddTarget(targetname,ref,robot);
    %progJoint.addMoveJ(target);
%end

%disp(TBW)
%disp(Pose_2_XYZRPW(TBW))

%UR5.moveL(Ps, Pe, robot);
%UR5.moveJ(Ps, Pe);

%UR5.moveJ(Ps, farthest*pi/180);
%UR5.moveJ(Ps, shortest*pi/180);