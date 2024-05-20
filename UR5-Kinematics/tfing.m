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

tic

%funky = 'Start->end1';
funky = 'Prog10';

RDK.Item(funky).setRunType(2)

RDK.Item(funky).RunProgram();
RDK.Item(funky).WaitFinished()

disp(toc)