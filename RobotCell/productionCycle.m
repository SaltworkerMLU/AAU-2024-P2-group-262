clear % Clear stored variables (Workspace)

RDK = Robolink; % Generate a Robolink object RDK. This object interfaces with RoboDK.
robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT); % Select robot
if robot.Valid() == 0
    error('No robot selected'); % Missing robot
end
ref = robot.Parent();

% Make used programs run on robot and not simulation
runType = 1; % 1 = simulation; 2 = IRL

RDK.Item('TopRed').setRunType(runType)
RDK.Item('TopBlack').setRunType(runType)
RDK.Item('TopBlue').setRunType(runType)

RDK.Item('BottomRed').setRunType(runType)
RDK.Item('BottomBlack').setRunType(runType)
RDK.Item('BottomBlue').setRunType(runType)

RDK.Item('FuseOne').setRunType(runType)
RDK.Item('FuseTwo').setRunType(runType)

RDK.Item('PCBZero').setRunType(runType)
RDK.Item('PCBTwo').setRunType(runType)

RDK.Item('AssembleZero').setRunType(runType) %2=PROGRAM_RUN_ON_ROBOT
RDK.Item('AssembleTwo').setRunType(runType)

while true
    clc % Clear terminal (Command Window)

    disp("Type a combination of the 3 characters:");
    disp("(1) R = red, G = black, B = blue, 0 = [end program]"); % User input
    disp("(2) 0 = 0 fuses, 1 = 1 fuse, 2 = 2 fuses"); % User input
    disp("(3) R = red, G = black, B = blue"); % User input
    txt = input("Where: (1) = top cover; (2) = fuses; (3) = bottom cover\n\n", "s");
    
    for i = 1:3:length(txt) % start by using i = 1... run for 1/3 * length(txt)

        % End program in respective iteration if character "0" is used
        if (txt(i) == "0")
            return; % End program
        end

        % Failsafe - Input must consist of 3 characters
        if length(txt) <= i %  because i = i + 3
            break
        end
        
        % Failsafes - make sure correct input is used
        if (txt(i) ~= "R" && txt(i) ~= "G" && txt(i) ~= "B")
            break % Input must use correct inputs
        end
        if (txt(i+1) ~= "0" && txt(i+1) ~= "1" && txt(i+1) ~= "2")
            break
        end
        if (txt(i+2) ~= "R" && txt(i+2) ~= "G" && txt(i+2) ~= "B")
            break
        end

        disp("Producing... " + txt(i) + txt(i+1) + txt(i+2))

        % Top cover -> the second digit of input txt
        if (txt(i) == "R")
            RDK.Item('TopRed').RunProgram();
            RDK.Item('TopRed').WaitFinished()
        end
        if (txt(i) == "G")
            RDK.Item('TopBlack').RunProgram();
            RDK.Item('TopBlack').WaitFinished()
        end
        if (txt(i) == "B")
            RDK.Item('TopBlue').RunProgram();
            RDK.Item('TopBlue').WaitFinished()
        end

        % Must pick up fuses first in case of bottom red cover...
        if (txt(i+2) == "R" && txt(i+1)=="1")
            RDK.Item('FuseOne').RunProgram();
            RDK.Item('FuseOne').WaitFinished()
        end
        if (txt(i+2) == "R" && txt(i+1)=="2")
            RDK.Item('FuseTwo').RunProgram();
            RDK.Item('FuseTwo').WaitFinished()
        end

        % Do bottom red before pcb to prevent gripper self-destruct
        if (txt(i+2) == "R")
            RDK.Item('BottomRed').RunProgram();
            RDK.Item('BottomRed').WaitFinished()
        end

        % ... Otherwise, pick up fuses after bottom red
        if (txt(i+2) ~= "R" && txt(i+1)=="1")
            RDK.Item('FuseOne').RunProgram();
            RDK.Item('FuseOne').WaitFinished()
        end
        if (txt(i+2) ~= "R" && txt(i+1)=="2")
            RDK.Item('FuseTwo').RunProgram();
            RDK.Item('FuseTwo').WaitFinished()
        end

        % Pick up pcb
        if (txt(i+1) == "0")
            RDK.Item('PCBZero').RunProgram();
            RDK.Item('PCBZero').WaitFinished()
        end
        if (txt(i+1) == "1" || txt(i+1) == "2") % One fuse & pcb assmuing two fuses
            RDK.Item('PCBTwo').RunProgram();
            RDK.Item('PCBTwo').WaitFinished()
        end
    
        % Bottom Cover -> the first digit of input txt
        % Instance of bottom red has been taken care of
        if (txt(i+2) == "G")
            RDK.Item('BottomBlack').RunProgram();
            RDK.Item('BottomBlack').WaitFinished()
        end
        if (txt(i+2) == "B")
            RDK.Item('BottomBlue').RunProgram();
            RDK.Item('BottomBlue').WaitFinished()
        end

        % Assemble depends on amount of fuses'
        if (txt(i+1) == "0")
            RDK.Item('AssembleZero').RunProgram();
            RDK.Item('AssembleZero').WaitFinished()
        else %if (txt(i+1) == "1" || txt(i+1) == "2")
            RDK.Item('AssembleTwo').RunProgram();
            RDK.Item('AssembleTwo').WaitFinished()
        end
    end
end