classdef UR5
    properties (Constant)
        % Modified Denavit-Hartenberg parameters (DH)
        alpha = pi/180 * [0 90 0 0 -90 90]; % radians
        a = [0 0 425 392 0 0]; % mm
        theta = pi/180 * [0 180 0 0 0 180]; % radians
        d = [89.2 0 0 109.3 94.75 82.5]; % mm

        % Predefined transform matrices of the UR5
        TB0 = [eye(4,3) [0;0;89.2;1]]; % Transform matrix from base to joint 0 % d(1) = 89.2
        T6W = diag([-1 -1 1 1]); % Diagonal 4x4-matrix using eigenvalues [-1, -1, 1, 1]
    end

    methods (Static)
        % Forward Kinematics makes use of 6DOF joint values of UR5 and
        % converts them into cartesian coordinates in a transform matrix.
        % OUTPUT: TBW (Base-wrist T) & T06 (joint0-joint6 T)
        function [TBW, T06] = forwardKinematics(joint, start, ender) % s = start joint; e = end joint
            IOtheta = zeros(1, ender); % a 1xj-matrix where j is length of "joint".
            for i = start:ender % The "theta" needed is ...
                IOtheta(i) = UR5.theta(i) + joint(i); 
            end % ... the sum of each element DH-theta and joint

            TBW = eye(4); % Set TBW to identity matrix
            for i = start:ender % For every joint
                % Forward kinematics of T{i-1}{i}
                temp = [cos(IOtheta(i))                     -sin(IOtheta(i))                    0                  UR5.a(i);
                        sin(IOtheta(i))*cos(UR5.alpha(i))   cos(IOtheta(i))*cos(UR5.alpha(i))  -sin(UR5.alpha(i)) -sin(UR5.alpha(i))*UR5.d(i);
                        sin(IOtheta(i))*sin(UR5.alpha(i))   cos(IOtheta(i))*sin(UR5.alpha(i))   cos(UR5.alpha(i))  cos(UR5.alpha(i))*UR5.d(i);
                        0   0   0   1];
                TBW = TBW * temp; % Multiplying prior T with i'th transform matrix.
            end
            T06 = UR5.TB0 \ TBW / UR5.T6W; % T06 is calculated using the equation: T06 = TB0^(-1) * TBW * T6W^(-1)
        end

        % Inverse Kinematics makes use of the transform matrix T06 to
        % calculate a set of 6DOF joint solutions in which the UR5 can
        % traverse to said transform matrix. 
        % OUTPUT: 512x6 matrix "solution" (8*2^6 = 512)
        function solutions = inverseKinematics(T06)
            Joint = zeros(8,6); % "Unique" solutions are provided as 8 sets of 6DOF "Joint values"
            for i = 1:8 % For all 8 solutions
                %theta1: Has 2 possible solutions
                P05 = T06*[0; 0; -UR5.d(6); 1]; % Distance between P5 and P6 is d(6)
                phi1 = atan2(P05(2), P05(1));
                phi2 = acos(UR5.d(4)/sqrt(P05(1)^2+P05(2)^2));
                phi = [phi2 -phi2]; % a 1x2 vector containing each possible solution of theta1...
                Joint(i,1) = phi1 + phi(1+mod(round((i-1)/2),2)) + pi/2; % theta1 % ... solution is chosen using equation 1+mod(round((i-1)/2),2)

                %theta5: Has 2 possible solutions
                T01 = UR5.TB0 \ UR5.forwardKinematics(Joint(i,:), 1, 1); % Account for included base d(1) in Forward Kinematics
                T16 = T01 \ T06 ; % T16 = T01^(-1) * T06
                phi1 = acos((-T16(2,4)-UR5.d(4)) / UR5.d(6));
                phi = [phi1 -phi1]; % a 1x2 vector containing each possible solution of theta5...
                Joint(i,5) = phi(1+mod(round((i-1)/4),2)); % theta5 % ... solution is chosen using equation 1+mod(round((i-1)/4),2)

                %theta6: Has 4 possible solution
                T60 = inv(T06);
                T60x = [T60(1,1); T60(2,1); T60(3,1)];
                T60y = [T60(1,2); T60(2,2); T60(3,2)];
                Y61 = -sin(Joint(i,1))*T60x + cos(Joint(i,1))*T60y;
                Joint(i,6) = atan2((Y61(2,1)/sin(Joint(i,5))),-Y61(1,1)/sin(Joint(i,5))); % theta6
                Joint(i,6) = Joint(i,6) + UR5.theta(6); % Align with DH-parameter
                if Joint(i,6) > pi % if value is above 180 degrees
                    Joint(i,6) = Joint(i,6) - 2*pi; % subtract by 360 degrees
                end

                % Halfway there - Calculating T14 to find theta2, theta3 & theta4
                T45 = UR5.forwardKinematics(Joint(i,:), 5, 5);
                T56 = UR5.forwardKinematics(Joint(i,:), 6, 6) / UR5.T6W; % Account for included wrist theta(6) in Forward Kinematics
                T14 = T16 / (T45 * T56);

                % theta3: Has 8 possible solutions
                T14xz = norm([T14(1,4) T14(3,4)]); % Length of T14x and T14z
                phi1 = acos((T14xz^2 - UR5.a(3)^2 - UR5.a(4)^2) / (2 * UR5.a(3) * UR5.a(4))); % Law of cosine
                phi = [phi1 -phi1]; % a 1x2 vector containing each possible solution of theta3...
                Joint(i,3) = phi(1+mod(round((i-1)/8),2)); % theta3 % ... solution is chosen using equation 1+mod(round((i-1)/8),2)

                % If either Joint(i,1), Joint(i,3) or Joint(i,5) is an imaginary number...
                if ~isreal(Joint(i,1)) || ~isreal(Joint(i,3)) || ~isreal(Joint(i,5)) 
                    for j = 1:6 % ...delete solution entirely
                        Joint(i,j) = NaN;
                    end
                    continue % skip solution
                end

                % theta2: Has 8 possible solutions
                phi1 = atan2(-T14 (3,4), -T14 (1,4));
                phi2 = asin(UR5.a(4) * sin(Joint(i,3)) / T14xz);
                Joint(i,2) = phi1 - phi2; % theta2

                % theta4: Has 8 possible solutions
                T12=UR5.forwardKinematics(Joint(i,:), 2, 2);
                T23=UR5.forwardKinematics(Joint(i,:), 3, 3);
                T34 = (T12 * T23) \ T14;
                Joint(i,4) = atan2(T34(2,1),T34(1,1)); % theta4
            end

            Joint = 180/pi*Joint; % Convert to degrees
            Joint = unique(Joint,'rows'); % Remove duplicate solutions

            for i = length(Joint):-1:1 % Countdown...
                if isnan(Joint(i))
                    Joint(i,:) = []; %...  to remove 'NaN' rows
                end
            end
            
            solutions = zeros(size(Joint,1)*64, 6); % 512 possible solutions...
            
            for i = 1:64 % ... 8 of which are unique (512/8 = 64)
                temp = Joint;
                % Configuration of which joints to change by 360 degrees
                % using i in a binary context, e.g. 001000 = 4 & 000000 = 0
                config = [1-mod(i,2) 1-mod(round(i/2),2) mod(round((i-1)/8),2) mod(round((i-1)/16),2) mod(round((i-1)/32),2) mod(round((i-1)/64),2)];
                for j = 1:size(Joint,1) % For all 8 unique solutions
                    for k = 1:6 % For all 6 joints
                        if config(k) == 0 % If config has not changed... %lastConfig(k)
                            continue % ... pass
                        end
                        if temp(j,k) + 360 > 360
                            temp(j,k) = temp(j,k) - 360;
                        else
                            temp(j,k) = temp(j,k) + 360;
                        end
                    end
                    solutions((i-1)*size(Joint,1)+ j,:) = temp(j,:); % Set i'th set of joint solutions
                end
            end
        end
        
        % Calculates execution time in seconds of a moveJ() movement
        function tf = moveJtf(P0, Pf, v_max, a_max)
            P0 = 180/pi * P0; % Convert to degrees
            Pf = 180/pi * Pf; % Convert to degrees
            
            % Using moveJ, tf depends on the joint which moves the most
            tf_a = max(abs(sqrt(4*(Pf - P0)/a_max)));
            tf_v = max(abs((Pf - P0)/v_max)); % t = ds/v
            tb = v_max / a_max; % t = v / a

            if 2*tb > tf_a % If acceleration does not reach max
                tf = tf_a;
            else % Otherwise, if acceleration reaches max
                tf = tf_v + tb;
            end
            disp("Execution time of moveJ(): " + tf + "s");
        end
        
        % Calculates execution time in seconds of a moveL() movement
        function tf = moveLtf(P0, Pf, v_max, a_max)
            TBS = UR5.forwardKinematics(P0, 1, 6); % Use only TBW of P0
            TBE = UR5.forwardKinematics(Pf, 1, 6); % Use only TBW of Pf

            Tse = TBS \ TBE; % Compute Tse (Angle-axis representation)
            theta = acos((Tse(1,1)+Tse(2,2)+Tse(3,3)-1)/2); % Find axis angle
            Ps = [0 0 0 0]; % Start frame position (origo)
            Pe = [Tse(1,4) Tse(2,4) Tse(3,4) theta]; % end postion relative to start frame
            
            % Using moveL, tf depends on the length of traversed distance
            tf_a = sqrt(4*norm(Pe - Ps)/a_max); 
            tf_v = norm(Pe - Ps)/v_max;
            tb = v_max / a_max; % t = v / a

            if 2*tb > tf_a % If acceleration does not reach max
                tf = tf_a;
            else % Otherwise, if acceleration reaches max
                tf = tf_v + tb;
            end
            disp("Execution time of moveL(): " + tf + "s");
        end

        % Calculates and plots moveJ() command using UR5 and cubic polynomial
        % https://se.mathworks.com/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html
        function moveJ(P0, Pf, v_max, a_max)
            tf = UR5.moveJtf(P0, Pf, v_max, a_max);
            tInterval = 0:tf; %Time interval displayed using the method % 0:timeStep=1:UR5.moveLtf(P0, Pf)

            % Plan trajectory of Cubic Polynomial
            a0 = P0;
            a1 = 0; % P0dot = 0
            a2 = 3/(tf.^2)*(Pf-P0);
            a3 = -2/(tf.^3)*(Pf-P0);

            % Plotting
            figure;
            ur5_RBT = loadrobot('universalUR5','DataFormat','row');
            show(ur5_RBT,P0,'PreservePlot',false,'Frames','off');
            hold on
            axis([-1 1 -1 1 -0.1 1.5]);

            jointConfigArray = zeros(length(P0), length(tInterval)-1);
            for i=1:length(tInterval)
                % Interpolate simulated joint positions to get configuration at current time
                jointConfigArray(:,i)=a0+a1*tInterval(i)+a2*tInterval(i)^2+a3*tInterval(i)^3; % Aquire position
                configNow = jointConfigArray(:,i).'; % Transpose
                poseNow = UR5.forwardKinematics(configNow, 1, 6);
                poseNow(1,4) = poseNow(1,4) / 1000; % Convert from mm to m
                poseNow(2,4) = poseNow(2,4) / 1000; % Convert from mm to m
                poseNow(3,4) = poseNow(3,4) / 1000; % Convert from mm to m
                configNow(1) = configNow(1) + pi; % An error in MabLab plotting system accounted for
                show(ur5_RBT,configNow,'PreservePlot',false,'Frames','off');
                jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
                drawnow; % Draw trajectory iteratively
            end
            %legend(jointSpaceMarker, {'Defined in Joint-Space'}); % Add a legend
            title('MoveJ()') % Add a title
        end
   
        % Calculates and plots moveL() command using UR5 and cubic polynomial
        function moveL(P0, Pf, v_max, a_max)
            tf = UR5.moveLtf(P0, Pf, v_max, a_max);
            tInterval = 0:tf; %Time interval displayed using the method % 0:timeStep=1:UR5.moveLtf(P0, Pf)
            
            % Cartesian space is used
            taskInit = UR5.forwardKinematics(P0, 1, 6); %UR5.TB0 \ poseNow / UR5.T6W
            taskFinal = UR5.forwardKinematics(Pf, 1, 6);

            % Convert to MatLab plot units
            for i = 1:2 % Reverse x and y-axis
                taskInit(i,:) = -taskInit(i,:); 
                taskFinal(i,:) = -taskFinal(i,:);
            end
            for i = 1:3 % mm -> m
                taskInit(i,4) = taskInit(i,4) / 1000; 
                taskFinal(i,4) = taskFinal(i,4) / 1000;
            end

            Tse = taskInit \ taskFinal; % Tse = TBS \ TBE; %Compute Tse (Angle-axis representation)
            theta = acos((Tse(1,1)+Tse(2,2)+Tse(3,3)-1)/2); %Find axis angle
            Ps = [0 0 0 0]; % Start frame position (origo)
            Pe = [Tse(1,4) Tse(2,4) Tse(3,4) theta]; % end postion relative to start frame

            % Plan trajectory of Cubic Polynomial
            a0 = Ps;
            a1 = 0; % P0dot = 0
            a2 = 3/(tf.^2)*(Pe-Ps);
            a3 = -2/(tf.^3)*(Pe-Ps);
            
            % Motion models
            ur5_RBT = loadrobot('universalUR5','DataFormat','row');
            endEffector = "tool0"; % In case another robot is used, analyse ur5_RBT properties/variables
            tsMotionModel = taskSpaceMotionModel('RigidBodyTree',ur5_RBT,'EndEffectorName',endEffector); % Cartesian space-trajectory motion

            % Velocity is constant, thus testVel is constant. Use t=0
            [~, testVel] = transformtraj(taskInit, taskFinal, [tInterval(1); tInterval(end)], 0);
            [tTask,stateTask] = ode15s(@(t,state) derivative(tsMotionModel, state, transformtraj(taskInit, taskFinal, [tInterval(1); tInterval(end)], t), testVel),[tInterval(1); tInterval(end)],[P0, zeros(size(P0))]);

            % Plotting
            figure;
            show(ur5_RBT,P0,'PreservePlot',false,'Frames','off'); % Extra params to update UR5 position
            hold on
            axis([-1 1 -1 1 -0.1 1.5])
            for i=1:length(tInterval)
                % Interpolate simulated joint positions to get configuration at current time
                configNow = interp1(tTask,stateTask(:,1:6),tInterval(i)); % Interpolate using time, state, joint [1,6], current time
                configNow(1) = configNow(1) + pi;
                P=a0+a1*tInterval(i)+a2*tInterval(i).^2+a3*tInterval(i).^3; % Aquire position
                theta=P(4); % aquire theta from P
                if (abs(theta) < 0.000001)
                    k = [0;0;0];
                else % theta presumed Positive
                    k=1/(2*sin(theta))*[Tse(3,2)-Tse(2,3);Tse(1,3)-Tse(3,1);Tse(2,1)-Tse(1,2)];
                end
                TSI= [k(1)*k(1)*(1-cos(theta))+cos(theta)     k(1)*k(2)*(1-cos(theta))-k(3)*sin(theta) k(1)*k(3)*(1-cos(theta))+k(2)*sin(theta) P(1);
                        k(1)*k(2)*(1-cos(theta))+k(3)*sin(theta)  k(2)*k(2)*(1-cos(theta))+cos(theta)    k(2)*k(3)*(1-cos(theta))-k(1)*sin(theta) P(2) ;
                        k(1)*k(3)*(1-cos(theta))-k(2)*sin(theta)  k(3)*k(2)*(1-cos(theta))+k(1)*sin(theta) k(3)*k(3)*(1-cos(theta))+cos(theta) P(3);
                        0 0 0 1;]; % Convert [x y z theta] to 4x4 transform matrix
                poseNow=taskInit*TSI;% TBI=TBS*TSI;
                show(ur5_RBT,configNow,'PreservePlot',false,'Frames','off'); % use this to show ideal linear trajectory
                taskSpaceMarker = plot3(-poseNow(1,4),-poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20); % Use negative values to account for MatLab plot error
                drawnow; % Draw trajectory iteratively
            end
            %legend(taskSpaceMarker, {'Defined in Task-Space'}); % Add a legend
            title('MoveL()') % Add a title
        end

        % Calculates and plots moveJ() command using UR5 and cubic polynomial
        % ALTERNATE METHOD - Spoiler alert: It's not 100% accurate
        % https://se.mathworks.com/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html
        function moveLalt(P0, Pf, v_max, a_max)
            tf = UR5.moveLtf(P0, Pf, v_max, a_max);
            tInterval = 0:tf; %Time interval displayed using the method % 0:timeStep=1:UR5.moveLtf(P0, Pf)

            % Cartesian space is used
            TBS = UR5.forwardKinematics(P0, 1, 6);
            TBE = UR5.forwardKinematics(Pf, 1, 6);
            Tse = TBS \ TBE; %Compute Tse (Angle-axis representation)
            theta = acos((Tse(1,1)+Tse(2,2)+Tse(3,3)-1)/2); %Find axis angle
            Ps = [0 0 0 0]; % Start frame position (origo)
            Pe = [Tse(1,4) Tse(2,4) Tse(3,4) theta]; % end postion relative to start frame

            % Plan trajectory of Cubic Polynomial
            a0 = Ps;
            a1 = 0; % P0dot = 0
            a2 = 3/(tf.^2)*(Pe-Ps);
            a3 = -2/(tf.^3)*(Pe-Ps);

            % Plotting
            figure;
            ur5_RBT = loadrobot('universalUR5','DataFormat','row');
            show(ur5_RBT,P0,'PreservePlot',false,'Frames','off'); % Extra params to update UR5 position
            hold on
            axis([-1 1 -1 1 -0.1 1.5])
            for i=1:length(tInterval)
                % Interpolate simulated joint positions to get configuration at current time
                P=a0+a1*tInterval(i)+a2*tInterval(i).^2+a3*tInterval(i).^3; % Aquire position
                theta=P(4); % aquire theta from P
                if (abs(theta) < 0.000001)
                    k = [0;0;0];
                else % theta presumed Positive
                    k=1/(2*sin(theta))*[Tse(3,2)-Tse(2,3);Tse(1,3)-Tse(3,1);Tse(2,1)-Tse(1,2)];
                end
                TSI= [k(1)*k(1)*(1-cos(theta))+cos(theta)     k(1)*k(2)*(1-cos(theta))-k(3)*sin(theta) k(1)*k(3)*(1-cos(theta))+k(2)*sin(theta) P(1);
                        k(1)*k(2)*(1-cos(theta))+k(3)*sin(theta)  k(2)*k(2)*(1-cos(theta))+cos(theta)    k(2)*k(3)*(1-cos(theta))-k(1)*sin(theta) P(2) ;
                        k(1)*k(3)*(1-cos(theta))-k(2)*sin(theta)  k(3)*k(2)*(1-cos(theta))+k(1)*sin(theta) k(3)*k(3)*(1-cos(theta))+cos(theta) P(3);
                        0 0 0 1;]; % Convert [x y z theta] to 4x4 transform matrix
                poseNow=taskInit*TSI;% TBI=TBS*TSI;
                % I'm gonna do what's called a "pro gamer move"
                % Done correctly in terms of maths but still slightly misaligns :(
                solset = UR5.inverseKinematics(UR5.TB0 \ poseNow / UR5.T6W); % UR5.TB0 \ TBW / UR5.T6W
                solset = solset * pi/180;
                nearest = solset(1,:);
                for j = 2:8
                    if norm(nearest - P0) > norm(solset(j,:) - P0)
                        nearest = solset(j,:);
                    end
                end
                nearest(1) = nearest(1) + pi;
                show(ur5_RBT,nearest,'PreservePlot',false,'Frames','off'); % use this to show actual linear trajectory
                taskSpaceMarker = plot3(poseNow(1,4)/1000,poseNow(2,4)/1000,poseNow(3,4)/1000,'b.','MarkerSize',20);
                drawnow; % Draw trajectory iteratively
            end
            %legend(taskSpaceMarker, {'Defined in Task-Space'}); % Add a legend
            title('Alternative MoveL()') % Add a title
        end
        
        % Plots the UR5 in 6DOF joint position P
        function spot(P)
            figure;
            P(1) = P(1) + pi;  % An error in MabLab plotting system accounted for
            show(loadrobot('universalUR5','DataFormat','row'),P,'PreservePlot',false,'Frames','off'); % Extra params to update UR5 position
            hold on
            axis([-1 1 -1 1 -0.1 1.5])
            title('start') % Add a title
        end
    end
end