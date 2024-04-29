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
        a_max = 0.9; %2922
        v_max = 10;
    end

    methods (Static)
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

        function solutions = inverseKinematics(T06)
            % Solutions are provided as 8 sets of "Joint values"
            Joint = zeros(8,6);
            for i = 1:8 % For all 8 solutions
                %theta1: Has 2 possible solutions
                P05 = T06*[0; 0; -UR5.d(6); 1]; % Distance between P5 and P6 is d(6)
                phi1 = atan2(P05(2), P05(1));
                phi2 = acos(UR5.d(4)/sqrt(P05(1)^2+P05(2)^2));
                phi = [phi2 -phi2]; % a 1x2 vector containing each possible solution of theta1...
                Joint(i,1) = phi1 + phi(1+mod(round((i-1)/2),2)) + pi/2; % theta1 % ... solution is chosen using equation 1+mod(round((i-1)/2),2)

                %theta5: Has 2 possible solutions (depending on theta1)
                T01 = UR5.TB0 \ UR5.forwardKinematics(Joint(i,:), 1, 1); % T01 = TB0^(-1) * TB1, essentially d(1) is disregarded, as it is nullified using TB0
                T16 = T01 \ T06 ; % T16 = T01^(-1) * T06
                phi1 = acos((-T16(2,4)-UR5.d(4))/UR5.d(6));
                phi = [phi1 -phi1]; % a 1x2 vector containing each possible solution of theta5...
                Joint(i,5) = phi(1+mod(round((i-1)/4),2)); % theta5 % ... solution is chosen using equation 1+mod(round((i-1)/4),2)

                %theta6: Has 4 possible solution (depending on theta1)
                T60 = inv ( T06 ) ;
                T60x = [ T60(1 ,1) ; T60(2 ,1) ; T60(3 ,1) ];
                T60y = [ T60(1 ,2) ; T60(2 ,2) ; T60(3 ,2) ];
                Y61 = -sin(Joint(i,1)) * T60x + cos(Joint(i,1)) * T60y;
                Joint(i,6) = atan2( (Y61(2,1)/sin(Joint(i,5))),-Y61(1,1)/sin(Joint(i,5)) ) ; % theta6 %  - pi
                Joint(i,6) = Joint(i,6) + UR5.theta(6);
                if Joint(i,6) > pi % if value is above 180 degrees
                    Joint(i,6) = Joint(i,6) - 2*pi; % subtract by 360 degrees
                end

                % Halfway there - Calculating theta2 , theta3 & theta4
                T45 = UR5.forwardKinematics(Joint(i,:), 5, 5);
                T56 = UR5.forwardKinematics(Joint(i,:), 6, 6) / UR5.T6W; % Replace P05 by calculating T56
                T14 = T16 / ( T45 * T56 );

                % theta3: Has 8 possible solutions (depending on theta1 and theta6)
                T14xz = norm([T14(1,4) T14(3,4)]); % Length of T14x and T14z
                phi1 = acos((T14xz^2 - UR5.a(3)^2 - UR5.a(4)^2)/(2 * UR5.a(3) * UR5.a(4)));
                phi = [phi1 -phi1];
                Joint(i,3) = phi(1+mod(round((i-1)/8),2)); % theta3 % ... solution is chosen using equation 1+mod(round((i-1)/4),2)

                % If either Joint(i,1), Joint(i,3) or Joint(i,5) is an imaginary number...
                if ~isreal(Joint(i,1)) || ~isreal(Joint(i,3)) || ~isreal(Joint(i,5)) 
                    for j = 1:6 % ...delete solution entirely
                        Joint(i,j) = NaN;
                    end
                    continue % skip solution
                end

                % theta2: Has 8 possible solutions (depending on theta1 and theta6)
                phi1 = atan2 ( - T14 (3 ,4) , - T14 (1 ,4) ) ;
                phi2 = asin ( UR5.a(4) * sin (Joint(i,3)) / T14xz ) ;
                Joint(i,2) = phi1 - phi2; % theta2

                % theta4: Has 8 possible solutions (depending on theta1 and theta6)
                T12=UR5.forwardKinematics(Joint(i,:), 2, 2);
                T23=UR5.forwardKinematics(Joint(i,:), 3, 3);
                T34 = (T12 * T23) \ T14;
                Joint(i,4) = atan2 ( T34 (2 ,1) , T34 (1 ,1) ) ; % theta4
            end

            Joint = 180/pi*Joint; % Convert to degrees
            Joint = unique(Joint,'rows'); % Remove duplicate solutions

            for i = length(Joint):-1:1 % Countdown to remove 'NaN' rows
                if isnan(Joint(i))
                    Joint(i,:) = [];
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
        
        function tf = moveJtf(P0, Pf)
            P0 = 180/pi * P0; % Convert to degrees
            Pf = 180/pi * Pf; % Convert to degrees

            tf_a = max(abs(sqrt(4*(Pf - P0)/UR5.a_max)));
            tf_v = max(abs((Pf - P0)/UR5.v_max)); % t = ds/v
            tb = UR5.v_max / UR5.a_max; % t = v / a

            if 2*tb > tf_a % If acceleration reaches max.
                tf = tf_a;
            else % if velocity hits max
                tf = tf_v + tb;
            end
            disp("Execution time of moveJ(): " + tf + "s");
        end

        function tf = moveLtf(P0, Pf)
            TBS = UR5.forwardKinematics(P0, 1, 6); % Use only TBW of P0
            TBE = UR5.forwardKinematics(Pf, 1, 6); % Use only TBW of Pf

            Tse = TBS \ TBE; %Compute Tse (Angle-axis representation)
            theta = acos((Tse(1,1)+Tse(2,2)+Tse(3,3)-1)/2); %Find axis angle
            Ps = [0 0 0 0]; % Start frame position (origo)
            Pe = [Tse(1,4) Tse(2,4) Tse(3,4) theta]; % end postion relative to start frame

            tf_a = sqrt(4*norm(Pe - Ps)/UR5.a_max); 
            tf_v = norm(Pe - Ps)/UR5.v_max;
            tb = UR5.v_max / UR5.a_max; % t = v / a

            if 2*tb > tf_a % If acceleration reaches max.
                tf = tf_a;
            else % if velocity hits max
                tf = tf_v + tb;
            end
            disp("Execution time of moveL(): " + tf + "s");
        end

        function stateDot = exampleHelperTimeBasedJointInputs(motionModel, timeInterval, configWaypoints, t, state)
            % This function is for internal use only and may be removed in a future
            % release

            %exampleHelperTimeBasedJointInputs Pass time-varying inputs to the jointSpaceMotionModel derivative
            %   Since the jointSpaceMotionModel derivative method is updated at an
            %   instant in time, a wrapper function is needed to provide time-varying
            %   tracking inputs. This function computes the value of the B-spline
            %   trajectory at an instant in time, t, and provides that input to the
            %   derivative of the associated jointSpaceMotionModel at that same
            %   instant. The resultant state derivative can be passed to an ODE solver
            %   to compute the tracking behavior.

            % Copyright 2019 The MathWorks, Inc.

            % Use a B-spline curve to ensure the trajectory is smooth and moves
            % through the waypoints with non-zero velocity
            [qd, qdDot] = bsplinepolytraj(configWaypoints,  timeInterval , t);
    
            % Compute state derivative
            stateDot = derivative(motionModel, state, [qd; qdDot]);
        end

        % https://se.mathworks.com/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html
        function moveJ(P0, Pf)
            ur5_RBT = loadrobot('universalUR5','DataFormat','row');
            endEffector = "tool0"; % In case another robot is used, analyse ur5_RBT properties/variables

            % Joint space-trajectory
            tInterval = 0:UR5.moveJtf(P0, Pf); %Time interval displayed using the method % 0:timeStep=1:UR5.moveLtf(P0, Pf)
            jointConfigArray = cubicpolytraj([P0',Pf'],[tInterval(1); tInterval(end)],tInterval);
            jsMotionModel = jointSpaceMotionModel('RigidBodyTree',ur5_RBT,'MotionType','PDControl');
            [tJoint,stateJoint] = ode15s(@(t,state) UR5.exampleHelperTimeBasedJointInputs(jsMotionModel,[tInterval(1); tInterval(end)],jointConfigArray,t,state), [tInterval(1); tInterval(end)],[P0,zeros(size(P0))]);

            % Plotting
            figure;
            show(ur5_RBT,P0,'PreservePlot',false,'Frames','off');
            hold on
            axis([-1 1 -1 1 -0.1 1.5]);
            for i=1:length(tInterval)
                % Interpolate simulated joint positions to get configuration at current time
                configNow = interp1(tJoint,stateJoint(:,1:6),tInterval(i)); % Interpolate using time, state, joint [1,6], current time
                poseNow = getTransform(ur5_RBT,configNow,endEffector);
                show(ur5_RBT,configNow,'PreservePlot',false,'Frames','off');
                jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
                drawnow; % Draw trajectory iteratively
            end
            %legend(jointSpaceMarker, {'Defined in Joint-Space'}); % Add a legend
            title('MoveJ()') % Add a title
        end

        function stateDot = exampleHelperTimeBasedTaskInputs(motionModel, timeInterval, initialTform, finalTform, t, state)
            % This function is for internal use only and may be removed in a future
            % release

            %exampleHelperTimeBasedTaskInputs Pass time-varying inputs to the taskSpaceMotionModel derivative
            %   Since the taskSpaceMotionModel derivative method is updated at an
            %   instant in time, a wrapper function is needed to provide time-varying
            %   tracking inputs. This function computes the value of the transform
            %   trajectory at an instant in time, t, and provides that input to the
            %   derivative of the associated taskSpaceMotionModel at that same instant.
            %   The resultant state derivative can be passed to an ODE solver to
            %   compute the tracking behavior.

            % Copyright 2019 The MathWorks, Inc.

            [refPose, refVel] = transformtraj(initialTform, finalTform, timeInterval, t);

            stateDot = derivative(motionModel, state, refPose, refVel);
        end
        
        % https://se.mathworks.com/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html
        function moveL(P0, Pf)
            ur5_RBT = loadrobot('universalUR5','DataFormat','row');
            endEffector = "tool0"; % In case another robot is used, analyse ur5_RBT properties/variables
            
            % Task space-trajectory
            taskInit = getTransform(ur5_RBT,P0,endEffector);
            taskFinal = getTransform(ur5_RBT,Pf,endEffector);
            tInterval = 0:UR5.moveLtf(P0, Pf); %Time interval displayed using the method % 0:timeStep=1:UR5.moveLtf(P0, Pf)
            tsMotionModel = taskSpaceMotionModel('RigidBodyTree',ur5_RBT,'EndEffectorName',endEffector); % Cartesian space-trajectory motion
            [tTask,stateTask] = ode15s(@(t,state) UR5.exampleHelperTimeBasedTaskInputs(tsMotionModel,[tInterval(1); tInterval(end)],taskInit,taskFinal,t,state),[tInterval(1); tInterval(end)],[P0, zeros(size(P0))]);
            
            % Plotting
            figure;
            show(ur5_RBT,P0,'PreservePlot',false,'Frames','off'); % Extra params to update UR5 position
            hold on
            axis([-1 1 -1 1 -0.1 1.5])
            for i=1:length(tInterval)
                % Interpolate simulated joint positions to get configuration at current time
                configNow = interp1(tTask,stateTask(:,1:6),tInterval(i)); % Interpolate using time, state, joint [1,6], current time
                poseNow = getTransform(ur5_RBT,configNow,endEffector);
                show(ur5_RBT,configNow,'PreservePlot',false,'Frames','off');
                taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
                drawnow; % Draw trajectory iteratively
            end
            %legend(taskSpaceMarker, {'Defined in Task-Space'}); % Add a legend
            title('MoveL()') % Add a title
        end

        function J = jacobian(joint)
            % ATTENTION A WORK IN PROGRESS ATTENTION
            % https://www.mdpi.com/2218-6581/11/6/137#B27-robotics-11-00137
            % jacobian is calculated using the equation 
            % det(J) = 𝑠3𝑠5𝑎2𝑎3(𝑐2𝑎2+𝑐23𝑎3+𝑠234𝑑5)
            % Narrowed down using equation: det(J) = J1 * J2, where:
            % J1 = 𝑠3𝑠5𝑎2𝑎3
            % J2 = (𝑐2𝑎2+𝑐23𝑎3+𝑠234𝑑5)
            J1 = sin(joint(3))*sin(joint(5))*UR5.a(3)*UR5.a(4);
            J2 = cos(joint(2))*UR5.a(3) + cos(joint(2)+joint(3))*UR5.a(4)+sin(joint(2)+joint(3)+joint(4))*UR5.d(5);
            J = J1 * J2;
        end
    end
end