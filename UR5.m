classdef UR5
    properties (Constant)
        % Modified Denavit-Hertenberg parameters (DH)
        alpha = pi/180 * [0 90 0 0 -90 90]; % radians
        a = [0 0 425 392 0 0]; % mm
        theta = pi/180 * [0 180 0 0 0 180]; % radians
        d = [89.2 0 0 109.3 94.75 82.5]; % mm

        % Predefined transform matrices of the UR5
        TB0 = [eye(4,3) [0;0;89.2;1]]; % Transform matrix from base to joint 0 % d(1) = 89.2
        T6W = diag([-1 -1 1 1]); % Diagonal 4x4-matrix using eigenvalues [-1, -1, 1, 1]
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
                        0                                   0                                   0                   1];
                TBW = TBW * temp; % Multiplying prior T with i'th transform matrix.
            end
            T06 = UR5.TB0 \ TBW / UR5.T6W; % T06 is calculated using the equation: T06 = TB0^(-1) * TBW * T6W^(-1)
        end

        function Joint = inverseKinematics(T06)
            % Solutions are provided as "Joint values" = Joint(solution, joint)
            Joint = zeros(8,6);
            for i = 1:8 % For all 8 solutions
                %theta1: Has 2 possible solutions
                P05 = T06*[0; 0; -UR5.d(6); 1]; % Distance between P5 and P6 is d(6)
                phi1 = atan2(P05(2), P05(1));
                phi2 = [acos(UR5.d(4)/sqrt(P05(1)^2+P05(2)^2)) -acos(UR5.d(4)/sqrt(P05(1)^2+P05(2)^2))]; % a 1x2 vector containing each possible solution of theta1...
                Joint(i,1) = phi1 + phi2(1+mod(round((i-1)/2),2)) + pi/2; % theta1 % ... solution is chosen using equation 1+mod(round((i-1)/2),2)

                %theta5: Has 2 possible solutions (depending on theta1)
                T01 = UR5.TB0 \ UR5.forwardKinematics(Joint(i,:), 1, 1); % T01 = TB0^(-1) * TB1, essentially d(1) is disregarded, as it is nullified using TB0
                T16 = T01 \ T06 ; % T16 = T01^(-1) * T06
                phi1 = [-acos((-T16(2,4)-UR5.d(4))/UR5.d(6)) acos((-T16(2 ,4)-UR5.d(4))/UR5.d(6))]; % a 1x2 vector containing each possible solution of theta5...
                Joint(i,5) = phi1(1+mod(round((i-1)/4),2)); % theta5 % ... solution is chosen using equation 1+mod(round((i-1)/4),2)

                %theta6: Has 4 possible solution (depending on theta1)
                T60 = inv ( T06 ) ;
                Y61 = -sin(Joint(i,1)) * [ T60(1 ,1) ; T60(2 ,1) ; T60(3 ,1) ] + cos(Joint(i,1)) * [ T60(1 ,2) ;T60(2 ,2) ; T60(3 ,2) ];
                Y61x = Y61 (1 ,1) ;
                Y61y = Y61 (2 ,1) ;
                Joint(i,6) = atan2 ( ( Y61y / sin ( Joint(i,5)) ) , -Y61x / sin ( Joint(i,5)) ) ; % theta6 %  - pi

                % Halfway there - Calculating theta2 , theta3 & theta4
                T45 = UR5.forwardKinematics(Joint(i,:), 5, 5);
                T56 = UR5.forwardKinematics(Joint(i,:), 6, 6); % Replace P05 by calculating T56
                T14 = T16 / ( T45 * T56 );

                % theta3: Has 8 possible solutions (depending on theta1 and theta6)
                T14xz = norm([T14(1,4) T14(3,4)]); % Length of T14x and T14z
                phi1 = [acos((T14xz^2 - UR5.a(3)^2 - UR5.a(4)^2)/(2 * UR5.a(3) * UR5.a(4))) -acos((T14xz^2 - UR5.a(3)^2 - UR5.a(4)^2)/(2 * UR5.a(3) * UR5.a(4)))];
                Joint(i,3) = phi1(1+mod(round((i-1)/8),2)); % theta3 % ... solution is chosen using equation 1+mod(round((i-1)/4),2)

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
            
            % Betragt dette som et lille plaster på invers kinematik metoden.
            for i = 1:length(Joint)
                Joint(i,6) = Joint(i,6) + 180;
            end

            Joint = unique(Joint,'rows'); % Remove duplicates

            for i = length(Joint):-1:1 % Countdown to remove 'NaN' rows
                if isnan(Joint(i))
                    Joint(i,:) = [];
                end
            end      
        end
        
        function moveJ(P0, Pf)
            RDK = Robolink; % Generate a Robolink object RDK. This object interfaces with RoboDK.
            robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT); % Select robot
            if robot.Valid() == 0
            error('No robot selected'); % Missing robot
            end
            ref = robot.Parent();

            P0 = 180/pi * P0; % Convert to degrees
            Pf = 180/pi * Pf; % Convert to degrees
            P0dot = [0 0 0 0 0 0]; % Assuming no start velocity
            Pfdot = [0 0 0 0 0 0]; % Assuming no end velocity

            tf = max(abs(sqrt((4*(Pf - P0))/50))); % function time for moveJ()
            disp("Execution time of moveJ(): " + tf);
            t=0:0.002:tf; % Control frequency: 500 hz
            %t=0:0.05:tf; % Control frequency: 20 hz
            
            % Cubic polynomal parameters
            a0 = P0;
            a1 = P0dot;
            a2 = 3/(tf.^2)*(Pf-P0)-2/tf*P0dot-1/tf*Pfdot;
            a3 = -2/(tf.^3)*(Pf-P0)+1/(tf.^2)*(Pfdot+P0dot);
            
            progJoint = RDK.AddProgram('MoveJ Cubic');
            progJoint.addMoveJ(RDK.Item('start')); %MoveJ (start)
            
            % Calculate frames in 6x1 vector [x y z gamma beta alpha]
            for i=2:(length(t)-1)
                P=a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3; % Aquire position
                robot.setJoints(P); % via. point in RoboDK
                targetname = sprintf('TargetJ%i',i);
                target = RDK.AddTarget(targetname,ref,robot);
                progJoint.addMoveJ(target);
            end
            progJoint.addMoveJ(RDK.Item('end')); %MoveJ (end)
        end

        function moveL(P0, Pf, robot)
            TBS = UR5.forwardKinematics(P0, 1, 6); % Use only TBW of P0
            TBE = UR5.forwardKinematics(Pf, 1, 6); % Use only TBW of Pf

            startLoc = Pose_2_XYZRPW(TBS)'; % Acquire XYZRPW of TBS
            endLoc = Pose_2_XYZRPW(TBE)'; % Acquire XYZRPW of TBE
            tf = max(abs(sqrt((4*norm(endLoc - startLoc))/50))); % function time for moveJ()
            t=0:0.05:tf; % 20fps  
            disp("Execution time of moveL(): " + tf);
            
            Tse = TBS \ TBE; %Compute Tse (Angle-axis representation)
            theta = acos((Tse(1,1)+Tse(2,2)+Tse(3,3)-1)/2); %Find axis angle
            Ps = [0 0 0 0]; % Start frame position (origo)
            Pe = [Tse(1,4) Tse(2,4) Tse(3,4) theta]; % end postion relative to start frame
            Psdot = [0 0 0 0]; % Assuming no via points from "start" to "end"
            Pedot = [0 0 0 0]; % Assuming no via points from "start" to "end"
            k=1/(2*sin(theta)) * [Tse(3,2)-Tse(2,3);Tse(1,3)-Tse(3,1);Tse(2,1)-Tse(1,2)];

            % Cubic polynomal parameters
            a0 = Ps;
            a1 = Psdot;
            a2 = 3/(tf.^2)*(Pe-Ps)-2/tf*Psdot-1/tf*Pedot;
            a3 = -2/(tf.^3)*(Pe-Ps)+1/(tf.^2)*(Pedot+Psdot);

            % Calculate frames in 4x1 vector [x y z theta]
            for i=2:(length(t)-1)
                P=a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3; % Aquire position
                theta=P(4); % aquire theta from P
                TSI= [k(1)*k(1)*(1-cos(theta))+cos(theta)     k(1)*k(2)*(1-cos(theta))-k(3)*sin(theta) k(1)*k(3)*(1-cos(theta))+k(2)*sin(theta) P(1);
                     k(1)*k(2)*(1-cos(theta))+k(3)*sin(theta)  k(2)*k(2)*(1-cos(theta))+cos(theta)    k(2)*k(3)*(1-cos(theta))-k(1)*sin(theta) P(2) ;
                    k(1)*k(3)*(1-cos(theta))-k(2)*sin(theta)  k(3)*k(2)*(1-cos(theta))+k(1)*sin(theta) k(3)*k(3)*(1-cos(theta))+cos(theta) P(3);
                    0 0 0 1;]; % Convert [x y z theta] to 4x4 transform matrix
                TBI=TBS*TSI; 
                robot.setPose(TBI); % via. point in RoboDK
            end
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
