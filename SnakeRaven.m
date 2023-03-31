classdef SnakeRaven
    properties (Access = public)
        design %design structure
        calibration %calibration structure
        Right %Right 1 or left 0 arm
        q %joint vector
        qU %Upper limit joint vector
        qL %Lower limit joint vector
        mj %motor angle vector
        Tend %Endeffector Pose
        W %control weight
        %target_pose %desired Tend
    end
    methods
        %% Initialise SnakeRaven
        function obj = SnakeRaven()
            obj = init_SnakeRaven(obj);
        end
        function obj = init_SnakeRaven(obj)  
            %Initialise SnakeRaven
            %One Module Design
            design1 = struct('alpha',1.24,'n',3,'d',1.62,'w',4,...
            'M',1,'tooltransform',txyz(0,0,5),'qL',0,'qU',0);

            %Two Module Design
            design2 = struct('alpha',[0.2 0.88],'n',[3 3],'d',[1 1],'w',4,...
            'M',2,'tooltransform',txyz(0,0,5),'qL',0,'qU',0);

            %Choose design to simulate (m=2) by default:
            obj.design = design2;
            
            %Raven arm variable right by default
            obj.Right = 1;
            
            %Compute Design Joint Limits:
            %Raven x rotation
            qrxL = -2*pi; qrxU = 2*pi;%radians
            %Raven y rotation
            qryL = -2*pi; qryU = 2*pi;
            %Raven z translation
            qrzL = -300; qrzU = 300; %mm

            %Joint Limits RAVEN level:
            obj.qL = zeros(3+2*obj.design.M,1);
            obj.qU = zeros(3+2*obj.design.M,1);
            
            obj.qL(1:3) = [qrxL; qryL; qrzL];
            obj.qU(1:3) = [qrxU; qryU; qrzU];

            %Segment Pan Tilt Joint Limit calculation:
            for ii = 1:obj.design.M
                %Continuum Joints: Lower Upper
                theta_max = (obj.design.alpha(ii)*obj.design.n(ii))/2; %Maximum bending
                %pan
                qipL = -theta_max;    qipU = theta_max;
                %tilt
                qitL = -theta_max;    qitU = theta_max;
                %Append segment configurations to whole configuration space:
                obj.qL((4+(ii-1)*2):(5+(ii-1)*2),:) = [qipL; qitL];
                obj.qU((4+(ii-1)*2):(5+(ii-1)*2),:) = [qipU; qitU];
            end
            
            if obj.design.M>1
                if obj.Right==1
                    %q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0, deg2rad(-5),deg2rad(0),deg2rad(1),deg2rad(-45)]';
                    %q0 = [ 0, 0, 25, 0.3, 0.2, 0.3, 0.2];
                    q0 = [deg2rad(-39.5967),deg2rad(-77.9160),50, deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0)]';
                else
                    q0 = [deg2rad(39.5967),deg2rad(-102.0840),0, 0,0,0,0]';
                end
                obj.calibration = struct('rate',ones(7,1),'offset',zeros(7,1));
            else
                if obj.Right==1
                    q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0 ,deg2rad(1),deg2rad(-30)]';
                else
                    q0 = [deg2rad(39.5967),deg2rad(-102.0840),0 ,deg2rad(0),deg2rad(0)]';
                end
                obj.calibration = struct('rate',ones(5,1),'offset',zeros(5,1));
            end
                
            obj.q = q0;
            %Defining a Tend pose:
            obj.Tend = FK(obj);
            %Gain
            obj.W = diag([1 1 1 5 5 5]);

            %Define motor angles
            obj.mj = joint2motor(obj.q,obj.design,obj.calibration);
        end
        
       %% Forward Kinematics 
        function T = FK(obj)
            obj.Tend = SnakeRavenFK(obj.Right,obj.design,obj.q);
            T = obj.Tend;
        end
        %% Manipulator Jacobian
        function J = Jacobian(obj)
            J = SnakeRavenJacobian(obj.Tend,obj.Right,obj.design,obj.q);
        end
        %% Joint update
        function obj = update(obj,dq)
            %Joint speed limit
            dq_limit = 0.5;
            if norm(dq)>dq_limit
                dq = cap_mag(dq,dq_limit);
            end
            %Integrate the joint step
            obj.q = obj.q + dq;

            %Ensure joint limits are satisfied and send motor command
            [obj.q,hit] = applyJointLimits(obj.q,obj.qL,obj.qU); 

            %Display error when joint limit is reached:
            if hit==true
                disp('Joint-Limit Saturation')
            end
        end
        %% Inverse Kinematics loop
        function [success,obj] = IK(obj,target_pose)
            %obj.target_pose = target_pose;
            moving = true; count = 1;
            %Controller Speed:
            dx_limit = 1; %mm/iteration
            tol_error = 0.01; %Error magnitude threshold tol_error = 0.1;
            while(moving)
                count = count + 1;
                %Input mj to joint angles
                obj.q = motor2joint(obj.mj,obj.design,obj.calibration);
                %Compute Forward Kinematics
                obj.Tend = FK(obj);
                %Measure the Error between transforms 
                dx = trans2dx(obj.Tend,target_pose);
                Xerror = norm(dx);
                disp('Current Error:')
                disp(Xerror) 
                    %Check Target if within some pose error:
                if Xerror<tol_error
                    moving = false;
                    disp('Target was reached')
                    success = true;
                else
                    %Measure error and apply speed limit:
                    if Xerror>dx_limit
                        dx = cap_mag(dx,dx_limit);
                    end

                    %Calculate Jacobian
                    J = SnakeRavenJacobian(obj.Tend,obj.Right,obj.design,obj.q);

                    %Calculate psuedo-inverse Jacobian avoiding joint limits:        
                    inv_J = (J'*J + eye(length(obj.q))^2)\J'; %damped least squares

                    %Calculate the update step Weighted Damped Least Squares:
                    dq = inv_J*obj.W*dx;

                    %Integrate the joint step
                    obj.q = obj.q + dq;

                    %Ensure joint limits are satisfied and send motor command
                    [obj.q,hit] = applyJointLimits(obj.q,obj.qL,obj.qU); 

                    %Display error when joint limit is reached:
                    if hit==true
                        disp('Joint-Limit Saturation')
                    end

                    %Output: Compute Motor Values for those joint values:
                    obj.mj = joint2motor(obj.q,obj.design,obj.calibration);

                    %if time runs out
                    if count>50
                        disp('Time for simulation expired');
                        moving = false;
                        success = false;
                    end
                end
            end
        end
        %% Update only in Inverse Kinematics
        function obj = IK_update(obj,target_pose)
                %Controller Speed:
                dx_limit = 0.2; %mm/iteration 0.2
                %Input mj to joint angles
                obj.q = motor2joint(obj.mj,obj.design,obj.calibration);
                %Compute Forward Kinematics
                obj.Tend = FK(obj);
                %Measure the Error between transforms 
                dx = trans2dx(obj.Tend,target_pose);
                Xerror = norm(dx);
                disp('Current Error:')
                disp(Xerror)
                if Xerror>dx_limit
                   dx = cap_mag(dx,dx_limit);
                end
                %Calculate Jacobian
                J = SnakeRavenJacobian(obj.Tend,obj.Right,obj.design,obj.q);
                %Calculate psuedo-inverse Jacobian avoiding joint limits:        
                inv_J = (J'*J + eye(length(obj.q))^2)\J'; %damped least squares
                %Calculate the update step Weighted Damped Least Squares:
                dq = inv_J*obj.W*dx;
                %Integrate the joint step
                obj.q = obj.q + dq;
                %Ensure joint limits are satisfied and send motor command
                [obj.q,hit] = applyJointLimits(obj.q,obj.qL,obj.qU); 
                %Display error when joint limit is reached:
                if hit==true
                    disp('Joint-Limit Saturation')
                end
                %Output: Compute Motor Values for those joint values:
                obj.mj = joint2motor(obj.q,obj.design,obj.calibration);
                obj.q = motor2joint(obj.mj,obj.design,obj.calibration);
                obj.Tend = FK(obj);
        end
    end
end