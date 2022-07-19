% Test Position Control Simulation

%Target point:
Target = [10, 10, 120]';
q1 = 0; q2 = 0; q3 = 0; %mm
q4 = 0; q5 = deg2rad(30); q6 = 0; %radians

%Plot
figure(1); clf;

%Constants
%For joint limit avoidance
Lower_Joint_Limits = [0, 0, 0, -4*pi]';
Upper_Joint_Limits = [61159/1250, 50685/1250, 55135/1250, 4*pi]';
r = 175.73; %mm


%Run loop
success = false;
disp('Starting Snake Robot Simulation')
timer = 0;
speed_limit = 5; %mm/loop

while (success==false)
    
    %Plot robot arm and find forward Kinematics
    [Base_T_tool] = plot_snake_robot(q1,q2,q3,q4,q5,q6,Target);
    
    %Extract position
    X = Base_T_tool(1:3,4);
    
    %Compute Error
    dX = Target - X;
    
    %Logic to stop simulation based on target position
    if (norm(dX) < 1)
        success = true;
        disp('Simulation finished')
    end   
    
    %Control speed in case of big magnitude
    if (norm(dX)>speed_limit)
        dX = speed_limit*(dX/norm(dX));
    end
    
    %Compute Jacobian
    J = SnakeRobotJacobian(q2,q3,q5,r);
    
    %Compute Inverse
    inv_J = dampedLeastSquaresInverse(J,[q1,q2,q3,q5]',Lower_Joint_Limits,Upper_Joint_Limits);
    
    %Compute Update step
    dQ = inv_J*dX;
    
    %Compute new joint targets
    q1 = q1 + dQ(1);
    q2 = q2 + dQ(2);
    q3 = q3 + dQ(3);
    q5 = q5 + dQ(4);
    
    %Enforce Joint Limits in the model
    [q1,q2,q3] = enforceJointLimits(q1,q2,q3,Lower_Joint_Limits,Upper_Joint_Limits);
    
    %Increment timer
    timer = timer + 1;
    
    %Simulation time out
    if (timer>100)
        success = true;
        disp('Failed to converge, Time out')
    end
          
end

function [q1,q2,q3] = enforceJointLimits(q1,q2,q3,Lower_Joint_Limits,Upper_Joint_Limits)
    %Enforces the prismatic tube joint limits
    Q = [q1,q2,q3];
    for ii = 1:3
        qmin = Lower_Joint_Limits(ii);
        qmax = Upper_Joint_Limits(ii);
        q = Q(ii);
        
        if (q<qmin)
            Q(ii) = qmin;
        elseif (q>qmax)
            Q(ii) = qmax;
        end
    end
    
    q1 = Q(1); q2 = Q(2); q3 = Q(3);

end
