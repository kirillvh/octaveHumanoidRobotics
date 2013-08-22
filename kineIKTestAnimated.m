clear all;
close all;

%Create a Manipulator
M = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;


M = addJoint( M,  0 , 0, 0.2 , 0);
M = addJoint( M,  0 , 0, 0.2 , 0);
M = addJoint( M,  0 , 0, 0.2 , 0);
M = addJoint( M,  0 , 0, 0.2 , 0);
M = addJoint( M,  0 , 0, 0.2 , 0);
M = addJoint( M,  0 , 0, 0.2 , 0);


position_ref = zeros(3,1);
DOF = size(M.joints,1);

%M.joints(1).angle = pi/2;
%M.joints(2).angle = -pi;
%M.joints(3).angle = pi/2;

positionRef = [];
positionResponse_log = [];
positionErr_log = [];
path = [];

for y = 0:0.01:DOF*0.2
    
    position_ref = [y+0.2; y; 0];
    %position_ref = [0; 0.2; 0];
    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    jointAnglesRef = kineIK(M.joints, position_ref,...
                            'maxiter', 100,...
                            'kX', 1,...
                            'maxposerr', 0.0001,...
                            'maxroterr', 0.0001,...
                            'mode', 'dls',...
                            'kTheta', 1,...
                            'orientation', [sqrt(0.5) 0 0 -sqrt(0.5)],...
                            'useJointsAngles', true);
                        
    path = [path ; jointAnglesRef'];
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end
end

Animate2D(M.joints, path, 1);