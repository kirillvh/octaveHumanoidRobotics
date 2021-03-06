clear all;
close all;

%Create a Manipulator
M = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;


M = addJoint( M,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);
M = addJoint( M,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);
M = addJoint( M,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);
M = addJoint( M,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);

position_ref = zeros(3,1);

%generate position reference
DOF = size(M.joints,1);
%Lets iter from some range of positions

disp('--------------------------------------------------------------------')
disp('---------------TEST CASE 1 - JACOBIAN TRANSPOSE---------------------')
disp('--------------------------------------------------------------------')

%for plotting data later
positionRef = [];
positionResponse_log = [];
positionErr_log = [];

for y = 0:0.01:DOF*0.2
    
    position_ref = [0; y; 0];

    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    jointAnglesRef = IK(M.joints, position_ref,...
                            'maxiter', 100,...
                            'kX', 0.5,...
                            'maxposerr', 0.001,...
                            'mode', 'transp',...
                            'kTheta', 1,...
                            'orientation', [1 0 0 0],...
                            'ikMode','com');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    com = calcCOM(M.joints);
    position_response = com(1:3);
    positionErr = position_ref - position_response;
    positionErr_log = [positionErr_log; positionErr'];
    positionResponse_log = [positionResponse_log; position_response'];
end

plot(positionErr_log);
axis([0 20*DOF -1 1]);
title('TEST CASE 1 - JACOBIAN TRANSPOSE')

disp('--------------------------------------------------------------------')
disp('--------------TEST CASE 2 - JACOBIAN PSEUDO INVERSE-----------------')
disp('--------------------------------------------------------------------')

positionRef = [];
positionResponse_log = [];
positionErr_log = [];


for y = 0:0.01:DOF*0.2
    
    position_ref = [0; y; 0];

    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    jointAnglesRef = IK(M.joints, position_ref,...
                            'maxiter', 100,...
                            'kX', 0.5,...
                            'maxposerr', 0.001,...
                            'mode', 'pinv',...
                            'kTheta', 1,...
                            'orientation', [1 0 0 0],...
                            'ikMode','com');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    com = calcCOM(M.joints);
    position_response = com(1:3);
    positionErr = position_ref - position_response;
    positionErr_log = [positionErr_log; positionErr'];
    positionResponse_log = [positionResponse_log; position_response'];
end

figure
plot(positionErr_log);
axis([0 20*DOF -1 1]);
title('TEST CASE 2 - JACOBIAN PSEUDO INVERSE')

disp('--------------------------------------------------------------------')
disp('-------------------------TEST CASE 3 - DLS--------------------------')
disp('--------------------------------------------------------------------')

positionRef = [];
positionResponse_log = [];
positionErr_log = [];


for y = 0:0.01:DOF*0.2
    
    position_ref = [0; y; 0];

    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    jointAnglesRef = IK(M.joints, position_ref,...
                            'maxiter', 100,...
                            'kX', 1,...
                            'maxposerr', 0.001,...
                            'mode', 'dls',...
                            'kTheta', 1,...
                            'orientation', [1 0 0 0],...
                            'ikMode', 'kine',...
                            'ikMode','com');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    com = calcCOM(M.joints);
    position_response = com(1:3);
    positionErr = position_ref - position_response;
    positionErr_log = [positionErr_log; positionErr'];
    positionResponse_log = [positionResponse_log; position_response'];
end

figure
plot(positionErr_log);
axis([0 20*DOF -1 1]);
title('TEST CASE 3 - DLS')
	