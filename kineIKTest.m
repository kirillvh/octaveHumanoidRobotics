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
    
    jointAnglesRef = kineIK(M.joints, position_ref, 'maxiter', 1, 'kX', 0.5, 'maxposerr', 0.001, 'mode', 'transp');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    fk = forwardKine(M.joints);
    position_response = fk(1:3,4);
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
    
    jointAnglesRef = kineIK(M.joints, position_ref, 'maxiter', 20, 'kX', 0.5, 'maxposerr', 0.001, 'mode', 'pinv');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    fk = forwardKine(M.joints);
    position_response = fk(1:3,4);
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
    
    jointAnglesRef = kineIK(M.joints, position_ref, 'maxiter', 20, 'kX', 1, 'maxposerr', 0.001, 'mode', 'dls');
    
    for i=1:DOF
        M.joints(i).angle = jointAnglesRef(i);
    end

    fk = forwardKine(M.joints);
    position_response = fk(1:3,4);
    positionErr = position_ref - position_response;
    positionErr_log = [positionErr_log; positionErr'];
    positionResponse_log = [positionResponse_log; position_response'];
end

figure
plot(positionErr_log);
axis([0 20*DOF -1 1]);
title('TEST CASE 3 - DLS')
%disp('position_ref and position_response should be almost the same')
%disp('watch out for singularities, the IK algo wont be able to solve if a unreachable pos_ref is given')
	