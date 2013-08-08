clear all;

%Create a Manipulator
M = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;

M = addJoint( M,  0 , 0, 0.1 , 0);
M = addJoint( M,  0 , 0, 0.1 , 0);

position_ref = zeros(3,1);

%for plotting data later
positionRef = [];
positionResponse_log = [];

%generate position reference
position_ref = [0.1; 0.1; 0]
	
%there will also be a comIK() later on, so thats why I choose this name
jointAnglesRef = kineIK(M.joints, position_ref);

M.joints(1).angle = jointAnglesRef(1);
M.joints(2).angle = jointAnglesRef(2);

position_response = forwardKine(M.joints)(1:3,4)


disp('position_ref and position_response should be almost the same')
disp('watch out for singularities, the IK algo wont be able to solve if a unreachable pos_ref is given')
	