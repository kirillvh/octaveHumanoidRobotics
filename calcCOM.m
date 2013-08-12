function COM = calcCOM(joints ) 
%let COM be a 4x1 vector, the first three components are the COM position w.r.t the 
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l145
%http://www.elysium-labs.com/robotics-corner/learn-robotics/biped-basics/center-of-mass/
top = zeros(3,1);
bottom = 0;
T = eye(4,4);
	for i=1:size(joints,1)
         		T = forwardKine(joints, i, size(joints,1));
         		top = top + T(1:3,4)*joints(i).mass;
         		bottom = bottom + joints(i).mass;
	end
	
COM = [top;bottom];
end