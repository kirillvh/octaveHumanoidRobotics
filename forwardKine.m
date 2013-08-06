function T = forwardKine(joints) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/joint-state-parameterization-and-forward-kinematics/
T= eye(4,4);
	for i=1:size(joints,1)
		T = T * jointTransform(joints(i));
	end
end