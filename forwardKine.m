function T = forwardKine(joints, varargin) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/joint-state-parameterization-and-forward-kinematics/
switch length(varargin)
    case 0
        initial = 1;
        final = size(joints,1);
    case 1
        initial = varargin{1};
        final = size(joints,1);
    case 2 
        initial = varargin{1};
        final = varargin{2};
    otherwise
        disp('Too Many Arguments, using default case')
        initial = 1;
        final = size(joints,1);
end
T= eye(4,4);
	for i=initial:final
		T = T * jointTransform(joints(i));
	end
end