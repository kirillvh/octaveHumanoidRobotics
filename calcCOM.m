function COM = calcCOM(joints, varargin ) 
%let COM be a 4x1 vector, the first three components are the COM position w.r.t the 
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l145
%http://www.elysium-labs.com/robotics-corner/learn-robotics/biped-basics/center-of-mass/
switch length(varargin)
    case 0
        initial = 1;
        final = size(joints,1);
    case 1
        initial = varargin{1};
        final = size(joints,1);
    case 2 
        initial = varargin{1};
        if(varargin{2} > size(joints,1)) 
            disp('calcCOM() error: final joint is beyond manipulators DOF')
            disp('Using final joint as last joint')
            final = size(joints,1);
        else
            final = varargin{2};
        end
    otherwise
        disp('Too Many Arguments, using default case')
        initial = 1;
        final = size(joints,1);
end
pos = zeros(3,1);
mass = 0;
T = eye(4,4);
	for i=initial:final
        %Changed the FK to be from Base to current link and not from
        %current link to tooltip, still I believe we should implement
        %a real COM position and not as it is at the joint
        
        %Kirill: Indeed, Ok done. 
        %We need to be careful about definition here
        % I am assuming that the joints COM vector points from the tip(not base) of the joint to its COM position
        
        %I think I do not agree, the COM postion need to me transformed to
        %the base Frame anyway so it is not just a sum.
        %I created a comTransform function that will create Tranformation
        %matrix using the COM position and not d and offset.
        %So here we get the Transformation matrix to i-1 and use the
        %comTransform as the last joint.
        
        Ti = eye(4)*comTransform(joints(i));
        if(i>1)
            T = forwardKine(joints, 1, i-1);       
        else
            T = eye(4,4);
        end
        T = T*Ti;
        pos = pos + (T(1:3,4))*joints(i).mass;
        mass = mass + joints(i).mass;
	end
%We should divide the COM position by the total Mass...	
COM = [pos/mass;mass];
end