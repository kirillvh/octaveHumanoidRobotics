function Jac = comJacobian(joints, varargin) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/biped-basics/com-jacobian/
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l272
    useRobot = false;
    AuxCOM = zeros(4,1);
    argl = length(varargin);   
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('comJacobian: Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'Robot')
                R = varargin{ii+1};
                useRobot = true;
            end        
        end
    end
    if(useRobot == true)
        RobotCOM = calcRobotCOM(R);
        ManCOM = calcCOM(joints);
        AuxCOM(4) = RobotCOM(4) - ManCOM(4);
       %AuxCOM(1:3) = (RobotCOM(4)*RobotCOM(1:3)-ManCOM(4)*ManCOM(1:3))/AuxCOM(4);
    end
    DOF = size(joints,1); %the number of joints
    mass = AuxCOM(4);
    for i=1:DOF
        mass = mass+joints(i).mass; 
    end
    Jac = zeros(6,DOF); %Jacobian is 6xN (6 = 3 linear + 3 rotational axes)
    pL = zeros(3,1);
    rotn = eye(3);
    for i=1:DOF
        if(i>1)
            T = eye(4)*jointTransform(joints(i-1));
            rotn = rotn*ROT(T);
            fk = forwardKine(joints, 1, i-1);
            pL = TRANS(fk);
        end
        zn = rotn(:,3);
        pCOM = calcCOM(joints, i);
        %if(useRobot == true)
            % Should we use total Robot mass or AuxCOM(4)+pCOM(4)?
           %pCOM(1:3) = (pCOM(1:3)*pCOM(4)+AuxCOM(1:3)*AuxCOM(4))/(AuxCOM(4)+pCOM(4));
           
        %end
        xn = -pL + pCOM(1:3);
        Jac(:,i) = vertcat((pCOM(4)/mass)*cross(zn,xn),zn);
    end
end