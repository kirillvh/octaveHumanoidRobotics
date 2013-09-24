function joints_ref = IK(joints, pos_ref, varargin) 
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l899
%http://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
% to think about: http://mi.ams.eng.osaka-u.ac.jp/pub/2011/tro2011sugihara.pdf
    %default values
    maxiter = 1000;
    maxposerr = 0.005;
    maxroterr = 0.0001;
    kTheta = 1;
    kX = 0.05;
    mode = 0;
    dump = 0.1;
    ikMode=0; % 0 = KinematicMode, 1 = COMMode
    KinematicMode=0;
    COMMode=1;
    WorldFrameMode = 0;
    TooltipFrameMode = 1;
    frameMode = WorldFrameMode;
    
    useRobot = false;
    
    %local variables
    DOF = size(joints,1);
    argl = length(varargin);
    error = zeros(6,1);
    joints_ref = ones(DOF,1);
    dtheta = zeros(DOF,1);
    useOrientation = false;
    OErr = zeros(1,3);
    error_log = [];
    useJointsAngles = false;
    % Feed optional configurations
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'maxiter')
                maxiter = varargin{ii+1};
            end
            if strcmp(varargin{ii},'kX')
                kX = varargin{ii+1};
            end
            if strcmp(varargin{ii},'kTheta')
                kTheta = varargin{ii+1};
            end
            if strcmp(varargin{ii},'maxposerr')
                maxposerr = varargin{ii+1};
            end
            if strcmp(varargin{ii},'maxroterr')
                maxroterr = varargin{ii+1};
            end
            if strcmp(varargin{ii},'dump')
                dump = varargin{ii+1};
            end
            if strcmp(varargin{ii},'useJointsAngles')
                useJointsAngles = varargin{ii+1};
            end
            if strcmp(varargin{ii},'orientation')
                ORef = varargin{ii+1};
                useOrientation = true;
            end
            if strcmp(varargin{ii},'mode')
                if(strcmp(varargin{ii+1},'pinv'))
                    mode = 0;
                end
                if(strcmp(varargin{ii+1},'transp'))
                    mode = 1;
                end
                if(strcmp(varargin{ii+1},'dls'))
                    mode = 2;
                end
            end
            if strcmp(varargin{ii},'ikMode')
                if(strcmp(varargin{ii+1},'kine'))
                    ikMode = KinematicMode;
                end
                if(strcmp(varargin{ii+1},'com'))
                    ikMode = COMMode;
                end
            end
            if strcmp(varargin{ii},'frameMode')
                if(strcmp(varargin{ii+1},'world'))
                    frameMode = WorldFrameMode;
                end
                if(strcmp(varargin{ii+1},'tooltip'))
                    frameMode = TooltipFrameMode;
                end
            end
            if strcmp(varargin{ii},'Robot')
                R = varargin{ii+1};
                useRobot = true;
            end
        end
    end
    
    %Load initial joints reference as their current angle (useful for small
    %movements...)
    if useJointsAngles == true
        for i=1:DOF
            joints_ref(i,1) = joints(i).angle;
        end
    end
    
    for cntr = 1:maxiter
        % Feed Joints Angles
        for i=1:DOF
            joints(i).angle = joints_ref(i,1);
        end
        % Get TransForm Matrix
        fk = forwardKine(joints);
        % Get Translation
        if ikMode == KinematicMode
            x = fk(1:3,4);
        elseif ikMode == COMMode
            if(useRobot == false)
                xCom= calcCOM(joints);
            else
                xCom = calcRobotCOM(R);
            end
            x = xCom(1:3);
        end
        % Calc Jacobian
        if ikMode == KinematicMode
            jac = kineJacobian(joints);
        elseif ikMode == COMMode
            if(useRobot == false)
                jac = comJacobian(joints);
            else
                jac = comJacobian(joints, 'Robot', R);
            end
            if(frameMode == TooltipFrameMode)
                jac = comJacChangeFrame(jac, joints,xCom);
                rot = ROT(fk);
                trans = TRANS(fk);
                x = rot'*(x-trans);
            end
        end
        % Calc PseudoInverse
        switch mode
            case 0
                %use Pseudo Inverse
                ijac = pinv(jac);
            case 1
                %use Tanspose (need to implement optimal alpha)
                ijac = jac';
            case 2
                %use Tanspose (need to implement optimal alpha)
                ijac = jac';
            otherwise
                ijac = pinv(jac);
        end
        % Calc the error
        error(1:3) = (pos_ref - x);
        if useOrientation == true
            rot = ROT(fk);
            OCurr = calcQuaternion(rot);
            OCurr(2:4) = -OCurr(2:4);
            OComp = multQuaternions(ORef,OCurr);
            error(4:6) = rot*OComp(2:4)';
        end      
        %error_log = [error_log; error];
        if(max(abs(error(1:3))) < maxposerr && max(abs(error(4:6))) < maxroterr)
            break;
        end
        error(1:3) = kX*error(1:3);
        error(4:6) = kTheta*error(4:6);
        % Calc new joint angles
        if mode == 2
            dtheta = (ijac*inv(jac*ijac+(dump^2)*eye(6)))*error;
            joints_ref  = joints_ref + dtheta;
        else
            joints_ref  = joints_ref +ijac*error;
        end
        
        % Lets just remove the accumulated angle in excess
        joints_ref = mod(joints_ref, 2*pi);
    end
    cntr
    %plot(error_log);
end