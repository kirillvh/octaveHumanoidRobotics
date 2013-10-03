function [joints_ref_Left, joints_ref_Right] = IKHumanoid(R, L2CRef, L2RRef, varargin) 
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
    
    jointsLeft = R.manipulators(1).joints;
    jointsRight = R.manipulators(2).joints;
    
    %local variables
    DOF = size(jointsLeft,1);
    argl = length(varargin);
    error = zeros(12,1);
    joints_ref_Left = ones(DOF,1);
    joints_ref_Right = ones(DOF,1);
    
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
        end
    end
    
    %Load initial joints reference as their current angle (useful for small
    %movements...)
    if useJointsAngles == true
        for i=1:DOF
            joints_ref_Left(i,1) = jointsLeft(i).angle;
            joints_ref_Right(i,1) = jointsRight(i).angle;
        end
    end
    
    for cntr = 1:maxiter
        % Feed Joints Angles
        for i=1:DOF
            jointsLeft(i).angle = joints_ref_Left(i,1);
            jointsRight(i).angle = joints_ref_Right(i,1);
            R.manipulators(1).joints = jointsLeft;
            R.manipulators(2).joints = jointsRight;
        end
        % Get TransForm Matrix
        fk = forwardKine(jointsLeft);
        % Get Translation
                xCom = calcRobotCOM(R);
            x = xCom(1:3);
        % Calc Jacobian

                jac = comJacobian(jointsLeft, 'Robot', R);

                jac = comJacChangeFrame(jac, jointsLeft,xCom);
                rot = ROT(fk);
                trans = TRANS(fk);
                x = rot'*(x-trans);

                ijac = jac';
                
                
%now for the swing leg part
%error(1:6) = 0;%reset the error vector so it can be used again
fk_Right = forwardKine(jointsRight);
rot_b2l =  rot';%transform from base to left leg
rot_6x6 = zeros(6,6);
rot_6x6(1:3,1:3) = rot_b2l;
rot_6x6(4:6,4:6) = rot_b2l;
jac_right =rot_6x6*kineJacobian(jointsRight);%transform the right legs kinematic jacobian from base to left leg
ijac_right = jac_right';


        % Calc the error for the left leg by measuring the position error between the left foot and the COM(while in the left foot's frame)
        error(1:3) = (L2CRef - x);
        % Calc the orientation error for the left leg, this concerns the orientation between the left foot and the base link
        if useOrientation == true
            rot = ROT(fk)';
            OCurr = calcQuaternion(rot);
            OCurr(2:4) = -OCurr(2:4);
            OComp = multQuaternions(ORef,OCurr);
            error(4:6) = rot*OComp(2:4)';
        end      
        
        % Calc the error for the right leg by measuring the 
        error(7:9) = L2RRef ... %this is the desired L2R position
         - rot_b2l*(-TRANS(fk) +TRANS(fk_Right)); %this is the actual L2R position as seen from the left leg
        % Calc the error for the right leg, this concerns the orientation between the left foot and the right foot
        if useOrientation == true
            rot_r2b = ROT(fkRight);
            OCurr = calcQuaternion(rot_b2l*rot_r2b' ); % rot_b2l*rot_r2b' should give the rotation matrix from the left to the right leg, not sure about it, need to test
            OCurr(2:4) = -OCurr(2:4);
            OComp = multQuaternions(ORef,OCurr);
            error(10:12) = rot_r2b *OComp(2:4)';
        end  
        

        %error_log = [error_log; error];
        if(max(abs(error(1:3))) < maxposerr && max(abs(error(4:6))) < maxroterr && ...
           max(abs(error(7:9))) < maxposerr && max(abs(error(10:12))) < maxroterr )
            break;
        end
        error(1:3) = kX*error(1:3);
        error(4:6) = kTheta*error(4:6);
        error(7:9) = kX*error(7:9);
        error(10:12) = kTheta*error(10:12);
        
% Calc new joint angles for left leg
            dtheta = (ijac*inv(jac*ijac+(dump^2)*eye(6)))*error(1:6);
            joints_ref_Left  = joints_ref_Left + dtheta;
            
% Calc new joint angles for right leg
            dtheta = (ijac_right*inv(jac_right*ijac_right+(dump^2)*eye(6)))*error(7:12);
            joints_ref_Right  = joints_ref_Right + dtheta;
           
        % Lets just remove the accumulated angle in excess
        joints_ref_Left = mod(joints_ref_Left, 2*pi);
        joints_ref_Right = mod(joints_ref_Right, 2*pi);
    end
    cntr
    %plot(error_log);
end