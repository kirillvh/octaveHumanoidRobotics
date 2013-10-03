function [baseLeg_jointRefs, swingLeg_jointRefs] = bipedCOMIK(R, baseIndex, swingIndex, base2COMVec, base2SwingVec, base2SwingQuart, varargin) 
%BIPEDCOMIK Summary of this function goes here
%   Detailed explanation goes here
    maxiter = 1000;
    maxposerr = 0.005;
    maxroterr = 0.0001;
    kTheta = 1;
    kX = 0.05;
    dump = 0.1;
    
    baseLeg_DOF = DOF(R.manipulators(baseIndex).joints);
    swingLeg_DOF = DOF(R.manipulators(swingIndex).joints);
    
    baseLeg_Error = zeros(6,1);
    swingLeg_Error = zeros(6,1);
    
    baseLeg_jointRefs = ones(baseLeg_DOF,1);
    swingLeg_jointRefs = ones(swingLeg_DOF,1);
    
      
    argl = length(varargin);
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
        end
    end
    
    
    base2SwingRot = calcRotationMatrix(base2SwingQuart);
    
    base2SwingT = eye(4,4);
    base2SwingT(1:3,1:3) = base2SwingRot;
    base2SwingT(1:3,4) = base2SwingVec;
    
    for i=1:baseLeg_DOF
        baseLeg_jointRefs(i,1) = R.manipulators(baseIndex).joints(i).angle;
    end
    
    for i=1:swingLeg_DOF
        swingLeg_jointRefs(i,1) = R.manipulators(swingIndex).joints(i).angle;
    end
    
    for cntr = 1:maxiter
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % GET ROBOT JOINTS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        baseLeg_joints = R.manipulators(baseIndex).joints;
        swingLeg_joints = R.manipulators(swingIndex).joints;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CALCULATE FK AND COM POSITION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        baseLeg_fk = forwardKine(baseLeg_joints);
        baseLeg_rot = ROT(baseLeg_fk);
        baseLeg_trans = TRANS(baseLeg_fk);
        
        swingLeg_fk = forwardKine(swingLeg_joints);
        swingLeg_rot = ROT(swingLeg_fk);
        swingLeg_trans = TRANS(swingLeg_fk);        
        
        COM = calcRobotCOM(R);        
        COM_x = COM(1:3);        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CHANGE BASE2SWING VECTOR TO ROBOT BASE FRAME
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               
        SwingRefFK = baseLeg_fk*base2SwingT;
        SwingRot =  ROT(SwingRefFK);
        SwingVec = TRANS(SwingRefFK);
        SwingQuart = calcQuaternion(SwingRot);
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CHNAGE COM POSITION TO BASE LEG FRAME
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        baseLeg_trans = baseLeg_rot'*(COM_x-baseLeg_trans);
        %baseLeg_rot = baseLeg_rot'*baseLeg_rot;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CALCULATE BASE LEG ERROR
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %baseLeg_OCurr = calcQuaternion(baseLeg_rot);
        %baseLeg_OCurr(2:4) = -baseLeg_OCurr(2:4);
        %baseLeg_OComp = multQuaternions(base2COMQuart,baseLeg_OCurr);
        baseLeg_Error(1:3) = base2COMVec - baseLeg_trans;
        %baseLeg_Error(4:6) = baseLeg_rot*baseLeg_OComp(2:4)';       
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CALCULATE SWING LEG ERROR
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        swingLeg_OCurr = calcQuaternion(swingLeg_rot);
        swingLeg_OCurr(2:4) = -swingLeg_OCurr(2:4);
        swingLeg_OComp = multQuaternions(SwingQuart,swingLeg_OCurr);
        
        swingLeg_Error(1:3) = SwingVec - swingLeg_trans;
        %swingLeg_Error(4:6) = swingLeg_rot*swingLeg_OComp(2:4)';
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CHECK IF ERROR IS WITHIN TOLERANCE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if(max(abs(swingLeg_Error(1:3))) < maxposerr && max(abs(swingLeg_Error(4:6))) < maxroterr && ...
           max(abs(baseLeg_Error(1:3))) < maxposerr && max(abs(baseLeg_Error(4:6))) < maxroterr)
            break;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ERROR GAIN MULTIPLICATION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        swingLeg_Error(1:3) = kX*swingLeg_Error(1:3);
        swingLeg_Error(4:6) = kTheta*swingLeg_Error(4:6);
        
        baseLeg_Error(1:3) = kX*baseLeg_Error(1:3);
        baseLeg_Error(4:6) = kTheta*baseLeg_Error(4:6);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CALCULATE TRANSPOSE JACOBIAN
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        swingLeg_jac = kineJacobian(swingLeg_joints);
        swingLeg_ijac = swingLeg_jac';
        
        baseLeg_COMjac = comJacobian(baseLeg_joints, 'Robot', R);
        baseLeg_COMjac = comJacChangeFrame(baseLeg_COMjac, baseLeg_joints, COM);   
        baseLeg_COMijac = baseLeg_COMjac';
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ERROR CORRECTION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        swingLeg_dTheta = (swingLeg_ijac*inv(swingLeg_jac*swingLeg_ijac+(dump^2)*eye(6)))*swingLeg_Error;
        swingLeg_jointRefs  = swingLeg_jointRefs + swingLeg_dTheta ;
        
        baseLeg_dTheta = (baseLeg_COMijac*inv(baseLeg_COMjac*baseLeg_COMijac+(dump^2)*eye(6)))*baseLeg_Error;
        baseLeg_jointRefs  = baseLeg_jointRefs + baseLeg_dTheta ;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % UPDATE ANGLES
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        for i=1:baseLeg_DOF
             R.manipulators(baseIndex).joints(i).angle = baseLeg_jointRefs(i,1);
        end

        for i=1:swingLeg_DOF
            R.manipulators(swingIndex).joints(i).angle = swingLeg_jointRefs(i,1);
        end
                
    end
    cntr
end

    

