function [ output_args ] = bipedCOMIK(R, baseIndex, swingIndex, base2COMVec, base2COMQuart, base2SwingVec, base2SwingQuart, varargin) 
%BIPEDCOMIK Summary of this function goes here
%   Detailed explanation goes here
    maxiter = 1000;
    maxposerr = 0.005;
    maxpoterr = 0.0001;
    kTheta = 1;
    kX = 0.05;
    dump = 0.1;
    
    baseLeg_DOF = DOF(R.manipulators(baseIndex).joints);
    swingLeg_DOF = DOF(R.manipulators(swingIndex).joints);
    
    baseLeg_Error = zeros(6,1);
    swingLeg_Error = zeros(6,1);
    
    baseLeg_dTheta = zeros(baseLeg_DOF,1);
    swingLeg_dTheta = zeros(swingLeg_DOF,1);
    
    baseLeg_OErr = zeros(1,3);
    swingLeg_OErr = zeros(1,3);
    
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
    
    base2COMRot = calcRotationMatrix(base2COMQuart);
    swing2COMRot = calcRotationMatrix(swing2COMQuart);
    
    for cntr = 1:maxiter
        
        baseLeg_joints = R.manipulators(baseIndex).joints;
        swingLeg_joints = R.manipulators(swingIndex).joints;

        baseLeg_fk = forwardKine(baseLeg_joints);
        baseLeg_rot = ROT(baseLeg_fk);
        baseLeg_trans = TRANS(baseLeg_fk);
        
        swingLeg_fk = forwardKine(swingLeg_joints);
        swingLeg_rot = ROT(swingLeg_fk);
        swingLeg_trans = TRANS(swingLeg_fk);

        COM = calcRobotCOM(R);        
        COM_x = COM(1:3);
        
        baseLeg_COMjac = comJacobian(baseLeg_joints, 'Robot', R);
        baseLeg_COMjac = comJacChangeFrame(baseLeg_COMjac, baseLeg_joints, COM);   
        
        SwingVec = baseLeg_rot(base2SwingVec-baseLeg_trans);
        SwingRot =  baseLeg_rot*base2SwingR;
        
        baseLeg_trans = baseLeg_rot'*(COM_x-baseLeg_trans);
        baseLeg_rot = baseLeg_rot'*baseLeg_rot;
        
        baseLeg_OCurr = calcQuaternion(baseLeg_rot);
        baseLeg_OCurr(2:4) = -baseLeg_OCurr(2:4);
        baseLeg_OComp = multQuaternions(base2COMQuart,baseLeg_OCurr);
        
        baseLeg_Error(1:3) = base2COMVec - baseLeg_trans;
        baseLeg_Error(4:6) = baseLeg_rot*baseLeg_OComp(2:4)';
        
        swingLeg_jac = kineJacobian(swingLeg_joints);
        
        
        
        
    end
end

    

