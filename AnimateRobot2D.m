function  AnimateRobot2D(R, path, varargin)
%ANIMATE2D Summary of this function goes here
%   Detailed explanation goes here
    
    frames = size(path, 1);
    plotCOM = false;
    argl = length(varargin);
    
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'plotCOM')
                plotCOM = varargin{ii+1};
            end        
        end
    end
    
   
    
    nM = size(R.manipulators,1);
    for f = 1:frames
        clf;
        n = 1;
        for j = 1:nM
            X = [0];
            Y = [0];
            DOF = size(R.manipulators(j).joints, 1);
            for i = 1:DOF
                R.manipulators(j).joints(i).angle = path(f,n);
                fk = forwardKine(R.manipulators(j).joints, 1, i);
                EPt = fk(1:2,4);
                X = [X EPt(1)];
                Y = [Y EPt(2)];
                IPt = EPt;
                n = n+1;
            end
            plot(X,Y,'-or',...
                'LineWidth',4,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize', 10);
            axis([-1 1 -1 1]);
            hold on;
        end
        if(plotCOM == true)        
            comPOS = calcRobotCOM(R);
            plot(comPOS(1),comPOS(2),'-or',...
                'LineWidth',4,...
                'MarkerEdgeColor','b',...
                'MarkerFaceColor','b',...
                'MarkerSize', 10);     
        end
        hold off;
        drawnow; 
        pause(0.1);
    end
end


