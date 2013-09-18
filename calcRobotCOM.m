function COM = calcRobotCOM(R)
%CALCROBOTCOM Summary of this function goes here
%   Detailed explanation goes here
    COM = zeros(4,1);
    nM = size(R.manipulators, 1);
    for i=1:nM
        iCOM = calcCOM(R.manipulators(i).joints);
        COM(1:3) = COM(1:3)*COM(4)+iCOM(1:3)*iCOM(4);
        COM(4) = COM(4)+iCOM(4);
        COM(1:3) = COM(1:3)/COM(4);
    end
end

