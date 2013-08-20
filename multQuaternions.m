function Qret = multQuaternions(Q1, Q2)
%MULTQUATERNIONS Summary of this function goes here
%   Detailed explanation goes here
% Reference: http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
% page 2
    Qw1 = Q1(1);  
    Qv1 = Q1(2:4);
    Qw2 = Q2(1);  
    Qv2 = Q2(2:4);

    Qretw = Qw1*Qw2-Qv1*Qv2';
    Qretv = Qw1*Qv2+Qw2*Qv1+cross(Qv1,Qv2);
    Qret = [Qretw Qretv];
end

