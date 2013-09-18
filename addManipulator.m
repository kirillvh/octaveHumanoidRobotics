function Rout = addManipulator(Rin, M)
%ADDMANIPULATOR Summary of this function goes here
%   Detailed explanation goes here
    Rin.manipulators = [Rin.manipulators; M];
    Rout = Rin;
end

