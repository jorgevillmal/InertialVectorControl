function [xd,b1d] = trajectory(i)
% The "trajectory" MATLA function receives as input the iteration of the
% FOR loop and outputs the x, y, an z reference signals
%
% Inputs:
% - i, iteration of the FOR loop
%
% Outputs:
% - xd, the desired position x, y, and z component \in \mathbb{R}. This is
% expressed in compliance to the ArxiV paper
% - bld, it is a vector \in \mathbb{R}^3. Please, take a look at the
% reference paper.
%

segment = 1/200;
segment2 = 1/300;

if i<=100
    x = 0;
    y = 0;
    z = -0.015;
    b1d = [1;0;0];
end
if i>100 && i<=300
    z = -0.015 - (i-100)*segment;
    x = 0;
    y = 0;
    b1d = [1;0;0];
end
if i>300 && i<=400
    x = 0;
    y = 0;
    z = -1.015;
    b1d = [1;0;0];
end
if i>400 && i<=700
     z = -1.015;
     x = (i-400)*segment2;
     y = -x;
     b1d = [1;0;0];
end
 
if i>700
    z=-1.015;
    x=1;
    y=-1;
    b1d = [1;0;0];
end

xd = [x;y;z];
end

