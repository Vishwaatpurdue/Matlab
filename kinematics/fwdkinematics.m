%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [T,Armconf,pos] = fwdkinematics(robot, q, i)
%   It computes the homogenous transformation matrix for the robot for a specific angle with
%   respect to base coordinate system.
%   It also provides the Arm configuration along with position
%   The Arm configuration will be like [L/R,A/B,F/NF]
%   The position is (x,y,z,r,p,y) the catesian coordinate with euler angles 
%   Also Refer inversekinematics,invJacobian  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2016, by VISHVESWARAN JOTHI
%
% This file is part of PURDUE-ART (A Robotics Toolbox for Education).
% 
% PURDUE-ART is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% PURDUE-ART is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with PURDUE-ART.  If not, see <http://www.gnu.org/licenses/>.

function [T,Armconf,pos] = fwdkinematics(robot, q,i)
d=eval(robot.DH.d);
theta=eval(robot.DH.theta);
a=eval(robot.DH.a);
alpha=eval(robot.DH.alpha);

switch nargin
    case 2
n=length(theta);
T=robot.T0;
for i=1:n 
T=T*dh(theta(i), d(i), a(i), alpha(i)); 
end
%if there is a tool attached to it, consider it in the computation of 
% direct kinematics
 if isfield(robot, 'tool')
     T=T*robot.tool.TCP; %dh(theta(i), d(i), a(i), alpha(i)); 
 end

    case 3
n=i;
T=robot.T0;
for i=1:n 
T=T*dh(theta(i), d(i), a(i), alpha(i)); 
end  
end
% Obtaining Arm configuration
Armconf=decisionEqn(robot,q);
% Obtaining Position and euler angles of the end effector /tool
pos=zeros(6,1);
pos(1:3)=T(1:3,4);
pos(4:6)=hom2eul(T(1:3,1:3));
end