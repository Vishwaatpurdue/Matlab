%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   J0 = jacob0(robot, q)
%   It computes the Jacobian for the robot for a specific angle with
%   respect to base coordinate system.
%   Also Refer jacobn,invJacobian  
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
function Jn = jacob0(robot, q)

%Initialize J
Jn = [];
for i=1:robot.DOF,
   Ji = jacob0_i(robot, q, i);
   % Add submatrix
   Jn = [Jn Ji];
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Computes the value of the submatrix joint i which contributes to the (either rotational or prismatic) in the
%   angular or linear velocity of the end effector.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ji=jacob0_i(robot, q, i)
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);
T=eye(4);
for j=1:i-1,
    T=T*dh(theta(j), d(j), a(j), alpha(j));    
end
% Obtain z_i-1
zi_1 = T(1:3,3);
% position vector of i-1
pi_1=T(1:3,4);
Tend=directkinematic(robot,q);
%Obtain p vector
p=Tend(1:3,4);
i_1pn=p-pi_1;
% generating a 3X1 zero vector
zero=zeros(3,1);
%If ith joint is rotational
if robot.kind(i) == 'R'
    Ji = [cross(zi_1,i_1pn); zi_1];
%If ith joint is prismatic
else 
    Ji = [zi; zero];
end
end
