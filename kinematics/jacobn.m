%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Jn = jacobn(robot, q)
%   It computes the Jacobian for the robot for a specific angle with
%   respect to end effector coordinate system(HTM).
%   Also Refer jacob0,invJacobian 
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
function Jn = jacobn(robot, q)

%Initialize J
Jn = [];
for i=1:robot.DOF,
   Ji = jacobn_i(robot, q, i);
   % Add submatrix
   Jn = [Jn Ji];
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Computes the value of the submatrix joint i which contributes to the (either rotational or prismatic) in the
%   angular or linear velocity of the end effector.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ji=jacobn_i(robot, q, i)
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);
n=robot.DOF;
T=eye(4);
for j=i:n,
    T=T*dh(theta(j), d(j), a(j), alpha(j));    
end
% Obtain n,s,a,p from the HTM
n=T(1:3,1);
s=T(1:3,2);
a=T(1:3,3);
p=T(1:3,4);
% Obtain z component of n,s,a
z_nsa=[n(3);s(3);a(3)];
% Obtain z component of pXn,pXs,pXa
pCross_nsa=[p(1)*n(2)-p(2)*n(1);
            p(1)*s(2)-p(2)*s(1);
            p(1)*a(2)-p(2)*a(1);];
%If ith joint is rotational
if robot.kind(i) == 'R'
    Ji = [pCross_nsa; z_nsa];
%If ith joint is prismatic
else 
    Ji = [z_nsa; zero];
end
end
