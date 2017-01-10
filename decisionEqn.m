%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Armconf = decisionEqn(robot, q)
%   It also provides the Arm configuration from the decision equation.
%   The Arm configuration will be like [L/R,A/B,F/NF]
%   Also Refer fwdkinematics, inversekinematics_puma560L  
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

function Armconf = decisionEqn(robot, q)
Armconf=zeros(3,1);

theta=q;
d=eval(robot.DH.d);
a=eval(robot.DH.a);
alpha=eval(robot.DH.alpha);
s23=sin(q(2)+q(3));
c23=cos(q(2)+q(3));
% Obtaining ARM
Armconf(1)=sign(-d(4)*s23-a(3)*c23-a(2)*cos(q(2)));
% Obtaining ELBOW
Armconf(2)=Armconf(1)*sign(d(4)*cos(q(3))-a(3)*sin(q(3)));
T=directkinematic(robot,q);
n=T(1:3,1);
s=T(1:3,2);
T4=eye(4);
for i=1:4
T4=T4*dh(theta(i), d(i), a(i), alpha(i));
z4=T4(1:3,3);
% Obtaining WRIST
if(s'*z4==0)
Armconf(3)=sign(n'*z4);
else
Armconf(3)=sign(s'*z4);
end
end
