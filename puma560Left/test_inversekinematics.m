%Test script to test the inverse kinematics
%
%
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

clear;
clc;
q_prompt='Enter the q values in degrees for finding the Jacobian of PUMA560:\n Like a array [0 90 180 0 0 0]:';
q = [0 0 0 0 0 0];  
fprintf(q_prompt);
%for i = 1:size(q,2)
q=input('');
%end
robot=load_robot('UNIMATE', 'puma560Left');
q = deg2radm(q);
choice=''; % You can give choice as geometric, circ, hfang
[T,Armconf,pos]= fwdkinematics(robot, q);
%Call the inversekinematic for this robot
qinv = inversekinematic_puma560L(robot, T,choice,Armconf);
qop=rad2degm(qinv)
%check that all of them are feasible solutions!
%and every Ti equals T
if size(qinv,2)>=2
for i=1:8,
     Ti = directkinematic(robot, qinv(:,i))
end

for i=1:8
	drawrobot3d(robot, qinv(:,i))
	pause(1);
end
else
    Ti = directkinematic(robot, qinv)
    drawrobot3d(robot, qinv)
end