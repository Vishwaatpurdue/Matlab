%Test script to test the Jacobian
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
q_prompt='Enter the q values in degrees for finding the Jacobian of PUMA560:';
q = [0 0 0 0 0 0];  
fprintf(q_prompt);
for i = 1:size(q,2)
q(i)=input('');
end
robot=load_robot('UNIMATE', 'puma560Left');
%q = [0 0 0 0 0 0];  
% If you enter values in degrees use the below function to convert to %radians
q = deg2radm(q);
J0=jacob0(robot,q)
Jn=jacobn(robot,q)
Jinv=invJacobian(J0)
Jinv=invJacobian(Jn)
