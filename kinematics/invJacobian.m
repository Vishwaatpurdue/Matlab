%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Jinv = invJacobian(robot, J)
%   It computes the Jacobian for the robot for a specific angle with
%   respect to end effector coordinate system(HTM).
%   Also Refer jacob0,jacobn 
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

function Jinv = invJacobian(robot, J)
DetJ=det(J);
if DetJ==0
    sprintf('\n It is in singular position \n Inverse of the Jacobian does not exist');
    Jinv='';
elseif size(J,1)==size(J,2)
    Jinv=inv(J);
elseif size(J,1)>size(J,2)
    Jinv=inv(J'*J)*J';
else
    Jinv=J'*inv(J*J');
end
end