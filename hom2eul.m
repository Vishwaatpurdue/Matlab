%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Converting transformation matrix into euler values
%   rpy=hom2eul(T);
%   
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
function rpy=hom2eul(T)
rpy=zeros(3,1);
if abs(T(1,3)) < eps && abs(T(2,3)) < eps
% singularity
gamma = 0;
beta = atan2(T(1,3), T(3,3));
alpha = atan2(T(2,1), T(2,2));
rpy=[gamma;beta;alpha];
else
    gamma = atan2(T(2,1),T(1,1));
    sp = sin(rpy(1,1));
    cp = cos(rpy(1,1));
    beta = atan2(-T(3,1), cp*T(1,1) + sp*T(2,1));
    alpha = atan2(sp*T(1,3) - cp*T(2,3), cp*T(2,2) - sp*T(1,2));
    rpy=[gamma;beta;alpha];
end
end