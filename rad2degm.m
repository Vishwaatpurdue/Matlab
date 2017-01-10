% Transform degrees to radians of a given matrix or a vector

%   Example
%   deg=randi(180,10,10);
%   rad=deg2radm(deg);
%   Also look into rad2degm,rad2deg,deg2rad

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
% along with PURDUE-ART.  If not, see <http://www.gnu.org/licenses/>.function deg = deg2radm(rad)
function deg=rad2degm(rad)
if(ismatrix(rad)==1)
    [row,col]=size(rad);
    deg=zeros(size(rad));
    for r=1:row
        for c=1:col
            deg(r,c)=rad(r,c)*180/pi;
        end
    end
elseif(isvector(rad)==1)
    deg=zeros(size(rad));
    for loop=1:size(rad)
        deg(loop)=rad(loop)*180/pi;
    end
elseif(isscalar(rad)==1)
    deg=rad*180/pi;
else
    sprintf('Input is empty');
    deg='';
end