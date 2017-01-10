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
function rad=deg2radm(deg)
if(ismatrix(deg)==1)
    [row,col]=size(deg);
    rad=zeros(size(deg));
    for r=1:row
        for c=1:col
            rad(r,c)=deg(r,c)/180*pi;
        end
    end
elseif(isvector(deg)==1)
    rad=zeros(size(deg));
    for loop=1:size(deg)
        rad(loop)=deg(loop)/180*pi;
    end
elseif(isscalar(deg)==1)
    rad=deg/180*pi;
else
    sprintf('Input is empty');
    rad='';
end