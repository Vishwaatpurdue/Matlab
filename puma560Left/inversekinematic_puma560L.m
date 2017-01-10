%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_PUMA560L(robot, T,choice,decval)	
%   Solves the inverse kinematic problem for the UNIMATE PUMA 560 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_PUMA560 returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%   If decision vector is provided it will give the one solution that fits
%   for the decision equation.
%   The equation are used from Lee's Book.
%   
%   Example code:
%
%   robot=load_robot('UNIMATE', 'puma560Left');
%   q = [0 0 0 0 0 0];	
%   choice='';
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T, choice);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:8,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%	See also DIRECTKINEMATIC.
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
function q = inversekinematic_puma560L(robot, T,choice,Armconf)
switch nargin
    case 2
        choice='geometric';
        % Initializing the Decision parameters
        ARM=[1,-1];
        EL=[1,-1];
        WRIST=[1,-1];
        FL=[0,1];
        K=ARM'*EL;
        decval=[-1 -1 -1 -1  1  1  1  1
                -1 -1  1  1 -1 -1  1  1
                -1  1 -1  1 -1  1 -1  1
                 0  0  0  0  0  0  0  0];
    case 3
        % Initializing the Decision parameters
        ARM=[1,-1];
        EL=[1,-1];
        WRIST=[1,-1];
        FL=[0,1];
        K=ARM'*EL;
        decval=[-1 -1 -1 -1  1  1  1  1
                -1 -1  1  1 -1 -1  1  1
                -1  1 -1  1 -1  1 -1  1
                 0  0  0  0  0  0  0  0];

    case 4
        % Initializing the Decision parameters
        decval=Armconf;
        ARM=Armconf(1);
        EL=Armconf(2);
        WRIST=Armconf(3);
        FL=0;
        K=ARM*EL;
end
%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);
%See geometry at the reference for this robot
L6=d(6);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
% Here T does not have tool in it.
T=inv(robot.T0)*T;
P=T(:,4)-L6*T(:,3);
Px=P(1);
Py=P(2);
Pz=P(3);


% Solving for q using geometric approach
if (strcmp(choice,'geometric')==0)
    if size(decval,2)>=2
    % Solve for theta1
    % Note: In these equations theta is -180 to +180 deg
    % Eqn 3.127
    
    theta11=atan2(-ARM(1)*Py*sqrt(Px^2+Py^2-d(2)^2)-Px*d(2),-ARM(1)*Px*sqrt(Px^2+Py^2-d(2)^2)+Py*d(2));
    theta12=atan2(-ARM(2)*Py*sqrt(Px^2+Py^2-d(2)^2)-Px*d(2),-ARM(2)*Px*sqrt(Px^2+Py^2-d(2)^2)+Py*d(2));
    
    % Solve for theta2
    % Note: In these equations theta is -180 to +180 deg
    R=sqrt(Px^2+Py^2+Pz^2-d(2)^2); % Eqn-3.129
    r=sqrt(Px^2+Py^2-d(2)^2); % Eqn-3.129
    theta21=SolveTheta2(R,r,K(1,1),ARM(1),d,a,Pz);
    theta22=SolveTheta2(R,r,K(1,2),ARM(1),d,a,Pz);
    theta23=SolveTheta2(R,r,K(2,1),ARM(2),d,a,Pz);
    theta24=SolveTheta2(R,r,K(2,2),ARM(2),d,a,Pz);
    
    % Solve for theta3
    % Note: In these equations theta is -180 to +180 deg
    theta31=SolveTheta3(R,K(1,1),d,a);
    theta32=SolveTheta3(R,K(1,2),d,a);
    theta33=theta32;
    theta34=theta31;
    q=[theta11 theta11 theta11 theta11 theta12 theta12 theta12 theta12;
       theta21 theta21 theta22 theta22 theta23 theta23 theta24 theta24;
       theta31 theta31 theta32 theta32 theta33 theta33 theta34 theta34;
       0        0       0       0       0       0       0       0;
       0        0       0       0       0       0       0       0;
       0        0       0       0       0       0       0       0];
    %Solve for theta 4 to theta6
    % Note: In these equations theta is -180 to +180 deg
    for i=1:2:8
    theta426_1=SolveTheta426(q(:,i),WRIST(1),T,robot,FL(1));
    theta426_2=SolveTheta426(q(:,i),WRIST(2),T,robot,FL(1));
    q(4:6,i)=theta426_1;
    q(4:6,i+1)=theta426_2;
    end
    else
        theta1=atan2(-ARM*Py*sqrt(Px^2+Py^2-d(2)^2)-Px*d(2),-ARM*Px*sqrt(Px^2+Py^2-d(2)^2)+Py*d(2));
        % Solve for theta2
        % Note: In these equations theta is -180 to +180 deg
        R=sqrt(Px^2+Py^2+Pz^2-d(2)^2); % Eqn-3.129
        r=sqrt(Px^2+Py^2-d(2)^2); % Eqn-3.129
        theta2=SolveTheta2(R,r,K,ARM,d,a,Pz);
        % Solve for theta3
        % Note: In these equations theta is -180 to +180 deg
        theta3=SolveTheta3(R,K,d,a);
        q=[theta1;theta2;theta3;0;0;0];
        theta426=SolveTheta426(q,WRIST,T,robot,0);
        q(4:6)=theta426;
    end
elseif (strcmp(choice,'circ')==0)
    if size(decval,2)>=2
    for i=1:8
        q(:,i)=solve_circ(d,a,P,T,decval(:,i));
    end
    else
        q=solve_circ(d,a,P,T,decval);
    end
elseif (strcmp(choice,'hfang')==0)
    if size(decval,2)>=2
    for i=1:8
        q(:,i)=solve_hfang(d,a,P,T,decval(:,i));
    end
    else
        q=solve_hfang(d,a,P,T,decval);
    end
else
    sprintf('Enter the choice properly(geometric,circ,hfang) \n');
end
end
function theta2=SolveTheta2(R,r,K,ARM,d,a,Pz)
Salpha=-Pz/R; % Eqn 3.130
Calpha=-ARM*r/R; % Eqn 3.131
Cbeta=(a(2)^2+R^2-d(4)^2-a(3)^2)/(2*a(2)*R); % Eqn 3.132
Sbeta=sqrt(1-Cbeta^2); % Eqn 3.133
Stheta2=Salpha*Cbeta+K*Calpha*Sbeta; % Eqn 3.134
Ctheta2=Calpha*Cbeta-K*Salpha*Sbeta; % Eqn 3.135
theta2=atan2(Stheta2,Ctheta2); % Eqn 3.136
end
function theta3=SolveTheta3(R,K,d,a)
Cphi=(a(2)^2+d(4)^2+a(3)^2-R^2)/(2*a(2)*sqrt(d(4)^2+a(3)^2)); % Eqn 3.138
Sphi=K*sqrt(1-Cphi^2); % Eqn 3.139
Sbeta=d(4)/sqrt(d(4)^2+a(3)^2); % Eqn 3.140
Cbeta=abs(a(3))/sqrt(d(4)^2+a(3)^2); % Eqn 3.140
Stheta3=Sphi*Cbeta-Cphi*Sbeta; % Eqn 3.142

Ctheta3=Cphi*Cbeta+Sphi*Sbeta; % Eqn 3.143

theta3=atan2(Stheta3,Ctheta3); % Eqn 3.144
end
function theta426=SolveTheta426(q,WRIST,T,robot,FL)
theta426=zeros(3,1);
n=T(1:3,1);
s=T(1:3,2);
a=T(1:3,3);
T3=dh(robot, q, 1)*dh(robot, q, 2)*dh(robot, q, 3);
% T3=fwdkinematic(robot, q(1:3));
% x3=T3(1:3,1);
% y3=T3(1:3,2);
z3=T3(1:3,3);
z4=cross(z3,a)/norm(cross(z3,a));
z41=cross(z3,a)/sqrt(sum(cross(z3,a).^2));
if(z4==0)
    %omega=0; % Denegerate case
    M=WRIST;
elseif(s'*z4~=0)
    omega=s'*z4;
    M=WRIST*sign(omega);
else
    omega=n'*z4;
    M=WRIST*sign(omega);
end
% theta4
% S4=-M*(z4'*x3);
% C4=M*(z4'*y3);
% theta426(1)=atan2(S4,C4);

% Eqn 3.151
theta426(1)=atan2(M*(cos(q(1))*a(2)-sin(q(1))*a(1)),M*(cos(q(1))*cos(q(2)+q(3))*a(1)+sin(q(1))*cos(q(2)+q(3))*a(2)-sin(q(2)+q(3))*a(3)));
q(4)=theta426(1);

% Solving for theta5
% Eqn 3.153
T4=T3*dh(robot, q, 4);
%S5=a'*T4(1:3,1);
%C5=-(a'*T4(1:3,2));
%num=S5;den=C5;
num=(cos(q(1))*cos(q(2)+q(3))*cos(q(4))-sin(q(1))*sin(q(4)))*a(1)+(sin(q(1))*cos(q(2)+q(3))*cos(q(4))+cos(q(1))*sin(q(4)))*a(2)-sin(q(2)+q(3))*cos(q(4))*a(3);
den=cos(q(1))*sin(q(2)+q(3))*a(1)+sin(q(1))*sin(q(2)+q(3))*a(2)+cos(q(2)+q(3))*a(3);
theta426(2)=atan2(num,den);
% Display case is degenerate if theta5=0
q(5)=theta426(2);

if q(5)==0
sprintf('Denegerate Case');
end
T5=T4*dh(robot, q, 5);
y5=T5(1:3,2);
S6=n'*y5;
C6=s'*y5;
theta426(3)=atan2(S6,C6); % Eqn 3.155
% Assume Flip is not set
if FL==1
    theta426(1)=theta426(1)+pi;
    theta426(2)=-theta426(2);
    theta426(3)=theta426(3)+pi;
end
end
function q=solve_circ(d,a,P,T,decval)
q=zeros(6,1);
ARM=decval(1);
EL=decval(2);
WRIST=decval(3);
FL=decval(4);
% theta1
r=sqrt(P(1)^2+P(2)^2);
q(1)=atan2(P(2),P(1))-atan2(d(2),-ARM*sqrt(r^2-d(2)^2)); % Eqn 3.104

% theta3
D=(cos(q(1))*P(1)+sin(q(1))*P(2))^2+P(3)^2-d(4)^2-a(3)^2-a(2)^2;
e2=4*a(2)^2*a(3)^2+4*a(2)^2*d(4)^2;
q(3)=atan2(D,ARM*EL*sqrt(e2-D^2))-atan2(a(3),d(4)); % Eqn 3.105

ARM2=ARM;
% theta2
f=(cos(q(1))*P(1)+sin(q(1))*P(2));
h2=d(4)^2+a(2)^2+a(3)^2+2*a(2)*d(4)*sin(q(3))+2*a(2)*a(3)*cos(q(3));
% Eqn 3.106
q(2)=atan2(f,ARM2*sqrt(h2-f^2))-atan2(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2),d(4)*cos(q(3))-a(3)*sin(q(3)));
% Finding ARM2
ARM2=sign(cos(q(2))*(d(4)*cos(q(3))-a(3)*sin(q(3)))-sin(q(2))*(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2)));
if ARM2==0
    ARM2=1;
end
% theta2
f=(cos(q(1))*P(1)+sin(q(1))*P(2));
h2=d(4)^2+a(2)^2+a(3)^2+2*a(2)*d(4)*sin(q(3))+2*a(2)*a(3)*cos(q(3));
% Eqn 3.106
q(2)=atan2(f,ARM2*sqrt(h2-f^2))-atan2(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2),d(4)*cos(q(3))-a(3)*sin(q(3)));

%theta4
% Eqn 3.110
n=T(1:3,1);
s=T(1:3,2);
ap=T(1:3,3);
num=cos(q(1))*ap(2)-sin(q(1))*ap(1);
den=cos(q(1))*cos(q(2)+q(3))*ap(1)+sin(q(1))*cos(q(2)+q(3))*ap(2)-sin(q(2)+q(3))*ap(3);
q(4)=atan2(WRIST*num,WRIST*den);
%theta5
%Eqn 3.111
num=(cos(q(1))*cos(q(2)+q(3))*cos(q(4))-sin(q(1))*sin(q(4)))*ap(1)+...
    (sin(q(1))*cos(q(2)+q(3))*cos(q(4))+cos(q(1))*sin(q(4)))*ap(2)-...
    cos(q(4))*sin(q(2)+q(3))*ap(3);
den=cos(q(1))*sin(q(2)+q(3))*ap(1)+sin(q(1)*sin(q(2)+q(3))*ap(2))+cos(q(2)+q(3))*ap(3);
q(5)=atan2(num,den);
%theta6
%Eqn 3.112
num=(-sin(q(1))*cos(q(4))-cos(q(1))*cos(q(2)+q(3))*sin(q(4)))*n(1)+...
    (cos(q(1))*cos(q(4))-sin(q(1))*cos(q(2)+q(3))*sin(q(4)))*n(2)+...
    sin(q(2)+q(3))*sin(q(4))*n(3);
den=(-sin(q(1))*cos(q(4))-cos(q(1))*cos(q(2)+q(3))*sin(q(4)))*s(1)+...
    (cos(q(1))*cos(q(4))-sin(q(1))*cos(q(2)+q(3))*sin(q(4)))*s(2)+...
    sin(q(2)+q(3))*sin(q(4))*s(3);
q(6)=atan2(num,den);
if FL==1
    q(4)=q(4)+pi;
    q(5)=-q(5);
    q(6)=q(6)+pi;
end
end
function q=solve_hfang(d,a,P,T,decval)
q=zeros(6,1);
ARM=decval(1);
EL=decval(2);
WRIST=decval(3);
FL=decval(4);
% theta1
r=sqrt(P(1)^2+P(2)^2);
q(1)=2*atan2(-P(1)-ARM*sqrt(r^2-d(2)^2),d(2)+P(2)); % Eqn 3.107

% theta3
D=(cos(q(1))*P(1)+sin(q(1))*P(2))^2+P(3)^2-(d(4)^2+a(3)^2+a(2)^2);
e2=4*a(2)^2*a(3)^2+4*a(2)^2*d(4)^2;
q(3)=2*atan2(2*a(2)*d(4)-ARM*EL*sqrt(e2-D^2),D+2*a(2)*a(3)); % Eqn 3.108


ARM2=ARM;
% theta2
f=(cos(q(1))*P(1)+sin(q(1))*P(2));
h2=d(4)^2+a(2)^2+a(3)^2+2*a(2)*d(4)*sin(q(3))+2*a(2)*a(3)*cos(q(3));
% Eqn 3.109
num=(d(4)*cos(q(3))-a(3)*sin(q(3)))-ARM2*sqrt(h2-f^2);
den=f+(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2));
q(2)=2*atan2(num,den);
% Finding ARM2
ARM2=sign(cos(q(2))*(d(4)*cos(q(3))-a(3)*sin(q(3)))-sin(q(2))*(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2)));
if ARM2==0
    ARM2=1;
end

num=(d(4)*cos(q(3))-a(3)*sin(q(3)))-ARM2*sqrt(h2-f^2);
den=f+(d(4)*sin(q(3))+a(3)*cos(q(3))+a(2));
q(2)=2*atan2(num,den);
%theta4
% Eqn 3.110
n=T(1:3,1);
s=T(1:3,2);
ap=T(1:3,3);
num=cos(q(1))*ap(2)-sin(q(1))*ap(1);
den=cos(q(1))*cos(q(2)+q(3))*ap(1)+sin(q(1))*cos(q(2)+q(3))*ap(2)-sin(q(2)+q(3))*ap(3);
q(4)=atan2(WRIST*num,WRIST*den);
%theta5
%Eqn 3.111
num=(cos(q(1))*cos(q(2)+q(3))*cos(q(4))-sin(q(1))*sin(q(4)))*ap(1)+...
    (sin(q(1))*cos(q(2)+q(3))*cos(q(4))+cos(q(1))*sin(q(4)))*ap(2)-...
    cos(q(4))*sin(q(2)+q(3))*ap(3);
den=cos(q(1))*sin(q(2)+q(3))*ap(1)+sin(q(1)*sin(q(2)+q(3))*ap(2))+cos(q(2)+q(3))*ap(3);
q(5)=atan2(num,den);
%theta6
%Eqn 3.112
num=(-sin(q(1))*cos(q(4))-cos(q(1))*cos(q(2)+q(3))*sin(q(4)))*n(1)+...
    (cos(q(1))*cos(q(4))-sin(q(1))*cos(q(2)+q(3))*sin(q(4)))*n(2)+...
    sin(q(2)+q(3))*sin(q(4))*n(3);
den=(-sin(q(1))*cos(q(4))-cos(q(1))*cos(q(2)+q(3))*sin(q(4)))*s(1)+...
    (cos(q(1))*cos(q(4))-sin(q(1))*cos(q(2)+q(3))*sin(q(4)))*s(2)+...
    sin(q(2)+q(3))*sin(q(4))*s(3);
q(6)=atan2(num,den);
if FL==1
    q(4)=q(4)+pi;
    q(5)=-q(5);
    q(6)=q(6)+pi;
end
end