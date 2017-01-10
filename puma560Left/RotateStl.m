%% Mat lab file to rotate the Stl files to match the Right arm of the Puma 
% Model to the Left Arm as specified in Lee's Book

% Rotate the link1 3d file to suit the Left Arm Configuration
% load the stl file using stl_read


% [F,V,C]= stl_read('link1.stl');
% patch('Faces', F, 'Vertices', V, 'FaceVertexCData', (1:5106)', 'FaceColor', 'flat');
% view(3)
% axis equal
% go=input('Enter to continue');
% % create a rotation matrix for 180 degree
% Rz=[ cos(pi), sin(pi), 0 
%     -sin(pi), cos(pi), 0 
%      0,         0, 1 ];
% V=V*Rz';
% patch('Faces', F, 'Vertices', V, 'FaceVertexCData', (1:5106)', 'FaceColor', 'flat');
% view(3)
% axis equal
% stlwrite('link1test.stl',F,V);

%% The bove sectin is not needed use coord_change file to rotate the 
%stl file and save it as in required name.
% Link0 - No issues
% Link1
T=[ cos(pi), sin(pi),  0, 0 
    -sin(pi), cos(pi), 0, 0 
     0,         0,     1, 0
     0,         0,     0, 1]; 
% Link2          
T=[1 0 0 0
    0 1 0 0
    0 0 1 0.15005
    0 0 0 1];
% Link3
T=[ cos(pi), sin(pi),  0, 0 
    -sin(pi), cos(pi), 0, 0 
     0,         0,     1, 0
     0,         0,     0, 1];
% Link4
T=[ cos(pi), sin(pi),  0, 0 
    -sin(pi), cos(pi), 0, 0 
     0,         0,     1, 0
     0,         0,     0, 1]; 
%Link5 - No issues
%LInk6 - No issues