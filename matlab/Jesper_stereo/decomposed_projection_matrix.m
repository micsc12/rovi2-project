clc, clear;

projection_l = [1473.179202, 0.000000, 500.873203, 0.000000; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];
projection_r = [1473.179202, 0.000000, 500.873203, -175.374968; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];

camera_l = [1316.084092, 0.000000, 506.400063; 0.000000, 1316.417843, 372.835099; 0.000000, 0.000000, 1.000000]
camera_r = [1318.204959, 0.000000, 517.054064; 0.000000, 1318.239559, 463.621512; 0.000000, 0.000000, 1.000000]
rect_l = [0.999966, -0.004955, 0.006583; 0.004947, 0.999987, 0.001256; -0.006590, -0.001223, 0.999978]

[K_l, Rc_w_l, Pc_l, pp_l, pv_l] = decomposecamera(projection_l);
[K_r, Rc_w_r, Pc_r, pp_r, pv_r] = decomposecamera(projection_r);


%% P = K A H
% (KA)^-1 P  = H

inv_l = inv(camera_l)*projection_l

%%
[R Q] = rq3(projection_l(1:3,1:3)) %where M is an invertible 3x3 matrix, 
%and C is a column-vector representing the camera's position in world coordinates. 
%Some calibration software provides a 4x4 matrix, which adds an extra row to preserve the z-coordinate. 
%In this case, just drop the third row to get a 3x4 matrix.

%% Jesper trying stuff with the projection matrix
%Formula from http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
simple=zeros(3,4);
simple(1,1)=1;
simple(2,2)=1;
simple(3,3)=1;
H=zeros(4,4);
%eye(4)
H(1:3,1:3)=Rc_w_l;
H(4,4)=1;
H(4,1:3)=Pc_l;
P_l=camera_l*simple*H;
H_from_shit=pinv(camera_l*simple)*projection_l
normc(H_from_shit(1:3,1:3))


%% Using code from the dear guys

%********************************************************************
% 1) LOADING THE STEREO CALIBRATION MATRIX
%********************************************************************


% intrinsic
KAl =   camera_l;

KAr =   camera_r;

H=eye(4);

% extrinsic
Hl =   H;

H(1,4)=Pc_r(1);
Hr =   H;
%********************************************************************
% 2) COMPUTE P
%********************************************************************

Pl = [ KAl [0 0 0]'] * Hl;
Pr = [ KAr [0 0 0]'] * Hr;
%********************************************************************
% 3) COMPUTE THE OPTICAL CENTER FOR LEFT AND RIGHT ( C=-pow(-R,-1)*t )
%********************************************************************
%Pr(1,4)=Pr(1,4)*-1;
%hej2=Pr(1,4);
%Pl=projection_l;
%Pr=projection_r;

PXl = Pl(1:3,1:3);
PXr = Pr(1:3,1:3);

pxl = Pl(1:3,4);
pxr = Pr(1:3,4);

Cl =  [(-inv(PXl) * pxl); 1];
Cr =  [(-inv(PXr) * pxr); 1];

%********************************************************************
% 4) COMPUTE EPIPOLES (e = P * C)
%********************************************************************

el = Pl * Cr;
er = Pr * Cl;

%********************************************************************
% 5) COMPUTE FUNDAMENTAL MATRIX F12
%********************************************************************

erx = [ 0 -er(3) er(2);
    er(3) 0 -er(1);
    -er(2) er(1) 0];

Flr = erx * Pr * pinv(Pl);

%Flr


%% Jesper trying stuff from: https://www.youtube.com/watch?v=5wuWKTJCV1w


translation_as_skew = [ 0       -Pc_r(3)      Pc_r(2);
                Pc_r(3)   0           -Pc_r(1);
                -Pc_r(2)  Pc_r(1)       0    ];
E=translation_as_skew*Rc_w_r; % Essential matrix
%http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node21.html
%Flr=transpose(pinv(K_l))*E*pinv(K_r)
%Flr=transpose(pinv(camera_r))*E*pinv(camera_l)
M1=[1 0 0 0;
    0 1 0 0;
    0 0 1 0]
M2= [Rc_w_r Pc_r]


 % Coordinate set 1
%p1=[337;312;1]; % left
%p2=[273;407;1]; % right
%p1=[337-1024/2;312-768/2;1]; % left
%p2=[273-1024/2;407-768/2;1]; % right

%New coordinate set 2
%p1=[440;515;1]; % left
%p2=[354;610;1]; % right

%New coordinate set 3
%p1=[123;578;1]; % left
%p2=[43;667;1]; % right

%Image 66 from samples:
p1=[579;481;1]; % left
p2=[505;575;1]; % right
%p2=[785;1000;1]; % right
p_left=[579;481;1]; % left
p_right=[505;575;1]; % right


% Image from calibration data
p1=[316;324;1]; % left
p2=[243;418;1]; % right
p_left=[316;324;1]; % left
p_right=[243;418;1]; % right


result=transpose(p2)*Flr*p1


% left: x 337 y 312
% right: x 273 y 407
p1x = [ 0       -p1(3)      p1(2);
        p1(3)   0           -p1(1);
        -p1(2)  p1(1)       0    ];
p2x = [ 0       -p2(3)      p2(2);
        p2(3)   0           -p2(1);
        -p2(2)  p2(1)       0    ];

line=Flr*p_left

%factor=1/(line(1)^2+line(2)^2)

factor=sqrt(1/(line(1)^2+line(2)^2))

line(1)=line(1)*factor;
line(2)=line(2)*factor;
line(3)=line(3)*factor;
line

xa=0;
xb=1024;
ya=(line(3)-line(1)*xa)/line(2)
yb=(line(3)-line(1)*xb)/line(2)

image_right=imread('right66');
figure(1);

hold on
%image(image_right)
%line([xa xb],[ya yb]);
hold off

% l2_param=E*p2;
% xa=0;
% xb=1024;
% ya=(l2_param(3)-l2_param(1)*xa)/l2_param(2)
% yb=(l2_param(3)-l2_param(1)*xb)/l2_param(2)
% A_right=imread('right66');

% Calculating 3D point
% A = [ p1x * M1 ; p2x * M2]
% 
% [U,D,V] = svd(A);
% 
% P=V(:,4);
% Plest = P/P(4)