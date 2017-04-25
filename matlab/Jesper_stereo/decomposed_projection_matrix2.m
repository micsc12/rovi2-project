clc, clear;

projection_l = [1473.179202, 0.000000, 500.873203, 0.000000; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];
projection_r = [1473.179202, 0.000000, 500.873203, -175.374968; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];

camera_l = [1316.084092, 0.000000, 506.400063; 0.000000, 1316.417843, 372.835099; 0.000000, 0.000000, 1.000000]
camera_r = [1318.204959, 0.000000, 517.054064; 0.000000, 1318.239559, 463.621512; 0.000000, 0.000000, 1.000000]
rect_l = [0.999966, -0.004955, 0.006583; 0.004947, 0.999987, 0.001256; -0.006590, -0.001223, 0.999978]

[K_l, Rc_w_l, Pc_l, pp_l, pv_l] = decomposecamera(projection_l);
[K_r, Rc_w_r, Pc_r, pp_r, pv_r] = decomposecamera(projection_r);

%load('stereo_param.mat');

%camera_l=stereoParams.CameraParameters1.IntrinsicMatrix;
%camera_r=stereoParams.CameraParameters2.IntrinsicMatrix;

%translation=stereoParams.TranslationOfCamera2;
%rotation=stereoParams.RotationOfCamera2;

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

H(1,4)=-Pc_r(1);
%H(1,4)=projection_r(1,4)/1000;
Hr =   H;
%********************************************************************
% 2) COMPUTE P
%********************************************************************

Pl = [ KAl [0 0 0]'] * Hl;
Pr = [ KAr [0 0 0]'] * Hr;
%Pr(1,4)=projection_r(1,4)
%********************************************************************
% 3) COMPUTE THE OPTICAL CENTER FOR LEFT AND RIGHT ( C=-pow(-R,-1)*t )
%********************************************************************
%Pl=[1473.179202, 0.000000, 500.873203, 0.000000; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];
%Pr=[1473.179202, 0.000000, 500.873203, 175.374968; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];

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

%Flr = erx * Pr * pinv(Pl);

%Flr


%% Jesper trying stuff from: https://www.youtube.com/watch?v=5wuWKTJCV1w


translation_as_skew = [ 0       -Pc_r(3)      Pc_r(2);
                Pc_r(3)   0           -Pc_r(1);
                -Pc_r(2)  Pc_r(1)       0    ];
E=translation_as_skew*Rc_w_r; % Essential matrix
%http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node21.html
Flr=transpose(pinv(camera_r))*E*pinv(camera_l)
M1=[1 0 0 0;
    0 1 0 0;
    0 0 1 0]
M2= [Rc_w_r Pc_r]



%Image 66 from samples:
p1=[579;481;1]; % left
p2=[505;575;1]; % right
%p2=[785;1000;1]; % right
p_left=[579;481;1]; % left
p_right=[505;575;1]; % right


% Image from calibration data
%p1=[316;324;1]; % left
%p2=[243;418;1]; % right
p_left2=[316;324;1]; % left
p_right2=[243;418;1]; % right

result=transpose(p2)*Flr*p1

line=Flr*p_left
factor=sqrt(1/(line(1)^2+line(2)^2))

line(1)=line(1)*factor;
line(2)=line(2)*factor;
line(3)=line(3)*factor;
line

xa=0;
xb=1024;
ya=(line(3)-line(1)*xa)/line(2)
yb=(line(3)-line(1)*xb)/line(2)

%% Trying to get 3d points
p_right
Minf = [pinv(PXl)*p_left; 0]; % computing point at infinity
mr = Pr*Minf % reprojection of point at infinity in right image
%error=mr(1)-p_right(1)

% p_right2
% Minf2 = [pinv(PXl)*p_left2; 0]; % computing point at infinity
% mr2 = Pr*Minf2 % reprojection of point at infinity in right image
% %error=mr2(1)-p_right2(1)
% 
% p_left2
% Minf2 = [pinv(PXr)*p_right2; 0]; % computing point at infinity
% mr2 = Pl*Minf2 % reprojection of point at infinity in right image
% %error=mr2(1)-p_right2(1)


M_inf = inv(PXr)*p_right; % computing point at infinity
    
Minf = Minf(1:3);
M_inf = M_inf(1:3);

mu1 = cross(Cl(1:3),Minf) / norm(Minf);
v1 = Minf / norm(Minf);

mu2 = cross(Cr(1:3),M_inf) / norm(M_inf);
v2 = M_inf / norm(M_inf);

% 9) COMPUTE INTERSECTION OF THOSE LINES

M1 = ((v1 * cross(v2,mu2)' - (v1 * v2') * v1 * cross(v2,mu1)') / (norm(cross(v1,v2))^2)) * v1 + cross(v1,mu1);
M2 = ((v2 * cross(v1,mu1)' - (v2 * v1') * v2 * cross(v1,mu2)') / (norm(cross(v2,v1))^2)) * v2 + cross(v2,mu2);

avgM = M1 + (M2-M1)/2;


% 10) PRINT RESULT
M1
M2
avgM
