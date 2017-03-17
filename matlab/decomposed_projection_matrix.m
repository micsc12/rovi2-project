projection_l = [1473.179202, 0.000000, 500.873203, 0.000000; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];
projection_r = [1473.179202, 0.000000, 500.873203, -175.374968; 0.000000, 1473.179202, 421.267643, 0.000000; 0.000000, 0.000000, 1.000000, 0.000000];

camera_l = [1316.084092, 0.000000, 506.400063; 0.000000, 1316.417843, 372.835099; 0.000000, 0.000000, 1.000000]

rect_l = [0.999966, -0.004955, 0.006583; 0.004947, 0.999987, 0.001256; -0.006590, -0.001223, 0.999978]

[K_l, Rc_w_l, Pc_l, pp_l, pv_l] = decomposecamera(projection_l);
[K_r, Rc_w_r, Pc_r, pp_r, pv_r] = decomposecamera(projection_r);


%% P = K A H
% (KA)^-1 P  = H

inv_l = inv(camera_l)*projection_l

%%
[R Q] = rq3(projection_l(1:3,1:3))where M is an invertible 3x3 matrix, and C is a column-vector representing the camera's position in world coordinates. Some calibration software provides a 4x4 matrix, which adds an extra row to preserve the z-coordinate. In this case, just drop the third row to get a 3x4 matrix.