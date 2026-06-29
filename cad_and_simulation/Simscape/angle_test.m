q_ref = [0.996188036723854, 0.00425790764049721, 0.0871272705745011, 0.000323161005497921];
q_meas = [0.996188036723854, 0.00425790764049721, 0.0871272705745011, 0.000323161005497921];
q_meas = q_meas / quatnorm(q_meas);

q_e = quatmultiply(quatconj(q_meas),q_ref)

q_e = [1 0.00 0.00 0.001];

q_e = q_e/quatnorm(q_e);

[a_z, a_y, a_x] = quat2angle(q_e, "ZYX")