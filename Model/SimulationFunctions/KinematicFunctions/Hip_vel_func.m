function velH = Hip_vel_func(in1,in2)
%HIP_VEL_FUNC
%    VELH = HIP_VEL_FUNC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    30-Sep-2020 10:41:48

x8 = in1(8,:);
x9 = in1(9,:);
velH = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,x8,x9,0.0,0.0],[4,4]);
