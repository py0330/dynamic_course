%% ur5 robot

% 用户可以自定义q dq ddq qf，分别代表驱动位置、速度、加速度、力
% 会输出：
% actuation_force : 电机出力，根据动力学反解计算
% actuation_force2 : 电机出力，根据通用的动力学方程来计算反解
% input_accleration : 输入加速度，根据动力学正解计算的电机加速度 

% 符号定义如下：
% cm : constraint matrix（就是某个关节的约束矩阵）
% pm : pose matrix（位姿矩阵）
% vs : velocity screw(twist)（速度螺旋）
% j1 : joint 1（关节1）
% m1 : motion 1（驱动1）

% 转动关节的约束矩阵
cm=[
1,0,0,0,0
0,1,0,0,0
0,0,1,0,0
0,0,0,1,0
0,0,0,0,1
0,0,0,0,0];

% 关节的位置与角度，z轴是关节的转动轴
j1_rpy = [0.0 0.0 0.0];
j1_xyz = [0.0 0.0 0.089159];

j2_rpy = [0.0 0.0 -pi/2];
j2_xyz = [0.0, 0.13585, 0.089159];

j3_rpy = [0.0 0.0 -pi/2];
j3_xyz = [0.425, 0.13585 - 0.1197, 0.089159];

j4_rpy = [0.0 0.0 -pi/2];
j4_xyz = [0.425 + 0.39225, 0.13585 - 0.1197, 0.089159];

j5_rpy = [0.0 0.0 0.0];
j5_xyz = [0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159];

j6_rpy = [0.0 0.0 -pi/2];
j6_xyz = [0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465];

pm_j1o = [eul2rotm(j1_rpy,'ZYX'), j1_xyz'
    0,0,0,1];
j1_vso = Tv(pm_j1o)*[0;0;0;0;0;1];
j1_cmo = Tf(pm_j1o)*cm;
m1_cmo = Tf(pm_j1o)*[0;0;0;0;0;1];

pm_j2o = [eul2rotm(j2_rpy,'ZYX'), j2_xyz'
    0,0,0,1];
j2_vso = Tv(pm_j2o)*[0;0;0;0;0;1];
j2_cmo = Tf(pm_j2o)*cm;
m2_cmo = Tf(pm_j2o)*[0;0;0;0;0;1];

pm_j3o = [eul2rotm(j3_rpy,'ZYX'), j3_xyz'
    0,0,0,1];
j3_vso = Tv(pm_j3o)*[0;0;0;0;0;1];
j3_cmo = Tf(pm_j3o)*cm;
m3_cmo = Tf(pm_j3o)*[0;0;0;0;0;1];

pm_j4o = [eul2rotm(j4_rpy,'ZYX'), j4_xyz'
    0,0,0,1];
j4_vso = Tv(pm_j4o)*[0;0;0;0;0;1];
j4_cmo = Tf(pm_j4o)*cm;
m4_cmo = Tf(pm_j4o)*[0;0;0;0;0;1];

pm_j5o = [eul2rotm(j5_rpy,'ZYX'), j5_xyz'
    0,0,0,1];
j5_vso = Tv(pm_j5o)*[0;0;0;0;0;1];
j5_cmo = Tf(pm_j5o)*cm;
m5_cmo = Tf(pm_j5o)*[0;0;0;0;0;1];

pm_j6o = [eul2rotm(j6_rpy,'ZYX'), j6_xyz'
    0,0,0,1];
j6_vso = Tv(pm_j6o)*[0;0;0;0;0;1];
j6_cmo = Tf(pm_j6o)*cm;
m6_cmo = Tf(pm_j6o)*[0;0;0;0;0;1];

% 求出起始位置各个杆件的惯量
I0o = [eye(3)*4.0, zeros(3,3);zeros(3,3), eye(3)];
pm = [eye(3), j1_xyz' + [0.0 0.00193 -0.02561]';0,0,0,1];
I1o = Tf(pm) * [eye(3)*3.7, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j2_xyz' + [0.2125 -0.024201 0.0]';0,0,0,1];
I2o = Tf(pm) * [eye(3)*8.393, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j3_xyz' + [0.110949 0.0 0.01634]';0,0,0,1];
I3o = Tf(pm) * [eye(3)*2.275, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j4_xyz';0,0,0,1];
I4o = Tf(pm) * [eye(3)*1.219, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j5_xyz';0,0,0,1];
I5o = Tf(pm) * [eye(3)*1.219, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j6_xyz';0,0,0,1];
I6o = Tf(pm) * [eye(3)*0.1879, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';

% end effector（末端）
ee_rpy = [pi, pi/2, 0];
ee_xyz = [0.425 + 0.39225, 0.13585 - 0.1197 + 0.093 + 0.0823, 0.089159 - 0.09465];

pm_eeo = [eul2rotm(ee_rpy,'ZYX'), ee_xyz'
    0,0,0,1];
%% input
if ~exist('q','var')
    q = [0.1,0.2,0.3,0.4,0.5,0.6]';
end
if(~exist('dq','var'))
    dq = [0.1,0.2,0.3,0.4,0.5,0.6]';
end
if(~exist('ddq','var'))
    ddq =  [0.1,0.2,0.3,0.4,0.5,0.6]';
end
if(~exist('qf','var'))
    % 输入力
    qf = -[1.36817604774081,-43.5520354075637,-7.09576204454976, 3.28091697296027,1.20096689859854,1.27283062013759]';
end
%% problem 1： 位置正解
P0 = eye(4);
P1 = P(j1_vso*q(1));
P2 = P1*P(j2_vso*q(2));
P3 = P2*P(j3_vso*q(3));
P4 = P3*P(j4_vso*q(4));
P5 = P4*P(j5_vso*q(5));
P6 = P5*P(j6_vso*q(6));

ee = P6*pm_eeo;
%% problem 2： 速度雅可比
J = [Tv(P0)*j1_vso, Tv(P1)*j2_vso, Tv(P2)*j3_vso, Tv(P3)*j4_vso, Tv(P4)*j5_vso, Tv(P5)*j6_vso];
%% problem 3： 计算所有杆件的速度
% step 1
j1_cm = j1_cmo;
j2_cm = Tf(P1) * j2_cmo;
j3_cm = Tf(P2) * j3_cmo;
j4_cm = Tf(P3) * j4_cmo;
j5_cm = Tf(P4) * j5_cmo;
j6_cm = Tf(P5) * j6_cmo;

m1_cm = m1_cmo;
m2_cm = Tf(P1) * m2_cmo;
m3_cm = Tf(P2) * m3_cmo;
m4_cm = Tf(P3) * m4_cmo;
m5_cm = Tf(P4) * m5_cmo;
m6_cm = Tf(P5) * m6_cmo;

% Constraint force matrix is as follow:
%    Fix R1 R2 R3 R4 R5 R6 M1 M2 M3 M4 M5 M6
% GR  1  -1                -1
% L1      1 -1              1 -1  
% L2         1 -1              1 -1
% L3            1 -1              1 -1
% L4               1 -1              1 -1
% L5                  1 -1              1 -1
% L6                     1                 1
C=[
eye(6,6),       -j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),     -m1_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6),      j1_cm,     -j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      m1_cm,     -m2_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5),      j2_cm,     -j3_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1),      m2_cm,     -m3_cm, zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5),      j3_cm,     -j4_cm, zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1),      m3_cm,     -m4_cm, zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5),      j4_cm,     -j5_cm, zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1),      m4_cm,     -m5_cm, zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j5_cm,     -j6_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1),      m5_cm,     -m6_cm
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j6_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1),      m6_cm];

% step 2
cv = [zeros(36,1);dq];

% step 3
v = C'\cv;

v0 = v(1:6);
v1 = v(7:12);
v2 = v(13:18);
v3 = v(19:24);
v4 = v(25:30);
v5 = v(31:36);
v6 = v(37:42);
%% problem 4： 根据C来计算雅可比
CT_inv = inv(C');
J2 = CT_inv(end-5:end,end-5:end);
%% problem 5： 加速度输入输出关系
dJ = [Cv(v0)*Tv(P0)*j1_vso, Cv(v1)*Tv(P1)*j2_vso, Cv(v2)*Tv(P2)*j3_vso, Cv(v3)*Tv(P3)*j4_vso, Cv(v4)*Tv(P4)*j5_vso, Cv(v5)*Tv(P5)*j6_vso];
aee = J*ddq + dJ*dq;
%% problem 6： 求所有杆件的加速度
% step 1
dC=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),   -Cf(v0)*m1_cm, zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1)
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v0)*m1_cm,-Cf(v1)*m2_cm, zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1)
zeros(6,6), zeros(6,5),    Cf(v1)*j2_cm,-Cf(v2)*j3_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,1),    Cf(v1)*m2_cm,-Cf(v2)*m3_cm, zeros(6,1),    zeros(6,1),    zeros(6,1)
zeros(6,6), zeros(6,5),    zeros(6,5),    Cf(v2)*j3_cm,-Cf(v3)*j4_cm, zeros(6,5),    zeros(6,5),    zeros(6,1),    zeros(6,1),    Cf(v2)*m3_cm,-Cf(v3)*m4_cm, zeros(6,1),    zeros(6,1)
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v3)*j4_cm,-Cf(v4)*j5_cm, zeros(6,5),    zeros(6,1),    zeros(6,1),    zeros(6,1),    Cf(v3)*m4_cm,-Cf(v4)*m5_cm, zeros(6,1)
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v4)*j5_cm,-Cf(v5)*j6_cm, zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1),    Cf(v4)*m5_cm,-Cf(v5)*m6_cm
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v5)*j6_cm, zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1),    zeros(6,1),    Cf(v5)*m6_cm];

ca = [zeros(36,1);ddq] - dC'*v;
% step 2
a = C'\ca;

a0 = a(1:6);
a1 = a(7:12);
a2 = a(13:18);
a3 = a(19:24);
a4 = a(25:30);
a5 = a(31:36);
a6 = a(37:42);
%% problem 7： 动力学逆解
% step 1
I0=Tf(P0) * I0o * Tf(P0)';
I1=Tf(P1) * I1o * Tf(P1)';
I2=Tf(P2) * I2o * Tf(P2)';
I3=Tf(P3) * I3o * Tf(P3)';
I4=Tf(P4) * I4o * Tf(P4)';
I5=Tf(P5) * I5o * Tf(P5)';
I6=Tf(P6) * I6o * Tf(P6)';

I=blkdiag(I0,I1,I2,I3,I4,I5,I6);

% step 2
g=[0,0,-9.8,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0;
f1=-I1*g + Cf(v1)*I1*v1;
f2=-I2*g + Cf(v2)*I2*v2;
f3=-I3*g + Cf(v3)*I3*v3;
f4=-I4*g + Cf(v4)*I4*v4;
f5=-I5*g + Cf(v5)*I5*v5;
f6=-I6*g + Cf(v6)*I6*v6;

fp=[f0;f1;f2;f3;f4;f5;f6];

% step 3
ca = [zeros(36,1);ddq] - dC'*v;

% step 4
A = [-I, C;C', zeros(42,42)];

b = [fp;ca];

x = A\b;

% x 包含了所有的杆件加速度和所有的约束力，现在列出六个驱动力
actuation_force = x(end-5:end);
%% problem 8： 动力学正解
% step 0 regenerate C
C2=[
eye(6,6),       -j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),
zeros(6,6),      j1_cm,     -j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),
zeros(6,6), zeros(6,5),      j2_cm,     -j3_cm, zeros(6,5), zeros(6,5), zeros(6,5),
zeros(6,6), zeros(6,5), zeros(6,5),      j3_cm,     -j4_cm, zeros(6,5), zeros(6,5),
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5),      j4_cm,     -j5_cm, zeros(6,5),
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j5_cm,     -j6_cm,
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j6_cm,];

dC2=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),   
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    
zeros(6,6), zeros(6,5),    Cf(v1)*j2_cm,-Cf(v2)*j3_cm, zeros(6,5),    zeros(6,5),    zeros(6,5),    
zeros(6,6), zeros(6,5),    zeros(6,5),    Cf(v2)*j3_cm,-Cf(v3)*j4_cm, zeros(6,5),    zeros(6,5),    
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v3)*j4_cm,-Cf(v4)*j5_cm, zeros(6,5),    
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v4)*j5_cm,-Cf(v5)*j6_cm, 
zeros(6,6), zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    zeros(6,5),    Cf(v5)*j6_cm];

% step 1
I0=Tf(P0) * I0o * Tf(P0)';
I1=Tf(P1) * I1o * Tf(P1)';
I2=Tf(P2) * I2o * Tf(P2)';
I3=Tf(P3) * I3o * Tf(P3)';
I4=Tf(P4) * I4o * Tf(P4)';
I5=Tf(P5) * I5o * Tf(P5)';
I6=Tf(P6) * I6o * Tf(P6)';

I=blkdiag(I0,I1,I2,I3,I4,I5,I6);

% step 2
g=[0,0,-9.8,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0-m1_cm*qf(1);
f1=-I1*g + Cf(v1)*I1*v1+m1_cm*qf(1)-m2_cm*qf(2);
f2=-I2*g + Cf(v2)*I2*v2+m2_cm*qf(2)-m3_cm*qf(3);
f3=-I3*g + Cf(v3)*I3*v3+m3_cm*qf(3)-m4_cm*qf(4);
f4=-I4*g + Cf(v4)*I4*v4+m4_cm*qf(4)-m5_cm*qf(5);
f5=-I5*g + Cf(v5)*I5*v5+m5_cm*qf(5)-m6_cm*qf(6);
f6=-I6*g + Cf(v6)*I6*v6+m6_cm*qf(6);

fp2=[f0;f1;f2;f3;f4;f5;f6];

% step 3
dcv2 = zeros(36,1);
ca2 = -dC2'*v+dcv2;

% step 4
A = [-I, C2; C2' zeros(36,36)];
b = [fp2;ca2];

x=A\b;

aj1 = x(7:12)-x(1:6) - Cv(v0)*v1;
aj2 = x(13:18)-x(7:12) - Cv(v1)*v2;
aj3 = x(19:24)-x(13:18) - Cv(v2)*v3;
aj4 = x(25:30)-x(19:24) - Cv(v3)*v4;
aj5 = x(31:36)-x(25:30) - Cv(v4)*v5;
aj6 = x(37:42)-x(31:36) - Cv(v5)*v6;

input_accleration = [norm(aj1(4:6));norm(aj2(4:6));norm(aj3(4:6));norm(aj4(4:6));norm(aj5(4:6));norm(aj6(4:6))];
%% problem 9： 写成动力学通用形式
A = [
-I, C
 C', zeros(42,42)    ];

B = inv(A);

M = B(end-5:end,end-5:end);
h = B(end-5:end,:)*[fp;- dC'*v];

actuation_force2 = M*ddq+h;