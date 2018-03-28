%% two bar robot

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
j1_xyz = [0.0 0.0 0.0];

j2_rpy = [0.0 0.0 0.0];
j2_xyz = [1.0 0.0 0.0];

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

% 求出起始位置各个杆件的惯量
I0o = eye(6);
I1o = eye(6);
I2o = eye(6);

% end effector（末端）
ee_rpy = [0.0 0.0 0.0];
ee_xyz = [1.0 1.0 0.0];

pm_eeo = [eul2rotm(ee_rpy,'ZYX'), ee_xyz'
    0,0,0,1];
%% input

% 判断当前工作区是否存在q等变量，如果存在，那么我们使用工作区的q，否则会使用默认输入
if ~exist('q','var')
    q = [0.6,-0.3]';
end
if(~exist('dq','var'))
    dq = [0.5, 0.3]';
end
if(~exist('ddq','var'))
    ddq =  [-0.1, 0.2]';
end
if(~exist('qf','var'))
    % 输入力
    qf = -[-0.115252880597922,0.369413700577896]';
end
%% problem 1： 位置正解
P0 = eye(4);
P1 = P(j1_vso*q(1));
P2 = P1*P(j2_vso*q(2));

ee = P2*pm_eeo;
%% problem 2： 速度雅可比
J = [Tv(P0)*j1_vso, Tv(P1)*j2_vso];
%% problem 3： 计算所有杆件的速度
% step 1
j1_cm = j1_cmo;
j2_cm = Tf(P1) * j2_cmo;

m1_cm = m1_cmo;
m2_cm = Tf(P1) * m2_cmo;

C=[
eye(6,6),       -j1_cm, zeros(6,5),     -m1_cm, zeros(6,1),
zeros(6,6),      j1_cm,     -j2_cm,      m1_cm,     -m2_cm,
zeros(6,6), zeros(6,5),      j2_cm, zeros(6,1),      m2_cm,];

% step 2
cv = [zeros(16,1);dq];

% step 3
v = C'\cv;

v0 = v(1:6);
v1 = v(7:12);
v2 = v(13:18);
%% problem 4： 根据C来计算雅可比
CT_inv = inv(C');
J2 = CT_inv(end-5:end,end-1:end);
%% problem 5： 加速度输入输出关系
dJ = [Cv(v0)*Tv(P0)*j1_vso, Cv(v1)*Tv(P1)*j2_vso];
aee = J*ddq + dJ*dq;
%% problem 6： 求所有杆件的加速度
% step 1
dC=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5),    -Cf(v0)*m1_cm, zeros(6,1)
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm, Cf(v0)*m1_cm,-Cf(v1)*m2_cm
zeros(6,6), zeros(6,5),    Cf(v1)*j2_cm,zeros(6,1),    Cf(v1)*m2_cm];

ca = [zeros(16,1);ddq] - dC'*v;
% step 2
a = C'\ca;

a0 = a(1:6);
a1 = a(7:12);
a2 = a(13:18);
%% problem 7： 动力学逆解
% step 1
I0=Tf(P0) * I0o * Tf(P0)';
I1=Tf(P1) * I1o * Tf(P1)';
I2=Tf(P2) * I2o * Tf(P2)';

I=blkdiag(I0,I1,I2);

% step 2
g=[0,0,-9.8,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0;
f1=-I1*g + Cf(v1)*I1*v1;
f2=-I2*g + Cf(v2)*I2*v2;

fp=[f0;f1;f2];

% step 3
ca = [zeros(16,1);ddq] - dC'*v;

% step 4
A = [-I, C;C', zeros(18,18)];
b = [fp;ca];
x = A\b;

% x 包含了所有的杆件加速度和所有的约束力，现在列出六个驱动力
actuation_force = x(end-1:end);
%% problem 8： 动力学正解
% step 0 regenerate C
C2=[
eye(6,6),       -j1_cm, zeros(6,5),
zeros(6,6),      j1_cm,     -j2_cm,
zeros(6,6), zeros(6,5),      j2_cm,];

dC2=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5),  
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm,
zeros(6,6), zeros(6,5),    Cf(v1)*j2_cm,];

% step 1
I0=Tf(P0) * I0o * Tf(P0)';
I1=Tf(P1) * I1o * Tf(P1)';
I2=Tf(P2) * I2o * Tf(P2)';

I=blkdiag(I0,I1,I2);

% step 2
g=[0,0,-9.8,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0-m1_cm*qf(1);
f1=-I1*g + Cf(v1)*I1*v1+m1_cm*qf(1)-m2_cm*qf(2);
f2=-I2*g + Cf(v2)*I2*v2+m2_cm*qf(2);

fp2=[f0;f1;f2];

% step 3
dcv2 = zeros(16,1);
ca2 = -dC2'*v+dcv2;

% step 4
A = [-I, C2; C2' zeros(16,16)];
b = [fp2;ca2];

x=A\b;

aj1 = x(7:12)-x(1:6) - Cv(v0)*v1;
aj2 = x(13:18)-x(7:12) - Cv(v1)*v2;

input_accleration = [norm(aj1(4:6));norm(aj2(4:6));];
%% problem 9： 写成动力学通用形式
A = [-I, C
 C', zeros(18,18)    ];

B = inv(A);

M = B(end-1:end,end-1:end);
h = B(end-1:end,:)*[fp;- dC'*v];

actuation_force2 = M*ddq+h;