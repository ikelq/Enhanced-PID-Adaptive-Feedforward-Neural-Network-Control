%% 
% Author: Qiong Liu,Dongyu Li,Shuzhi Sam Ge
% Email: liuqiong_yl@outlook.com
% Description: Adaptive Feedforward Neural Network Control with 
%              an Optimized Hidden Node Distribution
% The control law of the PID controller is 
% $\tau=K_2 r + K_I \int r$, where the control gain $K_I=[0.05,0;0,0.05]$.
% The technical details can be seen in the paper

%% parameter of dynamics
m1 = 2;  %unit is 'kg' %The model is referred "Aptive Nerual Network Contol..." written by Sam Ge on page57 
m2 = 0.85;

l1 = 0.35;  %unit is 'm'
l2 = 0.31;  % li is the length of ith link i
lc1 = 1/2 * l1; % lci the center of mass to joint of ith link
lc2 = 1/2 * l2;

I1 = 1/4*m1*l1^2;%1825795.31e-09 ;  % moment of inertial
I2 = 1/4*m2*l2^2;%26213426.68e-09 ;

g = 9.81;

p(1) = m1 * lc1.^2 + m2 * l1^2 + I1;
p(2) = m2 * lc2.^2 + I2;
p(3) = m2 * l1 * lc2;
p(4) = m1 * lc2 + m2 * l1;
p(5) = m2 * lc2;

%% begin simulation 
T=2000;
size=0.01;
t=0:size:T;
n=length(t);
% 16 8
% K1=2*diag([10  5]);
% K2=1*diag([10  5]);
% K1=2*diag([9  3]);
% K2=1*diag([9  3]);
% K1=2*diag([5  5]);
% K2=diag([5  5]);
%initial state
i=1;
%x(1)=1;x(2)=1;x(3)=1;x(4)=1; 
x(1)=0;x(2)=0;x(3)=0;x(4)=0; 

e1=[0;0];
de1=[0;0];
q=zeros(2,length(t));
%q(:,1)=[x(1);x(2)];
dq=zeros(2,length(t));
ddq=zeros(2,length(t));
e=zeros(2,length(t));
de=zeros(2,length(t));
Tau=zeros(2,length(t));
normW=zeros(2,length(t));
Td=[0,0]';
% qr=[sin(t);sin(t)];
% dqr=[cos(t);cos(t)];
% ddqr=-[sin(t);sin(t)];
qr=    [sin(t);cos(t)];
dqr=  [cos(t);-sin(t)];
ddqr=[-sin(t);-cos(t)];

% qr=[sin(0.5*t);sin(0.5*t)];
% dqr=[0.5*cos(0.5*t);0.5*cos(0.5*t)];
% ddqr=-0.25*[sin(0.5*t);0.25*sin(0.5*t)];

%  hidden node at reference trajectory
k=1;

% 
Ta=[0;0];   % for Ki
Kp=[30  18];
Kd=[10  6];
% Ki=2*[0.1;0.1];
% Kp=[40 15];
% Kd=[12  9];
Ki=0.5*[0.1;0.1];
K1=diag(Kp./Kd);
K2=diag(Kd);
K1=2*diag([1  1]);
K2=10*diag([1  1]);
K1=3*diag([1  0.6]);
K2=10*diag([1  0.6]);

%% the first step
for i=2:n

M=[p(1)+p(2)+2*p(3)*cos(x(3)) p(2)+p(3)*cos(x(3));
    p(2)+p(3)*cos(x(3)) p(2)];
C=[-p(3)*x(4)*sin(x(3)) -p(3)*(x(2)+x(4))*sin(x(3));
    p(3)*x(2)*sin(x(3))  0];
G=[p(4)*g*cos(x(1)) + p(5)*g*cos(x(1)+x(3)); p(5)*g*cos(x(1)+x(3))];
J=[-l1*sin(x(1))+l2*sin(x(1)+x(3))  -l2*sin(x(1)+x(3));  l1*cos(x(1))+l2*cos(x(1)+x(3))  l2*cos(x(1)+x(3)) ];

e1=[qr(1,i-1)-x(1);qr(2,i-1)-x(2)];
de1=[dqr(1,i-1)-x(3);dqr(2,i-1)-x(4)];
r=de1+K2*e1;
e(:,i-1)=e1;
de(:,i-1)=de1;


Ta=Ta+Ki.*r;
% Td=J'*[0,20]';
% if i>100001
%     Td=J'*[0,8]';
% end




e_RBF=M*(ddqr(:,i)+K2*de1)+C*(dqr(:,i)+K2*e1)+G+Td-Ta;
%e_RBF=M*(ddqr(:,i))+C*(dqr(:,i))+G-Ta;
ee_RBF(:,i)=e_RBF;
ee_Kr(:,i)=K1*r;
TTa(:,i)=Ta;

Tau(:,i)=K1*r+Ta+ 0*randn(2,1) ;
ddq(:,i)=M\(Tau(:,i)-Td-C*dq(:,i-1)-G);
dq(:,i)=dq(:,i-1)+size*ddq(:,i-1);
q(:,i)=q(:,i-1)+size*dq(:,i-1)+1/2*size^2*ddq(:,i-1);

x(1)=q(1,i);
x(2)=q(2,i);
x(3)=dq(1,i);
x(4)=dq(2,i);

end
e_q=qr-q;

last_100_seconds_e_q=e_q(:,length(t)-10000:length(t));
%mean_e_q_last_10=mean (( last_10_seconds_e_q').^2)
max_eq_100= max(last_100_seconds_e_q')
last_100_seconds_ee_RBF=ee_Kr(:,length(t)-10000:length(t));
%mean_ee_RBF_last_10=mean (( last_10_seconds_ee_RBF).^2,2)
max_eRBF_100= max(last_100_seconds_ee_RBF')



label_y="Tracking error [rad]";
legend_y = [ "error_1","error_2" ];
plot_line(t,e_q','t [s]',label_y,legend_y,[-13,0.4])

save("PID", 'q', 'e_q','Tau')

