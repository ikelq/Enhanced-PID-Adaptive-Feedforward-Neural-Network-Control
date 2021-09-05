%% 
% Author: Qiong Liu,Dongyu Li,Shuzhi Sam Ge
% Email: liuqiong_yl@outlook.com
% Description: Adaptive Feedforward Neural Network Control with 
%              an Optimized Hidden Node Distribution
% The control law of the  model-based feedforward controller is 
% $\tau = K_{2} r+ M(q_d)\ddot{q_d}+C(q_d,\dot{q}_d)\dot{q}_d+G(q_d)$, 
% in which the dynamics parameters are accurate.
% The technical details can be seen in the paper

% @ARTICLE{Qiong2021,  author={Q. {Liu} and D. {Li} and S. S. {Ge} and Z. {Ouyang}}, 
% journal={IEEE Transactions on Artificial Intelligence},   
% title={Adaptive Feedforward Neural Network Control with an Optimized Hidden Node Distribution},   
% year={2021},  volume={},  number={},  pages={1-1},  
% doi={10.1109/TAI.2021.3074106}}
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
% x(1)=1;x(2)=1;x(3)=1;x(4)=1; 
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
Mu(:,k)=[qr(:,i);dqr(:,i);ddqr(:,i)]';
distance_node=0.1;
width=1; % for RBF ,variance =width^2
gamma1=4; % for updating Weight
gamma2=2; % for updating Weight
%length(qr)


Node=length(Mu);   % number of nodes
W=zeros(Node,2);
WW(:,:,1)=W;
dw=zeros(Node,2);
dwf=dw;

normS(1)=0;

% 
K1=3*diag([1  0.6]);
K2=10*diag([1  0.6]);
TI=[0;0];
Ki=0*[0.1;0.1];

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
% %  te1=0.3*tanh(K2*e1);
% %  tde1=0.3*tanh(K2*de1);
% % te1=min(max(K2*e1,-0.1),0.1);
% % tde1=min(max(K2*de1,-0.1),0.1);
% %r=K2*e1;
%     %Z=[sin(q(:,i-1));(dqr(:,i)+K2*e1)*0.2;(ddqr(:,i)+K2*de1)*0.2];
%         Z=[qr(:,i-1);dqr(:,i);ddqr(:,i)];
%     %Z=[q(:,i-1);(dqr(:,i)+K2*e1);(ddqr(:,i)+K2*de1)];
%     %Z=[sin(q(:,i-1));(dqr(:,i)+te1);(ddqr(:,i)+tde1)];
%     %Z=min(max(Z,-1.2),1.2);
%   % Z=1.1*tanh(Z);
%     %Z=[q(:,i-1);(dqr(:,i));ddqr(:,i)];
%     %function [ S ] = RBF( Z, Mu,variance,node )     %prototype of function RBF
%     %b=min(max(50*abs(e1)),0.1) ;
%     
%     S=RBF(Z,Mu,width,Node ) ;                     % RBF method is used in calculating S     
%     
%     normS(i)=norm(S);
%     dw(:,1)=gamma1*S*r(1);              % updating law as stated
%     dw(:,2)=gamma2*S*r(2);
%     
%     %dw(:,1)=gamma1*(S*de1(1)+sigma1*W(:,1));              % updating law as stated
%     %dw(:,2)=gamma2*(S*de1(2)+sigma2*W(:,2));
e(:,i-1)=e1;
de(:,i-1)=de1;


TI=TI+Ki.*r;
% Td=J'*[0,20]';
% if i>100001
%     Td=J'*[0,8]';
% end
% de(:,i-1)=de1;
%dW(1)=-gamma*(S*r(1)+sigma1*W1_crt);
% the next step
%Tau(:,i)=K1*r+M*(ddqr(:,i)+K2*de1)+C*(dqr(:,i)+K2*e1)+G;
% Ta=W'*S;



% Calculator C(qr,dqr)
dqr1=dqr(:,i)+K2*e1;
x(3)=dqr1(1);
x(4)=dqr1(2);
C1=[-p(3)*x(4)*sin(x(3)) -p(3)*(x(2)+x(4))*sin(x(3));
    p(3)*x(2)*sin(x(3))  0];

% Ta=M*(ddqr(:,i)+K2*de1)+C*(dqr(:,i)+K2*e1)+G;
Ta=M*(ddqr(:,i))+C*(dqr(:,i))+G;
e_RBF=M*(ddqr(:,i)+K2*de1)+C*(dqr(:,i)+K2*e1)+G+Td-Ta;
%e_RBF=M*(ddqr(:,i))+C*(dqr(:,i))+G-Ta;
ee_RBF(:,i)=e_RBF;
ee_Kr(:,i)=K1*r;
TTa(:,i)=Ta;

Tau(:,i)=K1*r+Ta+TI+ 0*randn(2,1) ;
ddq(:,i)=M\(Tau(:,i)-Td-C*dq(:,i-1)-G);
dq(:,i)=dq(:,i-1)+size*ddq(:,i-1);
q(:,i)=q(:,i-1)+size*dq(:,i-1)+1/2*size^2*ddq(:,i-1);
% dq(:,i)=dq(:,i-1)+size*ddq(:,i-1) ;
% q(:,i)=q(:,i-1)+size*dq(:,i-1) ;
x(1)=q(1,i);
x(2)=q(2,i);
x(3)=dq(1,i);
x(4)=dq(2,i);


%     W(:,1)=dw(:,1)*size+W(:,1);                     % Weights for next iteration
%     W(:,2)=dw(:,2)*size+W(:,2);
%     Norm_W1(i)=sqrt(W(:,1)'*W(:,1));                    % Norm W1 & W2
%     Norm_W2(i)=sqrt(W(:,2)'*W(:,2));
%     
%     WW(:,:,i)=W;
end
e_q=qr-q;

label_y="Tracking error [rad]";
legend_y = [ "error_1","error_2" ];
plot_line(t,e_q','t [s]',label_y,legend_y,[-13,0.4])
% plot_local_detial ([0.2 0.4 0.4 0.2], t,e_q',[90 100])
% print('e_q_b0', '-depsc','-r600')

% label_y=["Output of RBFNNs [N]","Errors of RBFNNs [N]"];
% legend_y=["RBF_1","RBF_2","error_1","error_2"];
% plot_2line(t,TTa',ee_RBF','t [s]',label_y,legend_y,[-13,7.5;-13,7.5])
% plot_local_detial ([0.2 0.3 0.2 0.1], t,ee_RBF',[90 100])
% print('error_b0','-depsc','-r600')


last_10_seconds_e_q=e_q(:,length(t)-10000:length(t));
mean_e_q_last_10=mean (( last_10_seconds_e_q').^2);
max_eq_10= max(last_10_seconds_e_q')
last_10_seconds_ee_RBF=ee_Kr(:,length(t)-10000:length(t));
mean_ee_RBF_last_10=mean (( last_10_seconds_ee_RBF).^2,2);
max_eRBF_10= max(last_10_seconds_ee_RBF')


% last_20_seconds_e_q=e_q(:,200000-10000:200000);
% mean_e_q_last_20=mean (( last_20_seconds_e_q').^2)
% max_eq_20= max(last_20_seconds_e_q')
% last_20_seconds_ee_RBF=ee_RBF(:,200000-10000:200000);
% mean_ee_RBF_last_20=mean (( last_20_seconds_ee_RBF).^2,2)
% max_eRBF_20= max(last_20_seconds_ee_RBF')
%   
% figure
% plot(t,TTa(1,:),t,TTa(2,:))
% set (gca,'position',[0.1,0.1,0.8,0.8] );
% xlabel('t [s]'); ylabel('Output of RBFNN [N]','Position',[-13,7]);
%  legend('\tau_1','\tau_2')
%  print('Tau_b0','-depsc')
 
 
%  figure
%  plot(t,normS)
%  set (gca,'position',[0.1,0.1,0.8,0.8] );
%  legend('Norm S')
%  print('Norm_s0','-depsc')
 
 
 
 



% WW1=WW(Node-30:Node,1,:);
%  WW2=WW(Node-30:Node,2,:);
%  WW11=WW1(:,:);
%  WW22=WW2(:,:);
% 
% 
% label_y=["Evolving W_{700-729}";"Norm W"];
% legend_y=[""];
% plot_2line(t,WW11',Norm_W1','t [s]',label_y,legend_y,[-13,0.75;-13,7.5]);
% 
% ylabel("\textbf{Evolving $\hat{W}_{700-729}$}",'interpreter','latex')
% 
% % print('Norm_W1_b0','-depsc', '-r600')
% 
% 
% 
% label_y=["\textbf{Evolving $\hat{W}_{700-730}$}";"Norm W"];
% legend_y=[""];
% plot_2line(t,WW22',Norm_W2','t [s]',label_y,legend_y,[-13,0;-13,2]);
% 
% ylabel("\textbf{Evolving $\hat{W}_{700-729}$}",'interpreter','latex')
% % print('Norm_W2_b0','-depsc', '-r600')
% 

% figure;
% subplot(2,1,1)
% set (gca,'position',[0.1,0.55,0.8,0.4] )
% plot(t,Norm_W1')
% xlabel('t [s]'); ylabel('Norm W1','Position',[-13,5])
% subplot(2,1,2)
% set (gca,'position',[0.1,0.1,0.8,0.4] )
% plot(t,Norm_W2');xlabel('t [s]');ylabel('Norm W2','Position',[-13,2]) 
%  print('Norm_W_b0','-depsc')


 
% figure;
% plot(linspace(0,Time,STeps),eps');
% title('Adaptive Neural Netwok Control with the Full State Feedback');
% xlabel('t [s]'); ylabel('Approximation errors');% approximation error btw Neural network and the model
% 
% figure;plot(tout,eout); title('Adaptive Neural Netwok Control with the Full State Feedback');
% xlabel('t [s]'); ylabel('Norm of errors ||z_1||');   % Norm Errors
 save("model_based_feedforward", 'q', 'e_q','Tau')

