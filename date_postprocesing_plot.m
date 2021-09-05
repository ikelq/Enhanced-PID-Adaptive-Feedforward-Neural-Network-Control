%% 
% Author: Qiong Liu,Dongyu Li,Shuzhi Sam Ge
% Email: liuqiong_yl@outlook.com
% Description: Adaptive Feedforward Neural Network Control with 
%              an Optimized Hidden Node Distribution
% plot the simulation results of four control schemes
% The technical details can be seen in the paper

% @ARTICLE{Qiong2021,  author={Q. {Liu} and D. {Li} and S. S. {Ge} and Z. {Ouyang}}, 
% journal={IEEE Transactions on Artificial Intelligence},   
% title={Adaptive Feedforward Neural Network Control with an Optimized Hidden Node Distribution},   
% year={2021},  volume={},  number={},  pages={1-1},  
% doi={10.1109/TAI.2021.3074106}}


clear
clc
close all
load("PID.mat")
q_PID=q;
e_q_PID=e_q;
Tau_PID=Tau;

load("model_based_feedforward.mat")
q_MB=q;
e_q_MB=e_q;
Tau_MB=Tau;

load("RBFNN_Lattice_hideen_node_3_6")
q_RBFL=q;
e_q_RBFL=e_q;
Tau_RBFL=Tau;

load("RBFNN_Optimized_hideen_node_20.mat")
q_RBF=q;
e_q_RBF=e_q;
Tau_RBF=Tau;
t1=1:20/size;
t2=1980/size:2000/size;

label_y=["Tracking [rad]","Tracking error [rad]"];
legend_y1=[ "RBFNN-O","RBFNN-L","MB","PID", "Desired"];
legend_y2=["RBFNN-O","RBFNN-L","MB","PID"];
plot_2line(t(t1),[ q(1,t1)',q_RBFL(1,t1)',q_MB(1,t1)',q_PID(1,t1)' , qr(1,t1)'],[e_q(1,t1)',e_q_RBFL(1,t1)',e_q_MB(1,t1)',e_q_PID(1,t1)'],'t [s]',...
    label_y,legend_y1,legend_y2,[-1.5,0;-1.5,0]);
plot_local_detial ([0.3 0.77 0.2 0.2],t(t1),[ q(1,t1)',q_RBFL(1,t1)',q_MB(1,t1)',q_PID(1,t1)' , qr(1,t1)'] ,[0 2]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Tracking_performance_11',...
%  '-depsc',  '-painters','-r600')


label_y=["Tracking [rad]","Tracking error  [rad]"];
legend_y1=[ "RBFNN-O","RBFNN-L","MB","PID", "Desired"];
legend_y2=["RBFNN-O","RBFNN-L","MB"];
plot_2line(t(t2),[ q(1,t2)',q_RBFL(1,t2)',q_MB(1,t2)',q_PID(1,t2)' , qr(1,t2)'], [e_q(1,t2)',e_q_RBFL(1,t2)', e_q_MB(1,t2)'],'t [s]',label_y,legend_y1,legend_y2,...
    [1978.5,0;1978.5,0]);
plot_local_detial ([0.3 0.77 0.2 0.2],t(t2),[ q(1,t2)',q_RBFL(1,t2)',q_MB(1,t2)',q_PID(1,t2)' , qr(1,t2)'] ,[1980 1982]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Tracking_performance_12',...
%  '-depsc',  '-painters','-r600')


label_y=["Tracking [rad]","Tracking error [rad]"];
legend_y1=[ "RBFNN-O","RBFNN-L","MB","PID", "Desired"];
legend_y2=["RBFNN-O","RBFNN-L","MB","PID"];
plot_2line(t(t1),[ q(2,t1)',q_RBFL(2,t1)',q_MB(2,t1)',q_PID(2,t1)' , qr(2,t1)'],[e_q(2,t1)',e_q_RBFL(2,t1)',e_q_MB(2,t1)',e_q_PID(2,t1)'],'t [s]',...
    label_y,legend_y1,legend_y2,[-1.5,0;-1.5,0.4]);
plot_local_detial ([0.3 0.77 0.2 0.2],t(t1),[ q(2,t1)',q_RBFL(2,t1)',q_MB(2,t1)',q_PID(2,t1)' , qr(2,t1)'] ,[0 2]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Tracking_performance_21',...
%  '-depsc', '-painters', '-r600')


label_y=["Tracking [rad]","Tracking error  [rad]"];
legend_y1=[ "RBFNN-O","RBFNN-L","MB","PID", "Desired"];
legend_y2=["RBFNN-O","RBFNN-L","MB"];
plot_2line(t(t2),[ q(2,t2)',q_RBFL(2,t2)',q_MB(2,t2)',q_PID(2,t2)' , qr(2,t2)'], [e_q(2,t2)',e_q_RBFL(2,t2)',e_q_MB(2,t2)'],'t [s]',label_y,legend_y1,legend_y2,...
    [1978.5,0;1978.5,0]);
plot_local_detial ([0.2 0.77 0.2 0.2],t(t2),[ q(2,t2)',q_RBFL(2,t2)',q_MB(2,t2)',q_PID(2,t2)' , qr(2,t2)'] ,[1982 1983]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Tracking_performance_22',...
%  '-depsc', '-painters', '-r600')

label_y=["Control output \tau_1 [N.m]","Control output \tau_2 [N.m]"];
legend_y1=["RBFNN-O","RBFNN-L","MB","PID"];
legend_y2=["RBFNN-O","RBFNN-L","MB","PID"];
plot_2line(t(t1), [Tau(1,t1)', Tau_RBFL(1,t1)', Tau_MB(1,t1)',Tau_PID(1,t1)'],   [Tau(2,t1)',Tau_RBFL(2,t1)', Tau_MB(2,t1)', Tau_PID(2,t1)'],'t [s]',label_y,legend_y1,legend_y2,...
   [-1,4;-1,0]);
plot_local_detial ([0.3 0.86 0.3 0.1],t(t1), [Tau(1,t1)', Tau_RBFL(1,t1)', Tau_MB(1,t1)',Tau_PID(1,t1)'] ,[0 2]);
plot_local_detial ([0.3 0.2 0.3 0.1],t(t1), [Tau(2,t1)', Tau_RBFL(2,t1)', Tau_MB(2,t1)',Tau_PID(2,t1)'] ,[0 2]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Control_output_1',...
%  '-depsc', '-painters', '-r600')


label_y=["Control output \tau_1 [N.m]","Control output \tau_2 [N.m]"];
legend_y1=["RBFNN-O","RBFNN-L","MB","PID"];
legend_y2=["RBFNN-O","RBFNN-L","MB","PID"];
plot_2line(t(t2), [Tau(1,t2)', Tau_RBFL(1,t2)',  Tau_MB(1,t2)',Tau_PID(1,t2)'],   [Tau(2,t2)', Tau_RBFL(2,t2)',  Tau_MB(2,t2)', Tau_PID(2,t2)'],'t [s]',label_y,legend_y1,legend_y2,...
    [1978.5,4.8;1978.5,0.75]);
plot_local_detial ([0.3 0.75 0.2 0.2],t(t2),  [Tau(1,t2)', Tau_RBFL(1,t2)',  Tau_MB(1,t2)',Tau_PID(1,t2)'],[1986 1988]);
plot_local_detial ([0.3 0.26 0.2 0.2],t(t2),  [Tau(2,t2)', Tau_RBFL(2,t2)',  Tau_MB(2,t2)',Tau_PID(2,t2)'],[1986 1988]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Control_output_2',...
%  '-depsc', '-painters', '-r600')





label_y=["RBFNN [N.m]","Error [N.m]","K_2e_2 [N.m]"];
legend_y1=["RBF_1","RBF_2"];
legend_y2=["error_1","error_2"];
legend_y3=["K_{21}e_{21}","K_{22}e_{21}"];
plot_3line(t(t1),TTa(:,t1)',ee_RBF(:,t1)',ee_Kr(:,t1)','t [s]',label_y,...
    legend_y1,legend_y2,legend_y3,[-1,3;-1,1;-1,1]);
% plot_local_detial ([0.5 0.12 0.2 0.1], t,ee_RBF',[90 100]);
% plot_local_detial ([0.5 0.52 0.2 0.1], t,ee_Kr',[90 100]);
print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Approximation_performance_1',...
 '-depsc',  '-painters','-r600')

label_y=["RBFNNs[N.m]","Errors [N.m]","K_2e_2  [N.m]"];
legend_y1=["RBF_1","RBF_2"];
legend_y2=["error_1","error_2"];
legend_y3=["K_{21}e_{21}","K_{22}e_{22}"];
plot_3line(t(t2),TTa(:,t2)',ee_RBF(:,t2)',ee_Kr(:,t2)','t [s]',...
    label_y,legend_y1,legend_y2,legend_y3,[1979,3;1979,0;1979,0]);
print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Approximation_performance_2',...
 '-depsc', '-painters', '-r600')


t1=1:100/size;
t2=1900/size:2000/size;
label_y=["Evolving W_1";"Evolving W_2"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t(t1),WW11(:,t1)',WW22(:,t1)','t [s]',label_y,legend_y1,legend_y2,[-6,1;-6,0.1]);
set(gca,'YLim',[-1 1])
set(gca,'YLim',[-1 4])
print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\weight1',...
 '-depsc', '-painters', '-r600')

label_y=["Evolving W_1";"Evolving W_2"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t(t2),WW11(:,t2)',WW22(:,t2)','t [s]',label_y,legend_y1,legend_y2,[1894,1.25;1894,0.2]);
set(gca,'YLim',[-1 1])
set(gca,'YLim',[-1 4])
print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\weight2',...
 '-depsc',  '-painters','-r600')







