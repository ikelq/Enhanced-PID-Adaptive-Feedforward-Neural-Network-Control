function plot_3line(x,y1,y2,y3,label_x,label_y,legend_y1,legend_y2,legend_y3,ylabel_position)
figure
set(gcf,'PaperUnits','inches','PaperPosition',[100 100 520 440],'PaperPositionMode','auto');
set(gcf)
subplot(3,1,1)
%set(gcf,'DefaultTextInterpreter','latex' )
plot(x,y1,'linewidth',1)
ylabel(label_y(1),'position',ylabel_position(1,:));
if legend_y1 ~= [""]
   legend(legend_y1) 
end
set (gca,'position',[0.1,0.757,0.8,0.23],'fontsize', 12,'linewidth',0.5) 
subplot(3,1,2)
plot(x,y2,'linewidth',1);
ylabel(label_y(2),'position',ylabel_position(2,:)) 
if legend_y2 ~= [""]
    %,'interpreter','latex'
   legend(legend_y2)
end
set (gca,'position',[0.1,0.44,0.8,0.23],'fontsize', 12,'linewidth',0.5) 
subplot(3,1,3)
plot(x,y3,'linewidth',1);
xlabel(label_x);
ylabel(label_y(3),'position',ylabel_position(3,:)) 
if legend_y3 ~= [""]
    %,'interpreter','latex'
   legend(legend_y3)
end

set (gca,'position',[0.1,0.12,0.8,0.23] ,'fontsize',12,'linewidth',0.5)
end
