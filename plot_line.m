function plot_line(x,y,label_x,label_y,legend_y,ylabel_position)
figure
%set(gcf,'DefaultTextInterpreter','latex' );
set(gcf,'PaperUnits','inches');
set(gcf,'PaperPosition',[100 100 520 440]);
set(gcf,'PaperPositionMode','auto')

plot(x,y,'linewidth',1)
xlabel(label_x);
ylabel(label_y,'position',ylabel_position);
legend(legend_y)
%,'interpreter','latex'
set (gca,'position',[0.1,0.08,0.8,0.88],'fontsize', 8,'linewidth',0.5) 
set(gcf,'PaperPositionMode','auto');
end
