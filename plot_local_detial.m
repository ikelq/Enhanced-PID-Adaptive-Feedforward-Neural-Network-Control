function plot_local_detial (axes_position, x,y, xlimit)
axes('Position',axes_position,'fontsize', 12,'linewidth',0.5);
plot(x,y, 'linewidth',1)
xlim(xlimit)
set (gca,'position',axes_position,'fontsize', 12,'linewidth',0.5) 
end
