function plot_trip_length(Geodesic_distance,trip_lengths)

close all

figure('units','normalized','position',[0.6 0.33 0.423 0.35]);

clf
plot(trip_lengths/60,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,:,1),1),2),4))*180/pi,'Color',0.6*ones(3,1),'LineStyle', '--','Linewidth',4,'MarkerSize',6)
hold on
plot(trip_lengths/60,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,:,2),1),2),4))*180/pi,'Color',0.35*ones(3,1),'LineStyle', ':','Linewidth',2,'MarkerSize',6)
plot(trip_lengths/60,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,:,3),1),2),4))*180/pi,'Color',0*ones(3,1),'LineStyle', '-','Linewidth',2,'MarkerSize',6)

ylabel('ylabel')
xlabel('xlabel')
 
grid on
axis([0.5 5 0 60])

xticks(0:5)

leg_handle = legend('legendlegendleg1','legendlegendleg2','legendlegendleg3');
set(leg_handle,'Position',[0.736 0.61 0.05 0.28])

end