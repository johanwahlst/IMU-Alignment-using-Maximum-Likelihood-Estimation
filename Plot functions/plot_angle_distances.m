function plot_angle_distances(Angles_distance,trip_lengths)

close all

for angle = 1:3

    figure('units','normalized','position',[0.5 0.33 0.423 0.32]);
    
    clf
    plot(trip_lengths/60,squeeze(mean(mean(mean(abs(Angles_distance(angle,:,:,:,:,1)),2),3),5))*180/pi,'Color',0.6*ones(3,1),'LineStyle', '--','Linewidth',4,'MarkerSize',6)
    hold on
    plot(trip_lengths/60,squeeze(mean(mean(mean(abs(Angles_distance(angle,:,:,:,:,2)),2),3),5))*180/pi,'Color',0.35*ones(3,1),'LineStyle', ':','Linewidth',2,'MarkerSize',6)
    plot(trip_lengths/60,squeeze(mean(mean(mean(abs(Angles_distance(angle,:,:,:,:,3)),2),3),5))*180/pi,'Color',0*ones(3,1),'LineStyle', '-','Linewidth',2,'MarkerSize',6)
    
    ylabel('ylabel')
    xlabel('xlabel')
     
    grid on
    
    xticks(0:5)

    leg_handle = legend('legendlegendleg1','legendlegendleg2','legendlegendleg3');
    set(leg_handle,'Position',[0.736 0.61 0.05 0.28])

    if(angle<3)
       axis([0.5 5 0 8])
    end

end

figure(1)
text(0.7, 7, 'tex', 'FontSize', 12);
figure(2)
text(0.7, 7, 'tex', 'FontSize', 12);
figure(3)
text(0.7, 52.5, 'tex', 'FontSize', 12);

end