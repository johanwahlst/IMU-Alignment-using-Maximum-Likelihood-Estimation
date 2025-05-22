function plot_noise_and_bias(Geodesic_distance,Param_values)

close all

for l = 1:4

    figure('units','normalized','position',[0.3 0.33 0.423 0.35]);
    
    hor_axes = zeros(Param_values,1);
    
    for k = 1:Param_values
        params = parameters(k,l,Param_values);
        if(l==1)
            hor_axes(k) = params.accelerometer_noise;
        elseif(l==2)
            hor_axes(k) = params.gyroscope_noise*180/pi;
        elseif(l==3)
            hor_axes(k) = params.accelerometer_bias;
        elseif(l==4)
            hor_axes(k) = params.gyroscope_bias*180/pi;
        end
    end

    semilogx(hor_axes,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,l,:,1),1),2),5))*180/pi,'Color',0.6*ones(3,1),'LineStyle', '--','Linewidth',4,'MarkerSize',6)    
    hold on
    semilogx(hor_axes,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,l,:,2),1),2),5))*180/pi,'Color',0.35*ones(3,1),'LineStyle', ':','Linewidth',2,'MarkerSize',6)    
    semilogx(hor_axes,squeeze(mean(mean(mean(Geodesic_distance(:,:,:,l,:,3),1),2),5))*180/pi,'Color',0*ones(3,1),'LineStyle', '-','Linewidth',2,'MarkerSize',6)
        
    ylabel('ylabel')
    xlabel('xlabel')
     
    grid on
    axis([hor_axes(1)-0.00001 hor_axes(end) 0 8])
    
    leg_handle = legend('legendlegendleg1','legendlegendleg2','legendlegendleg3');
    set(leg_handle,'Position',[0.736 0.2 0.05 0.28])
    
    if(hor_axes(1)>0.1)
        text(hor_axes(1)+0.085, 7, 'tex', 'FontSize', 12);
    else
        text(hor_axes(1)+0.002, 7, 'tex', 'FontSize', 12);
    end

    xticks([0.001 0.01 0.1 1 10 100])

end

end