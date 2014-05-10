function displayMPU6050(data)
	
	%period per sample
	period = 20*10^-3;

    samples = size(data, 1);
    samplesPerSec = 1 / period;
    sec = samples / samplesPerSec;
    
    acc_x(1,:) = data(:, 1);
    acc_y(1,:) = data(:, 2);
    acc_z(1,:) = data(:, 3);

    gyro_x(1,:) = data(:, 4);
    gyro_y(1,:) = data(:, 5);
    gyro_z(1,:) = data(:, 6);
    
    

    t = 1:1/samplesPerSec:sec + 1;
    t(1) = [];
    
% ==== 2d plot ====    
    subplot(1, 2, 1);
    plot(t, acc_x, t, acc_y, t, acc_z);
    xlabel('Time [s]');
    ylabel('Magnitude');
    title('Accelerometer (x, y, z)');
    
    subplot(1, 2, 2);
    plot(t, gyro_x, t, gyro_y, t, gyro_z);
    xlabel('Time [s]');
    ylabel('Magnitude');
    title('Gyrometer (x, y, z)');
%==================
%     for i = 1:samples
%         clf;
%         hold on;
%         plot3(acc_x(i), acc_y(i), acc_z(i), 'o');
%         plot3(0, 0, 0, 'x');
%         view(90, 90);
%       %  hold off;
%        % Frames(:, i) = getframe;
%        % pause(0.1);
%         pause(1/samplesPerSec);
%     end
  %  movie(Frames, 1);
end