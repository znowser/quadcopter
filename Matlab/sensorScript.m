
data = load('C:/Users/Erik/Desktop/moving32.m');
%120 sekunder långt
%lyft ca 20-30cm varande i 10s med start på 30s, 60s, 90s.
t = 120;


temp = transpose(data(:, 1));
press = transpose(data(:, 2));
height = transpose(data(:, 3));


interval = 0:t/size(temp, 2):t;
interval(end) = [];

plot(interval, height, interval, temp);
%average = mean(data);
%averageData = data - average;
%plot(data)
%variance = var(averageData);
%stddev = std(averageData);
%fprintf('variance: %.3f, standard deviation: %.3f \n', variance, stddev);