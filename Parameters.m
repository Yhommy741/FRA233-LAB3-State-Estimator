m = 0.3403;
k = 29.7438 ;
b = 0.05;
g = 9.81;

x = double(data.Data);
t = double(data.Time);

t = double(t(:));
x = double(x(:));
measured_ts = timeseries(x, t);