% 8-curve
T = 60;
t = 0:0.1:60;
x = 4*sin(2*pi/T*t);
y = 4*sin(4*pi/T*t);
z = cos(4*pi/T*t);

plot3(x,y,z);
grid on;
