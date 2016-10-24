%Define T, t
T = 60; %seconds
t = 0:1:30;

%Calculate the position
x=4*sind(2*pi*T*t);
y=4*sind(4*pi*T*t);
z=cosd(4*pi*T*t);

%Plot
figure
plot3(x,y,z);
