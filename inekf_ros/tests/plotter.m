filename = 'result/data_odo_xyz.csv';
odo = csvread(filename, 1, 0);
filename = 'result/data_filtered_xyz.csv';
filtered = csvread(filename, 1, 0);
fsize = 12;
figure; hold on, grid on
xlabel('x', 'fontsize', fsize, 'Interpreter', 'latex')
ylabel('y', 'fontsize', fsize, 'Interpreter', 'latex')
zlabel('z', 'fontsize', fsize, 'Interpreter', 'latex')
%plot(odo(:,2), odo(:,3));
plot3(odo(:,2), odo(:,3), odo(:,4));
hold on;
%plot(filtered(:,2), filtered(:,3));
plot3(filtered(:,2), filtered(:,3), filtered(:,4));
legend('Raw position', 'Filtered odometry', 'location', 'best')
set(gca, 'fontsize', fsize)

%{
R_earth = 6.371e6;
lat = odo(:,2) * pi / 180;
lon = odo(:,3) * pi / 180;
alt = odo(:,4);
r = 6371000 + alt;
z = r .* sin(lat);
q = r .* cos(lat);
x = q .* cos(lon);
y = q .* sin(lon);
figure(2);
plot3(x, y, z);
%}