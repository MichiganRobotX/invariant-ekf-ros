filename = 'result/data_base_xyz_vrx.csv';
base = csvread(filename, 1, 0);
filename = 'result/data_odo_xyz_vrx.csv';
odo = csvread(filename, 1, 0);
filename = 'result/data_filtered_xyz_vrx.csv';
filtered = csvread(filename, 1, 0);
fsize = 12;
figure; hold on, grid on
xlabel('x', 'fontsize', fsize, 'Interpreter', 'latex')
ylabel('y', 'fontsize', fsize, 'Interpreter', 'latex')
zlabel('z', 'fontsize', fsize, 'Interpreter', 'latex')
plot3(base(:,2)-base(1,2), base(:,3)-base(1,3), base(:,4)-base(1,4));
hold on;
plot3(odo(:,2), odo(:,3), odo(:,4));
hold on;
plot3(filtered(:,2), filtered(:,3), filtered(:,4));
%legend('Transformed GPS position', 'Filtered position', 'location', 'best')
legend('Base position', 'Transformed GPS position', 'Filtered position', 'location', 'best')
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