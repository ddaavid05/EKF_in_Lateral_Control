% Kézi fájlválasztás a .mat fájlokhoz
[file1, path1] = uigetfile('*.mat', 'Select the first .mat file');
if isequal(file1, 0)
    disp('User selected Cancel');
    return;
else
    file1 = fullfile(path1, file1);
end

[file2, path2] = uigetfile('*.mat', 'Select the second .mat file');
if isequal(file2, 0)
    disp('User selected Cancel');
    return;
else
    file2 = fullfile(path2, file2);
end

% Betöltjük a .mat fájlokat struktúrákba
data1 = load(file1);
data2 = load(file2);

% Kinyerjük a szükséges változókat minden fájlból
x1 = data1.out.vehiclePosition.cgPosition.x.Data;
y1 = data1.out.vehiclePosition.cgPosition.y.Data;
steering_angle1 = data1.out.out_steering_angle.Data; % Feltételezzük, hogy itt van tárolva a kormányszög

x2 = data2.out.vehiclePosition.cgPosition.x.Data;
y2 = data2.out.vehiclePosition.cgPosition.y.Data;
steering_angle2 = data2.out.out_steering_angle.Data; % Feltételezzük, hogy itt van tárolva a kormányszög

% Számoljuk ki a kívánt út mentén megtett távolságot
distance_desired_path1 = cumsum([0; sqrt(diff(x1).^2 + diff(y1).^2)]);
distance_desired_path2 = cumsum([0; sqrt(diff(x2).^2 + diff(y2).^2)]);

% Számoljuk ki a merőleges távolságokat a kívánt útra az első fájl esetén
distances_to_path1 = zeros(size(x1));
last_valid_distance1 = 0;
valid_indices1 = false(size(x1)); % Inicializáljuk az érvényes indexek tömbjét

for i = 1:length(x1)
    % Megtaláljuk a legközelebbi pontot a kívánt útvonalra
    [~, closest_point_index1] = min(sqrt((x1 - x1(i)).^2 + (y1 - y1(i)).^2));
    
    % Számoljuk ki a kívánt útvonalra vetített távolságot
    projected_distance1 = distance_desired_path1(closest_point_index1);
    
    % Csak akkor vesszük figyelembe a pontot, ha a távolságkülönbség kevesebb, mint 10 méter
    if abs(projected_distance1 - last_valid_distance1) < 10
        distances_to_path1(i) = projected_distance1; % Itt zárójel kell
        last_valid_distance1 = projected_distance1;
        valid_indices1(i) = true; % Érvényesként jelöljük ezt az indexet
    else
        distances_to_path1(i) = NaN; % Jelöljük ezt a távolságot NaN-nel
    end
end

% Számoljuk ki a merőleges távolságokat a kívánt útra a második fájl esetén
distances_to_path2 = zeros(size(x2));
last_valid_distance2 = 0;
valid_indices2 = false(size(x2)); % Inicializáljuk az érvényes indexek tömbjét

for i = 1:length(x2)
    % Megtaláljuk a legközelebbi pontot a kívánt útvonalra
    [~, closest_point_index2] = min(sqrt((x2 - x2(i)).^2 + (y2 - y2(i)).^2));
    
    % Számoljuk ki a kívánt útvonalra vetített távolságot
    projected_distance2 = distance_desired_path2(closest_point_index2);
    
    % Csak akkor vesszük figyelembe a pontot, ha a távolságkülönbség kevesebb, mint 10 méter
    if abs(projected_distance2 - last_valid_distance2) < 10
        distances_to_path2(i) = projected_distance2; % Itt zárójel kell
        last_valid_distance2 = projected_distance2;
        valid_indices2(i) = true; % Érvényesként jelöljük ezt az indexet
    else
        distances_to_path2(i) = NaN; % Jelöljük ezt a távolságot NaN-nel
    end
end

% Szűrjük ki a NaN értékeket és ábrázoljuk a kormányszöget a megtett távolság függvényében
figure;
plot(distances_to_path1(valid_indices1), steering_angle1(valid_indices1), 'LineWidth', 2, 'DisplayName', 'File 1');
hold on;
plot(distances_to_path2(valid_indices2), steering_angle2(valid_indices2), 'LineWidth', 2, 'DisplayName', 'File 2');
hold off;
xlabel('Distance Traveled along Desired Path (m)');
ylabel('Steering Angle (degrees)');
title('Steering Angle vs. Distance Traveled (Combined)');
legend('show');
grid on;


% -----------------------------------------------------------------------------------------------
% Készítünk egy új diagramot a data1 alapján
% Open and read the text file
filename = 'C:\Users\Admin\Documents\Dávid\Egyetem\Mester\Diploma2\Matlab_representation\Measurements_RealCar\egyetem_latlon.txt'; % replace with your actual file path
data = readtable(filename);

% Extract the longitude and latitude values
longitude = data.Longitude;
latitude = data.Latitude;

% Convert WGS84 coordinates to UTM
[utm_x, utm_y, utm_zone] = wgs2utm(latitude, longitude);

% Origin coordinates in UTM
X_orig = 353582.57;
Y_orig = 5260202.74;

% Convert UTM coordinates to external coordinate system
x_new = utm_x - X_orig;
y_new = utm_y - Y_orig;

% Combine plots into one figure using data1
figure;
plot(x_new, y_new, 'LineWidth', 2, 'DisplayName', 'Desired path');
hold on;
plot(x1, y1, 'LineWidth', 2, 'DisplayName', 'Vehicle Position (data1)');
hold off;
xlabel('X');
ylabel('Y');
legend('show');
grid on;

% Készítünk egy új diagramot a data2 alapján
% Combine plots into one figure using data2
figure;
plot(x_new, y_new, 'LineWidth', 2, 'DisplayName', 'Desired path');
hold on;
plot(x2, y2, 'LineWidth', 2, 'DisplayName', 'Vehicle Position (data2)');
hold off;
xlabel('X');
ylabel('Y');
legend('show');
grid on;
