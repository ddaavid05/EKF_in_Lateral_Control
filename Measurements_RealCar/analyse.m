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

% Extract x and y values from the workspace variable
x = out.vehiclePosition.cgPosition.x.Data;
y = out.vehiclePosition.cgPosition.y.Data;

% Combine plots into one figure
figure;
plot(x_new, y_new, 'LineWidth', 2, 'DisplayName', 'Desired path');
hold on;
plot(x, y, 'LineWidth', 2, 'DisplayName', 'Vehicle Position');
hold off;
xlabel('X');
ylabel('Y');
legend('show');
grid on;


% --------------------------------------------------------------------------------------------------------
% Create a separate figure for steering angle vs. distance traveled

% Calculate the cumulative distance along the desired path
distance_desired_path = cumsum([0; sqrt(diff(x_new).^2 + diff(y_new).^2)]);

% Calculate the perpendicular distances to the desired path
distances_to_path = zeros(size(x));

% Extract steering angle data from the workspace variable
steering_angle = out.out_steering_angle.Data; % Assuming steering angle is stored here

last_valid_distance = 0;
valid_indices = false(size(x)); % Initialize the valid indices array

for i = 1:length(x)
    % Find the closest point on the desired path
    [~, closest_point_index] = min(sqrt((x_new - x(i)).^2 + (y_new - y(i)).^2));
    
    % Calculate the projected distance on the desired path
    projected_distance = distance_desired_path(closest_point_index);
    
    % Include point only if the distance difference is less than 10 meters
    if abs(projected_distance - last_valid_distance) < 10
        distances_to_path(i) = projected_distance;
        last_valid_distance = projected_distance;
        valid_indices(i) = true; % Mark this index as valid
    else
        distances_to_path(i) = NaN; % Mark this distance as NaN
    end
end

% Filter out NaN values and plot steering angle against distance traveled
figure;
plot(distances_to_path(valid_indices), steering_angle(valid_indices), 'LineWidth', 2);
xlabel('Distance Traveled along Desired Path (m)');
ylabel('Steering Angle (degrees)');
title('Steering Angle vs. Distance Traveled');
grid on;
