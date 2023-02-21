% Load Euler angle data

data = load('FILENAME.ext'); % Put file name in FILENAME.ext
% 나중에 real time으로 바꿔주어야 함

% Set up plot
figure;
axis([-1 1 -1 1 -1 1]); % Set the limits of the plot
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

% Loop over each frame and update the plot
for i = 1:size(data, 1)
    % Extract Euler angles for this frame
    roll = data(i, 1);
    pitch = data(i, 2);
    yaw = data(i, 3);

    % Compute rotation matrix
    R = rotationMatrix(roll, pitch, yaw);

    % Define unit vectors in the direction of the x, y, and z axes
    x_axis = [1 0 0];
    y_axis = [0 1 0];
    z_axis = [0 0 1];

    % Rotate each axis vector by the rotation matrix to get the new directions of the axes
    new_x_axis = R * x_axis';
    new_y_axis = R * y_axis';
    new_z_axis = R * z_axis';

    % Update the plot with the new axis directions
    plot3([0 new_x_axis(1)], [0 new_x_axis(2)], [0 new_x_axis(3)], 'r-', 'LineWidth', 2);
    hold on;
    plot3([0 new_y_axis(1)], [0 new_y_axis(2)], [0 new_y_axis(3)], 'g-', 'LineWidth', 2);
    plot3([0 new_z_axis(1)], [0 new_z_axis(2)], [0 new_z_axis(3)], 'b-', 'LineWidth', 2);
    hold off;

    % Set fixed axis ranges
    axis([-1 1 -1 1 -1 1]);

    % Set origin at the center
    set(gca, 'DataAspectRatio', [1 1 1], 'PlotBoxAspectRatio', [1 1 1], 'XLim', [-1 1], 'YLim', [-1 1], 'ZLim', [-1 1]);


end

function R = rotationMatrix(roll, pitch, yaw)

    % Compute rotation matrix using roll-pitch-yaw convention (intrinsic rotations)
    R_roll = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R_pitch = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R_yaw = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = R_yaw * R_pitch * R_roll;
end