clear all; close all; clc;

d_plot = false; % true for 3d plot
robot_data_csv_name = 'pose_data.csv';
ref_data_csv_name = 'ref_data.csv';
vertices_csv_name = 'vertices.csv';

% Specify the path to your CSV file
robot_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', robot_data_csv_name);
ref_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', ref_data_csv_name);
vert_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', vertices_csv_name);

% Read the CSV file
robot_data = csvread(robot_csvFilePath, 1, 0); % Assuming the first row contains headers
ref_data = csvread(ref_csvFilePath, 1, 0);
vert_data = csvread(vert_csvFilePath, 1, 0);

% Extract x, y, and z values
robot_x = robot_data(:, 2);
robot_y = robot_data(:, 3);
robot_z = robot_data(:, 4);

% Extract x, y, and z values
ref_x = ref_data(:, 1);
ref_y = ref_data(:, 2);
ref_z = ref_data(:, 3);

% Extract x, y, and z values
vert_x = vert_data(:, 1);
vert_y = vert_data(:, 2);
vert_z = vert_data(:, 3);

% Create a 3D plot
if d_plot
    figure;
    plot3(robot_x, robot_y, robot_z, 'LineWidth', 0.1);
    hold on;
    plot3(ref_x, ref_y, ref_z, 'LineWidth', 0.1);
    hold on;
    scatter3(vert_x, vert_y, vert_z, 'LineWidth', 1);
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend('Robot Pose', 'Reference Pose', 'Vertices');
    title('Welding Trajectory');
else
    figure;
    plot(robot_x, robot_y, 'LineWidth', 0.1);
    hold on;
    %plot(ref_x, ref_y, 'LineWidth', 0.1);
    hold on;
    scatter(vert_x, vert_y,'LineWidth', 1);
    grid on;
    xlabel('X');
    ylabel('Y');
    legend('Robot Pose', 'Reference Pose', 'Vertices');
    title('Welding Trajectory');
end

frechet(robot_x,robot_y, ref_x,ref_y)
% Optionally, you can customize the plot appearance further
% For example:
% legend('Pose Data');
% axis equal; % Ensure equal scaling on all axes

% Save the plot if needed
% saveas(gcf, 'pose_data_plot.png'); % Change the file extension as needed