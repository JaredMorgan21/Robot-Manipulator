data = readmatrix('interpolateQuestion5Position2.csv');
nonInterpolatedData = readmatrix('interpolateQuestion5Position2NonInterpolated.csv');

timestamp1 = data(:, 4);
joints1 = data(:, 1:3);
timestamp2 = nonInterpolatedData(:, 4);
joints2 = nonInterpolatedData(:, 1:3);

hold on
plot(timestamp1, joints1(:,1));
plot(timestamp1, joints1(:,2));
plot(timestamp1, joints1(:,3));
plot(timestamp2, joints2(:,1));
plot(timestamp2, joints2(:,2));
plot(timestamp2, joints2(:,3));
hold off

title('Motion Profile for Joints, Position (Degrees) vs. Time (ms)');
xlabel('Time (ms)');
ylabel('Joint Position (Degrees)');
legend({'Joint 1 Run 1','Joint 2 Run 1', 'Joint 3 Run 1', ...
    'Joint 1 Run 2','Joint 2 Run 2', 'Joint 3 Run 2'},'Location','southeast')
