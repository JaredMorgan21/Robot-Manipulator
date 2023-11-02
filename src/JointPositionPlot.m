A = readmatrix('jointsAndTimestep.csv');
timestamp = A(:, 4);
joints = A(:, 1:3);

figure;

% Line Plot
subplot(2,1,1)

hold on
plot(timestamp, joints(:, 1));
plot(timestamp, joints(:, 2));
plot(timestamp, joints(:, 3));
hold off

title('Motion Profile for Joints, Position (Degrees) vs. Time (ms)');
xlabel('Time (ms)');
ylabel('Joint Position (Degrees)');
legend({'Joint 1','Joint 2', 'Joint 3'},'Location','southeast')

% Histogram
subplot(2,1,2)
differentialTimestamp = timestamp(2:end) - timestamp(1:end - 1)
hist(differentialTimestamp)

title('Timestamps Between Readings (ms)');
xlabel('Time (ms)');
ylabel('Frequency');

