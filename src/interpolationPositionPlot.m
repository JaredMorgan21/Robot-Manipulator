firstData = readmatrix('interpolateJointsAndTime1.csv');
secondData = readmatrix('interpolateJointsAndTime2.csv');

% grabs data and cuts it off at 6 seconds
timestamp1 = firstData(:, 4);
joints1 = firstData(:, 1:3);
timestamp1 = timestamp1(timestamp1 < 6000);
joints1 = joints1(timestamp1 < 6000, :);

timestamp2 = secondData(:, 4);
joints2 = secondData(:, 1:3);
timestamp2 = timestamp2(timestamp1 < 1000);
joints2 = joints2(timestamp1 < 1000, :);

figure;

% Line Plot
subplot(2,1,1)

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

% Histogram
subplot(2,1,2)
differentialTimestamp = timestamp(2:end) - timestamp(1:end - 1)
hist(differentialTimestamp)

title('Timestamps Between Readings (ms)');
xlabel('Time (ms)');
ylabel('Frequency');

mean(differentialTimestamp)
median(differentialTimestamp)
max(differentialTimestamp)
min(differentialTimestamp)