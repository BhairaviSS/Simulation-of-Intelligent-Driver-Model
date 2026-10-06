# Simulation-of-Intelligent-Driver-Model


# Code For IDM Execution
%% 
clear;
clc;

% Initializing some road parameters
roadLength = 878;
roadWidth = 3.5;

% Creating the Scenario for visualization
scenario = drivingScenario;          % Crete a driving scenario which comes from autonomous driving toolbox
roadCenter = [0 0;roadLength 0];     % Define the road center points     
road(scenario,roadCenter,roadWidth); % Create the road in the scenario
plot(scenario,'Centerline','on');    % Plot the road with centreline
hold on;

% Input parameters
numVehicle = 20;
vehicleLength = 5;
vehicleWidth = 3;
stopLine = 138;
timeStep = 0.1;
totalTime = 120;

% IDM Model parameters
maxAcceleration = 1.0;
comDeceleration = 1.5;
desiredSpeed = 15;
accExponent = 4;
minimumGap = 2.0;
desiredTimeGap = 1.0;

% Statistical Data Storage
timeStat = [];
gapStat = [];
speedStat = [];
accStat = [];

% Initializing the Vehicle Stats
vehicleSpeed = zeros(numVehicle,1); 
vehicleAcc = zeros(numVehicle,1);

% Initializing the position of vehicle on road
vehicleXcoord = initialization1(vehicleLength, minimumGap,stopLine,numVehicle);

refTime = 0;

% Loop through each time Step

while refTime <= totalTime

    %% Step 1: Update the traffic statistics
    [timeStat,gapStat,speedStat,accStat] = trafficStatistics1(timeStat, gapStat, speedStat, accStat,...
                                                 vehicleSpeed, vehicleAcc, roadLength,...
                                                 vehicleLength, vehicleXcoord, refTime);

    %% Step 2: Intelligent Drivers Model (IDM)
    [vehicleXcoord, vehicleSpeed, vehicleAcc] = IDMmodel1(vehicleXcoord, vehicleSpeed,accExponent,desiredSpeed, maxAcceleration, comDeceleration, timeStep, minimumGap, vehicleLength, roadLength, numVehicle, desiredTimeGap);

    %% Step 3: Visualization of vehicle position
    visualization1(vehicleXcoord, vehicleLength, vehicleWidth, numVehicle);

    refTime = refTime + timeStep;  % update the reference time 
end

plotfunction1(timeStat, gapStat, speedStat, accStat);



# Function Used in IDM Model Execution

# Initialisation function
function vehicleXcoord = initialization1(vehicleLength, minimumGap,stopLine,numVehicle)
 % this function Helps to find the position of the vehicle on the road
 % INPUT
 % vehicleLength 
 % minimumGap
 % stopLine

 % OUTPUT
 % vehicleXcoord

 % Calculating the spacing of the vehicle
 spacing = vehicleLength +minimumGap;

 % Finding the position of the vehicles
 vehicleXcoord = stopLine - (0:(numVehicle-1))' * spacing;
end



# Traffic Statistics function
function [timeStat,gapStat,speedStat,accStat] = trafficStatistics1(timeStat, gapStat, speedStat, accStat,...
                                                 vehicleSpeed, vehicleAcc, roadLength,...
                                                 vehicleLength, vehicleXcoord, refTime)
% Function is used to calculate the traffic statistics 

% Append the time statistics
timeStat = [timeStat;refTime];

% Initializing the vehicle indices which will be used in the Simulation
vehicleIndices = [1,5,10,15,20];

% Calculate the position of the first vehicle with respect to the road end 
gaps = roadLength - vehicleXcoord(1);

% Loop is added so as to get the gaps
% Calculating the gap between the current vehicle and the vehicle ahead
 for i =2:numel(vehicleIndices)
     idx = vehicleIndices(i);  % Current vehicle (follower)
     previdx = idx -1;   % Previous vehicle (Leader)
     gap = vehicleXcoord(previdx) - vehicleLength - vehicleXcoord(idx);
     gaps = [gaps,gap];   % append the gaps
 end

 % Append the Gap statistics
 gapStat = [gapStat;gaps];

 % Append the Speed statistics
 speedStat = [speedStat;vehicleSpeed(vehicleIndices)'];


 % Append the Acceleration statistics
 accStat = [accStat;vehicleAcc(vehicleIndices)'];
end


# IDM model function
function [vehicleXcoord, vehicleSpeed, vehicleAcc] = IDMmodel1(vehicleXcoord, vehicleSpeed,accExponent,desiredSpeed,...
                                                     maxAcceleration, comDeceleration, timeStep, minimumGap,...
                                                     vehicleLength, roadLength, numVehicle, desiredTimeGap)
% assigning the acceleration component
Acc = zeros(numVehicle,1);

%% Step 1: Calculating the relative Speed and the distance gap
% For first vehicle, the relative speed will be the speed of the vehicle itself
% For other, the relative speed is the difference between the speed of
% leading vehicle and the following vehicle

relativeSpeed = [vehicleSpeed(1); diff(vehicleSpeed)];

% For first vehicle, the gap is the diatnce between the road length and
% length of first vehicle
% For other, the gap is the distance to the leading vehicle minus the
% vehicle length

distanceGap = [roadLength - vehicleXcoord(1);...
              vehicleXcoord(1:end-1) - vehicleLength - vehicleXcoord(2:end)];

%% Step 2: Calculating the dynamic distance and the desired distance

dynamicGap = (vehicleSpeed .* relativeSpeed) ./ (2 * sqrt(maxAcceleration .* comDeceleration));

safeDistance = vehicleSpeed .* desiredTimeGap;

desiredDistaceGap = minimumGap + max(0,safeDistance + dynamicGap);

%% Step 3: Calculating  Acceleration  at each step

Acc = maxAcceleration .* (1 - (vehicleSpeed ./ desiredSpeed).^ accExponent - (desiredDistaceGap ./ distanceGap).^2);

%% Step 4: Update the Speed and Acceleration
newSpeed = vehicleSpeed + Acc.* timeStep;

newSpeed(newSpeed < 0) = 0;

vehicleAcc = (newSpeed - vehicleSpeed) ./ timeStep;

%% Step 5: Update the vehicle position
vehicleXcoord = vehicleXcoord + ((newSpeed + vehicleSpeed) / 2) .* timeStep ;
vehicleSpeed = newSpeed;

end



# Visualization function
function visualization1(vehicleXcoord, vehicleLength, vehicleWidth, numVehicle)
%Input
%output
% none. the function directly creates and deletes a rectangules for
% visualization.
%Precompute rectnagle positions for all vehicles
xPosition = vehicleXcoord - vehicleLength;
yPosition = - vehicleWidth / 2;

%Create rectangles for all vehicles in one batch
for i = 1:numVehicle
    rectangle('Position', [xPosition(i), yPosition, vehicleLength,vehicleWidth],...
        'FaceColor',[1,1,0],'EdgeColor','none');  %Yellow Rectangle
end
%Ensure proper plot scaling
axis equal;
xlim([min(xPosition) - vehicleLength, max(vehicleXcoord) + vehicleLength]); % Set limit for the x-axis to fit vehicle length
ylim([-vehicleWidth, vehicleWidth]); % Set limit for the y-axis to fit vehicle width

% Enhance Visualization
grid on;
xlabel('Road Length (m)'); % label x axis
ylabel('Road Width (m)');  % label y axis
title('Vehicle Initialization');

pause(0.005);

% Remove the existing rectangle from the plot
rectangles = findobj(gca,'Type','rectangle');   % Find existing rectangle objects
delete(rectangles);  % Delete all rectangles efficiently

%hold on; % Keep the cureent plot for future update

end


# Plot function
function plotfunction1(timeStat, gapStat, speedStat, accStat)

%% --- GAP PLOT ---
figure(2);
plot(timeStat(:,1), gapStat(:,1),'r', ...
     timeStat(:,1), gapStat(:,2),'g', ...
     timeStat(:,1), gapStat(:,3),'b', ...
     timeStat(:,1), gapStat(:,4),'c', ...
     timeStat(:,1), gapStat(:,5),'m');

xlabel('Time (s)');
ylabel('Gap (m)');
ylim([0, 35]);
legend('Car 1','Car 2','Car 3','Car 4','Car 5');
title('Gap vs Time');

%% --- SPEED PLOT ---
figure(3);
plot(timeStat(:,1), speedStat(:,1),'r', ...
     timeStat(:,1), speedStat(:,2),'g', ...
     timeStat(:,1), speedStat(:,3),'b', ...
     timeStat(:,1), speedStat(:,4),'c', ...
     timeStat(:,1), speedStat(:,5),'m');

xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0, 15]);
legend('Car 1','Car 2','Car 3','Car 4','Car 5');
title('Speed vs Time');

%% --- ACCELERATION PLOT ---
figure(4);
plot(timeStat(:,1), accStat(:,1),'r', ...
     timeStat(:,1), accStat(:,2),'g', ...
     timeStat(:,1), accStat(:,3),'b', ...
     timeStat(:,1), accStat(:,4),'c', ...
     timeStat(:,1), accStat(:,5),'m');

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
ylim([-3, 3]);
legend('Car 1','Car 2','Car 3','Car 4','Car 5');
title('Acceleration vs Time');

end
