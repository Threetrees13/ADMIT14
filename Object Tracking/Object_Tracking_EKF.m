% Load the trained object detector
load('trained_detector.mat');

x_position = 0;
y_position = 0;

% Initialize Kalman filter parameters
x = [x_position; y_position; 0]; % Initial state [x_position; y_position; theta]
P = eye(3);    % Initial covariance matrix

% Process noise covariance
Q = diag([0.1, 0.1, 0.05]);

% Measurement noise covariance
R = diag([1, 1]);

% Time step
dt = 1; % Adjust as needed

% Define the state transition function for the EKF
function x_next = stateTransitionFcn(x, u, dt)
    theta = x(3);
    v = u(1);
    omega = u(2);
    
    x_next = x + [v * cos(theta) * dt; 
                  v * sin(theta) * dt; 
                  omega * dt];
end

% Define the measurement function for the EKF
function z = measurementFcn(x)
    z = [x(1); x(2)];  % We measure x and y positions
end

% Define the Jacobian of the state transition function
function F = jacobianStateFcn(x, u, dt)
    theta = x(3);
    v = u(1);
    
    F = [1, 0, -v * sin(theta) * dt;
         0, 1,  v * cos(theta) * dt;
         0, 0, 1];
end

% Define the Jacobian of the measurement function
function H = jacobianMeasurementFcn(~)
    H = [1, 0, 0;
         0, 1, 0];
end

% Read video
V = VideoReader('B4K11-04-154219-154432.mp4');
outputVideoFile = 'Tracked_EKF.mp4';
videoWriter = VideoWriter(outputVideoFile, 'MPEG-4');
open(videoWriter);

trackedLocations = [];
kfInitialized = false;

while hasFrame(V)
    frame = readFrame(V);
    [bbox, score, label] = detect(trained_detector, frame, 'MiniBatchSize', 32);
    threshold = 0.5;
    idx = score >= threshold;
    bbox = bbox(idx, :);
    label = label(idx);
    
    if ~isempty(bbox)
        strongestBbox = selectStrongestBbox(bbox, score(idx), 'OverlapThreshold', 0.5);
        centroid = [strongestBbox(1) + strongestBbox(3) / 2, strongestBbox(2) + strongestBbox(4) / 2];
        
        if ~kfInitialized
            % Initialize Kalman filter with the centroid of the first detection
            x = [centroid, 0]';
            kfInitialized = true;
        else
            % EKF Prediction
            u = [1; 0.1]; % Example control inputs, should be estimated
            x_pred = stateTransitionFcn(x, u, dt);
            F = jacobianStateFcn(x, u, dt);
            P_pred = F * P * F' + Q;

            % EKF Update
            z = centroid';
            H = jacobianMeasurementFcn(x_pred);
            K = P_pred * H' / (H * P_pred * H' + R);
            x = x_pred + K * (z - measurementFcn(x_pred));
            P = (eye(3) - K * H) * P_pred;
        end

        % Save tracked location
        trackedLocation = x(1:2)';
        trackedLocations = [trackedLocations; trackedLocation];
        detectedFrame = insertShape(frame, 'FilledCircle', [trackedLocation, 5], 'Color', 'green', 'LineWidth', 2);
    else
        % Predict if there are no detections
        u = [1; 0.1]; % Example control inputs, should be estimated
        x_pred = stateTransitionFcn(x, u, dt);
        F = jacobianStateFcn(x, u, dt);
        P_pred = F * P * F' + Q;
        x = x_pred;
        P = P_pred;

        % Save predicted location
        trackedLocation = x(1:2)';
        trackedLocations = [trackedLocations; trackedLocation];
        detectedFrame = insertShape(frame, 'FilledCircle', [trackedLocation, 5], 'Color', 'blue', 'LineWidth', 2);
    end
    
    % Write frame to video
    writeVideo(videoWriter, detectedFrame);
end

% Close video writer
close(videoWriter);