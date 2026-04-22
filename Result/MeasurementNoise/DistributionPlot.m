% Define the filenames of your sensor data
fileNames = {'0Nut.mat', '1Nut.mat', '2Nut.mat', '3Nut.mat', '4Nut.mat', '5Nut.mat'};

% Create a new figure window, sized appropriately for 6 subplots
figure('Name', 'Ultrasonic Sensor Distributions', 'Position', [100, 100, 1200, 800]);

% Loop through each file to load and plot the data
for i = 1:length(fileNames)
    % 1. Load the data structure from the .mat file
    dataStruct = load(fileNames{i});
    
    % 2. Extract the timeseries object (gets the first variable in the file dynamically)
    varNames = fieldnames(dataStruct);
    tsObject = dataStruct.(varNames{1});
    
    % 3. Extract the actual distance measurements array and convert to double
    sensorData = double(tsObject.Data);
    
    % Clean data by removing any missing/NaN values just in case
    sensorData = sensorData(~isnan(sensorData));
    
    % 4. Calculate Mean (mu) and Standard Deviation (sigma)
    mu = mean(sensorData);
    sigma = std(sensorData);
    
    % 5. Target the specific subplot (2 rows, 3 columns, position i)
    subplot(2, 3, i);
    hold on;
    
    % Plot a normalized histogram of the raw data
    histogram(sensorData, 'Normalization', 'pdf', 'FaceColor', [0.7 0.8 1]);
    
    % Create x and y values for the theoretical normal distribution bell curve
    x_values = linspace(min(sensorData), max(sensorData), 1000);
    y_values = normpdf(x_values, mu, sigma);
    
    % Plot the bell curve overlay
    plot(x_values, y_values, 'r-', 'LineWidth', 2);
    
    % 6. Add formatting, titles, and labels
    title(sprintf('File: %s\nMean = %.2f, Std Dev = %.2f', fileNames{i}, mu, sigma), 'Interpreter', 'none');
    xlabel('Measurement [mm]');
    ylabel('Probability Density');
    legend('Data Histogram', 'Normal Fit', 'Location', 'best');
    grid on;
    hold off;
end