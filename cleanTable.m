% Post simulation data. Note: files handled in CoppeliaSim.
% Function cleans the data in the force and time files.
function forceMat = cleanTable()

    timeTable = readtable('Time_Data.csv','ReadVariableNames',true);

    simTimeString = timeTable.x0_;
    simTime = str2double(simTimeString); % Simulation time

    % Replace NaN value at end of simulation time
    simTime(isnan(simTime)) = (simTime(end-1) + 0.05);

    dataTable = readtable('Force_Data.csv','PreserveVariableNames',true);

    % Extract force data from table
    forceTable = dataTable(:,1);
    x_forceString = forceTable(4:6:end,:);
    y_forceString = forceTable(5:6:end,:);
    z_forceString = forceTable(6:6:end,:);

    % Convert to array
    xArray = table2array(x_forceString);
    yArray = table2array(y_forceString);
    zArray = table2array(z_forceString);

    % Make sure arrays are same length
    simSteps = numel(zArray);
    xArray = xArray(1:simSteps);
    yArray = yArray(1:simSteps);

    % Convert string force data to numerical - 3 decimal place
    x_forceNumString = extractBetween(xArray,7,11);
    x_force = str2double(x_forceNumString);

    y_forceNumString = extractBetween(yArray,7,11);
    y_force = str2double(y_forceNumString);

    z_forceNumString = extractBetween(zArray,7,11);
    z_force = str2double(z_forceNumString);

    simTime(1,:) = 0.05;
    for i = 2:simSteps
        simTime(i,:) = 0.05+simTime(i-1,:);
    end

    vectorMat = [simTime x_force y_force z_force];

    for i = 1:length(vectorMat)
        forceMag(i) = sqrt(vectorMat(i,2)^2 + vectorMat(i,3)^2 + vectorMat(i,4)^2);
    end

    forceMat = [vectorMat(:,1) forceMag'];

    plot(forceMat(:,1), forceMat(:,2));
    ylabel('Force (N)');
    xlabel('Simulation Time (s)');
    
    addToFile(forceMat);
end