function addToFile(matrix)
    fileName = "TimeAndForceData_allSims.csv";
    checkEmpty = dir(fileName);
    if checkEmpty.bytes == 0
        writematrix(matrix,fileName);
    else
        dlmwrite(fileName, matrix, '-append','delimiter',' ', 'roffset', 1);
    end
end