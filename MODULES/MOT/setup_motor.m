function MOT = setup_mot(instruct, MOTmod)
%% Notes

% This script preps the motor for the Simulink block.

% 7/1/2016 -- Created

% 7/9/2016 -- Converted into function, prepping file for future creation of
% control files, building is now done solely on Simulink


switch instruct
    case 'create'
        % Initialize MOT data
        MOT = {};
        MOT.NAME = 'L1520T';
        MOT.LENGTH_SI = 531.000; %mm
        MOT.PROP_MASS_INIT_SI = 1773.00; % GRAMS

        % Thrust vs Time Data Table
        MOT.THRUST_TABLE = [
            0.0110  1484.5530
            0.0550  1506.3040
            0.0890  1468.2390
            0.1550  1560.6840
            0.2000  1571.5590
            0.3000  1582.4350
            0.4000  1598.7490
            0.5000  1620.5010
            0.6000  1642.2520
            0.7000  1653.1280
            0.8000  1674.8800
            0.9000  1691.1940
            1.0000 1685.7560
            1.1000 1696.6320
            1.2000 1691.1940
            1.4000 1680.3180
            1.5000 1664.0040
            1.6000 1636.8140
            1.7000 1620.5010
            1.8000 1604.1870
            1.9000 1576.9970
            2.0000 1560.6840
            2.1000 1538.9320
            2.2000 1500.8660
            2.2270 1435.6110
            2.2500 1125.6500
            2.2900 832.0020
            2.3660 647.1130
            2.4000 456.7850
            2.4400 244.7060
            2.5000 59.8170
            2.6000 0.0000];
        
    case 'update'
       MOT = MOTmod; 
        
        
end


end