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
        
        % PROPELLANT PROPERTIES - PBAN APCP
        MOT.PROP_MASS_INIT = 3.908796; % lbm
        MOT.RHO_PROP = 0.064*(12^3); % lb/in^3 -> lb/ft^3 
        MOT.PROP_RAD_INIT = 0.25/12; % inch->ft, initial bore radius
        MOT.FLAME_TEMP = 5800; % FAHRENHEIT
        MOT.BURN_RATE = 0.55/12; %IN/S -> ft/s
        MOT.PROP_LENGTH = 18.0/12; %IN-> ft
        MOT.PROP_CASE_DISP = 1.40/12; % IN -> ft
        
        % CASE DATA
        MOT.CASE_LENGTH = 20.9055/12; %in -> ft
        MOT.CASE_DIAM = 2.95276/12; % in -> ft
        MOT.CASE_MASS = 4.071938; % lbm
        MOT.CASE_CG = 8.9046/12; % IN -> ft
        MOT.THROAT_AREA = (0.5/12)^2 * pi ; % in^2 -> ft^2
        MOT.EXHAUST_AREA = (2.75*0.5/12)^2 * pi; % IN^2 -> ft^2
        MOT.CASE_LEAD_EDGE = 88.54744/12; %IN -> ft
        
        % Thrust Properties
        MOT.TOTAL_IMPULSE_SI = 3743.3947; % Ns
        MOT.THRUST_TABLE_SI = [ % N vs s
            0.0     0.0
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
        
        MOT.VEC_BODY = [1.0, 0.0, 0.0]; % THRUST IN THE X DIRECTION 
        MOT.X_THR = MOT.CASE_LENGTH;
        MOT.Y_THR = 0;
        MOT.Z_THR = 0;
    case 'update'
        MOT = MOTmod;
        
        % Thrust Properties update
        MOT.THRUST_TABLE = MOT.THRUST_TABLE_SI;
        MOT.THRUST_TABLE(:,2) = SI_TO_ENG(MOT.THRUST_TABLE_SI(:,2), 'Newtons', 'lbf');
        MOT.TOTAL_IMPULSE = SI_TO_ENG(MOT.TOTAL_IMPULSE_SI, 'Newtons', 'lbf');
        MOT.ISP = MOT.TOTAL_IMPULSE / (MOT.PROP_MASS_INIT);
        MOT.THRUST_TABLE_X = MOT.THRUST_TABLE(:,1);
        MOT.THRUST_TABLE_Y = MOT.THRUST_TABLE(:,2);
end

    function val_eng = SI_TO_ENG(val_si, si_str, eng_str)
        switch si_str
            case 'Newtons'
                if strcmp(eng_str, 'lbf')
                    cf = 0.224809;
                end
        end
        val_eng = cf * val_si;
    end

end