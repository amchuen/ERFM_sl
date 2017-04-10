function AERO = setup_aero(instruct, AEROmod)

% Generates important vehicle parameters needed for simulink calculations
% Variables are passed in a struct called AERO, which should be assigned to
% a AERO variable in workspace, which then the AERO_lib should be able to
% access and perform necessary aerodynamic calculations with.

% the "create" part should initialize general info about the geometry and mass

% the "update" part should then take the inputs and initialize the rest of
% the useable information useable by the solver



switch instruct

    case 'create'
        AERO = {};
        
        %% Nosecone properties
        % GENERAL
        AERO.NC.X_LE = 0;
        AERO.NC.SHAPE = 'OGIVE';
        AERO.NC.SHAPE_PARAM = 1.0;
        AERO.NC.RADIUS = 6.15/2; % in
        AERO.NC.LENGTH = 24; % in
        AERO.NC.DENSITY = 0.0467; % lbm/in^3
        AERO.NC.WALL_THICKNESS = 0.125; % in
        
        % SHOULDER
        AERO.NC.SHOULD_DIAM = 5.975; % in
        AERO.NC.SHOULD_LENGTH = 5.5; % in
        AERO.NC.SHOULD_THICKNESS = 0.125; % in
        
        %% AIRFRAME 1 - PAYLOAD FRAME
        % General
        AERO.AF(1).LENGTH = 37; % IN
        AERO.AF(1).OUTER_DIAM = 6.155; % IN
        AERO.AF(1).INNER_DIAM = 6.007; % IN
        AERO.AF(1).WALL_THICKNESS = 0.074; % IN
        AERO.AF(1).DENSITY = 0.123; % LBM/IN^3
        AERO.AF(1).X_LE = 24; 
        AERO.AF(1).X_TE = AERO.AF(1).X_LE+ AERO.AF(1).LENGTH;
        
        %% AIRFRAME 2 - MOTOR FRAME
        % General
        AERO.AF(2).LENGTH = 48; % IN
        AERO.AF(2).OUTER_DIAM = 6.155; % IN
        AERO.AF(2).INNER_DIAM = 6.007; % IN
        AERO.AF(2).WALL_THICKNESS = 0.074; % IN
        AERO.AF(2).DENSITY = 0.123; % LBM/IN^3
        AERO.AF(2).X_LE = AERO.AF(1).X_TE; 
        AERO.AF(2).X_TE = AERO.AF(1).X_LE+ AERO.AF(1).LENGTH;

        %% FIN SET
        % General
        AERO.FIN.BODY_DIAM = 6.15/12;
        AERO.FIN.X_LE = 94/12; % FT
        AERO.FIN.Y_LE = AERO.FIN.BODY_DIAM * 0.5;
        AERO.FIN.Z_LE = 0.0;
        AERO.FIN.COUNT = 4;
        AERO.FIN.LAMBDA_VEC = [0 90 180 270 360];
        AERO.FIN.C_ROOT = 13.5;
        AERO.FIN.C_TIP = 2.625;
        AERO.FIN.SPAN = 6.5;
        AERO.FIN.SWEEP = 9.625;
        AERO.FIN.S_AREA = 0.5*(AERO.FIN.C_ROOT + AERO.FIN.C_TIP)*AERO.FIN.SPAN;
        AERO.FIN.THICKNESS = 0.185;
        AERO.FIN.ROUNDED_LEADING_EDGE = 0; % square/rectangular edge chosen
        AERO.FIN.ROT_ANGLE = 0.0;
        
        for ii = 2:AERO.FIN(1).COUNT
            AERO.FIN(ii) = AERO.FIN(1);
            AERO.FIN(ii).ROT_ANGLE = ii * 2*pi/AERO.FIN(1).COUNT;
        end
        
        %% Air Properties
        
        AERO.GAS.GAMMA_VS_TEMP = [
            -40	1.401;...
            -20	1.401;...
            0	1.401;...
            10	1.401;...
            20	1.401;...
            30	1.401;...
            40	1.401;...
            50	1.401;...
            60	1.401;...
            70	1.401;...
            80	1.400;...
            90	1.400;...
            100	1.400;...
            120	1.400;...
            140	1.399;...
            160	1.399;...
            180	1.399;...
            200	1.398;...
            300	1.394;...
            400	1.389;...
            500	1.383;...
            750	1.367;...
            1000	1.351;...
            1500	1.329];
        
    case 'update'
        AERO = AEROmod;
        
        %% NOSECONE
        % GENERAL
        AERO.NC.RHO_TAN = (AERO.NC.RADIUS^2 + AERO.NC.LENGTH^2)/(2*AERO.NC.RADIUS);
        AERO.NC.RHO_CURVE = (AERO.NC.RADIUS^2 + AERO.NC.LENGTH^2)*(((2-AERO.NC.SHAPE_PARAM)*AERO.NC.LENGTH)^2 + (AERO.NC.SHAPE_PARAM*AERO.NC.RADIUS)^2)/(4*(AERO.NC.SHAPE_PARAM * AERO.NC.RADIUS)^2);
        AERO.NC.GEOM_XVALS = linspace(0, AERO.NC.LENGTH, 1000);
        AERO.NC.GEOM_YVALS = sqrt(AERO.NC.RHO_CURVE^2 - (AERO.NC.LENGTH/AERO.NC.SHAPE_PARAM - AERO.NC.GEOM_XVALS).^2) - sqrt(AERO.NC.RHO_CURVE^2 - (AERO.NC.LENGTH/AERO.NC.SHAPE_PARAM)^2);
        AERO.NC.PHI = 0;
        
        % SHOULDER        
        AERO.NC.SHOULD_XVALS = [AERO.NC.LENGTH, AERO.NC.LENGTH + AERO.NC.SHOULD_LENGTH];
        AERO.NC.SHOULD_YVALS = [AERO.NC.SHOULD_DIAM, AERO.NC.SHOULD_DIAM];
        
        % CP calculations
        AERO.NC.X_LE = 0;
        AERO.NC.Y_LE = 0;
        AERO.NC.AREA_LE = 0;
        AERO.NC.X_TE = AERO.NC.LENGTH;
        AERO.NC.DIAM_TE = AERO.NC.GEOM_YVALS(end)*2;
        AERO.NC.AREA_TE = pi * AERO.NC.GEOM_YVALS(end)^2;
        AERO.NC.AREA_REF = AERO.NC.AREA_TE;
        AERO.NC.VOL = trapz(AERO.NC.GEOM_XVALS, AERO.NC.GEOM_YVALS);
        AERO.NC.X_CP = (AERO.NC.LENGTH * AERO.NC.AREA_TE - AERO.NC.VOL)/(AERO.NC.AREA_TE - AERO.NC.AREA_LE);
        
        % DRAG        
        NC_DRAG(AERO.NC.SHAPE_PARAM);
        
        % MASS AND MOMENT OF INERTIA
        AERO.NC.MASS =  2.92479; % lbm
        AERO.NC.CG = [17.37117, 0, 0]; % IN
        % inertial tensor is relative to the nosecone tip
        AERO.NC.INERTIA_TENSOR = [ %lbm * in^2
            1033.71,    0.00,	0.00;...
            0.00,   19.77,  0.00;...
            0.00,   0.00,   1033.71];
        

        %% AIRFRAME 1 - PAYLOAD FRAME        
        % CP CALC
        AERO.AF(1).AREA_LE = AERO.AF(1).OUTER_DIAM ^2 * 0.25 * pi;
        AERO.AF(1).AREA_TE = AERO.AF(1).AREA_LE;
        AERO.AF(1).XVALS = [AERO.AF(1).X_LE, AERO.AF(1).X_TE];
        AERO.AF(1).YVALS = [AERO.AF(1).OUTER_DIAM, AERO.AF(1).OUTER_DIAM];
        
        AERO.AF(1).AREA_PLANAR = AERO.AF(1).OUTER_DIAM * AERO.AF(1).LENGTH;
        AERO.AF(1).X_CP = AERO.AF(1).LENGTH^2 * AERO.AF(1).OUTER_DIAM*0.5 / AERO.AF(1).AREA_PLANAR;
        AERO.AF(1).BODY_RADIUS_AVG = 0.5*(AERO.NC.AREA_PLAN + AERO.AF(1).AREA_PLANAR) / (AERO.NC.X_TE + AERO.AF(1).LENGTH);
        AERO.AF(1).BODY_AREA_WET = AERO.NC.AREA_WET + pi*AERO.AF(1).OUTER_DIAM*AERO.AF(1).LENGTH;
        
        % DRAG
        
        % MASS AND MOMENT OF INERTIA
        AERO.AF(1).MASS = 6.43; %LBM
        AERO.AF(1).CG = AERO.AF(1).LENGTH *0.5 + AERO.AF(1).X_LE;
        AERO.AF(1).INERTIA_TENSOR = [ %lbm * in^2
            28.00259,    0.00,	0.00;...
            0.00,   5834.79129,  0.00;...
            0.00,   0.00,   5834.79129];
        
        
       %% AIRFRAME 2 - PAYLOAD FRAME        
        % CP CALC
        AERO.AF(2).AREA_LE = AERO.AF(2).OUTER_DIAM ^2 * 0.25 * pi;
        AERO.AF(2).AREA_TE = AERO.AF(2).AREA_LE;
        AERO.AF(2).XVALS = [AERO.AF(2).X_LE, AERO.AF(2).X_TE];
        AERO.AF(2).YVALS = [AERO.AF(2).OUTER_DIAM, AERO.AF(2).OUTER_DIAM];
        
        AERO.AF(2).AREA_PLANAR = AERO.AF(2).OUTER_DIAM * AERO.AF(2).LENGTH;
        AERO.AF(2).X_CP = AERO.AF(2).LENGTH^2 * AERO.AF(2).OUTER_DIAM*0.5 / AERO.AF(2).AREA_PLANAR;
        AERO.AF(2).BODY_RADIUS_AVG = 0.5*AERO.AF(2).OUTER_DIAM;%(AERO.NC.AREA_PLAN + AERO.AF(2).AREA_PLANAR) / (AERO.NC.TRAIL_EDGE + AERO.AF(2).LENGTH);
%         AERO.AF(2).BODY_AREA_WET = AERO.NC.AREA_WET + pi*AERO.AF(2).BODY_DIAM*AERO.AF(2).LENGTH;
        
        % DRAG
        
        % MASS AND MOMENT OF INERTIA
        AERO.AF(2).MASS = 6.43; %LBM
        AERO.AF(2).CG = AERO.AF(2).LENGTH *0.5 + AERO.AF(2).X_LE;
        AERO.AF(2).INERTIA_TENSOR = [ %lbm * in^2
            28.00259,    0.00,	0.00;...
            0.00,   5834.79129,  0.00;...
            0.00,   0.00,   5834.79129];
        
        
        %% FIN
        FIN_CP_CALC('Trapezoid');
        AERO.FIN.AREA_REF = (0.5 * AERO.FIN.BODY_DIAM)^2;
        AERO.FIN.AREA_REF_DRAG = AERO.FIN.COUNT * AERO.FIN.THICKNESS * AERO.FIN.SPAN;
        AERO.FIN.COS_GAMMA = cos(atan((AERO.FIN.SWEEP + 0.5*(AERO.FIN.C_TIP - AERO.FIN.C_ROOT))/AERO.FIN.SPAN));
        FIN_MACH_INTERP();
        AERO.FIN.INTERFERE_CORR = [1, 1.0;...
                                  2, 1.0;...
                                  3, 1.0;...
                                  4, 1.0;...
                                  5, 0.948;...
                                  6, 0.913;...
                                  7, 0.854;...
                                  8, 0.810;...
                                  9, 0.750];
end

% return AERO;

    function NC_DRAG(type)
        % creates nosecone variables needed for CD calculation
        % assumes nosecone subsonic pressure drag is zeroed out for now,
        % may need fixing later if time permits
        switch type
            case 'Conical'                
                % DRAG CHARACTERISTICS
                AERO.NC.FINENESS_RATIO = 0.5*DIAM / LENGTH;
                AERO.NC.CD_PRESSURE_SUBSONIC = 0;
                AERO.NC.CD_TRANSONIC = 1.0 * sin(atan(0.5*DIAM / LENGTH));
                AERO.NC.CD_SUPERSONIC = (2.1 / (1 + 4 * (0.5 * DIAM / LENGTH)^2)) + (0.5 / sqrt((1 + 4 * (0.5 * DIAM / LENGTH)^2)*(1.3^2 - 1)));
                AERO.NC.DERIV_CD_MA_TRANSONIC = (4/2.4)*(1-0.5*AERO.NC.CD_TRANSONIC);
                AERO.NC.CD_SUBSONIC_HIGH_PARAM_A = AERO.NC.CD_TRANSONIC - AERO.NC.CD_PRESSURE_SUBSONIC;
                AERO.NC.CD_SUBSONIC_HIGH_PARAM_B = AERO.NC.DERIV_CD_MA_TRANSONIC / AERO.NC.CD_SUBSONIC_HIGH_PARAM_A;
                TRANSONIC_MAT = [
                    1.3^2 1.3 1
                    1 1 1
                    2 1 0];
                TRANSONIC_B = [AERO.NC.CD_SUPERSONIC; AERO.NC.CD_TRANSONIC; AERO.NC.DERIV_CD_MA_TRANSONIC];
                AERO.NC.TRANSONIC_CONSTANTS = TRANSONIC_MAT\TRANSONIC_B;
                
                AERO.NC.TYPE = type;
                AERO.NC.CONE_CORRECTION = 1.0;

            case 'Tangent Ogive'                
                % DRAG CHARACTERISTICS
                AERO.NC.FINENESS_RATIO = 0.5*DIAM / LENGTH;
                AERO.NC.CD_PRESSURE_SUBSONIC = 0.8*(sin(AERO.NC.PHI))^2;
                
                AERO.NC.CD_TRANSONIC = 1.0 * sin(atan(0.5*DIAM / LENGTH));
                AERO.NC.DERIV_CD_MA_TRANSONIC = (4/2.4)*(1-0.5*AERO.NC.CD_TRANSONIC);
                
                AERO.NC.CD_SUBSONIC_HIGH_PARAM_A = AERO.NC.CD_TRANSONIC - AERO.NC.CD_PRESSURE_SUBSONIC;
                AERO.NC.CD_SUBSONIC_HIGH_PARAM_B = AERO.NC.DERIV_CD_MA_TRANSONIC / AERO.NC.CD_SUBSONIC_HIGH_PARAM_A;
                
                AERO.NC.CD_SUPERSONIC = (2.1 / (1 + 4 * (0.5 * DIAM / LENGTH)^2)) + (0.5 / sqrt((1 + 4 * (0.5 * DIAM / LENGTH)^2)*(1.3^2 - 1)));
                
                TRANSONIC_MAT = [
                    1.3^2 1.3 1
                    1 1 1
                    2 1 0];
                TRANSONIC_B = [AERO.NC.CD_SUPERSONIC; AERO.NC.CD_TRANSONIC; AERO.NC.DERIV_CD_MA_TRANSONIC];
                AERO.NC.TRANSONIC_CONSTANTS = TRANSONIC_MAT\TRANSONIC_B;
                
                AERO.NC.TYPE = type;
                AERO.NC.CONE_CORRECTION = 0.72 + 0.82;
                

            case 'Elliptical'
                RAD_CHAR = DIAM * 0.5;
%                 X_VAL = linspace(0, LENGTH, 1000);
%                 Y_VAL_FUNC = @(X) RAD_CHAR * sqrt(1 - (1 - (X/LENGTH))^2);
                AERO.NC.CD_PRESSURE_SUBSONIC = 0;
                AERO.NC.TYPE = type;

%             case 'Parabolic'
%                 RAD_CHAR = DIAM * 0.5;
%                 X_VAL = linspace(0, LENGTH, 1000);
%                 Y_VAL_FUNC = @(X) RAD_CHAR * sqrt(1 - (1 - (X/LENGTH))^2);
% 

        end
        
%         X_LE_AREA = 0;
%         TRAIL_EDGE_AREA = pi * Y_VAL_FUNC(X_VAL(end))^2;
%         AREA_FUNC = @(X) (Y_VAL_FUNC(X)).^2 * pi;
%         VOL = integral(AREA_FUNC, 0, X_VAL(end));
%         AERO.NC.X_CP = BODY_CP_CALC(LENGTH, TRAIL_EDGE_AREA, VOL, X_LE_AREA);
%         AERO.NC.AREA_REF = pi * (DIAM * 0.5)^2;
        AERO.NC.AREA_PLAN = 2 * trapz(AERO.NC.GEOM_XVALS, AERO.NC.GEOM_YVALS);
        AERO.NC.AREA_WET = pi * AERO.NC.AREA_PLAN;

    end

    function FIN_CP_CALC(type)
        switch type
            case 'Trapezoid'
               AERO.FIN.X_F_SUBSONIC = (AERO.FIN.SWEEP / 3) * ( AERO.FIN.C_ROOT + 2 * AERO.FIN.C_TIP)/(AERO.FIN.C_ROOT + AERO.FIN.C_TIP) + (1/6)*(AERO.FIN.C_ROOT^2 + AERO.FIN.C_TIP^2 + AERO.FIN.C_ROOT*AERO.FIN.C_TIP)/(AERO.FIN.C_ROOT + AERO.FIN.C_TIP);
               AERO.FIN.S_AREA = 0.5 * (AERO.FIN.C_ROOT + AERO.FIN.C_TIP) * AERO.FIN.SPAN;
               AERO.FIN.MAC = integral(@(x) (((AERO.FIN.C_ROOT - AERO.FIN.C_TIP)/(0 - AERO.FIN.SPAN))*(x)).^2, 0, AERO.FIN.SPAN)/AERO.FIN.AREA;
               AERO.FIN.AREA_WET_DOUBLE_TOTAL = AERO.FIN.AREA * 2 * AERO.FIN.COUNT;
               AERO.FIN.Y_MAC = AERO.FIN.SPAN/3 * (AERO.FIN.C_ROOT + 2*AERO.FIN.C_TIP)/(AERO.FIN.C_ROOT + AERO.FIN.C_TIP);
       end
        
    end

    function FIN_MACH_INTERP()
        Mgre2 = @(x) (2*AERO.FIN.SPAN^2 * sqrt(x^2 - 1)/AERO.FIN.AREA - 0.67)/(2 * (2 * AERO.FIN.SPAN^2 * sqrt(x^2 - 1))/AERO.FIN.AREA - 1);
        dx = 0.0005;
        deriv = (Mgre2(2+dx) - Mgre2(2-dx))/(2 * dx);
        AERO.FIN.p_arr = [0.25; 0; Mgre2(2); deriv; 0; 0];
        AERO.FIN.p_mat = [NTH_DERIV(0.5, 5, 0); NTH_DERIV(0.5, 5, 1); NTH_DERIV(2, 5, 0); NTH_DERIV(2, 5, 1); NTH_DERIV(2,5,2); NTH_DERIV(2, 5, 3)];
        AERO.FIN.CP_COEFFS = AERO.FIN.p_mat\AERO.FIN.p_arr;
    end

    function array = NTH_DERIV(val, order, deriv)
       power = (order:-1:0) - deriv;
       coeff = ones(1, order+1);
       for i = 1:deriv
            coeff = coeff .* ((order:-1:0) - (i-1));
       end
       array = ((val * ones(1, order+1)) .^ power).* coeff;
    end
end

