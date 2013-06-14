%% Configuration
function main
clear all;

multithread = 0; % Short experiments should not use multithread, long ones benefit
Filename = 'rleg_47.mat';

fprintf('\n');
disp('ANU Datalogger Processor');
fprintf('Filename: %s\n', Filename);
if multithread == 1
    disp('Multithread:                           enabled');
else
    disp('Multithread:                           disabled');
end
fprintf('\n');

% Multithread
% Garbage collection
java.lang.System.gc()

if(matlabpool('size') > 0)
    matlabpool close
    pause(3);
end

%% Check enabled variables

twindow=10;
sign=0;
enabled_vars = ReadGMatlabDataFile('enabled_vars', Filename);

if enabled_vars(1) == 1
    disp('Time datalogging:                      enabled');
else
    disp('Time datalogging:                      disabled');
end

if enabled_vars(2) == 1
    disp('Execution times datalogging:           enabled');
else
    disp('Execution times datalogging:           disabled');
end

if enabled_vars(3) == 1
    disp('Raw IMU datalogging:                   enabled');
else
    disp('Raw IMU datalogging:                   disabled');
end

if enabled_vars(4) == 1
    disp('Input Voltage datalogging:            enabled');
else
    disp('Input Voltage datalogging:            disabled');
end

% if enabled_vars(5) == 1
%     disp('Output Voltage datalogging:                enabled');
% else
%     disp('Output Voltage datalogging:                disabled');
% end
disp('Output Voltage datalogging:                enabled');
fprintf('\n');

%% Process data

if(multithread == 1)
    % Start multithread
    matlabpool open 8
    spmd
        if(labindex == 1)
            if enabled_vars(1) == 1
                t_composite = ReadGMatlabDataFile('t', Filename);
                fprintf('Finisished reading t.\n');
                ts_composite = ReadGMatlabDataFile('Ts', Filename);
                fprintf('Finisished reading Ts.\n');
            end
        end
        
        if(labindex == 2)
            if enabled_vars(2) == 1
                t_control_exec_composite = ReadGMatlabDataFile('t_control_exec', Filename);
                fprintf('Finisished reading t_control_exec.\n');
                %t_ui_exec_composite = ReadGMatlabDataFile('t_ui_exec', Filename);
                %fprintf('Finisished reading t_ui_exec.\n');
            end
        end
                
        if(labindex == 3)
            if enabled_vars(3) == 1
                imu_accelerometer_x_raw_composite = ReadGMatlabDataFile('imu_accelerometer_x_raw', Filename);
                fprintf('Finisished reading imu_accelerometer_x_raw.\n');
                imu_accelerometer_y_raw_composite = ReadGMatlabDataFile('imu_accelerometer_y_raw', Filename);
                fprintf('Finisished reading imu_accelerometer_y_raw.\n');
                imu_accelerometer_z_raw_composite = ReadGMatlabDataFile('imu_accelerometer_z_raw', Filename);
                fprintf('Finisished reading imu_accelerometer_z_raw.\n');
                imu_magnetometer_x_raw_composite = ReadGMatlabDataFile('imu_magnetometer_x_raw', Filename);
                fprintf('Finisished reading imu_magnetometer_x_raw.\n');
                imu_magnetometer_y_raw_composite = ReadGMatlabDataFile('imu_magnetometer_y_raw', Filename);
                fprintf('Finisished reading imu_magnetometer_y_raw.\n');
                imu_magnetometer_z_raw_composite = ReadGMatlabDataFile('imu_magnetometer_z_raw', Filename);
                fprintf('Finisished reading imu_magnetometer_z_raw.\n');
                imu_gyrometer_x_raw_composite = ReadGMatlabDataFile('imu_gyrometer_x_raw', Filename);
                fprintf('Finisished reading imu_gyrometer_x_raw.\n');
                imu_gyrometer_y_raw_composite = ReadGMatlabDataFile('imu_gyrometer_y_raw', Filename);
                fprintf('Finisished reading imu_gyrometer_y_raw.\n');
                imu_gyrometer_z_raw_composite = ReadGMatlabDataFile('imu_gyrometer_z_raw', Filename);
                fprintf('Finisished reading imu_gyrometer_z_raw.\n');
                imu_valid_data_composite = ReadGMatlabDataFile('imu_valid_data', Filename);
                fprintf('Finisished reading imu_valid_data.\n');
            end
        end
        
        if(labindex == 4)
            if enabled_vars(4) == 1
                Vin0_composite = ReadGMatlabDataFile('Vin0', Filename);
                fprintf('Finisished reading Vin0.\n');
                Vin0_valid_data_composite = ReadGMatlabDataFile('Vin0_valid_data', Filename);
                fprintf('Finisished reading imu_valid_data.\n');
            end
            
        end
        
        if(labindex == 5)
%             if enabled_vars(5) == 1
%                 Out_composite = ReadGMatlabDataFile('Out', Filename);
%                 fprintf('Finisished reading Out.\n');
%                 OutRead_composite = ReadGMatlabDataFile('OutRead', Filename);
%                 fprintf('Finisished reading OutRead.\n');
%             end
             Out_composite = ReadGMatlabDataFile('Out', Filename);
                fprintf('Finisished reading Out.\n');
                OutRead_composite = ReadGMatlabDataFile('OutRead', Filename);
                fprintf('Finisished reading OutRead.\n');
        end    
    end
    
    if enabled_vars(1) == 1
        t = t_composite{1};
        Ts = ts_composite{1};
    end
    
    if enabled_vars(2) == 1
        t_control_exec = t_control_exec_composite{2};
        %t_ui_exec = t_ui_exec_composite{2};
    end
    
    if enabled_vars(3) == 1
        imu_accelerometer_x_raw = imu_accelerometer_x_raw_composite{6};
        imu_accelerometer_y_raw = imu_accelerometer_y_raw_composite{6};
        imu_accelerometer_z_raw = imu_accelerometer_z_raw_composite{6};
        imu_magnetometer_x_raw = imu_magnetometer_x_raw_composite{6};
        imu_magnetometer_y_raw = imu_magnetometer_y_raw_composite{6};
        imu_magnetometer_z_raw = imu_magnetometer_z_raw_composite{6};
        imu_gyrometer_x_raw = imu_gyrometer_x_raw_composite{6};
        imu_gyrometer_y_raw = imu_gyrometer_y_raw_composite{6};
        imu_gyrometer_z_raw = imu_gyrometer_z_raw_composite{6};
        imu_valid_data = imu_valid_data_composite{6};
    end
    
    if enabled_vars(4) == 1
        Vin0 = Vin0_composite{4};
        Vin0_valid_data = Vin0_valid_data_composite{4};
    end
    
%     if enabled_vars(5) == 1
%         Out = Out_composite{3};
%         OutRead = OutRead_composite{3};
%     end
 Out = Out_composite{3};
        OutRead = OutRead_composite{3};
    clear *_composite
    
    % End multithread
    matlabpool close
else % Not multithread
    if enabled_vars(1) == 1
        t = ReadGMatlabDataFile('t', Filename);
        Ts = ReadGMatlabDataFile('Ts', Filename);
    end
    
    if enabled_vars(2) == 1
        t_control_exec = ReadGMatlabDataFile('t_control_exec', Filename);
        %t_ui_exec = ReadGMatlabDataFile('t_ui_exec', Filename);
    end
     
    if enabled_vars(3) == 1
        imu_accelerometer_x_raw = ReadGMatlabDataFile('imu_accelerometer_x_raw', Filename);
        imu_accelerometer_y_raw = ReadGMatlabDataFile('imu_accelerometer_y_raw', Filename);
        imu_accelerometer_z_raw = ReadGMatlabDataFile('imu_accelerometer_z_raw', Filename);
        imu_magnetometer_x_raw = ReadGMatlabDataFile('imu_magnetometer_x_raw', Filename);
        imu_magnetometer_y_raw = ReadGMatlabDataFile('imu_magnetometer_y_raw', Filename);
        imu_magnetometer_z_raw = ReadGMatlabDataFile('imu_magnetometer_z_raw', Filename);
        imu_gyrometer_x_raw = ReadGMatlabDataFile('imu_gyrometer_x_raw', Filename);
        imu_gyrometer_y_raw = ReadGMatlabDataFile('imu_gyrometer_y_raw', Filename);
        imu_gyrometer_z_raw = ReadGMatlabDataFile('imu_gyrometer_z_raw', Filename);
        imu_valid_data = ReadGMatlabDataFile('imu_valid_data', Filename);
        
        imu_accelerometer_x_g = ReadGMatlabDataFile('imu_accelerometer_x_g', Filename);
        imu_accelerometer_y_g = ReadGMatlabDataFile('imu_accelerometer_y_g', Filename);
        imu_accelerometer_z_g = ReadGMatlabDataFile('imu_accelerometer_z_g', Filename);
        imu_magnetometer_x_B = ReadGMatlabDataFile('imu_magnetometer_x_B', Filename);
        imu_magnetometer_y_B = ReadGMatlabDataFile('imu_magnetometer_y_B', Filename);
        imu_magnetometer_z_B = ReadGMatlabDataFile('imu_magnetometer_z_B', Filename);
        imu_gyrometer_x_rads = ReadGMatlabDataFile('imu_gyrometer_x_rads', Filename);
        imu_gyrometer_y_rads = ReadGMatlabDataFile('imu_gyrometer_y_rads', Filename);
        imu_gyrometer_z_rads = ReadGMatlabDataFile('imu_gyrometer_z_rads', Filename);
        imu_calibratede_valid_data = ReadGMatlabDataFile('imu_calibrated_valid_data', Filename);
    end
    
    if enabled_vars(4) == 1
        Vin0 = ReadGMatlabDataFile('Vin0', Filename);
        Vin0_valid_data = ReadGMatlabDataFile('Vin0_valid_data', Filename);
    end
    
%     if enabled_vars(5) == 1
%         Out = ReadGMatlabDataFile('Out', Filename);
%         OutRead = ReadGMatlabDataFile('OutRead', Filename);
%     end
v_ctl = ReadGMatlabDataFile('v_ctl_read', Filename);
        v_ctl_read = ReadGMatlabDataFile('v_ctl_read', Filename);
end


[~, filename_noext, ~] = fileparts(Filename);
clear Filename multithread

eval(sprintf('save(''%s_processed.mat'')', filename_noext));

if enabled_vars(3) == 1
            sign=1;
            figure(1);
            annotation('textbox', [0 0.9 1 0.1], 'String', 'IMU - Accelerometer - Raw measurements', 'EdgeColor', 'none', 'HorizontalAlignment', 'center')
            subplot(321); myplot(t,'s',imu_accelerometer_x_raw,'X','raw','b',twindow,sign);
            subplot(323); myplot(t,'s',imu_accelerometer_y_raw,'Y','raw','b',twindow,sign);
            subplot(325); myplot(t,'s',imu_accelerometer_z_raw,'Z','raw','b',twindow,sign);
            subplot(322); myplot(t,'s',imu_accelerometer_x_g,'X','g','r',twindow,sign);
            subplot(324); myplot(t,'s',imu_accelerometer_y_g,'Y','g','r',twindow,sign);
            subplot(326); myplot(t,'s',imu_accelerometer_z_g,'Z','g','r',twindow,sign);
            
            figure(2);
            annotation('textbox', [0 0.9 1 0.1], 'String', 'IMU - Magnetometer', 'EdgeColor', 'none', 'HorizontalAlignment', 'center')
            subplot(321); myplot(t,'s',imu_magnetometer_x_raw,'X','raw','b',twindow,sign);
            subplot(323); myplot(t,'s',imu_magnetometer_y_raw,'Y','raw','b',twindow,sign);
            subplot(325); myplot(t,'s',imu_magnetometer_z_raw,'Z','raw','b',twindow,sign);
            subplot(322); myplot(t,'s',imu_magnetometer_x_B,'X','B','r',twindow,sign);
            subplot(324); myplot(t,'s',imu_magnetometer_y_B,'Y','B','r',twindow,sign);
            subplot(326); myplot(t,'s',imu_magnetometer_z_B,'Z','B','r',twindow,sign);
            
            figure(3);
            annotation('textbox', [0 0.9 1 0.1], 'String', 'IMU - Gyrometer', 'EdgeColor', 'none', 'HorizontalAlignment', 'center')
            subplot(321); myplot(t,'s',imu_gyrometer_x_raw,'X','raw','b',twindow,sign);
            subplot(323); myplot(t,'s',imu_gyrometer_y_raw,'Y','raw','b',twindow,sign);
            subplot(325); myplot(t,'s',imu_gyrometer_z_raw,'Z','raw','b',twindow,sign);
            subplot(322); myplot(t,'s',imu_gyrometer_x_rads,'X','rads','r',twindow,sign);
            subplot(324); myplot(t,'s',imu_gyrometer_y_rads,'Y','rads','r',twindow,sign);
            subplot(326); myplot(t,'s',imu_gyrometer_z_rads,'Z','rads','r',twindow,sign);
            
        end
        if enabled_vars(4) == 1
            sign=0;
            figure(4);
            annotation('textbox', [0 0.9 1 0.1], 'String', 'In', 'EdgeColor', 'none', 'HorizontalAlignment', 'center')
            myplot(t,'s',Vin0,'Vin0','bits','b',twindow,sign);
            %subplot(211); myplot(t,t_unitstring,Vin0,'Vin0',Vin0_unitstring,'b',twindow);
            %subplot(212); myplot(t,t_unitstring,Vin1,'Vin1',Vin1_unitstring,'b',twindow);
        end
        if enabled_vars(5) == 1
            sign=0;
            figure(5);
            annotation('textbox', [0 0.9 1 0.1], 'String', 'Out', 'EdgeColor', 'none', 'HorizontalAlignment', 'center')
            subplot(211); myplot(t,'s',v_ctl,'Written','bits','b',twindow,sign);
            subplot(212); myplot(t,'s',v_ctl_read,'Read','bits','b',twindow,sign);
        end
        



%% Prepare for estimator processing

% Ts is ready
% t is ready
% ax = imu_accelerometer_x_ms2;
% ay = imu_accelerometer_y_ms2;
% az = imu_accelerometer_z_ms2;
% accelerometer_validmeasure = imu_calibrated_valid_accelerometer_data;
% local_gravity = local_gravity';
% wx = imu_gyrometer_x_rads;
% wy = imu_gyrometer_y_rads;
% wz = imu_gyrometer_z_rads;
% gyrometer_validmeasure = imu_calibrated_valid_gyrometer_data;
% mx = imu_magnetometer_x_uT;
% my = imu_magnetometer_y_uT;
% mz = imu_magnetometer_z_uT;
% magnetometer_validmeasure = imu_calibrated_valid_magnetometer_data;
% local_magnetic = local_magnetic';
% gps_x = gps_calibrated_position_x;
% gps_y = gps_calibrated_position_y;
% gps_z = gps_calibrated_position_z;
% gps_vx = gps_calibrated_velocity_x;
% gps_vy = gps_calibrated_velocity_y;
% gps_vz = gps_calibrated_velocity_z;
% gps_validpmeasure = gps_calibrated_valid_data;
% gps_validvmeasure = gps_calibrated_valid_data;
% % We don't use sonar for now
% sonar_range = zeros(length(t), 1);
% sonar_validmeasure = zeros(length(t), 1);
% 
% eval(sprintf('save(''%s_localization.mat'', ''Ts'', ''t'', ''ax'', ''ay'', ''az'', ''accelerometer_validmeasure'', ''local_gravity'', ''wx'', ''wy'', ''wz'', ''gyrometer_validmeasure'', ''mx'', ''my'', ''mz'', ''magnetometer_validmeasure'', ''local_magnetic'', ''gps_x'', ''gps_y'', ''gps_z'', ''gps_vx'', ''gps_vy'', ''gps_vz'', ''gps_validpmeasure'', ''gps_validvmeasure'', ''sonar_range'', ''sonar_validmeasure'')', filename_noext));

%% Cleanup

clear all
end
function myplot(t,tunit,var,varname,varunit,color,twindow, sign)

kmax = min([length(t),length(var)]);

if(t(kmax)>twindow)
    taxismin = t(kmax) - twindow;
    taxismax = t(kmax);
else
    taxismin = 0;
    taxismax = twindow;
end
plot(t(1:kmax),var(1:kmax),color);
%ax = axis; axis([taxismin taxismax ax(3) ax(4)]);
if(sign==0)
    axis([taxismin taxismax -1 4096]);
else
    axis([taxismin taxismax -2050 2050]);
end
xlabel(sprintf('t [%s]',tunit));
ylabel(sprintf('%s [%s]',varname,varunit));

return;
end