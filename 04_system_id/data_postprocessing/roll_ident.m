clc
clear all

roll_data = load("roll_ident_2.mat");
roll_data.markers.time(3) = roll_data.markers.time(3) + 0.94;
%% Ploting Roll and pwm during flight
figure("Name",'Roll angle during entire test')
subplot(2,1,1)
plot(roll_data.att.time, roll_data.att.roll); xlim([50 150])
xlabel('Time (s)');
ylabel('Roll Angle (degrees)');
grid on;
for i=1 : length(roll_data.markers.time)
    xline(roll_data.markers.time(i),'--m', num2str(i), 'LabelVerticalAlignment','bottom')
end
title('Roll angle during entire flight')
legend('Roll [deg]')

u_roll_raw = (roll_data.rcou.c3 + roll_data.rcou.c2) - (roll_data.rcou.c4 + roll_data.rcou.c1);
subplot(2,1,2)
plot(roll_data.rcou.time, u_roll_raw); xlim([50 150])
xlabel('Time (s)');
ylabel('Control Input (Right leftPWM - rightPWM) [us]');
grid on;
for i=1 : length(roll_data.markers.time)
    xline(roll_data.markers.time(i),'--m', num2str(i), 'LabelVerticalAlignment','bottom')
end
title('Roll angle during entire flight')
legend('PWM')
title('Control Input during entire flight');
%%
% Extracting time markers
roll_markers_str = string(roll_data.markers.text);

noise_p_start_ind = find(contains(roll_markers_str, 'MARKER: NOISE_1_START'));
noise_p_stop_ind = find(contains(roll_markers_str, 'MARKER: NOISE_1_STOP'));
noise_np_start_ind = find(contains(roll_markers_str, 'MARKER: NOISE_2_START'));
noise_np_stop_ind = find(contains(roll_markers_str, 'MARKER: NOISE_2_STOP'));

noise_p_start_t = roll_data.markers.time(noise_p_start_ind);
noise_p_stop_t = roll_data.markers.time(noise_p_stop_ind);
noise_np_start_t = roll_data.markers.time(noise_np_start_ind);
noise_np_stop_t = roll_data.markers.time(noise_np_stop_ind);

% Preparing equal grid time vectors for interpolation to overcome not even
% sampling periouds due to nondeterministic cycle time of process in Linux
% operating system

fs = 50;
t_grid_p = noise_p_start_t:1/fs:noise_p_stop_t;
t_grid_np = noise_np_start_t:1/fs:noise_np_stop_t;

% Detrending input of the system
pitch = interp1(roll_data.att.time, roll_data.att.pitch, roll_data.rcou.time, 'linear');
u_roll_pitch_detrended = double(u_roll_raw) .* cos(pitch);

u_roll_working_point_p = mean(u_roll_pitch_detrended(roll_data.rcou.time > noise_p_start_t - 5 & roll_data.rcou.time < noise_p_start_t));
u_roll_working_point_np = mean(u_roll_pitch_detrended(roll_data.rcou.time > noise_np_start_t - 5 & roll_data.rcou.time < noise_np_start_t));
disp(["Working point of drone (PWM) with payload:", num2str(u_roll_working_point_p)]);
disp(["Working point of drone (PWM) without payload:", num2str(u_roll_working_point_np)]);

u_roll_interp_p = interp1(roll_data.rcou.time, u_roll_pitch_detrended, t_grid_p, 'linear');
u_roll_interp_np = interp1(roll_data.rcou.time, u_roll_pitch_detrended, t_grid_np, 'linear');

figure('Name','Detrening veryfication');
plot(t_grid_p, u_roll_interp_p,'-r',roll_data.rcou.time, u_roll_pitch_detrended,'.g'); xlim([65 70]); grid on;
legend( 'Interpolated','Real data');
xlabel('Time [s]'); ylabel('PWM');

u_roll_detrended_p = u_roll_interp_p - u_roll_working_point_p;
u_roll_detrended_np = u_roll_interp_np - u_roll_working_point_np;

% Detrending output of the system
y_roll_working_point_p = mean(roll_data.att.roll(roll_data.att.time > (noise_p_start_t - 5) & roll_data.att.time < noise_p_start_t));
y_roll_working_point_np = mean(roll_data.att.roll(roll_data.att.time > (noise_np_start_t - 5) & roll_data.att.time < noise_np_start_t));
disp(["Working point of drone with payload:", num2str(y_roll_working_point_p)]);
disp(["Working point of drone without payload:", num2str(y_roll_working_point_np)]);

y_roll_interp_p = interp1(roll_data.att.time, roll_data.att.roll, t_grid_p, 'linear');
y_roll_interp_np = interp1(roll_data.att.time, roll_data.att.roll, t_grid_np, 'linear');

y_roll_detrended_p = y_roll_interp_p - y_roll_working_point_p;
y_roll_detrended_np = y_roll_interp_np - y_roll_working_point_np;

figure('Name','Detrened data');
subplot(4,1,1);
plot(t_grid_p, u_roll_detrended_p); grid on;
xlabel('Time [s]'); ylabel('PWM');
title('Detrended imput MISO->SISO (With payload)')
axis tight;
subplot(4,1,2);
plot(t_grid_p, y_roll_detrended_p); grid on;
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Detrended output (With payload)')
axis tight;
subplot(4,1,3);
title('Detrended imput MISO->SISO (Without payload)')
plot(t_grid_np, u_roll_detrended_np); grid on;
xlabel('Time [s]'); ylabel('PWM');
axis tight;
subplot(4,1,4);
plot(t_grid_np, y_roll_detrended_np); grid on;
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Detrended output (Without payload)')
axis tight;

roll_data_p = iddata(y_roll_detrended_p', u_roll_detrended_p', 1/fs, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Altitude Deviation');
roll_data_np = iddata(y_roll_detrended_np', u_roll_detrended_np', 1/fs, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Altitude Deviation');
%% Armax system identyfication
best_fit_p = -Inf;
best_sys_p = [];
best_orders_p = [1 1 1 1];
disp('Searching optimal configuration for Payload Attached...');

for a=1:4
    for b=1:4
        for c=1:4 
            for k=1:2
                current_orders = [a b c k];
                try
                    opt = armaxOptions('Display','off');
                    temp_sys = armax(roll_data_p, current_orders, opt); 
                    [~, current_fit] = compare(roll_data_p, temp_sys);
                    if(current_fit > best_fit_p)
                        best_fit_p = current_fit;
                        best_sys_p = temp_sys;
                        best_orders_p = current_orders;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Attached [na nb nc nk]: ', num2str(best_orders_p)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_p), '%']);

best_fit_np = -Inf;
best_sys_np = [];
best_orders_np = [1 1 1 1];
disp('Searching optimal configuration for Payload Dettached...');

for a=1:4
    for b=1:4
        for c=1:4 
            for k=1:2
                current_orders = [a b c k];
                try
                    opt = armaxOptions('Display','off');
                    temp_sys = armax(roll_data_np, current_orders, opt); 
                    [~, current_fit] = compare(roll_data_np, temp_sys);
                    if(current_fit > best_fit_np)
                        best_fit_np = current_fit;
                        best_sys_np = temp_sys;
                        best_orders_np = current_orders;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Dettached [na nb nc nk]: ', num2str(best_orders_np)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_np), '%']);

figure('Name', 'ARMAX Model Validation (Optimal Configurations)');
subplot(2,1,1);
compare(roll_data_p, best_sys_p);
title(['Simulation Fit - Noise 1 (Payload Attached) | Orders: ', num2str(best_orders_p)]);
subplot(2,1,2);
compare(roll_data_np, best_sys_np);
title(['Simulation Fit - Noise 2 (Payload Detached) | Orders: ', num2str(best_orders_np)]);

% RESIDUAL ANALYSIS
figure('Name', 'Residual Analysis - Optimal Noise 1');
resid(roll_data_p, best_sys_p);
title('Residual Analysis - Optimal Noise 1 (Payload Attached)');

figure('Name', 'Residual Analysis - Optimal Noise 2');
resid(roll_data_np, best_sys_np);
title('Residual Analysis - Optimal Noise 2 (Payload Detached)');

%% IV4 System Identification (Roll Angle)
disp('========================================================');
disp('STARTING IDENTIFICATION FOR ROLL ANGLE USING IV4');
disp('========================================================');

best_fit_iv4_p = -Inf;
best_sys_iv4_p = [];
best_orders_iv4_p = [1 1 1]; % Array format for IV4 algorithm: [na nb nk]

disp('Searching optimal IV4 configuration for Payload Attached...');
for a=1:5
    for b=1:5
        % Removed 'c' loop because iv4 algorithm does not model noise characteristics
        for k=1:3
            current_orders = [a b k];
            try
                % iv4: Computes the Instrumental Variable estimate for the system model.
                % Bypasses the closed-loop correlation issue between input and noise.
                temp_sys = iv4(roll_data_p, current_orders); 
                
                % compare: Simulates the model output and calculates the Fit percentage.
                [~, current_fit] = compare(roll_data_p, temp_sys);
                if(current_fit > best_fit_iv4_p)
                    best_fit_iv4_p = current_fit;
                    best_sys_iv4_p = temp_sys;
                    best_orders_iv4_p = current_orders;
                end
            catch
                continue;
            end
        end
    end
end
disp(['Best IV4 orders for Payload Attached [na nb nk]: ', num2str(best_orders_iv4_p)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_iv4_p), '%']);

best_fit_iv4_np = -Inf;
best_sys_iv4_np = [];
best_orders_iv4_np = [1 1 1]; % Array format for IV4 algorithm: [na nb nk]

disp('Searching optimal IV4 configuration for Payload Detached...');
for a=1:4
    for b=1:4
        for k=1:2
            current_orders = [a b k];
            try
                % iv4: Computes the Instrumental Variable estimate
                temp_sys = iv4(roll_data_np, current_orders); 
                [~, current_fit] = compare(roll_data_np, temp_sys);
                if(current_fit > best_fit_iv4_np)
                    best_fit_iv4_np = current_fit;
                    best_sys_iv4_np = temp_sys;
                    best_orders_iv4_np = current_orders;
                end
            catch
                continue;
            end
        end
    end
end
disp(['Best IV4 orders for Payload Detached [na nb nk]: ', num2str(best_orders_iv4_np)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_iv4_np), '%']);

%% Model Validation Plots for IV4 (Roll Angle)
figure('Name', 'IV4 Model Validation (Roll Angle)');
subplot(2,1,1);
compare(roll_data_p, best_sys_iv4_p);
title(['IV4 Roll Angle Fit - Noise 1 (Payload Attached) | Orders: ', num2str(best_orders_iv4_p)]);

subplot(2,1,2);
compare(roll_data_np, best_sys_iv4_np);
title(['IV4 Roll Angle Fit - Noise 2 (Payload Detached) | Orders: ', num2str(best_orders_iv4_np)]);


% RESIDUAL ANALYSIS FOR IV4
% resid: Computes and plots the autocorrelation of residuals and cross-correlation with input.
figure('Name', 'Residual Analysis - Roll Angle (IV4)');
subplot(2,1,1);
resid(roll_data_p, best_sys_iv4_p);
title('Residual Analysis - Roll Angle IV4 (Payload Attached)');

subplot(2,1,2);
resid(roll_data_np, best_sys_iv4_np);
title(['Residual Analysis -upel agh' ...
    ' Roll Angle IV4 (Payload Detached)']);

%% NARX system identification
best_fit_p = -Inf;
best_sys_p = [];
best_orders_p = [1 1 1];
best_neurons_p = 5;

disp('Searching optimal configuration for Payload Attached (NARX)...');
for a=1:4
    for b=1:4
        % The 'c' parameter is omitted because NARX uses [na nb nk] format
        for k=1:2
            % Added a loop to find the optimal number of hidden neurons
            for neurons = [5, 10, 15, 20] 
                current_orders = [a b k];
                try
                    % Configure training options to suppress terminal output
                    opt = nlarxOptions('Display','off');
                    
                    % Define the neural network estimator with current number of neurons
                    net_estimator = idSigmoidNetwork('NumberOfUnits', neurons);
                    
                    % Train the Nonlinear ARX model
                    temp_sys = nlarx(roll_data_p, current_orders, net_estimator, opt); 
                    
                    % Compare model simulation with real data to get Fit percentage
                    [~, current_fit] = compare(roll_data_p, temp_sys);
                    
                    % Save the model if it achieved the highest Fit so far
                    if(current_fit > best_fit_p)
                        best_fit_p = current_fit;
                        best_sys_p = temp_sys;
                        best_orders_p = current_orders;
                        best_neurons_p = neurons;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Attached [na nb nk]: ', num2str(best_orders_p)]);
disp(['Best number of neurons: ', num2str(best_neurons_p)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_p), '%']);

best_fit_np = -Inf;
best_sys_np = [];
best_orders_np = [1 1 1];
best_neurons_np = 5;

disp('Searching optimal configuration for Payload Detached (NARX)...');
for a=1:4
    for b=1:4
        % The 'c' parameter is omitted because NARX uses [na nb nk] format
        for k=1:2
            for neurons = [5, 10, 15, 20, 25, 30] 
                current_orders = [a b k];
                try
                    % Configure training options to suppress terminal output
                    opt = nlarxOptions('Display','off');
                    
                    % Define the neural network estimator with current number of neurons
                    net_estimator = idSigmoidNetwork('NumberOfUnits', neurons);
                    
                    % Train the Nonlinear ARX model
                    temp_sys = nlarx(roll_data_np, current_orders, net_estimator, opt); 
                    
                    % Compare model simulation with real data to get Fit percentage
                    [~, current_fit] = compare(roll_data_np, temp_sys);
                    
                    % Save the model if it achieved the highest Fit so far
                    if(current_fit > best_fit_np)
                        best_fit_np = current_fit;
                        best_sys_np = temp_sys;
                        best_orders_np = current_orders;
                        best_neurons_np = neurons;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Detached [na nb nk]: ', num2str(best_orders_np)]);
disp(['Best number of neurons: ', num2str(best_neurons_np)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_np), '%']);

% Model Validation Plots for NARX
figure('Name', 'NARX Model Validation (Optimal Configurations)');
subplot(2,1,1);
compare(roll_data_p, best_sys_p);
title(['Simulation Fit - Noise 1 (Payload Attached) | Orders: ', num2str(best_orders_p), ' | Neurons: ', num2str(best_neurons_p)]);

subplot(2,1,2);
compare(roll_data_np, best_sys_np);
title(['Simulation Fit - Noise 2 (Payload Detached) | Orders: ', num2str(best_orders_np), ' | Neurons: ', num2str(best_neurons_np)]);

% RESIDUAL ANALYSIS
figure('Name', 'Residual Analysis - Optimal Noise 1 (NARX)');
resid(roll_data_p, best_sys_p);
title('Residual Analysis - Optimal Noise 1 (Payload Attached)');

figure('Name', 'Residual Analysis - Optimal Noise 2 (NARX)');
resid(roll_data_np, best_sys_np);
title('Residual Analysis - Optimal Noise 2 (Payload Detached)');

%% System Identification - Roll Rate (Gyroscope Data) using NARX
disp('========================================================');
disp('STARTING NEURAL NETWORK (NARX) IDENTIFICATION FOR ROLL RATE');
disp('========================================================');

% Calculating working points 5 seconds before the noise injection
y_gyr_working_point_p = mean(roll_data.imu.gyrX(roll_data.imu.time > (noise_p_start_t - 5) & roll_data.imu.time < noise_p_start_t));
y_gyr_working_point_np = mean(roll_data.imu.gyrX(roll_data.imu.time > (noise_np_start_t - 5) & roll_data.imu.time < noise_np_start_t));

disp(["Working point of drone (Gyro) with payload:", num2str(y_gyr_working_point_p)]);
disp(["Working point of drone (Gyro) without payload:", num2str(y_gyr_working_point_np)]);

% Extract unique timestamps and their corresponding indices to prevent interp1 error
[unique_imu_time, unique_idx] = unique(roll_data.imu.time, 'stable');

% Extract gyroscope data strictly at those unique indices
unique_gyrX = roll_data.imu.gyrX(unique_idx);

% Interpolation to match the 50Hz grid using the purified arrays
y_gyr_interp_p = interp1(unique_imu_time, unique_gyrX, t_grid_p, 'linear');
y_gyr_interp_np = interp1(unique_imu_time, unique_gyrX, t_grid_np, 'linear');

% Detrending to ensure simulation starts around zero
y_gyr_detrended_p = y_gyr_interp_p - y_gyr_working_point_p;
y_gyr_detrended_np = y_gyr_interp_np - y_gyr_working_point_np;

% Creating IDDATA objects for Roll Rate (Ensuring vectors are transposed)
gyr_data_p = iddata(y_gyr_detrended_p', u_roll_detrended_p', 1/fs, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Roll Rate Deviation');
gyr_data_np = iddata(y_gyr_detrended_np', u_roll_detrended_np', 1/fs, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Roll Rate Deviation');

% NARX Grid Search for Roll Rate
best_fit_gyr_p = -Inf;
best_sys_gyr_p = [];
best_orders_gyr_p = [1 1 1];
best_neurons_gyr_p = 5;

disp('Searching optimal configuration for Roll Rate Payload Attached (NARX)...');
for a=1:4
    for b=1:4
        % The 'c' parameter is omitted because NARX uses [na nb nk] format
        for k=1:2
            % Loop to find the optimal number of hidden neurons
            for neurons = [5, 10, 15, 20] 
                current_orders = [a b k];
                try
                    % Configure training options to suppress terminal output
                    opt = nlarxOptions('Display','off');
                    
                    % Define the neural network estimator
                    net_estimator = idSigmoidNetwork('NumberOfUnits', neurons);
                    
                    % Train the Nonlinear ARX model
                    temp_sys = nlarx(gyr_data_p, current_orders, net_estimator, opt); 
                    
                    % Compare model simulation with real data to get Fit percentage
                    [~, current_fit] = compare(gyr_data_p, temp_sys);
                    
                    % Save the model if it achieved the highest Fit so far
                    if(current_fit > best_fit_gyr_p)
                        best_fit_gyr_p = current_fit;
                        best_sys_gyr_p = temp_sys;
                        best_orders_gyr_p = current_orders;
                        best_neurons_gyr_p = neurons;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Roll Rate Payload Attached [na nb nk]: ', num2str(best_orders_gyr_p)]);
disp(['Best number of neurons: ', num2str(best_neurons_gyr_p)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_gyr_p), '%']);

best_fit_gyr_np = -Inf;
best_sys_gyr_np = [];
best_orders_gyr_np = [1 1 1];
best_neurons_gyr_np = 5;

disp('Searching optimal configuration for Roll Rate Payload Detached (NARX)...');
for a=1:4
    for b=1:4
        % The 'c' parameter is omitted because NARX uses [na nb nk] format
        for k=1:2
            for neurons = [5, 10, 15, 20, 25, 30] 
                current_orders = [a b k];
                try
                    % Configure training options to suppress terminal output
                    opt = nlarxOptions('Display','off');
                    
                    % Define the neural network estimator
                    net_estimator = idSigmoidNetwork('NumberOfUnits', neurons);
                    
                    % Train the Nonlinear ARX model
                    temp_sys = nlarx(gyr_data_np, current_orders, net_estimator, opt); 
                    
                    % Compare model simulation with real data to get Fit percentage
                    [~, current_fit] = compare(gyr_data_np, temp_sys);
                    
                    % Save the model if it achieved the highest Fit so far
                    if(current_fit > best_fit_gyr_np)
                        best_fit_gyr_np = current_fit;
                        best_sys_gyr_np = temp_sys;
                        best_orders_gyr_np = current_orders;
                        best_neurons_gyr_np = neurons;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Roll Rate Payload Detached [na nb nk]: ', num2str(best_orders_gyr_np)]);
disp(['Best number of neurons: ', num2str(best_neurons_gyr_np)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_gyr_np), '%']);

%% Model Validation Plots for NARX (Gyroscope)
figure('Name', 'NARX Model Validation (Gyroscope / Roll Rate)');
subplot(2,1,1);
compare(gyr_data_p, best_sys_gyr_p);
title(['Roll Rate Fit - Noise 1 (Payload Attached) | Orders: ', num2str(best_orders_gyr_p), ' | Neurons: ', num2str(best_neurons_gyr_p)]);
axis tight;

subplot(2,1,2);
compare(gyr_data_np, best_sys_gyr_np);
title(['Roll Rate Fit - Noise 2 (Payload Detached) | Orders: ', num2str(best_orders_gyr_np), ' | Neurons: ', num2str(best_neurons_gyr_np)]);
axis tight;

% RESIDUAL ANALYSIS FOR NARX (Gyroscope)
figure('Name', 'Residual Analysis - Roll Rate (NARX)');
subplot(2,1,1);
resid(gyr_data_p, best_sys_gyr_p);
title('Residual Analysis - Roll Rate (Payload Attached)');

subplot(2,1,2);
resid(gyr_data_np, best_sys_gyr_np);
title('Residual Analysis - Roll Rate (Payload Detached)');