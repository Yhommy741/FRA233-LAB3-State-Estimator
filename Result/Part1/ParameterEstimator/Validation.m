% =========================================================================
% Cross-Validation Script  —  5x5  (Model x Test)  [FAST VERSION]
% Model : m*x_ddot + c*x_dot + k*x = 0
%
% Speed improvements over previous version:
%   1. ANALYTICAL solution replaces ode45 entirely  (exact, ~100x faster)
%   2. lsqnonlin (Levenberg-Marquardt) replaces fminsearch (~5x fewer evals)
%   3. Single smart initial guess from data  (no 5-start loop)
%
% Parameter assignment per cell (i=model, j=test):
%   m = m_all(j)   <-- test nut mass       (physically correct)
%   c = c_all(i)   <-- model nut damping
%   k = k_all(i)   <-- model nut stiffness
% =========================================================================

clear; clc; close all;

%% -----------------------------------------------------------------------
%% Global LaTeX Renderer & Academic Font Settings
%% -----------------------------------------------------------------------
set(groot, 'defaultTextInterpreter',              'latex');
set(groot, 'defaultAxesTickLabelInterpreter',     'latex');
set(groot, 'defaultLegendInterpreter',            'latex');
set(groot, 'defaultColorbarTickLabelInterpreter', 'latex');
set(groot, 'defaultAxesFontName',  'Times New Roman');
set(groot, 'defaultTextFontName',  'Times New Roman');
set(groot, 'defaultAxesFontSize',  9);
set(groot, 'defaultAxesLineWidth', 0.8);

%% --- Known Parameters ---
m_all = [0.1888, 0.2269, 0.2649, 0.3027, 0.3403];               % [kg]
c_all = [0.0458417, 0.033614, 0.0459097, 0.0477507, 0.0409283];  % [Ns/m]
k_all = [31.17, 30.535, 30.497, 29.886, 30.387];                 % [N/m]

num_nuts   = 5;
nut_labels = {'1 Nut','2 Nut','3 Nut','4 Nut','5 Nut'};

%% --- lsqnonlin Options (Levenberg-Marquardt) ---
lsq_options = optimoptions('lsqnonlin', ...
    'Algorithm',            'levenberg-marquardt', ...
    'Display',              'off', ...
    'FunctionTolerance',    1e-10, ...
    'StepTolerance',        1e-10, ...
    'MaxFunctionEvaluations', 2000, ...
    'MaxIterations',        500);

%% --- Load All Trial-1 Data ---
fprintf('Loading Trial-1 data...\n');
T1_time = cell(num_nuts, 1);
T1_data = cell(num_nuts, 1);

for j = 1:num_nuts
    filename = sprintf('%dNut_1.mat', j);
    if ~isfile(filename)
        error('File not found: %s', filename);
    end
    raw        = load(filename);
    T1_time{j} = double(raw.data.Time(:));
    T1_data{j} = double(raw.data.Data(:));
    fprintf('  Loaded %s  (%d samples)\n', filename, length(T1_time{j}));
end

%% --- Pre-allocate ---
CV_RMSE = zeros(num_nuts, num_nuts);

%% =========================================================================
%% MAIN COMPUTATION LOOP  (analytical solution — no ode45)
%% =========================================================================
fprintf('\nRunning 5x5 cross-validation...\n');
tic;

for i = 1:num_nuts          % Model nut (row): c, k
    c = c_all(i);
    k = k_all(i);

    for j = 1:num_nuts      % Test nut (col): m and data
        m = m_all(j);

        t_meas = T1_time{j};
        x_meas = T1_data{j};

        % Derived system constants
        omega_n = sqrt(k/m);              % natural frequency
        zeta    = c / (2*sqrt(m*k));      % damping ratio

        % Smart initial guess: from first two data points
        dt   = t_meas(2) - t_meas(1);
        x0_0 = x_meas(1);
        v0_0 = (x_meas(2) - x_meas(1)) / dt;

        % Residual function using analytical solution (no ode45)
        res_fun = @(ic) analytical_response(ic, t_meas, omega_n, zeta) - x_meas;

        % lsqnonlin: Levenberg-Marquardt
        ic_opt = lsqnonlin(res_fun, [x0_0; v0_0], [], [], lsq_options);

        % Final simulation & RMSE with optimal IC
        x_sim         = analytical_response(ic_opt, t_meas, omega_n, zeta);
        RMSE          = sqrt(mean((x_meas - x_sim).^2));
        CV_RMSE(i, j) = RMSE;

        fprintf('  Model %d Nut | Test %d Nut | RMSE = %.4e m\n', i, j, RMSE);
    end
end

elapsed = toc;
fprintf('\nTotal computation time: %.2f seconds\n', elapsed);

%% =========================================================================
%% FIGURE 1 : 5x5 Validation Plot Grid
%% =========================================================================
fig1 = figure('Name', 'Cross-Validation Response Plots', 'NumberTitle', 'off');
fig1.Position = [30 30 1500 960];

for i = 1:num_nuts
    c = c_all(i);
    k = k_all(i);

    for j = 1:num_nuts
        m = m_all(j);

        t_meas  = T1_time{j};
        x_meas  = T1_data{j};
        omega_n = sqrt(k/m);
        zeta    = c / (2*sqrt(m*k));

        % Re-estimate IC for plot (reuse computation)
        dt   = t_meas(2) - t_meas(1);
        x0_0 = x_meas(1);
        v0_0 = (x_meas(2) - x_meas(1)) / dt;
        res_fun = @(ic) analytical_response(ic, t_meas, omega_n, zeta) - x_meas;
        ic_opt  = lsqnonlin(res_fun, [x0_0; v0_0], [], [], lsq_options);
        x_sim   = analytical_response(ic_opt, t_meas, omega_n, zeta);

        %% Subplot
        ax = subplot(num_nuts, num_nuts, (i-1)*num_nuts + j);

        plot(t_meas, x_meas, '.', 'Color', [0.20 0.40 0.75], 'MarkerSize', 2);
        hold on;
        plot(t_meas, x_sim,  '-', 'Color', [0.85 0.15 0.10], 'LineWidth', 1.1);
        grid on; box on;

        ax.FontSize      = 6.5;
        ax.GridAlpha     = 0.25;
        ax.GridLineStyle = '--';
        ax.TickDir       = 'out';
        ax.TickLength    = [0.02 0.02];

        if i == 1
            title(['Test: ' nut_labels{j}], ...
                  'FontSize', 8, 'FontWeight', 'bold', 'Interpreter', 'latex');
        end
        if j == 1
            ylabel(['Model: ' nut_labels{i} newline '$x$ (m)'], ...
                   'FontSize', 7, 'FontWeight', 'bold', 'Interpreter', 'latex');
        end
        if i == num_nuts
            xlabel('$t$ (s)', 'FontSize', 7.5, 'Interpreter', 'latex');
        end

        xl = xlim; yl = ylim;
        text(xl(1)+0.03*(xl(2)-xl(1)), yl(2)-0.05*(yl(2)-yl(1)), ...
             sprintf('$\\mathrm{RMSE}=%.2e$ m', CV_RMSE(i,j)), ...
             'FontSize', 5.2, 'Color', [0.7 0 0], ...
             'VerticalAlignment', 'top', 'Interpreter', 'latex');
    end
end

annotation('textbox', [0.913 0.962 0.085 0.025], ...
           'String', '$-\!-$ Model', 'Color', [0.85 0.15 0.10], ...
           'EdgeColor', 'none', 'FontSize', 8, 'Interpreter', 'latex', ...
           'FontName', 'Times New Roman');
annotation('textbox', [0.913 0.938 0.085 0.025], ...
           'String', '$\cdot\!\cdot$ Measured', 'Color', [0.20 0.40 0.75], ...
           'EdgeColor', 'none', 'FontSize', 8, 'Interpreter', 'latex', ...
           'FontName', 'Times New Roman');

annotation('textbox', [0 0.965 1 0.03], ...
           'String',      'Cross-Validation of Free-Vibration Model Parameters', ...
           'Interpreter', 'latex', 'FontSize', 11, 'FontWeight', 'bold', ...
           'FontName',    'Times New Roman', ...
           'HorizontalAlignment', 'center', 'EdgeColor', 'none');
annotation('textbox', [0 0.945 1 0.025], ...
           'String',      'Row: Model Parameters $(c_i, k_i)$ \quad|\quad Column: Test Data $(m_j)$', ...
           'Interpreter', 'latex', 'FontSize', 9, ...
           'FontName',    'Times New Roman', ...
           'HorizontalAlignment', 'center', 'EdgeColor', 'none', ...
           'Color',       [0.3 0.3 0.3]);

%% =========================================================================
%% FIGURE 2 : 5x5 RMSE Heatmap  (Green = low, Red = high)
%% =========================================================================
fig2 = figure('Name', 'Cross-Validation RMSE Table', 'NumberTitle', 'off');
fig2.Position = [200 120 720 600];

n_colors     = 256;
green_to_red = [linspace(0,   1, n_colors)', ...
                linspace(0.7, 0, n_colors)', ...
                zeros(n_colors, 1)];

imagesc(log10(CV_RMSE));
colormap(green_to_red);

cb                      = colorbar;
cb.Label.String         = '$\log_{10}(\mathrm{RMSE})$ (m)';
cb.Label.Interpreter    = 'latex';
cb.Label.FontSize       = 11;
cb.TickLabelInterpreter = 'latex';

set(gca, 'XTick', 1:num_nuts, 'XTickLabel', nut_labels, ...
         'YTick', 1:num_nuts, 'YTickLabel', nut_labels, ...
         'XAxisLocation', 'bottom', 'TickLength', [0 0], ...
         'FontSize', 10, 'LineWidth', 0.8);

xlabel('Test Configuration $(m_j)$', ...
       'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
ylabel('Model Configuration $(c_i,\;k_i)$', ...
       'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
title({'Cross-Validation RMSE Matrix', ...
       'Cell $(i,j)$: model $(c_i, k_i)$ with test mass $m_j$'}, ...
      'FontSize', 11, 'Interpreter', 'latex');

log_min = min(log10(CV_RMSE(:)));
log_max = max(log10(CV_RMSE(:)));

for i = 1:num_nuts
    for j = 1:num_nuts
        lval       = log10(CV_RMSE(i,j));
        brightness = (lval - log_min) / (log_max - log_min + eps);
        txt_color  = double(brightness > 0.6) * [1 1 1];

        text(j, i, sprintf('$%.3e$', CV_RMSE(i,j)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'FontSize', 9, 'FontWeight', 'bold', ...
             'Color', txt_color, 'Interpreter', 'latex');
    end
end

axis equal tight;

%% =========================================================================
%% CONSOLE: RMSE Matrix
%% =========================================================================
fprintf('\n');
fprintf('======================================================================\n');
fprintf('  5x5 Cross-Validation RMSE Matrix  [m]\n');
fprintf('  Rows = Model (c,k)   |   Cols = Test (m)\n');
fprintf('======================================================================\n');
fprintf('%-12s', 'Model\Test');
for j = 1:num_nuts; fprintf('%-15s', nut_labels{j}); end
fprintf('\n%s\n', repmat('-',1,87));
for i = 1:num_nuts
    fprintf('%-12s', nut_labels{i});
    for j = 1:num_nuts
        if i == j
            fprintf('%-15s', sprintf('[%.3e]', CV_RMSE(i,j)));
        else
            fprintf('%-15s', sprintf('%.3e', CV_RMSE(i,j)));
        end
    end
    fprintf('\n');
end

%% =========================================================================
%% LOCAL FUNCTIONS
%% =========================================================================

function x = analytical_response(ic, t, omega_n, zeta)
% Closed-form solution of m*x'' + c*x' + k*x = 0
% Handles underdamped (zeta < 1), critically damped (zeta = 1),
% and overdamped (zeta > 1) cases.
%
% ic      : [x0; v0]
% t       : time vector
% omega_n : natural frequency (rad/s)
% zeta    : damping ratio

    x0 = ic(1);
    v0 = ic(2);
    t  = t(:);

    if zeta < 1
        % --- Underdamped ---
        omega_d = omega_n * sqrt(1 - zeta^2);
        A = x0;
        B = (v0 + zeta*omega_n*x0) / omega_d;
        x = exp(-zeta*omega_n*t) .* (A*cos(omega_d*t) + B*sin(omega_d*t));

    elseif zeta == 1
        % --- Critically damped ---
        A = x0;
        B = v0 + omega_n*x0;
        x = exp(-omega_n*t) .* (A + B.*t);

    else
        % --- Overdamped ---
        omega_d = omega_n * sqrt(zeta^2 - 1);
        r1 = -zeta*omega_n + omega_d;
        r2 = -zeta*omega_n - omega_d;
        A  =  (v0 - r2*x0) / (r1 - r2);
        B  = -(v0 - r1*x0) / (r1 - r2);
        x  = A*exp(r1*t) + B*exp(r2*t);
    end
end