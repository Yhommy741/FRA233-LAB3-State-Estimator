%% =========================================================
%  Spring Constant Experiment -- Data Visualization
%  Academic Plot Script
%  NOTE: Save this file as "spring_analysis.m" (NOT "plot.m")
%        Naming a script "plot.m" shadows MATLAB's built-in
%        plot() function and causes errors.
%  Font: LaTeX Interpreter throughout
% ==========================================================

clc; clear; close all;

%% ---------------------------------------------------------
%  1. Experimental Data
% ----------------------------------------------------------
F = [0, 0, 0, ...
     0.369837, 0.369837, 0.369837, ...
     0.740655, 0.740655, 0.740655, ...
     1.116378, 1.116378, 1.116378, ...
     1.483272, 1.483272, 1.483272, ...
     1.856052, 1.856052, 1.856052];   % Force (N)

x = [0, 0, 0, ...
     0.0125, 0.013, 0.013, ...
     0.025,  0.0245, 0.025, ...
     0.037,  0.037,  0.038, ...
     0.0495, 0.05,   0.05,  ...
     0.0625, 0.0625, 0.063];          % Displacement (m)

%% ---------------------------------------------------------
%  2. Linear Regression  (F = k*x + c)
% ----------------------------------------------------------
p_kx   = polyfit(x, F, 1);   % p(1)=k, p(2)=c
k      = p_kx(1);
c      = p_kx(2);
x_fit  = linspace(min(x), max(x), 200);
F_fit  = polyval(p_kx, x_fit);

% R^2 for Fig 2
F_mean  = mean(F);
SS_tot  = sum((F - F_mean).^2);
SS_res  = sum((F - polyval(p_kx, x)).^2);
R2_fig2 = 1 - SS_res / SS_tot;

%% ---------------------------------------------------------
%  3. Figure 1 -- Result of Experiment
%     x-axis : Force (N)   |   y-axis : Displacement (m)
% ----------------------------------------------------------
fig1 = figure('Units','centimeters','Position',[2 2 16 12], ...
              'Color','w');

% Trendline for this orientation (predictor=F, response=x)
p_Fx   = polyfit(F, x, 1);
F_fit1 = linspace(min(F), max(F), 200);
x_fit1 = polyval(p_Fx, F_fit1);

% R^2 for Fig 1
x_mean  = mean(x);
R2_fig1 = 1 - sum((x - polyval(p_Fx, F)).^2) / sum((x - x_mean).^2);

ax1 = axes('Parent', fig1);
hold(ax1, 'on');

h_data1  = scatter(ax1, F, x, 52, 'o', ...
    'MarkerEdgeColor', [0.12 0.31 0.58], ...
    'MarkerFaceColor', [0.45 0.67 0.88], ...
    'LineWidth', 1.2);

h_trend1 = line(ax1, F_fit1, x_fit1, ...
    'Color',     [0.80 0.10 0.10], ...
    'LineWidth', 1.8);

hold(ax1, 'off');

% Labels
xlabel(ax1, 'Applied Force, $F$ (N)',  'Interpreter','latex','FontSize',13);
ylabel(ax1, 'Displacement, $x$ (m)',   'Interpreter','latex','FontSize',13);
title(ax1,  '\textbf{Result of Experiment: Applied Force vs.\ Displacement}', ...
            'Interpreter','latex','FontSize',14);

% Equation annotation
eq_str1 = sprintf('$x = %.4f\\,F %+.4f$\n$R^{2} = %.4f$', ...
                   p_Fx(1), p_Fx(2), R2_fig1);
text(ax1, 0.05, 0.92, eq_str1, ...
     'Units','normalized','Interpreter','latex','FontSize',11, ...
     'VerticalAlignment','top', ...
     'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.5 0.5 0.5]);

legend(ax1, [h_data1, h_trend1], ...
       {'Experimental data','Linear trendline'}, ...
       'Interpreter','latex','FontSize',11,'Location','northwest');

set(ax1, 'TickLabelInterpreter','latex','FontSize',11, ...
         'Box','on','LineWidth',0.9, ...
         'XGrid','on','YGrid','on', ...
         'GridLineStyle','--','GridAlpha',0.35);
ax1.XAxis.MinorTick = 'on';
ax1.YAxis.MinorTick = 'on';

%% ---------------------------------------------------------
%  4. Figure 2 -- Spring Constant Graph
%     x-axis : Displacement (m)  |  y-axis : Force (N)
% ----------------------------------------------------------
fig2 = figure('Units','centimeters','Position',[20 2 16 12], ...
              'Color','w');

ax2 = axes('Parent', fig2);
hold(ax2, 'on');

h_data2  = scatter(ax2, x, F, 52, 's', ...
    'MarkerEdgeColor', [0.10 0.40 0.10], ...
    'MarkerFaceColor', [0.56 0.83 0.56], ...
    'LineWidth', 1.2);

h_trend2 = line(ax2, x_fit, F_fit, ...
    'Color',     [0.80 0.10 0.10], ...
    'LineWidth', 1.8);

hold(ax2, 'off');

% Build equation string for annotation & legend
if c >= 0
    eq_sign = '+';
else
    eq_sign = '-';
end
eq_latex = sprintf('$F = %.2f\\,x %s %.4f$\n$R^{2} = %.4f$', ...
                    k, eq_sign, abs(c), R2_fig2);
leg_eq   = sprintf('$F = %.2f\\,x %s %.4f$', k, eq_sign, abs(c));

% Labels
xlabel(ax2, 'Displacement, $x$ (m)',  'Interpreter','latex','FontSize',13);
ylabel(ax2, 'Applied Force, $F$ (N)', 'Interpreter','latex','FontSize',13);
title(ax2,  '\textbf{Spring Constant: Displacement vs.\ Applied Force}', ...
            'Interpreter','latex','FontSize',14);

% Equation annotation box
text(ax2, 0.05, 0.92, eq_latex, ...
     'Units','normalized','Interpreter','latex','FontSize',12, ...
     'VerticalAlignment','top', ...
     'BackgroundColor',[1 1 1 0.75],'EdgeColor',[0.5 0.5 0.5]);

legend(ax2, [h_data2, h_trend2], ...
       {'Experimental data', leg_eq}, ...
       'Interpreter','latex','FontSize',11,'Location','northwest');

set(ax2, 'TickLabelInterpreter','latex','FontSize',11, ...
         'Box','on','LineWidth',0.9, ...
         'XGrid','on','YGrid','on', ...
         'GridLineStyle','--','GridAlpha',0.35);
ax2.XAxis.MinorTick = 'on';
ax2.YAxis.MinorTick = 'on';

%% ---------------------------------------------------------
%  5. Console Summary
% ----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('  Spring Constant Analysis Results\n');
fprintf('========================================\n');
fprintf('  Spring constant  k = %.4f  N/m\n', k);
fprintf('  Intercept        c = %.6f  N\n',   c);
fprintf('  R^2 (Fig 1)        = %.4f\n',       R2_fig1);
fprintf('  R^2 (Fig 2)        = %.4f\n',       R2_fig2);
fprintf('  Fitted equation:  F = %.4f*x + (%.6f)\n', k, c);
fprintf('========================================\n\n');