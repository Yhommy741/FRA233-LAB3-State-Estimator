% =========================================================================
%  MSDPlot_Early.m
%  MSD Kalman Filter — First 5 Seconds
%  Focus: Responsiveness vs Smoothness comparison
%
%  Layout: 4 Figures × 2 subplots
%    subplot(2,1,1)  Position  (0–5 s)  + Smoothness annotation
%    subplot(2,1,2)  Error     (0–5 s)
%
%  Figures:
%    Fig 1: Fixed R, Vary Q  — x₀ = 4 cm
%    Fig 2: Fixed R, Vary Q  — x₀ = 8 cm
%    Fig 3: Fixed Q, Vary R  — x₀ = 4 cm
%    Fig 4: Fixed Q, Vary R  — x₀ = 8 cm
% =========================================================================

% ── Path ──────────────────────────────────────────────────────────────────
base = 'C:\Users\Yhommy\OneDrive\Documents\GitHub\FRA233-LAB3-State-Estimator\Result\Part3\';

% ── Load & Rename ─────────────────────────────────────────────────────────
tmp = load(fullfile(base,'MSD_1_4.mat')); d_m4_4 = tmp.data;
tmp = load(fullfile(base,'MSD_1_8.mat')); d_m4_8 = tmp.data;
tmp = load(fullfile(base,'MSD_2_4.mat')); d_m5_4 = tmp.data;
tmp = load(fullfile(base,'MSD_2_8.mat')); d_m5_8 = tmp.data;
clear tmp;
fprintf('Load สำเร็จ\n');

% ── LaTeX & Font ──────────────────────────────────────────────────────────
set(groot,'defaultTextInterpreter',           'latex');
set(groot,'defaultAxesTickLabelInterpreter',  'latex');
set(groot,'defaultLegendInterpreter',         'latex');
set(groot,'defaultAxesFontName',              'Times New Roman');
set(groot,'defaultTextFontName',              'Times New Roman');
set(groot,'defaultAxesFontSize',              10);

% ── Colors (Wong Colorblind-Safe) ─────────────────────────────────────────
c_raw = [0.00 0.00 0.00];   % Black
c_kf1 = [0.84 0.37 0.00];   % Vermilion   — Q/R = 1e-1  responsive
c_kf2 = [0.00 0.45 0.70];   % Deep Blue   — Q/R = 1e-3  balanced
c_kf3 = [0.00 0.62 0.45];   % Teal Green  — Q/R = 1e-5  smooth

% ── Time window ───────────────────────────────────────────────────────────
T_END = 5.0;   % seconds

% ── Legend labels ─────────────────────────────────────────────────────────
lbl_m4 = {'Raw Measurement', ...
    'KF1:\ $Q/R=10^{-1}$,\ $q=7.5\times10^{-8}$,\ $R=7.5\times10^{-7}\ \mathrm{m}^{2}$', ...
    'KF2:\ $Q/R=10^{-3}$,\ $q=7.5\times10^{-10}$,\ $R=7.5\times10^{-7}\ \mathrm{m}^{2}$', ...
    'KF3:\ $Q/R=10^{-5}$,\ $q=7.5\times10^{-12}$,\ $R=7.5\times10^{-7}\ \mathrm{m}^{2}$'};

lbl_m5 = {'Raw Measurement', ...
    'KF1:\ $Q/R=10^{-1}$,\ $q=10^{-9}$,\ $R=10^{-8}\ \mathrm{m}^{2}$', ...
    'KF2:\ $Q/R=10^{-3}$,\ $q=10^{-9}$,\ $R=10^{-6}\ \mathrm{m}^{2}$', ...
    'KF3:\ $Q/R=10^{-5}$,\ $q=10^{-9}$,\ $R=10^{-4}\ \mathrm{m}^{2}$'};

lbl_err = { ...
    'KF1:\ $Q/R=10^{-1}$', ...
    'KF2:\ $Q/R=10^{-3}$', ...
    'KF3:\ $Q/R=10^{-5}$'};

W = 17; H = 16;

% =========================================================================
%  Figure 1 — Fixed R Vary Q, x₀ = 4 cm
% =========================================================================
fig1 = make_fig(W, H, 1);
sgtitle('\textbf{MSD Kalman Filter: Fixed $R$, Vary $Q$,\ $x_0 = 4$ cm\ (first 5 s)}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); pos_plot(d_m4_4, c_raw,c_kf1,c_kf2,c_kf3, lbl_m4, T_END);
subplot(2,1,2); err_plot(d_m4_4, c_kf1,c_kf2,c_kf3, lbl_err, T_END);

% =========================================================================
%  Figure 2 — Fixed R Vary Q, x₀ = 8 cm
% =========================================================================
fig2 = make_fig(W, H, 2);
sgtitle('\textbf{MSD Kalman Filter: Fixed $R$, Vary $Q$,\ $x_0 = 8$ cm\ (first 5 s)}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); pos_plot(d_m4_8, c_raw,c_kf1,c_kf2,c_kf3, lbl_m4, T_END);
subplot(2,1,2); err_plot(d_m4_8, c_kf1,c_kf2,c_kf3, lbl_err, T_END);

% =========================================================================
%  Figure 3 — Fixed Q Vary R, x₀ = 4 cm
% =========================================================================
fig3 = make_fig(W, H, 3);
sgtitle('\textbf{MSD Kalman Filter: Fixed $Q$, Vary $R$,\ $x_0 = 4$ cm\ (first 5 s)}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); pos_plot(d_m5_4, c_raw,c_kf1,c_kf2,c_kf3, lbl_m5, T_END);
subplot(2,1,2); err_plot(d_m5_4, c_kf1,c_kf2,c_kf3, lbl_err, T_END);

% =========================================================================
%  Figure 4 — Fixed Q Vary R, x₀ = 8 cm
% =========================================================================
fig4 = make_fig(W, H, 4);
sgtitle('\textbf{MSD Kalman Filter: Fixed $Q$, Vary $R$,\ $x_0 = 8$ cm\ (first 5 s)}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); pos_plot(d_m5_8, c_raw,c_kf1,c_kf2,c_kf3, lbl_m5, T_END);
subplot(2,1,2); err_plot(d_m5_8, c_kf1,c_kf2,c_kf3, lbl_err, T_END);

% ── Export ────────────────────────────────────────────────────────────────
% figs  = {fig1,fig2,fig3,fig4};
% names = {'MSD_VQ_4cm_5s','MSD_VQ_8cm_5s','MSD_VR_4cm_5s','MSD_VR_8cm_5s'};
% for i = 1:4
%     exportgraphics(figs{i},[names{i} '.pdf'],'ContentType','vector');
% end


% =========================================================================
%  LOCAL — make_fig
% =========================================================================
function f = make_fig(W, H, idx)
    col = mod(idx-1, 2);
    row = floor((idx-1) / 2);
    x0  = 1  + col*(W+2);
    y0  = 30 - row*(H+3);
    f   = figure('Color','white','Units','centimeters','Position',[x0 y0 W H]);
end

% =========================================================================
%  LOCAL — crop: keep only t <= T_END
% =========================================================================
function [tc, xc] = crop(t, x, T_end)
    idx = t <= T_end;
    tc  = t(idx);
    xc  = x(idx);
end

% =========================================================================
%  LOCAL — pos_plot: Position Response (0 to T_END)
% =========================================================================
function pos_plot(ds, c_raw, c_kf1, c_kf2, c_kf3, lbls, T_end)

    t_all = ds{4}.Values.Time;
    raw   = ds{4}.Values.Data;
    k1    = ds{1}.Values.Data;
    k2    = ds{2}.Values.Data;
    k3    = ds{3}.Values.Data;

    [t, raw] = crop(t_all, raw, T_end);
    [~, k1]  = crop(t_all, k1,  T_end);
    [~, k2]  = crop(t_all, k2,  T_end);
    [~, k3]  = crop(t_all, k3,  T_end);

    hold on; box on;
    h0 = plot(t, raw, 'Color',c_raw, 'LineWidth',1.8, 'LineStyle','-');
    h1 = plot(t, k1,  'Color',c_kf1, 'LineWidth',1.5, 'LineStyle','-');
    h2 = plot(t, k2,  'Color',c_kf2, 'LineWidth',1.5, 'LineStyle','--');
    h3 = plot(t, k3,  'Color',c_kf3, 'LineWidth',1.5, 'LineStyle','-.');
    hold off;

    ylabel('Displacement, $x$ (m)', 'FontSize',11);
    title('\textbf{Position Response}', 'FontSize',12);

    legend([h0,h1,h2,h3], lbls, ...
        'Location',  'southeast', ...
        'FontSize',  7.5, ...
        'Box',       'on', ...
        'EdgeColor', [0.25 0.25 0.25], ...
        'Color',     [1 1 1]);

    ax = ax_style(gca, t);
    ax.XTickLabel = {};

    % ── Smoothness annotation — top left (computed over full window) ───
    sm1 = std(diff(k1));
    sm2 = std(diff(k2));
    sm3 = std(diff(k3));
    ann = sprintf(['$\\mathrm{Smoothness} = \\mathrm{std}(\\mathrm{diff}(x))$\n' ...
                   '$\\mathrm{Smoothness}_1 = %.3e$\n' ...
                   '$\\mathrm{Smoothness}_2 = %.3e$\n' ...
                   '$\\mathrm{Smoothness}_3 = %.3e$'], sm1, sm2, sm3);
    xl = ax.XLim; yl = ax.YLim;
    text(xl(1)+0.02*(xl(2)-xl(1)), yl(2)-0.02*(yl(2)-yl(1)), ann, ...
        'FontSize',            8, ...
        'Interpreter',         'latex', ...
        'VerticalAlignment',   'top', ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor',     [1 1 1 0.80], ...
        'EdgeColor',           [0.40 0.40 0.40], ...
        'LineWidth',           0.6, ...
        'Margin',              3);
end

% =========================================================================
%  LOCAL — err_plot: Estimation Error (0 to T_END)
% =========================================================================
function err_plot(ds, c_kf1, c_kf2, c_kf3, lbls, T_end)

    t_all = ds{4}.Values.Time;
    raw   = ds{4}.Values.Data;
    k1    = ds{1}.Values.Data;
    k2    = ds{2}.Values.Data;
    k3    = ds{3}.Values.Data;

    [t, raw] = crop(t_all, raw, T_end);
    [~, k1]  = crop(t_all, k1,  T_end);
    [~, k2]  = crop(t_all, k2,  T_end);
    [~, k3]  = crop(t_all, k3,  T_end);

    e1 = k1 - raw;
    e2 = k2 - raw;
    e3 = k3 - raw;

    hold on; box on;
    yline(0,'Color',[0.55 0.55 0.55],'LineWidth',0.8,'LineStyle','--', ...
          'HandleVisibility','off');
    h1 = plot(t, e1, 'Color',c_kf1, 'LineWidth',1.4, 'LineStyle','-');
    h2 = plot(t, e2, 'Color',c_kf2, 'LineWidth',1.4, 'LineStyle','--');
    h3 = plot(t, e3, 'Color',c_kf3, 'LineWidth',1.4, 'LineStyle','-.');
    hold off;

    xlabel('Time, $t$ (s)',   'FontSize',11);
    ylabel('Error, $e$ (m)', 'FontSize',11);
    title('\textbf{Estimation Error} $(e = \hat{x} - z)$', 'FontSize',12);

    legend([h1,h2,h3], lbls, ...
        'Location',  'southeast', ...
        'FontSize',  8, ...
        'Box',       'on', ...
        'EdgeColor', [0.25 0.25 0.25], ...
        'Color',     [1 1 1]);

    ax_style(gca, t);
end

% =========================================================================
%  LOCAL — ax_style
% =========================================================================
function ax = ax_style(ax, t)
    ax.FontSize           = 10;
    ax.FontName           = 'Times New Roman';
    ax.GridAlpha          = 0.20;
    ax.GridColor          = [0.40 0.40 0.40];
    ax.GridLineStyle      = ':';
    ax.MinorGridAlpha     = 0.12;
    ax.MinorGridLineStyle = ':';
    ax.XMinorTick         = 'on';
    ax.YMinorTick         = 'on';
    ax.TickDir            = 'in';
    ax.TickLength         = [0.012 0.025];
    ax.LineWidth          = 0.9;
    ax.XLim               = [t(1) t(end)];
    ax.Box                = 'on';
    grid on;
end