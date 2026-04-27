% =========================================================================
%  Part2Plot.m
%  Kalman Filter — CD / CVD / CAD  ×  Step / Free Vibration
%
%  6 Figures, each 2×1 subplot:
%    Top    : KF Response + Smoothness annotation (S = std(diff(x)))
%    Bottom : Estimation Error  (e = KF − Raw)
%
%  Legend shows: Q value, Q/R ratio
%  Smoothness box: formula + 3 values (Response only)
%
%  R = 7.5e-7 m^2  (fixed for all models)
% =========================================================================

R = 7.5e-7;   % Measurement Noise Covariance [m^2]

% ── Path ──────────────────────────────────────────────────────────────────
base = 'C:\Users\Yhommy\OneDrive\Documents\GitHub\FRA233-LAB3-State-Estimator\Result\Part2\';

% ── Load & Rename ─────────────────────────────────────────────────────────
tmp = load(fullfile(base,'data_cd_step.mat'));   data_cd_step  = tmp.data;
tmp = load(fullfile(base,'data_cvd_step.mat'));  data_cvd_step = tmp.data;
tmp = load(fullfile(base,'data_cad_step.mat'));  data_cad_step = tmp.data;
tmp = load(fullfile(base,'data_cd_vib.mat'));    data_cd_vib   = tmp.data;
tmp = load(fullfile(base,'data_cvd_vib.mat'));   data_cvd_vib  = tmp.data;
tmp = load(fullfile(base,'data_cad_vib.mat'));   data_cad_vib  = tmp.data;
clear tmp;
fprintf('Load สำเร็จ\n');

% ── LaTeX ─────────────────────────────────────────────────────────────────
set(groot,'defaultTextInterpreter',          'latex');
set(groot,'defaultAxesTickLabelInterpreter', 'latex');
set(groot,'defaultLegendInterpreter',        'latex');

% ── Colors ────────────────────────────────────────────────────────────────
c_raw = [0.00 0.00 0.00];
c_kf1 = [0.84 0.37 0.00];   % Vermilion
c_kf2 = [0.00 0.45 0.70];   % Deep Blue
c_kf3 = [0.00 0.62 0.45];   % Teal Green

% ── Q values ──────────────────────────────────────────────────────────────
%  CD  : q  (scalar)     = [7.5e-8, 7.5e-9, 7.5e-10]
%  CVD/CAD: sigma_a      = [1e-1,   1e-3,   1e-5   ]
Q_cd  = [7.5e-8, 7.5e-9, 7.5e-10];
Q_cvc = [1e-1,   1e-3,   1e-5   ];

% ── Build legend labels with Q and Q/R ratio ──────────────────────────────
function lbls = make_lbl_sig(param_name, Q_vals, R, prefix)
    % prefix: 'q' or '\sigma_a'
    lbls = cell(1,4);
    lbls{1} = 'Raw Measurement';
    for i = 1:3
        ratio = Q_vals(i) / R;
        if strcmp(param_name,'q')
            lbls{i+1} = sprintf(['KF%d: $%s = %.1e\\ \\mathrm{m}^{2}$,' ...
                                  '\\ $Q/R = %.2g$'], ...
                                  i, prefix, Q_vals(i), ratio);
        else
            lbls{i+1} = sprintf(['KF%d: $%s = 10^{%d}\\ (\\mathrm{m/s}^2)^2$,' ...
                                  '\\ $\\sigma_a/R = %.2g$'], ...
                                  i, prefix, round(log10(Q_vals(i))), ratio);
        end
    end
end

function lbls = make_lbl_err(param_name, Q_vals, R, prefix)
    lbls = cell(1,3);
    for i = 1:3
        ratio = Q_vals(i) / R;
        if strcmp(param_name,'q')
            lbls{i} = sprintf(['KF%d: $%s = %.1e$,' ...
                                '\\ $Q/R = %.2g$'], ...
                                i, prefix, Q_vals(i), ratio);
        else
            lbls{i} = sprintf(['KF%d: $%s = 10^{%d}$,' ...
                                '\\ $\\sigma_a/R = %.2g$'], ...
                                i, prefix, round(log10(Q_vals(i))), ratio);
        end
    end
end

lbl_cd_sig  = make_lbl_sig('q',   Q_cd,  R, 'q');
lbl_cvc_sig = make_lbl_sig('sa',  Q_cvc, R, '\sigma_a');
lbl_cd_err  = make_lbl_err('q',   Q_cd,  R, 'q');
lbl_cvc_err = make_lbl_err('sa',  Q_cvc, R, '\sigma_a');

W = 17; H = 18;

% =========================================================================
%  Figure 1 — CD Step
% =========================================================================
fig1 = make_fig(W,H,1);
sgtitle('\textbf{Constant Dynamic (CD) --- Step Input}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cd_step,  c_raw,c_kf1,c_kf2,c_kf3, lbl_cd_sig);
subplot(2,1,2); err_plot(data_cd_step,  c_kf1,c_kf2,c_kf3, lbl_cd_err);

% =========================================================================
%  Figure 2 — CD Free Vibration
% =========================================================================
fig2 = make_fig(W,H,2);
sgtitle('\textbf{Constant Dynamic (CD) --- Free Vibration}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cd_vib,   c_raw,c_kf1,c_kf2,c_kf3, lbl_cd_sig);
subplot(2,1,2); err_plot(data_cd_vib,   c_kf1,c_kf2,c_kf3, lbl_cd_err);

% =========================================================================
%  Figure 3 — CVD Step
% =========================================================================
fig3 = make_fig(W,H,3);
sgtitle('\textbf{Constant Velocity Dynamic (CVD) --- Step Input}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cvd_step, c_raw,c_kf1,c_kf2,c_kf3, lbl_cvc_sig);
subplot(2,1,2); err_plot(data_cvd_step, c_kf1,c_kf2,c_kf3, lbl_cvc_err);

% =========================================================================
%  Figure 4 — CVD Free Vibration
% =========================================================================
fig4 = make_fig(W,H,4);
sgtitle('\textbf{Constant Velocity Dynamic (CVD) --- Free Vibration}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cvd_vib,  c_raw,c_kf1,c_kf2,c_kf3, lbl_cvc_sig);
subplot(2,1,2); err_plot(data_cvd_vib,  c_kf1,c_kf2,c_kf3, lbl_cvc_err);

% =========================================================================
%  Figure 5 — CAD Step
% =========================================================================
fig5 = make_fig(W,H,5);
sgtitle('\textbf{Constant Acceleration Dynamic (CAD) --- Step Input}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cad_step, c_raw,c_kf1,c_kf2,c_kf3, lbl_cvc_sig);
subplot(2,1,2); err_plot(data_cad_step, c_kf1,c_kf2,c_kf3, lbl_cvc_err);

% =========================================================================
%  Figure 6 — CAD Free Vibration
% =========================================================================
fig6 = make_fig(W,H,6);
sgtitle('\textbf{Constant Acceleration Dynamic (CAD) --- Free Vibration}', ...
        'FontSize',13,'Interpreter','latex');
subplot(2,1,1); sig_plot(data_cad_vib,  c_raw,c_kf1,c_kf2,c_kf3, lbl_cvc_sig);
subplot(2,1,2); err_plot(data_cad_vib,  c_kf1,c_kf2,c_kf3, lbl_cvc_err);

% ── Export ────────────────────────────────────────────────────────────────
% figs  = {fig1,fig2,fig3,fig4,fig5,fig6};
% names = {'CD_Step','CD_Vib','CVD_Step','CVD_Vib','CAD_Step','CAD_Vib'};
% for i = 1:6
%     exportgraphics(figs{i},[names{i} '.pdf'],'ContentType','vector');
% end


% =========================================================================
%  LOCAL — make_fig
% =========================================================================
function f = make_fig(W, H, idx)
    col = mod(idx-1, 3);
    row = floor((idx-1) / 3);
    x0  = 1  + col*(W+2);
    y0  = 30 - row*(H+4);
    f   = figure('Color','white','Units','centimeters', ...
                 'Position',[x0 y0 W H]);
end

% =========================================================================
%  LOCAL — sig_plot: Response + Smoothness annotation on top subplot only
% =========================================================================
function sig_plot(ds, c_raw, c_kf1, c_kf2, c_kf3, lbls)

    t   = ds{1}.Values.Time;
    k1  = ds{1}.Values.Data;
    k2  = ds{2}.Values.Data;
    k3  = ds{3}.Values.Data;
    raw = ds{4}.Values.Data;

    hold on; box on;
    h0 = plot(t, raw, 'Color',c_raw, 'LineWidth',1.8, 'LineStyle','-');
    h1 = plot(t, k1,  'Color',c_kf1, 'LineWidth',1.4, 'LineStyle','-');
    h2 = plot(t, k2,  'Color',c_kf2, 'LineWidth',1.4, 'LineStyle','--');
    h3 = plot(t, k3,  'Color',c_kf3, 'LineWidth',1.4, 'LineStyle','-.');
    hold off;

    ylabel('Displacement, $d$ (m)', 'FontSize',10);
    title('\textbf{Response}', 'FontSize',11);

    legend([h0,h1,h2,h3], lbls, ...
        'Location',  'southeast', ...
        'FontSize',  7.5, ...
        'Box',       'on', ...
        'EdgeColor', [0.3 0.3 0.3], ...
        'Color',     'white');

    ax = ax_style(gca, t);
    ax.XTickLabel = {};

    % ── Smoothness annotation ─────────────────────────────────────────
    % Formula header + 3 values, colour-coded labels
    sm1 = std(diff(k1));
    sm2 = std(diff(k2));
    sm3 = std(diff(k3));
    ann = sprintf(['$\\mathrm{Smoothness} = \\mathrm{std}(\\mathrm{diff}(x))$\n' ...
                   '$\\mathrm{Smoothness}_1 = %.3e$\n' ...
                   '$\\mathrm{Smoothness}_2 = %.3e$\n' ...
                   '$\\mathrm{Smoothness}_3 = %.3e$'], sm1, sm2, sm3);

    xl = ax.XLim; yl = ax.YLim;
    xpos = xl(1) + 0.02*(xl(2)-xl(1));
    ypos = yl(2) - 0.02*(yl(2)-yl(1));
    text(xpos, ypos, ann, ...
        'FontSize',    7.5, ...
        'Interpreter', 'latex', ...
        'VerticalAlignment',   'top', ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [1 1 1 0.75], ...
        'EdgeColor',       [0.45 0.45 0.45], ...
        'LineWidth',       0.6, ...
        'Margin',          3);
end

% =========================================================================
%  LOCAL — err_plot: Estimation Error (no smoothness)
% =========================================================================
function err_plot(ds, c_kf1, c_kf2, c_kf3, lbls)

    t   = ds{1}.Values.Time;
    k1  = ds{1}.Values.Data;
    k2  = ds{2}.Values.Data;
    k3  = ds{3}.Values.Data;
    raw = ds{4}.Values.Data;

    e1 = k1 - raw;
    e2 = k2 - raw;
    e3 = k3 - raw;

    hold on; box on;
    yline(0,'Color',[0.55 0.55 0.55],'LineWidth',0.8,'LineStyle','--', ...
          'HandleVisibility','off');
    h1 = plot(t, e1, 'Color',c_kf1, 'LineWidth',1.3, 'LineStyle','-');
    h2 = plot(t, e2, 'Color',c_kf2, 'LineWidth',1.3, 'LineStyle','--');
    h3 = plot(t, e3, 'Color',c_kf3, 'LineWidth',1.3, 'LineStyle','-.');
    hold off;

    xlabel('Time, $t$ (s)',  'FontSize',10);
    ylabel('Error, $e$ (m)', 'FontSize',10);
    title('\textbf{Estimation Error} $(e = \hat{x} - z)$', 'FontSize',11);

    legend([h1,h2,h3], lbls, ...
        'Location',  'southeast', ...
        'FontSize',  7.5, ...
        'Box',       'on', ...
        'EdgeColor', [0.3 0.3 0.3], ...
        'Color',     'white');

    ax_style(gca, t);
end

% =========================================================================
%  LOCAL — ax_style
% =========================================================================
function ax = ax_style(ax, t)
    ax.FontSize           = 9;
    ax.GridAlpha          = 0.20;
    ax.GridColor          = [0.40 0.40 0.40];
    ax.GridLineStyle      = ':';
    ax.MinorGridAlpha     = 0.10;
    ax.MinorGridLineStyle = ':';
    ax.XMinorTick         = 'on';
    ax.YMinorTick         = 'on';
    ax.TickDir            = 'in';
    ax.TickLength         = [0.012 0.025];
    ax.LineWidth          = 0.8;
    ax.XLim               = [t(1) t(end)];
    grid on;
end