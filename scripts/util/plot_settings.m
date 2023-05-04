%% Plot settings
% get(groot, 'factory'); % See "factory default" settings
% get(groot, 'factoryAxes'); % See factory default settings for Axes objects

% Line drawing settings
set(groot, 'defaultLineLineWidth', 1) % Factory width is 0.5
set(groot, 'defaultStairLineWidth', 1)

% Grid settings
set(groot, 'defaultAxesXGrid', 'on')
set(groot, 'defaultAxesYGrid', 'on')
set(groot, 'defaultAxesZGrid', 'on')
set(groot, 'defaultAxesXMinorGrid', 'on', 'defaultAxesXMinorGridMode','manual')
set(groot, 'defaultAxesYMinorGrid', 'on', 'defaultAxesYMinorGridMode','manual')
set(groot, 'defaultAxesZMinorGrid', 'on', 'defaultAxesZMinorGridMode','manual')
set(groot, 'defaultAxesMinorGridAlpha', 0.25)
set(groot, 'defaultAxesMinorGridColor', [0.1, 0.1, 0.1])

% Tick settings
set(groot, 'defaultAxesXMinorTick', 'on')
set(groot, 'defaultAxesYMinorTick', 'on')
set(groot, 'defaultAxesZMinorTick', 'on')
set(groot, 'defaultAxesTickLabelInterpreter', 'latex')
set(groot, 'defaultAxesXAxisLocation', 'bottom')
set(groot, 'defaultAxesYAxisLocation', 'left')

% Legend settings
set(groot, 'defaultLegendInterpreter', 'latex')

% Title and axis label settings
set(groot, 'defaultTextInterpreter', 'latex')
set(groot, 'defaultTextFontSize', 10)
set(groot, 'defaultAxesLabelFontSizeMultiplier', 1.4000)
set(groot, 'defaultAxesTitleFontSizeMultiplier', 1.6000)
LEGEND_FONT_SIZE = 12;
LABEL_FONT_SIZE = 14;
TITLE_FONT_SIZE = 16;
SGTITLE_FONT_SIZE = 20;

%% Don't forget titles, etc.
% title('A title: $\theta$', 'FontSize', TITLE_FONT_SIZE)
% xlabel('Time [s]', 'FontSize', LABEL_FONT_SIZE)
% ylabel('Angle [deg]', 'FontSize', LABEL_FONT_SIZE)
% legend('EKF', 'KF', 'Encoders');
% xlim([0, PLOT_LENGTH*ts])

%% Functions
% function title_obj = ptitle(title_string)
%     title_obj = title(title_string, 'FontSize', 16);
% end
% 
% function xlabel_obj = pxlabel(label_string)
%     xlabel_obj = xlabel(label_string, "FontSize", 14);
% end
% factoryAxesFontSize


