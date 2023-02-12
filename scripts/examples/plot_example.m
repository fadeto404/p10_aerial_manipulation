%% Plot example
t = 0:0.01:2*pi;
a = 3*sin(3*pi*t + pi/4) + 1;
b = -2*a + 1;
c = cos(t);

figure(1); hold on;
plot(t, a, t, b, t, c);
title("Some curves: a, b, and c");
legend('a', 'b', 'c');
xlabel('Time [s]');
ylabel('f(t) [-]');
xlim([min(t), max(t)]);

run ../util/plot_settings.m
figure(2); hold on;
plot(t, a, t, b, t, c);
title("Some curves: $a$, $b$, and $c$", 'FontSize', TITLE_FONT_SIZE);
legend('$a$', '$b$', '$c$');
xlabel('Time $t$ [s]', 'FontSize', LABEL_FONT_SIZE);
ylabel('$f(t)$ [-]', 'FontSize', LABEL_FONT_SIZE);
xlim([min(t), max(t)]);