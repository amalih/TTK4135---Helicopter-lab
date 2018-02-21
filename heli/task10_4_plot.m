%PLotting script

%% Plotting
t = 0:delta_t:delta_t*(length(u1)-1);

figure
subplot(611)
plot(plot_travel.time, plot_travel.signals.values);
xlabel('Travel')
subplot(612)
plot(plot_travelRate.time, plot_travelRate.signals.values);
xlabel('Travel rate')
subplot(615)
plot(plot_elev.time, plot_elev.signals.values);
xlabel('Elevation')
subplot(616)
plot(plot_elevRate.time, plot_elev.signals.values);
xlabel('Elevation rate')
subplot(613)
plot(plot_pitch.time, plot_pitch.signals.values);
xlabel('Pitch')
subplot(614)
plot(plot_pitchRate.time, plot_pitchRate.signals.values);
xlabel('Pitch rate')
