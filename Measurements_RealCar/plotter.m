

% Adatok betöltése a workspace-ből
out = evalin('base', 'out');

% Idő vektor kivonása a 'countinous_state' és kiválasztott struktúrából
time_meas = out.measurementsEKF.Time;
time_pred = out.predictionsEKF.Time;

% Az összes adatoszlop ábrázolása külön ábrákon
data_columns = size(out.measurementsEKF.Data, 2); % Adatoszlopok száma
for i = 1:data_columns
    % Switch-case az egyedi y-címkék megadásához 
    switch i 
        case 1
            state_variable = 'Lateral position error'; 
        case 2
            state_variable = 'Lateral velocity error';
        case 3
            state_variable = 'Yaw error';
        case 4
            state_variable = 'Yaw rate error';
        otherwise
            state_variable = (['Data:', num2str(i)]);
    end
    
    figure;
    plot(time_meas, out.measurementsEKF.Data(:, i), 'LineWidth', 1.5);
    hold on;
    plot(time_pred, out.predictionsEKF.Data(:, i), 'LineWidth', 1.5);
    ylim([-5, 5])
    hold off;
    xlabel('Time (s)');
    ylabel(state_variable);
    title([state_variable, ' comparison']);
    legend('Measurement', "Predicted", 'Location', 'best');
    grid on;
end
