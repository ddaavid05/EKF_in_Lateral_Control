function show_datas_on_figures
    % Adatstruktúrák nevének listája
    data_structures = {'discrete_c2d_state', 'discrete_calculated_state', 'discrete_extended_state', 'discrete_state'};

    % Létrehozunk egy felhasználói felületet
    f = figure('Position', [100, 100, 400, 150], 'Name', 'Adatstruktúra kiválasztása');
    popup = uicontrol('Style', 'popupmenu', 'String', data_structures, ...
        'Position', [100, 80, 200, 25]);
    btn = uicontrol('Style', 'pushbutton', 'String', 'Plot Data', ...
        'Position', [150, 30, 100, 25], 'Callback', @(src, event) plotData(popup, data_structures));

    % Plot adatokat kiválasztó funkció
    function plotData(popup, data_structures)
        idx = popup.Value; % Kiválasztott index
        struct_name = data_structures{idx}; % Kiválasztott struktúra neve

        % Adatok betöltése a workspace-ből
        out = evalin('base', 'out');

        % Idő vektor kivonása a 'countinous_state' és kiválasztott struktúrából
        time_continuous = out.countinous_state.Time;
        time_discrete = out.(struct_name).Time;

        % Az összes adatoszlop ábrázolása külön ábrákon
        data_columns = size(out.countinous_state.Data, 2); % Adatoszlopok száma
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
            plot(time_continuous, out.countinous_state.Data(:, i), 'LineWidth', 1.5);
            hold on;
            plot(time_discrete, out.(struct_name).Data(:, i), 'LineWidth', 1.5);
            hold off;
            xlabel('Time (s)');
            ylabel(state_variable);
            title([state_variable, ' comparisom']);
            legend('Continuous State', "Discrete State", 'Location', 'best');
            grid on;
        end
    end
end
