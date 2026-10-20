function simulacion_pendulo_articulado()
    % =========================================================================
    % SIMULACIÓN INTERACTIVA: Péndulo sobre masa con movimiento horizontal
    % Asignatura: Mecánica Analítica | Grado en Física (UCO)
    % =========================================================================

    % 1. CREACIÓN DE LA VENTANA INTERACTIVA (UIFigure)
    fig = uifigure('Name', 'Simulador - Péndulo Articulado sobre Guía Horizontal (UCO)', 'Position', [100, 100, 1250, 700]);

    % Panel de Control (Izquierda)
    panelCtrl = uipanel(fig, 'Title', 'Parámetros del Sistema (Edición)', 'Position', [20, 20, 340, 660], ...
        'FontSize', 12, 'FontWeight', 'bold');

    % Panel de Visualización / Animación (Derecha)
    panelPlot = uipanel(fig, 'Position', [380, 20, 850, 660]);
    ax = uiaxes(panelPlot, 'Position', [40, 40, 770, 580]);
    title(ax, 'Simulación Gráfica en Tiempo Real');
    xlabel(ax, 'Posición X (m)'); ylabel(ax, 'Posición Y (m)');
    grid(ax, 'on'); axis(ax, 'equal');

    %% 2. CONTROLES DESLIZANTES (SLIDERS) PARA CADA PARÁMETRO
    
    % --- Masa del carro (m1) ---
    uilabel(panelCtrl, 'Position', [20, 585, 300, 22], 'Text', 'Masa del carro, m_1 (kg):');
    s_m1 = uislider(panelCtrl, 'Position', [20, 565, 300, 3], 'Limits', [0.5, 10], 'Value', 2.0);
    lbl_m1 = uilabel(panelCtrl, 'Position', [260, 585, 60, 22], 'Text', num2str(s_m1.Value));
    s_m1.ValueChangedFcn = @(sld, event) updateLabel(lbl_m1, sld);

    % --- Masa del péndulo (m2) ---
    uilabel(panelCtrl, 'Position', [20, 505, 300, 22], 'Text', 'Masa del péndulo, m_2 (kg):');
    s_m2 = uislider(panelCtrl, 'Position', [20, 485, 300, 3], 'Limits', [0.1, 5.0], 'Value', 1.0);
    lbl_m2 = uilabel(panelCtrl, 'Position', [260, 505, 60, 22], 'Text', num2str(s_m2.Value));
    s_m2.ValueChangedFcn = @(sld, event) updateLabel(lbl_m2, sld);

    % --- Constante elástica (k) ---
    uilabel(panelCtrl, 'Position', [20, 425, 300, 22], 'Text', 'Constante del muelle, k (N/m):');
    s_k = uislider(panelCtrl, 'Position', [20, 405, 300, 3], 'Limits', [5, 100], 'Value', 20.0);
    lbl_k = uilabel(panelCtrl, 'Position', [260, 425, 60, 22], 'Text', num2str(s_k.Value));
    s_k.ValueChangedFcn = @(sld, event) updateLabel(lbl_k, sld);

    % --- Longitud del péndulo (l) ---
    uilabel(panelCtrl, 'Position', [20, 345, 300, 22], 'Text', 'Longitud del péndulo, l (m):');
    s_l = uislider(panelCtrl, 'Position', [20, 325, 300, 3], 'Limits', [0.5, 3.0], 'Value', 1.5);
    lbl_l = uilabel(panelCtrl, 'Position', [260, 345, 60, 22], 'Text', num2str(s_l.Value));
    s_l.ValueChangedFcn = @(sld, event) updateLabel(lbl_l, sld);

    % --- Ángulo Inicial del Péndulo (theta0 en grados) ---
    uilabel(panelCtrl, 'Position', [20, 265, 300, 22], 'Text', 'Ángulo Inicial \theta_0 (grados):');
    s_th = uislider(panelCtrl, 'Position', [20, 245, 300, 3], 'Limits', [-90, 90], 'Value', 45);
    lbl_th = uilabel(panelCtrl, 'Position', [260, 265, 60, 22], 'Text', num2str(s_th.Value));
    s_th.ValueChangedFcn = @(sld, event) updateLabel(lbl_th, sld);

    %% 3. BOTÓN DE EJECUCIÓN
    uibutton(panelCtrl, 'Position', [40, 160, 260, 45], ...
        'Text', 'Ejecutar Simulación', ...
        'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0 0.4470 0.7410], 'FontColor', [1 1 1], ...
        'ButtonPushedFcn', @(btn,event) ejecutarSimulacion(s_m1.Value, s_m2.Value, s_k.Value, s_l.Value, s_th.Value, ax));

    % Instrucciones
    uilabel(panelCtrl, 'Position', [20, 20, 300, 120], ...
        'WordWrap', 'on', ...
        'Text', 'Instrucciones:\n1. Modifica los deslizadores de masas, muelle y longitud.\n2. Pulsa el botón para resolver el sistema dinámico y visualizar la animación exacta del carro y el péndulo.');

end

function updateLabel(lbl, sld)
    lbl.Text = sprintf('%.1f', sld.Value);
end

%% 4. MOTOR DE CÁLCULO (ODE45) Y ANIMACIÓN GRÁFICA
function ejecutarSimulacion(m1, m2, k, l, th_deg, ax)
    % Constantes físicas
    g = 9.81;
    l0 = 1.0; % Longitud natural de referencia del muelle

    % Condiciones iniciales: [x; dx; theta; dtheta]
    x0 = 0.5;      % Posición inicial del carro
    dx0 = 0.0;     % Velocidad inicial del carro
    theta0 = deg2rad(th_deg);
    dtheta0 = 0.0; % Velocidad angular inicial

    y0 = [x0; dx0; theta0; dtheta0];
    tspan = [0 12];

    % Resolver EDOs no lineales del sistema acoplado
    options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
    [t, y] = ode45(@(t, y) eqs_acopladas(t, y, m1, m2, k, l0, l, g), tspan, y0, options);

    x_carro = y(:,1);
    theta = y(:,3);

    % Posiciones cartesianas de la masa m2 (péndulo)
    x_pendulo = x_carro + l * sin(theta);
    y_pendulo = -l * cos(theta);

    % Configurar límites de los ejes dinámicamente
    xmin = min(x_carro) - l - 0.5;
    xmax = max(x_carro) + l + 0.5;
    axis(ax, [xmin xmax -l-1.0 0.8]);
    grid(ax, 'on'); hold(ax, 'off');

    % Dibujar elementos estáticos iniciales
    plot(ax, [xmin, xmax], [0, 0], 'k-', 'LineWidth', 3); % Suelo horizontal
    hold(ax, 'on');
    
    h_muelle = plot(ax, [0, x_carro(1)], [0.2, 0.2], 'Color', [0.4 0.4 0.4], 'LineWidth', 2);
    h_carro = rectangle(ax, 'Position', [x_carro(1)-0.3, 0.0, 0.6, 0.4], 'FaceColor', [0.7 0.8 1], 'Curvature', 0.2);
    h_varilla = plot(ax, [x_carro(1), x_pendulo(1)], [0.2, y_pendulo(1)], 'r-', 'LineWidth', 2);
    h_masa2 = plot(ax, x_pendulo(1), y_pendulo(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    title(ax, sprintf('Péndulo sobre Carro (m_1=%.1f, m_2=%.1f, k=%.1f, l=%.1f)', m1, m2, k, l));

    % Bucle de animación fluida en tiempo real
    skip = max(1, floor(length(t)/200));
    for i = 1:skip:length(t)
        if ~isvalid(ax), break; end
        
        xc = x_carro(i);
        xp = x_pendulo(i);
        yp = y_pendulo(i);

        set(h_muelle, 'XData', [0, xc], 'YData', [0.2, 0.2]);
        set(h_carro, 'Position', [xc-0.3, 0.0, 0.6, 0.4]);
        set(h_varilla, 'XData', [xc, xp], 'YData', [0.2, yp]);
        set(h_masa2, 'XData', xp, 'YData', yp);
        
        drawnow;
    end
end

%% 5. SISTEMA DE ECUACIONES DIFERENCIALES ACOPLADAS (EULER-LAGRANGE)
function dydt = eqs_acopladas(~, y, m1, m2, k, l0, l, g)
    x      = y(1);
    dx     = y(2);
    theta  = y(3);
    dtheta = y(4);

    % Abreviaturas trigonométricas
    cost = cos(theta);
    sint = sin(theta);

    % Denominador común de las aceleraciones acopladas
    den = m1 + m2 * (1 - cost^2); % Equivalente a m1 + m2*sint^2

    % Ecuación para la aceleración del carro (ddx) y del péndulo (ddtheta)
    ddx = (m2 * l * dtheta^2 * sint + m2 * g * sint * cost - k * (x - l0)) / den;
    ddtheta = (-ddx * cost - g * sint) / l;

    dydt = [dx; ddx; dtheta; ddtheta];
end