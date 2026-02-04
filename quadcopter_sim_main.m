%% --- SCHRITT 2: PARAMETERDEFINITION (quadcopter_sim_main.m) ---
% 1. Physikalische Konstanten (basierend auf Tabelle 1)
m       = 0.468;        % Masse [kg]
g       = 9.81;         % Erdbeschleunigung [m/s^2]
l       = 0.225;        % Abstand Rotor-Schwerpunkt [m]
Ixx     = 4.856e-3;     % Roll-Trägheitsmoment [kg*m^2]
Iyy     = 4.856e-3;     % Pitch-Trägheitsmoment [kg*m^2]
Izz     = 8.801e-3;     % Yaw-Trägheitsmoment [kg*m^2]
k       = 2.980e-6;     % Schubfaktor (Lift-Konstante)
b       = 1.140e-7;     % Momentenfaktor (Drag-Konstante für Yaw)
Ax      = 0.25;         % Reibungskoeffizient X-Richtung [kg/s]
Ay      = 0.25;         % Reibungskoeffizient Y-Richtung [kg/s]
Az      = 0.25;         % Reibungskoeffizient Z-Richtung [kg/s]

% 2. Dynamische Konstanten (Zusammenfassung für die Funktion)
I       = [Ixx, Iyy, Izz]; % Vektor der Trägheitsmomente
constants = [m, g, l, k, b, I, Ax, Ay, Az, Ixx, Iyy, Izz]; % Übergabe an Dynamik-Funktion

% 3. Anfangsbedingungen
% Zustandsvektor: x = [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]
x0      = 0; y0      = 0; z0      = 0;    % Position [m]
phi0    = 0; theta0  = 0; psi0    = 0;    % Orientierung [rad]
dx0     = 0; dy0     = 0; dz0     = 0;    % Lineare Geschwindigkeiten [m/s]
p0      = 0; q0      = 0; r0      = 0;    % Winkelgeschwindigkeiten [rad/s]
initial_state = [x0, y0, z0, phi0, theta0, psi0, dx0, dy0, dz0, p0, q0, r0];

% 4. Simulationsparameter
Tspan = [0 60];         % Simulationsdauer (0 bis 60 Sekunden)

%% --- SCHRITT 5: SIMULATION UND LÖSUNG ---
% Numerische Integration des Systems mit ode45
[T, X] = ode45(@(t, X) quadcopter_dynamics(t, X, constants), Tspan, initial_state);

%% --- VISUALISIERUNG DER ERGEBNISSE ---
figure;
% 1. Positionsdiagramm (x, y, z)
subplot(3,1,1); 
plot(T, X(:, 1), 'r', 'LineWidth', 2); hold on; % x-Position
plot(T, X(:, 2), 'g', 'LineWidth', 2);          % y-Position
plot(T, X(:, 3), 'b', 'LineWidth', 2);          % z-Position
xlabel('Zeit (s)');
ylabel('Position (m)');
title('1. Positionsänderung (x, y, z)');
legend('x', 'y', 'z', 'Location', 'best');
grid on;

% 2. Geschwindigkeitsdiagramm (dx, dy, dz)
subplot(3,1,2); 
plot(T, X(:, 7), 'r', 'LineWidth', 2); hold on; % Geschwindigkeit dx
plot(T, X(:, 8), 'g', 'LineWidth', 2);          % Geschwindigkeit dy
plot(T, X(:, 9), 'b', 'LineWidth', 2);          % Geschwindigkeit dz
xlabel('Zeit (s)');
ylabel('Geschwindigkeit (m/s)');
title('2. Lineare Geschwindigkeitsänderung (dx, dy, dz)');
legend('dx', 'dy', 'dz', 'Location', 'best');
grid on;

% 3. Orientierungsdiagramm (Roll, Pitch, Yaw)
subplot(3,1,3); 
plot(T, rad2deg(X(:, 4)), 'r', 'LineWidth', 2); hold on; % Roll (phi)
plot(T, rad2deg(X(:, 5)), 'g', 'LineWidth', 2);          % Pitch (theta)
plot(T, rad2deg(X(:, 6)), 'b', 'LineWidth', 2);          % Yaw (psi)
xlabel('Zeit (s)');
ylabel('Winkel (Grad)');
title('3. Orientierungsänderung (Roll, Pitch, Yaw)');
legend('\phi (Roll)', '\theta (Pitch)', '\psi (Yaw)', 'Location', 'best');
grid on;

%% --- 3D-TRAJEKTORIEN-PLOT ---
figure; 
% Darstellung der X, Y, Z Positionen im Raum
plot3(X(:, 1), X(:, 2), X(:, 3), 'k-', 'LineWidth', 2); hold on;
% Markierung von Start- und Endpunkt
plot3(X(1, 1), X(1, 2), X(1, 3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start (Grün)
plot3(X(end, 1), X(end, 2), X(end, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Ende (Rot)
xlabel('X-Position (m)');
ylabel('Y-Position (m)');
zlabel('Z-Position (m)');
title('Flugtrajektorie des Quadcopters');
grid on;
axis equal; 
view(3);    

%% --- SCHRITT 6: DATENEXPORT FÜR BLENDER ---
% Speicherung von Zeit, Position und Winkeln für die 3D-Visualisierung.
% Winkel bleiben im Bogenmaß (Radiant) für die Verarbeitung in Blender-Python.
data_to_export = [T, X(:, 1), X(:, 2), X(:, 3), X(:, 4), X(:, 5), X(:, 6)];
filename = 'drone_data.csv';
writematrix(data_to_export, filename);
fprintf('Erfolgreich! "%s" wurde erstellt.\n', filename);
disp(['Dateipfad: ', pwd, '\', filename]);