function demo_robot()
%DEMO_ROBOT  Escena 3D del proyecto Pick & Place (solo visualizacion).
%
%   Muestra el robot en tres configuraciones: q_safe, toma y colocacion,
%   junto con la banda transportadora, el objeto, el fixture de destino y
%   el obstaculo definidos en escenario.json.
%
%   Las configuraciones de toma y colocacion son las soluciones de
%   referencia (rama frontal, codo arriba) calculadas en PYTHON y leidas de
%   resultados/configuraciones_referencia.csv (python/exportar_configuraciones.py).
%   MATLAB no calcula ningun angulo: solo dibuja y verifica.

[robot, P] = construir_robot();
esc = P.esc;

qs = { P.q_safe, ...
       leer_configuracion(P, 'Pick'), ...
       leer_configuracion(P, 'Place') };
titulos = {'q_{safe} (posicion segura)', 'Toma (Pick)', 'Colocacion (Place)'};

% Comprobar que los angulos de referencia corresponden al escenario actual
fprintf('\n=== demo_robot: verificacion de poses ===\n');
verificar_pose(P, qs{2}, esc.toma, 'toma');
verificar_pose(P, qs{3}, esc.colocacion, 'colocacion');

figure('Name', 'Robot 5 GDL - escena Pick & Place', 'Color', 'w', ...
       'Position', [60 80 1500 540]);
for k = 1:3
    ax = subplot(1, 3, k);
    dibujar_robot(ax, robot, P, qs{k});
    hold(ax, 'on');
    dibujar_escena(ax, P, k);
    [p, ~] = P.fk(qs{k});
    title(ax, sprintf('%s\nTCP = (%.3f, %.3f, %.3f) m', titulos{k}, p));
    ajustar_ejes(ax);
end
end


% =========================================================================
function q = leer_configuracion(P, tarea)
% Fila 'frontal, codo arriba' de la tarea en el CSV generado por Python.
ruta = fullfile(P.rutaResultados, 'configuraciones_referencia.csv');
if ~isfile(ruta)
    error('demo_robot:sinConfiguraciones', ...
        'No existe %s. Ejecute primero: python python/exportar_configuraciones.py', ruta);
end
tabla = readtable(ruta, 'TextType', 'string', 'Delimiter', ',');
fila = tabla(tabla.tarea == tarea & tabla.rama == "frontal, codo arriba", :);
assert(height(fila) == 1, 'No se encontro una unica configuracion de %s.', tarea);
q = deg2rad([fila.theta1_deg, fila.theta2_deg, fila.theta3_deg, ...
             fila.theta4_deg, fila.theta5_deg]);
end


% =========================================================================
function verificar_pose(P, q, pose, nombre)
[p, R] = P.fk(q);
e_p   = 1000 * norm(p - pose.p(:).');                          % mm
e_apr = rad2deg(atan2(norm(cross(R(:,3), [0;0;-1])), R(:,3).' * [0;0;-1]));
psi   = atan2(R(2,1), R(1,1));
e_psi = abs(rad2deg(atan2(sin(psi - deg2rad(pose.psi_deg)), cos(psi - deg2rad(pose.psi_deg)))));
if e_p < 1e-6 && e_apr < 1e-6 && e_psi < 1e-6
    fprintf('[OK   ] %-11s error pos %.1e mm, aprox %.1e deg, giro %.1e deg\n', nombre, e_p, e_apr, e_psi);
else
    warning('demo_robot:pose', ['Los angulos de "%s" no corresponden al escenario actual ' ...
        '(%.2f mm, %.2f deg, %.2f deg). Actualicelos desde Python.'], nombre, e_p, e_apr, e_psi);
end
end


% =========================================================================
function dibujar_robot(ax, robot, P, q)
% Robot del rigidBodyTree + eslabones gruesos + pinza.
if ~isempty(robot)
    show(robot, q, 'Parent', ax, 'Frames', 'off', 'PreservePlot', false);
    hold(ax, 'on');
    % Puntos de las articulaciones tomados del propio rigidBodyTree
    T1 = getTransform(robot, q, 'cintura');
    T2 = getTransform(robot, q, 'brazo');
    T3 = getTransform(robot, q, 'antebrazo');
    T5 = getTransform(robot, q, 'pinza');
    pts = [0 0 0; T1(1:3,4).'; T2(1:3,4).'; T3(1:3,4).'; T5(1:3,4).'];
    R = T5(1:3, 1:3);
else
    hold(ax, 'on');
    pts = P.puntos(q);
    [~, R] = P.fk(q);
end

% Pedestal de la base
dibujar_cilindro(ax, [0 0], 0.07, 0, 0.06, [0.35 0.35 0.40], 1.0);

% Eslabones y articulaciones
plot3(ax, pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.95 0.55 0.10], 'LineWidth', 7);
plot3(ax, pts(2:4,1), pts(2:4,2), pts(2:4,3), 'o', 'MarkerSize', 9, ...
      'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerEdgeColor', 'k');

% Pinza: dos dedos separados a lo largo de x_t, apuntando segun a
tcp = pts(5, :);  x_t = R(:,1).';  a = R(:,3).';
abertura = 0.035;  largo = 0.04;
for s = [-1 1]
    base_dedo = tcp + s*abertura*x_t - largo*a;
    punta     = tcp + s*abertura*x_t;
    plot3(ax, [base_dedo(1) punta(1)], [base_dedo(2) punta(2)], [base_dedo(3) punta(3)], ...
          'k-', 'LineWidth', 4);
end
travesano = [tcp - abertura*x_t - largo*a; tcp + abertura*x_t - largo*a];
plot3(ax, travesano(:,1), travesano(:,2), travesano(:,3), 'k-', 'LineWidth', 4);
plot3(ax, tcp(1), tcp(2), tcp(3), 'p', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
end


% =========================================================================
function dibujar_escena(ax, P, estado)
% estado: 1 = q_safe, 2 = toma, 3 = colocacion
esc = P.esc;
p_toma = esc.toma.p(:).';        psi_toma = deg2rad(esc.toma.psi_deg);
p_col  = esc.colocacion.p(:).';  psi_col  = deg2rad(esc.colocacion.psi_deg);
h_apr  = esc.altura_aproximacion_m;
dim_obj = [0.04 0.06 0.05];      % [a lo largo de x_t, transversal, alto]

% Piso
patch(ax, [-0.4 1.0 1.0 -0.4], [-0.8 -0.8 0.9 0.9], [0 0 0 0], ...
      [0.93 0.93 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.6);

% Banda transportadora (superficie a la altura de la base del objeto)
z_banda = p_toma(3) - dim_obj(3)/2;
dibujar_caja(ax, [p_toma(1), -0.30, z_banda/2], [0.20 0.50 z_banda], 0, [0.25 0.25 0.28], 1.0);

% Fixture de destino
z_fix = p_col(3) - dim_obj(3)/2;
dibujar_caja(ax, [p_col(1), p_col(2), z_fix/2], [0.12 0.12 z_fix], 0, [0.45 0.55 0.70], 1.0);

% Obstaculo y su zona de holgura
o = esc.obstaculo;
dibujar_cilindro(ax, o.centro_xy(:).', o.radio, 0, o.altura, [0.85 0.20 0.20], 0.85);
dibujar_cilindro(ax, o.centro_xy(:).', o.radio + esc.holgura_m, 0, o.altura + esc.holgura_m, ...
                 [0.85 0.20 0.20], 0.12);

% Objeto: en la banda (antes/durante la toma) o en el fixture (colocado)
naranja = [1.0 0.65 0.0];
if estado == 3
    dibujar_caja(ax, p_col,  dim_obj, psi_col,  naranja, 1.0);
    dibujar_caja(ax, p_toma, dim_obj, psi_toma, naranja, 0.15);   % donde estaba
else
    dibujar_caja(ax, p_toma, dim_obj, psi_toma, naranja, 1.0);
    dibujar_caja(ax, p_col,  dim_obj, psi_col,  [0.2 0.7 0.3], 0.20); % destino
end

% Puntos de toma/colocacion y de aproximacion
marcas = [p_toma; p_toma + [0 0 h_apr]; p_col; p_col + [0 0 h_apr]];
plot3(ax, marcas(:,1), marcas(:,2), marcas(:,3), 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);
text(ax, p_toma(1), p_toma(2) - 0.08, p_toma(3) + h_apr + 0.04, 'Pick',  'FontWeight', 'bold');
text(ax, p_col(1),  p_col(2) + 0.06,  p_col(3)  + h_apr + 0.04, 'Place', 'FontWeight', 'bold');
text(ax, o.centro_xy(1), o.centro_xy(2), o.altura + 0.07, 'Obstaculo', 'Color', [0.7 0 0]);
end


% =========================================================================
function dibujar_caja(ax, centro, tam, yaw, color, alfa)
% Caja de dimensiones tam = [lx ly lz] centrada en 'centro', girada yaw sobre z.
[X, Y, Z] = ndgrid([-0.5 0.5], [-0.5 0.5], [-0.5 0.5]);
V = [X(:) Y(:) Z(:)] .* tam;
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
V = V * Rz.' + centro;
F = [1 3 7 5; 2 4 8 6; 1 2 6 5; 3 4 8 7; 1 2 4 3; 5 6 8 7];
patch(ax, 'Vertices', V, 'Faces', F, 'FaceColor', color, 'FaceAlpha', alfa, ...
      'EdgeColor', 0.4*color, 'EdgeAlpha', max(alfa, 0.3));
end


% =========================================================================
function dibujar_cilindro(ax, cxy, radio, z0, z1, color, alfa)
% Cilindro vertical de radio 'radio' entre z0 y z1, con tapa superior.
[X, Y, Z] = cylinder(radio, 40);
X = X + cxy(1);  Y = Y + cxy(2);  Z = z0 + Z*(z1 - z0);
surf(ax, X, Y, Z, 'FaceColor', color, 'FaceAlpha', alfa, 'EdgeColor', 'none');
patch(ax, X(2,:), Y(2,:), Z(2,:), color, 'FaceAlpha', alfa, 'EdgeColor', 'none');
end


% =========================================================================
function ajustar_ejes(ax)
axis(ax, 'equal');
axis(ax, [-0.40 1.00 -0.80 0.90 0 1.00]);
view(ax, 40, 25);           % vista frontal oblicua (rotable con el raton)
grid(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
end
