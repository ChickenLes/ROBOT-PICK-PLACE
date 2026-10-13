function [robot, P] = construir_robot(rutaJson)
%CONSTRUIR_ROBOT  Construye el robot Pick & Place de 5 GDL como rigidBodyTree
%   usando config/escenario.json, la MISMA fuente de verdad que lee Python.
%
%   [robot, P] = construir_robot()
%   [robot, P] = construir_robot(rutaJson)
%
%   Salidas
%   -------
%   robot : rigidBodyTree con DataFormat 'row'. La configuracion es un
%           vector fila q = [theta1 ... theta5] en radianes, identica a la
%           de Python. Vale [] si Robotics System Toolbox no esta instalado.
%   P     : estructura con geometria [m], limites [rad], escenario, ruta de
%           resultados/ y dos funciones auxiliares:
%             [p, R, phi, psi] = P.fk(q)   forma cerrada trigonometrica
%                 (p, phi, psi identicos a python/cinematica.fk; R = [x_t y_t a]
%                 solo para validar contra el rigidBodyTree)
%             pts = P.puntos(q)            5x3: base, hombro, codo, muneca, TCP
%
%   Tabla DH estandar. MATLAB usa el orden [a alpha d theta]:
%
%       i    a     alpha    d     theta
%       1    0     +90 deg  d1    theta1
%       2    a2    0        0     theta2
%       3    a3    0        0     theta3
%       4    0     +90 deg  0     theta4 + 90 deg   <- offset
%       5    0     0        d5    theta5
%
%   IMPORTANTE: en juntas revolutas MATLAB ignora la columna theta de la
%   tabla DH (la sustituye por la variable articular). Por eso el offset de
%   +90 deg de theta4 NO puede escribirse en la tabla. Se implementa con un
%   cuerpo FIJO intermedio que aplica una rotacion Rz(90 deg):
%
%       Rz(90 deg) * Rz(theta4) = Rz(theta4 + 90 deg)
%
%   Asi la configuracion de MATLAB es exactamente la q de Python, sin
%   conversiones. El marco del cuerpo 'pinza' es el TCP.
%
%   Este archivo NO resuelve ninguna optimizacion ni cinematica inversa.

if nargin < 1 || isempty(rutaJson)
    carpeta = fileparts(mfilename('fullpath'));
    rutaJson = fullfile(carpeta, '..', 'config', 'escenario.json');
end
P = leer_parametros(rutaJson);

if exist('rigidBodyTree') == 0 %#ok<EXIST>
    warning('construir_robot:sinToolbox', ...
        'Robotics System Toolbox no disponible: se devuelve robot = [].');
    robot = [];
    return
end

robot = rigidBodyTree('DataFormat', 'row');

%       a      alpha  d     theta (ignorado)
dh = [  0      pi/2   P.d1  0;
        P.a2   0      0     0;
        P.a3   0      0     0;
        0      pi/2   0     0;
        0      0      P.d5  0 ];

nombres = {'cintura', 'brazo', 'antebrazo', 'muneca', 'pinza'};
padre = robot.BaseName;

for i = 1:5
    if i == 4
        % Cuerpo fijo que materializa el offset de +90 deg de theta4
        cuerpo = rigidBody('offset_muneca');
        junta  = rigidBodyJoint('offset_theta4', 'fixed');
        setFixedTransform(junta, rotz_h(pi/2));
        cuerpo.Joint = junta;
        addBody(robot, cuerpo, padre);
        padre = 'offset_muneca';
    end

    cuerpo = rigidBody(nombres{i});
    junta  = rigidBodyJoint(sprintf('theta%d', i), 'revolute');
    setFixedTransform(junta, dh(i, :), 'dh');
    junta.PositionLimits = [P.q_min(i), P.q_max(i)];
    cuerpo.Joint = junta;
    addBody(robot, cuerpo, padre);
    padre = nombres{i};
end
end


% =========================================================================
function P = leer_parametros(ruta)
% Lee el JSON y convierte todo a SI y radianes.
cfg = jsondecode(fileread(ruta));
r   = cfg.robot;

P.d1 = r.d1;  P.a2 = r.a2;  P.a3 = r.a3;  P.d5 = r.d5;
lim     = deg2rad(r.limites_deg);          % 5x2
P.q_min = lim(:, 1).';
P.q_max = lim(:, 2).';
P.ell   = cfg.pinza.brazo_palanca_m;
P.esc   = cfg.escenario;
P.q_safe = deg2rad(cfg.escenario.q_safe_deg(:).');
P.phi_obj = deg2rad(cfg.pinza.phi_objetivo_deg);
P.cfg   = cfg;
P.rutaResultados = fullfile(fileparts(ruta), '..', 'resultados');

g = struct('d1', P.d1, 'a2', P.a2, 'a3', P.a3, 'd5', P.d5);
P.fk     = @(q) fk_cerrada(q, g);
P.puntos = @(q) puntos_brazo(q, g);
end


% =========================================================================
function [p, R, phi, psi] = fk_cerrada(q, g)
% p, phi, psi: mismas ecuaciones que cinematica.fk (Python).
% R: orientacion completa, solo para validacion.
t1 = q(1); t2 = q(2); t3 = q(3); t4 = q(4); t5 = q(5);
psi = t1 - t5;

e_r = [cos(t1); sin(t1); 0];      % radial horizontal
e_t = [-sin(t1); cos(t1); 0];     % horizontal perpendicular al plano
e_z = [0; 0; 1];                  % vertical

f3  = t2 + t3;                    % elevacion del antebrazo
phi = f3 + t4;                    % elevacion de la herramienta

r = g.a2*cos(t2) + g.a3*cos(f3) + g.d5*cos(phi);
z = g.d1 + g.a2*sin(t2) + g.a3*sin(f3) + g.d5*sin(phi);
p = (r*e_r + z*e_z).';

a   = cos(phi)*e_r + sin(phi)*e_z;
x4  = -sin(phi)*e_r + cos(phi)*e_z;
y4  = -e_t;
x_t =  cos(t5)*x4 + sin(t5)*y4;
y_t = -sin(t5)*x4 + cos(t5)*y4;
R = [x_t, y_t, a];
end


% =========================================================================
function pts = puntos_brazo(q, g)
% Base, hombro, codo, muneca y TCP (transcripcion de cinematica.puntos_brazo).
e_r = [cos(q(1)), sin(q(1)), 0];
e_z = [0, 0, 1];
d = @(elev) cos(elev)*e_r + sin(elev)*e_z;

base   = [0, 0, 0];
hombro = g.d1*e_z;
codo   = hombro + g.a2*d(q(2));
muneca = codo   + g.a3*d(q(2) + q(3));
tcp    = muneca + g.d5*d(q(2) + q(3) + q(4));
pts = [base; hombro; codo; muneca; tcp];
end


% =========================================================================
function T = rotz_h(ang)
% Matriz homogenea de rotacion pura alrededor de z.
T = [cos(ang) -sin(ang) 0 0;
     sin(ang)  cos(ang) 0 0;
     0         0        1 0;
     0         0        0 1];
end
