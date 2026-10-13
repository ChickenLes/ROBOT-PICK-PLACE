function ok = validar_fk()
%VALIDAR_FK  Demuestra que FK_Python(q) = FK_MATLAB(q).
%
%   ok = validar_fk()
%
%   Pruebas:
%     1. q_safe = [0 90 -90 -90 0] deg  ->  TCP = (0.30, 0, 0.50) m, pinza abajo.
%     2. Tabla de referencia calculada en PYTHON (resultados/referencia_fk.csv,
%        generada por python/exportar_referencia_fk.py): casos a mano, las 8
%        soluciones exactas de Pick/Place (psi = 0) y configuraciones aleatorias.
%        Se comparan posicion, eje de aproximacion a, eje de los dedos x_t,
%        phi y psi.
%     3. rigidBodyTree vs forma cerrada en 2000 configuraciones aleatorias,
%        incluyendo los puntos intermedios (hombro, codo, muneca).
%     4. Los limites articulares del arbol coinciden con escenario.json.
%
%   Si una prueba falla, la convencion de MATLAB y la de Python NO
%   coinciden y no debe continuarse con la simulacion.

tol = 1e-12;
[robot, P] = construir_robot();
hayToolbox = ~isempty(robot);
res = [];

fprintf('\n=== Validacion FK Python vs MATLAB ===\n');
if ~hayToolbox
    fprintf('AVISO: sin Robotics System Toolbox; solo se prueba la forma cerrada.\n');
end

% ------------------------------------------------------------------ 1
q = P.q_safe;
[p, R] = P.fk(q);
e = max([abs(p - [0.30 0 0.50]), abs(R(:,3).' - [0 0 -1])]);
res(end+1) = informar('q_safe, forma cerrada MATLAB', e, tol);
if hayToolbox
    T = getTransform(robot, q, 'pinza');
    e = max([abs(T(1:3,4).' - [0.30 0 0.50]), abs(T(1:3,3).' - [0 0 -1])]);
    res(end+1) = informar('q_safe, rigidBodyTree', e, tol);
    fprintf('        TCP(q_safe) = (%.6f, %.6f, %.6f) m\n', T(1:3,4));
end

% ------------------------------------------------------------------ 2
% Columnas: q [deg] (5) | p (3) | a (3) | x_t (3) | phi [deg] | psi [deg]
rutaRef = fullfile(P.rutaResultados, 'referencia_fk.csv');
if ~isfile(rutaRef)
    error('validar_fk:sinReferencia', ...
        'No existe %s. Ejecute primero: python python/exportar_referencia_fk.py', rutaRef);
end
REF = readmatrix(rutaRef, 'NumHeaderLines', 1);
nRef = size(REF, 1);

e_cf = 0;  e_ang = 0;  e_rbt = 0;
for k = 1:nRef
    q = deg2rad(REF(k, 1:5));
    ref = REF(k, 6:14);
    [p, R, phi, psi] = P.fk(q);
    e_cf  = max(e_cf, max(abs([p, R(:,3).', R(:,1).'] - ref)));
    e_ang = max(e_ang, max(abs(envolver([phi, psi] - deg2rad(REF(k, 15:16))))));
    if hayToolbox
        T = getTransform(robot, q, 'pinza');
        e_rbt = max(e_rbt, max(abs([T(1:3,4).', T(1:3,3).', T(1:3,1).'] - ref)));
    end
end
res(end+1) = informar(sprintf('Referencia Python vs forma cerrada MATLAB (%d q)', nRef), e_cf, tol);
res(end+1) = informar(sprintf('phi y psi Python vs MATLAB (%d q)', nRef), e_ang, tol);
if hayToolbox
    res(end+1) = informar(sprintf('Referencia Python vs rigidBodyTree (%d q)', nRef), e_rbt, tol);
end

% ------------------------------------------------------------------ 3
if hayToolbox
    fijar_semilla(2026);
    N = 2000;
    Q = P.q_min + (P.q_max - P.q_min) .* rand(N, 5);
    e_tcp = 0;  e_pts = 0;
    for k = 1:N
        q = Q(k, :);
        [p, R] = P.fk(q);
        T = getTransform(robot, q, 'pinza');
        e_tcp = max(e_tcp, max(abs([T(1:3,4).' - p, reshape(T(1:3,1:3) - R, 1, [])])));
        pts = P.puntos(q);
        o1 = getTransform(robot, q, 'cintura');    % hombro
        o2 = getTransform(robot, q, 'brazo');      % codo
        o3 = getTransform(robot, q, 'antebrazo');  % muneca
        e_pts = max(e_pts, max(abs([o1(1:3,4).' - pts(2,:), ...
                                    o2(1:3,4).' - pts(3,:), ...
                                    o3(1:3,4).' - pts(4,:)])));
    end
    res(end+1) = informar('rigidBodyTree vs forma cerrada, TCP y R (2000 q)', e_tcp, tol);
    res(end+1) = informar('rigidBodyTree vs forma cerrada, hombro/codo/muneca', e_pts, tol);
end

% ------------------------------------------------------------------ 4
if hayToolbox
    nombres = {'cintura', 'brazo', 'antebrazo', 'muneca', 'pinza'};
    e = 0;
    for i = 1:5
        cuerpo = getBody(robot, nombres{i});
        lim = cuerpo.Joint.PositionLimits;
        e = max(e, max(abs(lim - [P.q_min(i), P.q_max(i)])));
    end
    res(end+1) = informar('Limites articulares del arbol == JSON', e, tol);
    res(end+1) = informar('El arbol tiene exactamente 5 GDL', ...
                          double(numel(homeConfiguration(robot)) ~= 5), 0.5);
end

ok = all(res);
fprintf('\nResumen: %d/%d pruebas superadas.\n\n', sum(res), numel(res));
end


% =========================================================================
function ok = informar(nombre, err, tol)
ok = err <= tol;
if ok, etiqueta = 'OK   '; else, etiqueta = 'FALLA'; end
fprintf('[%s] %-52s error max = %.1e\n', etiqueta, nombre, err);
end

function a = envolver(a)
a = mod(a + pi, 2*pi) - pi;
end

function fijar_semilla(s)
try
    rng(s);
catch
    rand('seed', s); %#ok<RAND>
end
end
