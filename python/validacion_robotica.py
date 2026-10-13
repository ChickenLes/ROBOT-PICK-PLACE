"""
Herramientas de comprobacion de la cinematica.

- fk_dh:             misma cinematica por Denavit-Hartenberg (comprobacion
                     independiente y puente con el rigidBodyTree de MATLAB).
- ik_analitica:      configuraciones exactas que llevan la pinza a Pick y Place.
- dentro_de_limites: revisa los limites de cada junta.
"""
import numpy as np

from cinematica import envolver
from parametros import A2, A3, D1, D5, PHI_OBJ, Q_MAX, Q_MIN


# --------------------------------------------------------------------------
# Denavit-Hartenberg estandar
#   i   a    alpha   d    theta
#   1   0    +90     d1   theta1
#   2   a2   0       0    theta2
#   3   a3   0       0    theta3
#   4   0    +90     0    theta4 + 90
#   5   0    0       d5   theta5
# --------------------------------------------------------------------------
def _matriz_dh(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_dh(q):
    """Matriz homogenea 4x4 del TCP. Columna 3: eje de aproximacion; columna 1: eje de los dedos."""
    t1, t2, t3, t4, t5 = q
    tabla = [(0.0, np.pi / 2, D1, t1),
             (A2, 0.0, 0.0, t2),
             (A3, 0.0, 0.0, t3),
             (0.0, np.pi / 2, 0.0, t4 + np.pi / 2),
             (0.0, 0.0, D5, t5)]
    T = np.eye(4)
    for fila in tabla:
        T = T @ _matriz_dh(*fila)
    return T


# --------------------------------------------------------------------------
# Cinematica inversa analitica (pinza con inclinacion phi*)
# --------------------------------------------------------------------------
def ik_analitica(tarea, phi_obj=PHI_OBJ):
    """
    Devuelve las 4 soluciones {rama, q} (base frontal/trasera x codo arriba/abajo).
      1. theta1 apunta la base hacia el objetivo (o al lado opuesto, r < 0).
      2. Se retrocede d5 desde el TCP para obtener la muneca.
      3. Brazo de 2 barras (a2, a3) hasta la muneca: ley de cosenos.
      4. theta4 completa phi*;  theta5 = theta1 - psi*.
    """
    x, y, z = tarea['p']
    soluciones = []
    for base, s in (('frontal', 1.0), ('trasera', -1.0)):
        t1 = np.arctan2(y, x) + (0.0 if s > 0 else np.pi)
        r = s * np.hypot(x, y)
        u = r - D5 * np.cos(phi_obj)            # muneca respecto al hombro
        v = z - D5 * np.sin(phi_obj) - D1
        c3 = (u * u + v * v - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
        if abs(c3) > 1:
            continue                             # fuera del alcance
        for signo in (-1.0, 1.0):
            t3 = signo * np.arccos(c3)
            t2 = np.arctan2(v, u) - np.arctan2(A3 * np.sin(t3), A2 + A3 * np.cos(t3))
            t4 = phi_obj - t2 - t3
            t5 = t1 - tarea['psi']
            codo = 'arriba' if s * t3 < 0 else 'abajo'
            soluciones.append({'rama': f'{base}, codo {codo}',
                               'q': envolver(np.array([t1, t2, t3, t4, t5]))})
    return soluciones


def dentro_de_limites(q, tol=1e-9):
    return bool(np.all(q >= Q_MIN - tol) and np.all(q <= Q_MAX + tol))
