"""
Exporta la tabla de referencia FK que MATLAB (validar_fk.m) compara con
su rigidBodyTree. Asi MATLAB no guarda valores copiados a mano.

    python python/exportar_referencia_fk.py

Columnas: q [deg] (5) | p [m] (3) | eje de aproximacion a (3) |
          eje de los dedos x_t (3) | phi [deg] | psi [deg]
"""
import csv
import os

import numpy as np

from cinematica import envolver, fk
from parametros import PICK, PLACE, Q_MAX, Q_MIN, Q_SAFE, RUTA_RESULTADOS
from validacion_robotica import fk_dh, ik_analitica


def main():
    g = np.radians
    configuraciones = [g([0, 0, 0, 0, 0]), g([90, 0, 0, 0, 0]), g([0, 90, 0, 0, 0]), Q_SAFE]
    for tarea in (PICK, PLACE):
        configuraciones += [s['q'] for s in ik_analitica(tarea)]
    rng = np.random.default_rng(2026)
    configuraciones += list(Q_MIN + (Q_MAX - Q_MIN) * rng.random((6, 5)))

    filas = []
    for q in configuraciones:
        p, phi, psi = fk(q)
        T = fk_dh(q)
        assert np.allclose(T[:3, 3], p, atol=1e-12)
        filas.append([*np.degrees(q), *p, *T[:3, 2], *T[:3, 0],
                      np.degrees(envolver(phi)), np.degrees(envolver(psi))])

    os.makedirs(RUTA_RESULTADOS, exist_ok=True)
    ruta = os.path.join(RUTA_RESULTADOS, 'referencia_fk.csv')
    with open(ruta, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['q1_deg', 'q2_deg', 'q3_deg', 'q4_deg', 'q5_deg', 'px', 'py', 'pz',
                    'ax', 'ay', 'az', 'xtx', 'xty', 'xtz', 'phi_deg', 'psi_deg'])
        w.writerows([[repr(float(v)) for v in fila] for fila in filas])
    print(f'Escrito: {ruta} ({len(filas)} configuraciones)')


if __name__ == '__main__':
    main()
