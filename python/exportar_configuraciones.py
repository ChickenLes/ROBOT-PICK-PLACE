"""
Configuraciones de referencia para la demo de MATLAB: q_safe y las 4 soluciones
analiticas de Pick y de Place, con si respetan los limites y la altura del codo.

    python python/exportar_configuraciones.py

Escribe resultados/configuraciones_referencia.csv.
"""
import csv
import os

import numpy as np

from cinematica import puntos_brazo
from parametros import PICK, PLACE, Q_SAFE, RUTA_RESULTADOS
from validacion_robotica import dentro_de_limites, ik_analitica


def main():
    filas = [['tarea', 'rama', 'dentro_limites', 'z_codo_m',
              'theta1_deg', 'theta2_deg', 'theta3_deg', 'theta4_deg', 'theta5_deg']]
    filas.append(['q_safe', '-', True, puntos_brazo(Q_SAFE)[2, 2], *np.degrees(Q_SAFE)])
    for tarea in (PICK, PLACE):
        for s in ik_analitica(tarea):
            z_codo = puntos_brazo(s['q'])[2, 2]
            filas.append([tarea['nombre'], s['rama'], dentro_de_limites(s['q']), z_codo, *np.degrees(s['q'])])
            print(f'{tarea["nombre"]:6s} {s["rama"]:22s} limites: {"SI" if dentro_de_limites(s["q"]) else "no"}  '
                  f'q = {np.round(np.degrees(s["q"]), 4)} deg')

    os.makedirs(RUTA_RESULTADOS, exist_ok=True)
    ruta = os.path.join(RUTA_RESULTADOS, 'configuraciones_referencia.csv')
    with open(ruta, 'w', newline='', encoding='utf-8') as fh:
        csv.writer(fh).writerows(
            [filas[0]] + [[*r[:3], f'{r[3]:.6f}', *[f'{v:.10f}' for v in r[4:]]] for r in filas[1:]])
    print(f'\nEscrito: {ruta}')


if __name__ == '__main__':
    main()
