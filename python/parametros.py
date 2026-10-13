"""
Lee config/escenario.json, la UNICA fuente de verdad (la misma que lee MATLAB),
y expone los parametros en unidades SI y radianes.

La ruta se construye respecto a este archivo, no respecto a la carpeta
desde la que se ejecuta Python.
"""
import json
import os

import numpy as np

CARPETA = os.path.dirname(os.path.abspath(__file__))
RUTA_CONFIG = os.path.normpath(os.path.join(CARPETA, '..', 'config', 'escenario.json'))
RUTA_RESULTADOS = os.path.normpath(os.path.join(CARPETA, '..', 'resultados'))

with open(RUTA_CONFIG, encoding='utf-8') as _fh:
    CFG = json.load(_fh)

# --- Geometria [m] --------------------------------------------------------
_robot = CFG['robot']
D1 = _robot['d1']      # altura del hombro
A2 = _robot['a2']      # brazo
A3 = _robot['a3']      # antebrazo
D5 = _robot['d5']      # muneca -> punta de la pinza (TCP)

_lim = np.radians(np.array(_robot['limites_deg'], dtype=float))
Q_MIN = _lim[:, 0]
Q_MAX = _lim[:, 1]

# --- Pinza ----------------------------------------------------------------
_pinza = CFG['pinza']
ELL = _pinza['brazo_palanca_m']                  # l: convierte rad -> m
PHI_OBJ = np.radians(_pinza['phi_objetivo_deg'])  # pinza vertical: -pi/2
SIMETRIA_PI = _pinza['simetria_pi']
if SIMETRIA_PI:
    raise NotImplementedError('La version definitiva usa simetria_pi = false.')

# --- Escenario ------------------------------------------------------------
_esc = CFG['escenario']
Q_SAFE = np.radians(np.array(_esc['q_safe_deg'], dtype=float))

PICK = {'nombre': 'Pick',
        'p': np.array(_esc['toma']['p'], dtype=float),
        'psi': np.radians(_esc['toma']['psi_deg'])}
PLACE = {'nombre': 'Place',
         'p': np.array(_esc['colocacion']['p'], dtype=float),
         'psi': np.radians(_esc['colocacion']['psi_deg'])}
