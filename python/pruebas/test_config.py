import numpy as np

import parametros as P


def test_geometria_aprobada():
    assert (P.D1, P.A2, P.A3, P.D5) == (0.30, 0.35, 0.30, 0.15)


def test_decisiones_version_definitiva():
    assert P.SIMETRIA_PI is False
    assert P.PICK['psi'] == 0.0
    assert P.PLACE['psi'] == 0.0           # la version vieja usaba 90 deg
    assert P.ELL == 0.05
    assert np.isclose(P.PHI_OBJ, -np.pi / 2)


def test_limites_bien_formados():
    assert P.Q_MIN.shape == (5,) and P.Q_MAX.shape == (5,)
    assert np.all(P.Q_MIN < P.Q_MAX)


def test_q_safe_dentro_de_limites():
    assert np.all(P.Q_SAFE >= P.Q_MIN) and np.all(P.Q_SAFE <= P.Q_MAX)
