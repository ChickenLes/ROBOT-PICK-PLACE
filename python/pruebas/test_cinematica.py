import numpy as np

from cinematica import envolver, fk, puntos_brazo
from parametros import A2, A3, D1, D5, PHI_OBJ, PICK, PLACE, Q_MAX, Q_MIN, Q_SAFE
from validacion_robotica import dentro_de_limites, fk_dh, ik_analitica

TOL = 1e-12
rng = np.random.default_rng(2026)


def q_aleatorias(n):
    return Q_MIN + (Q_MAX - Q_MIN) * rng.random((n, 5))


def test_q_safe():
    p, phi, psi = fk(Q_SAFE)
    assert np.allclose(p, [0.30, 0.0, 0.50], atol=TOL)
    assert np.isclose(phi, -np.pi / 2) and np.isclose(psi, 0.0)


def test_casos_a_mano():
    g = np.radians
    casos = [([0, 0, 0, 0, 0], [0.80, 0.0, 0.30]),     # brazo estirado horizontal
             ([90, 0, 0, 0, 0], [0.0, 0.80, 0.30]),    # misma pose girada 90 deg
             ([0, 90, 0, 0, 0], [0.0, 0.0, 1.10])]     # brazo vertical
    for q_deg, p_esperado in casos:
        assert np.allclose(fk(g(q_deg))[0], p_esperado, atol=TOL)


def test_fk_trigonometrica_igual_a_dh():
    for q in q_aleatorias(2000):
        p, phi, _ = fk(q)
        T = fk_dh(q)
        assert np.allclose(T[:3, 3], p, atol=TOL)
        a = [np.cos(phi) * np.cos(q[0]), np.cos(phi) * np.sin(q[0]), np.sin(phi)]
        assert np.allclose(T[:3, 2], a, atol=TOL)       # eje de aproximacion


def test_psi_igual_a_dh_con_pinza_vertical():
    for q in q_aleatorias(2000):
        for vuelta in (0.0, 2 * np.pi):                  # phi = -90 y phi = 270
            q[3] = PHI_OBJ + vuelta - q[1] - q[2]
            T = fk_dh(q)
            assert np.allclose(T[:3, 2], [0, 0, -1], atol=TOL)
            psi_dh = np.arctan2(T[1, 0], T[0, 0])        # direccion de los dedos
            assert abs(envolver(psi_dh - fk(q)[2])) < 1e-10


def test_puntos_brazo():
    for q in q_aleatorias(200):
        pts = puntos_brazo(q)
        assert np.allclose(pts[-1], fk(q)[0], atol=TOL)
        largos = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        assert np.allclose(largos, [D1, A2, A3, D5], atol=TOL)


def test_ik_analitica_pick_place():
    for tarea in (PICK, PLACE):
        sols = ik_analitica(tarea)
        assert len(sols) == 4
        for s in sols:
            p, phi, psi = fk(s['q'])
            assert np.allclose(p, tarea['p'], atol=TOL)
            assert abs(envolver(phi - PHI_OBJ)) < 1e-12
            assert abs(envolver(psi - tarea['psi'])) < 1e-12


def test_ik_recupera_configuracion_original():
    for q in q_aleatorias(500):
        q[3] = PHI_OBJ - q[1] - q[2]
        q = envolver(q)
        p, _, psi = fk(q)
        sols = ik_analitica({'p': p, 'psi': psi})
        assert min(np.max(np.abs(envolver(s['q'] - q))) for s in sols) < 1e-8


def test_limites_permiten_pick_place_y_q_safe():
    assert dentro_de_limites(Q_SAFE)
    for tarea in (PICK, PLACE):
        dentro = [s['rama'] for s in ik_analitica(tarea) if dentro_de_limites(s['q'])]
        assert 'frontal, codo arriba' in dentro
