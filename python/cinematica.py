"""
Cinematica directa del robot Pick & Place de 5 GDL, obtenida solo con
seno, coseno y angulos acumulados (sin Denavit-Hartenberg).

    q = [theta1, theta2, theta3, theta4, theta5]   [rad]

1) Una barra de longitud L inclinada un angulo theta avanza
       horizontal  L cos(theta),     vertical  L sin(theta).

2) En el plano vertical del brazo los angulos se acumulan:
       brazo      a2   con angulo  theta2
       antebrazo  a3   con angulo  theta2 + theta3
       pinza      d5   con angulo  theta2 + theta3 + theta4 = phi

   Sumando los avances de las tres barras (el hombro esta a altura d1):
       r = a2 cos(theta2) + a3 cos(theta2+theta3) + d5 cos(phi)
       z = d1 + a2 sin(theta2) + a3 sin(theta2+theta3) + d5 sin(phi)

3) La base gira todo ese plano un angulo theta1:
       x = r cos(theta1),     y = r sin(theta1)

4) Orientacion de la pinza:
       phi = theta2 + theta3 + theta4     inclinacion (-pi/2 = vertical hacia abajo)
       psi = theta1 - theta5              giro de los dedos alrededor de la vertical
"""
import numpy as np

from parametros import A2, A3, D1, D5


def fk(q):
    """Devuelve (p, phi, psi) con p = [x, y, z] en metros."""
    t1, t2, t3, t4, t5 = q
    phi = t2 + t3 + t4

    r = A2 * np.cos(t2) + A3 * np.cos(t2 + t3) + D5 * np.cos(phi)
    z = D1 + A2 * np.sin(t2) + A3 * np.sin(t2 + t3) + D5 * np.sin(phi)

    x = r * np.cos(t1)
    y = r * np.sin(t1)

    psi = t1 - t5
    return np.array([x, y, z]), phi, psi


def puntos_brazo(q):
    """Base, hombro, codo, muneca y TCP (5x3). Util para dibujar y colisiones."""
    t1, t2, t3, t4, _ = q
    e_r = np.array([np.cos(t1), np.sin(t1), 0.0])   # direccion horizontal del plano
    e_z = np.array([0.0, 0.0, 1.0])

    def direccion(angulo):
        return np.cos(angulo) * e_r + np.sin(angulo) * e_z

    base = np.zeros(3)
    hombro = D1 * e_z
    codo = hombro + A2 * direccion(t2)
    muneca = codo + A3 * direccion(t2 + t3)
    tcp = muneca + D5 * direccion(t2 + t3 + t4)
    return np.array([base, hombro, codo, muneca, tcp])


def envolver(angulo):
    """Lleva cualquier angulo al intervalo [-pi, pi)."""
    return (np.asarray(angulo) + np.pi) % (2 * np.pi) - np.pi
