"""
SCARA 4-DOF Kinematics Library
Descripción:
  Este módulo implementa la cinemática directa (FK) e inversa (IK)
  para un robot SCARA con:
    - θ1, θ2 → juntas rotacionales (plano XY)
    - d      → junta prismática (eje Z)
    - φ_m    → rotación del gripper (motor propio)
  El gripper rota con θ1 + θ2 y además posee su propio motor (φ_m).
  Por lo tanto:
      φ_abs = θ1 + θ2 + φ_m (+ offset opcional)

Uso:
  import scara_kinematics as scara

  # Cinemática directa:
  x, y, z, phi_abs = scara.forward_kinematics(theta1, theta2, d, phi_m, L1, L2)

  # Cinemática inversa:
  solutions = scara.inverse_kinematics(x, y, z, phi_des, L1, L2)

"""

import math


# ===============================
# 🔹 Utilidades
# ===============================

def wrap_angle(angle):
    """Normaliza ángulo a [-pi, pi)"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def grados_a_radianes(grados):
    """Convierte un ángulo de grados a radianes."""
    return grados * math.pi / 180

def radianes_a_grados(radianes):
    """Convierte un ángulo de radianes a grados."""
    return radianes * 180 / math.pi
  
L1 = 160
L2 = 200
OFFSET_X = 135

# ===============================
# 🔹 Cinemática directa (FK)
# ===============================

def forward_kinematics(theta1, theta2, d, phi_m, offset=0):
    # Posición en XY
    theta1 = grados_a_radianes(theta1)
    theta2 = grados_a_radianes(theta2)
    offset = grados_a_radianes(offset)
    phi_m  = grados_a_radianes(phi_m)
    
    x = (L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)) + OFFSET_X
    y = (L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2))
    z = d
    # Orientación absoluta
    phi_abs = radianes_a_grados(theta1 + theta2 + phi_m + offset)
    return x, y, z, phi_abs


def inverse_kinematics(x, y, z, phi_des, offset=0,
                       joint_limits=None, phi_m_limits=None):

    x = x - OFFSET_X   
    y = y     
    phi_des = grados_a_radianes(phi_des)
    offset = grados_a_radianes(offset)
    solutions = []

    # Radio en el plano
    r2 = x * x + y * y
    cos2 = (r2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    # Chequeo de alcance
    if abs(cos2) > 1.0:
        return []  # Fuera del workspace

    # Dos posibles configuraciones: codo arriba / codo abajo
    for sign in (+1, -1):
        sin2 = sign * math.sqrt(max(0.0, 1 - cos2 * cos2))
        theta2 = math.atan2(sin2, cos2)
        theta1 = math.atan2(y, x) - math.atan2(L2 * sin2, L1 + L2 * cos2)
        theta1 = wrap_angle(theta1)
        theta2 = wrap_angle(theta2)

        # Desplazamiento Z
        d = z

        # Rotación necesaria del gripper
        phi_m = wrap_angle(phi_des - (theta1 + theta2 + offset))

        # 🔹 Convertir a grados
        theta1_deg = radianes_a_grados(theta1)
        theta2_deg = radianes_a_grados(theta2)
        phi_m_deg = radianes_a_grados(phi_m)

        # Verificación de límites
        ok = True
        if joint_limits:
            if 'theta1' in joint_limits and not (joint_limits['theta1'][0] <= theta1_deg <= joint_limits['theta1'][1]):
                ok = False
            if 'theta2' in joint_limits and not (joint_limits['theta2'][0] <= theta2_deg <= joint_limits['theta2'][1]):
                ok = False
            if 'd' in joint_limits and not (joint_limits['d'][0] <= d <= joint_limits['d'][1]):
                ok = False
        if phi_m_limits:
            lo, hi = phi_m_limits
            if not (lo <= phi_m_deg <= hi):
                ok = False

        solutions.append({
            'theta1': theta1_deg,
            'theta2': theta2_deg,
            'd': d,
            'phi_m': phi_m_deg,
            'ok': ok
        })

    return solutions