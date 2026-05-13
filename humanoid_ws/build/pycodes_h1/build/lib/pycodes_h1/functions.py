import numpy as np
import osqp
import scipy.sparse as sp
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi

# =========================
# DENAVIT HATTENBERG
# =========================

def dh(d, theta, a, alpha):
  """
  Calcular la matriz de transformacion homogenea asociada con los parametros
  de Denavit-Hartenberg.
  Los valores d, theta, a, alpha son escalares.
  """
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
  sth = np.sin(theta)
  cth = np.cos(theta)
  sa  = np.sin(alpha)
  ca  = np.cos(alpha)
  T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                [sth,  ca*cth, -sa*cth, a*sth],
                [0.0,      sa,      ca,     d],
                [0.0,     0.0,     0.0,   1.0]])
  return T

# =========================
# KFINE DE BRAZO IZQUIERDO
# =========================

def fkine_arm_left_unitree(q):
    """
    Cinemática directa del brazo izquierdo del Unitree H1-2
    
    q: numpy array de 7 elementos
       [shoulder_pitch,
        shoulder_roll,
        shoulder_yaw,
        elbow,
        wrist_roll,
        wrist_pitch,
        wrist_yaw]
    """
    # =========================
    # DH estructural 7R
    # =========================

    Tbase = np.array([
            [0.0, -1.0,  0.0, 0.0],
            [0.25881905, 0.0, 0.96592583,  0.20794643],
            [-0.96592583, 0.0, 0.25881905, 0.43937656],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    T1 = dh(0.0, q[0],                           0.0060011,  np.pi / 2.0)
    T2 = dh(0.0, q[1] - (np.pi / 2.0 + 0.2618),  0.0,        np.pi / 2.0)
    T3 = dh(-0.3276, q[2] + np.pi / 2.0,         0.006,     -np.pi / 2.0)
    T4 = dh(0.0, q[3] + np.pi / 2.0,             0.011,      np.pi / 2.0)
    T5 = dh(0.208, q[4] + np.pi,                 0.0,        np.pi / 2.0)
    T6 = dh(0.0, q[5] + np.pi / 2.0,             0.020,      np.pi / 2.0)
    T7 = dh(0.0, q[6],                           0.0,        0.0)

    # Transformación total
    T = Tbase.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)

    return T

# =========================
# KFINE DE BRAZO DERECHO
# =========================

def fkine_arm_right_unitree(q):
    """
    Cinemática directa del brazo derecho del Unitree H1-2
    
    q: numpy array de 7 elementos
       [shoulder_pitch,
        shoulder_roll,
        shoulder_yaw,
        elbow,
        wrist_roll,
        wrist_pitch,
        wrist_yaw]
    """
    # =========================
    # DH estructural 7R
    # =========================

    Tbase = np.array([
            [0.0, -1.0,  0.0, 0.0],
            [-0.25881905, 0.0, 0.96592583, -0.20794643],
            [-0.96592583, 0.0, -0.25881905, 0.43937656],
            [0.0, 0.0, 0.0, 1.0],
        ])
    
    T1 = dh(0.0, q[0],                           0.0060011,  np.pi / 2.0)
    T2 = dh(0.0, q[1] - (np.pi / 2.0 - 0.2618),  0.0,        np.pi / 2.0)
    T3 = dh(-0.3276, q[2] + np.pi / 2.0,         0.006,     -np.pi / 2.0)
    T4 = dh(0.0, q[3] + np.pi / 2.0,             0.011,      np.pi / 2.0)
    T5 = dh(0.208, q[4] + np.pi,                 0.0,        np.pi / 2.0)
    T6 = dh(0.0, q[5] + np.pi / 2.0,             0.020,      np.pi / 2.0)
    T7 = dh(0.0, q[6],                           0.0,        0.0)

    # Transformación total
    T = Tbase.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)

    return T

# =========================
# MATRIZ DE ROTACIÓN A CUATERNIONES
# =========================

def rot2quat(R):
  """
  Convertir una matriz de rotacion en un cuaternion

  Entrada:
   R -- Matriz de rotacion
  Salida:
   Q -- Cuaternion [ew, ex, ey, ez]

  """
  dEpsilon = 1e-6
  quat = 4*[0.,]

  quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
  if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
    quat[1] = 0.0
  else:
    quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
  if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
    quat[2] = 0.0
  else:
    quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
  if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
    quat[3] = 0.0
  else:
    quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

  return np.array(quat)

# =========================
# TF A CUATERNIONES
# =========================

def TF2xyzquat(T):
  """
  Convert a homogeneous transformation matrix into the a vector containing the
  pose of the robot.
  
  Input:
   T -- A homogeneous transformation
  Output:
   X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
        is Cartesian coordinates and the last part is a quaternion
  """
  quat = rot2quat(T[0:3,0:3])
  res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
  return np.array(res)


# =========================
# CUATERNIONES
# =========================

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_multiply(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

# =========================
# ERROR DE ORIENTACIÓN
# =========================

def orientation_error(qd, q):
    q_inv = quat_conjugate(q)
    qe = quat_multiply(qd, q_inv)
    return qe[1:]  # parte vectorial

# =========================
# ERROR TOTAL (POSE)
# =========================

def pose_error(xd, x):
    ep = xd[0:3] - x[0:3]
    eo = orientation_error(xd[3:], x[3:])
    return np.hstack((ep, eo))

# =========================
# JACOBIANO NUMÉRICO
# =========================

def numerical_jacobian(fkine, q, TF2xyzquat, delta=1e-6):
    n = len(q)
    J = np.zeros((6, n))

    x = TF2xyzquat(fkine(q))

    for i in range(n):
        dq = np.zeros(n)
        dq[i] = delta

        x_d = TF2xyzquat(fkine(q + dq))

        J[:, i] = (pose_error(x_d, x)) / delta

    return J

# =========================
# IK DAMPED LEAST SQUARES
# =========================

def ik_dls(fkine, TF2xyzquat, q0, xd, max_iter=100, tol=1e-4, lamb=0.01):
    q = q0.copy()

    for _ in range(max_iter):
        x = TF2xyzquat(fkine(q))
        e = pose_error(xd, x)

        if np.linalg.norm(e) < tol:
            return q

        J = numerical_jacobian(fkine, q, TF2xyzquat)

        JT = J.T
        JJ = J @ JT

        dq = JT @ np.linalg.inv(JJ + (lamb**2)*np.eye(6)) @ e

        q = q + dq

    return q

# =========================
# IK AVANZADO
# =========================

def ik_dls_advanced(
    fkine,
    TF2xyzquat,
    q0,
    xd,
    q_min,
    q_max,
    max_iter=1,
    tol=1e-4,
    lamb=0.01,
    k_null=0.1
):
    q = q0.copy()

    for _ in range(max_iter):
        x = TF2xyzquat(fkine(q))
        e = pose_error(xd, x)

        # 🔹 Pesos (posición vs orientación)
        W = np.diag([1, 1, 1, 0.3, 0.3, 0.3])
        e = W @ e

        if np.linalg.norm(e) < tol:
            return q

        J = numerical_jacobian(fkine, q, TF2xyzquat)

        # 🔹 Aplicar pesos al Jacobiano
        J = W @ J

        JT = J.T
        JJ = J @ JT

        # 🔹 Damped Least Squares
        J_pinv = JT @ np.linalg.inv(JJ + (lamb**2)*np.eye(6))

        # 🔹 Null-space (evitar límites articulares)
        q_mid = (q_min + q_max) / 2.0
        z = -k_null * (q - q_mid)

        # 🔹 Proyector de null-space
        I = np.eye(len(q))
        N = I - J_pinv @ J

        dq = J_pinv @ e + N @ z

        q = q + dq

        # 🔹 Límites articulares duros
        q = np.clip(q, q_min, q_max)

    return q


def rotation_error_SO3(R, Rd):
    """
    Error de orientación usando log(Rd * R^T)
    """
    Re = Rd @ R.T

    cos_theta = (np.trace(Re) - 1) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if abs(theta) < 1e-6:
        return np.zeros(3)

    w = (1/(2*np.sin(theta))) * np.array([
        Re[2,1] - Re[1,2],
        Re[0,2] - Re[2,0],
        Re[1,0] - Re[0,1]
    ])

    return theta * w


def pose_error_SE3(Td, T):
    """
    Error en SE(3): [posición; orientación]
    """
    R = T[:3,:3]
    p = T[:3,3]

    Rd = Td[:3,:3]
    pd = Td[:3,3]

    ep = pd - p
    eR = rotation_error_SO3(R, Rd)

    return np.hstack((ep, eR))


def numerical_jacobian_SE3(fkine, q, delta=1e-6):
    n = len(q)
    J = np.zeros((6, n))

    T = fkine(q)

    for i in range(n):
        dq = np.zeros(n)
        dq[i] = delta

        T_d = fkine(q + dq)

        e = pose_error_SE3(T_d, T)

        J[:, i] = e / delta

    return J


def ik_dls_SE3(
    fkine,
    q0,
    Td,
    max_iter=1,
    tol=1e-4,
    lamb=0.01
):
    q = q0.copy()

    for _ in range(max_iter):
        T = fkine(q)
        e = pose_error_SE3(Td, T)

        # 🔹 peso orientación (importante)
        W = np.diag([1, 1, 1, 0.5, 0.5, 0.5])
        e = W @ e

        if np.linalg.norm(e) < tol:
            return q

        J = numerical_jacobian_SE3(fkine, q)
        J = W @ J

        JT = J.T
        JJ = J @ JT

        dq = JT @ np.linalg.inv(JJ + (lamb**2)*np.eye(6)) @ e

        q = q + dq

    return q


def ik_dls_advanced_SE3(
    fkine,
    q0,
    Td,
    q_min,
    q_max,
    max_iter=1,
    tol=1e-4,
    lamb=0.01,
    k_null=0.1
):
    q = q0.copy()

    for _ in range(max_iter):
        T = fkine(q)
        e = pose_error_SE3(Td, T)

        # 🔹 pesos
        W = np.diag([1, 1, 1, 0.5, 0.5, 0.5])
        e = W @ e

        if np.linalg.norm(e) < tol:
            return q

        J = numerical_jacobian_SE3(fkine, q)
        J = W @ J

        JT = J.T
        JJ = J @ JT

        J_pinv = JT @ np.linalg.inv(JJ + (lamb**2)*np.eye(6))

        # 🔹 null-space
        q_mid = (q_min + q_max) / 2.0
        z = -k_null * (q - q_mid)

        I = np.eye(len(q))
        N = I - J_pinv @ J

        dq = J_pinv @ e + N @ z

        q = q + dq

        # 🔹 límites duros
        q = np.clip(q, q_min, q_max)

    return q


def differential_ik_SE3(
    fkine,
    q,
    Td,
    K=1.0,
    lamb=0.01
):
    # 🔹 pose actual
    T = fkine(q)

    # 🔹 error SE(3)
    e = pose_error_SE3(Td, T)

    # 🔹 ganancia
    e = K * e

    # 🔹 Jacobiano
    J = numerical_jacobian_SE3(fkine, q)

    # 🔹 DLS pseudo-inversa
    JT = J.T
    JJ = J @ JT

    J_pinv = JT @ np.linalg.inv(JJ + (lamb**2)*np.eye(6))

    # 🔹 velocidad articular
    dq = J_pinv @ e

    return dq


def qp_differential_ik_osqp(
    fkine,
    q,
    Td,
    q_min,
    q_max,
    dt,
    K=2.0,
    lambda_reg=1e-4
):
    """
    Control cinemático en espacio operacional usando QP (OSQP)

    Minimiza:
        || J dq - v ||^2 + lambda ||dq||^2

    sujeto a:
        dq_min <= dq <= dq_max
    """

    n = len(q)

    # 🔹 1. Cinemática directa actual
    T = fkine(q)

    # 🔹 2. Error en SE(3)
    e = pose_error_SE3(Td, T)

    # 🔹 3. Velocidad deseada en task space
    v = K * e

    # 🔹 4. Jacobiano
    J = numerical_jacobian_SE3(fkine, q)

    # 🔹 5. Construcción del QP
    H = J.T @ J + lambda_reg * np.eye(n)   # matriz cuadrática
    f = -J.T @ v                           # término lineal

    # 🔹 6. Límites articulares → límites en dq
    dq_min = (q_min - q) / dt
    dq_max = (q_max - q) / dt

    # 🔹 7. Formato OSQP (sparse)
    P = sp.csc_matrix(H)
    q_osqp = f

    A = sp.eye(n, format='csc')
    l = dq_min
    u = dq_max

    # 🔹 8. Resolver QP
    prob = osqp.OSQP()
    prob.setup(P, q_osqp, A, l, u, verbose=False)

    res = prob.solve()

    if res.x is None:
        return np.zeros(n)

    return res.x

def damped_pseudo_inverse(J, lambda_reg=1e-4):
    JT = J.T
    return JT @ np.linalg.inv(J @ JT + lambda_reg * np.eye(J.shape[0]))

def nullspace_projector(J, lambda_reg=1e-4):
    J_pinv = damped_pseudo_inverse(J, lambda_reg)
    I = np.eye(J.shape[1])
    N = I - J_pinv @ J
    return N

def joint_limit_avoidance(q, q_min, q_max, gain=0.5):
    q_center = (q_min + q_max) / 2.0
    z = -gain * (q - q_center)
    return z

def redundancy_resolution(J, q, q_min, q_max, lambda_reg=1e-4, gain=0.5):
    N = nullspace_projector(J, lambda_reg)
    z = joint_limit_avoidance(q, q_min, q_max, gain)
    dq_null = N @ z
    return dq_null

