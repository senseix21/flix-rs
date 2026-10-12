"""Small vector and quaternion helpers; tuples, no numpy.

Frames: body is FLU, world is right-handed with z up (Crazyswarm2 world, Flix NWU-like).
Quaternions are (w, x, y, z) and rotate body vectors into the world frame.
"""
import math


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def distance(a, b):
    return norm(sub(a, b))


def multiply(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def rotate(q, v):
    """Body vector to world."""
    w, x, y, z = q
    t = scale(cross((x, y, z), v), 2.0)
    return add(add(v, scale(t, w)), cross((x, y, z), t))


def unrotate(q, v):
    """World vector to body."""
    w, x, y, z = q
    return rotate((w, -x, -y, -z), v)


def from_axes(x, y, z):
    """Quaternion from orthonormal body axes expressed in world (rotation matrix columns)."""
    m00, m10, m20 = x
    m01, m11, m21 = y
    m02, m12, m22 = z
    trace = m00 + m11 + m22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = (0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        q = ((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        q = ((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        q = ((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s)
    q = normalize(q)
    return q if q[0] >= 0 else tuple(-v for v in q)


def tilt(q):
    """Angle between body z and world z, radians."""
    return math.acos(max(-1.0, min(1.0, rotate(q, (0.0, 0.0, 1.0))[2])))


def yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
