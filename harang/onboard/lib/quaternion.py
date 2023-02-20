import numpy as np
import math

class quaternion:
    
    def e_to_q(r): # 오일러각 -> 쿼터니언
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def q_to_e(q): # 쿼터니언 -> 오일러각
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]
    
    def multi(q1,q2): # 쿼터니언 곱
        (a_1, a_2, a_3, a_4) = (q1[0], q1[1], q1[2], q1[3])
        (b_1, b_2, b_3, b_4) = (q2[0], q2[1], q2[2], q2[3])
        x = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4
        y = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3
        z = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2
        w = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1
        return [x, y, z, w]

    def norm(q): # 쿼터니언 정규화
        (a_1, a_2, a_3, a_4) = (q[0], q[1], q[2], q[3])
        q_0 = a_1/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_1 = a_2/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_2 = a_3/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_3 = a_4/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        return [q_0, q_1, q_2, q_3]