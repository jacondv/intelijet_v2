import math
import numpy as np
from scipy.optimize import  fsolve, root_scalar
# Các hệ số của đa thức bậc 6, bạn thay bằng giá trị thật nếu cần

from shared.config_loader import CONFIG as cfg

ax6= 0.000000000000000014180561937502
bx5= -0.000000000000056332634126525600
cx4= 0.000000000088249620640769400000
dx3= -0.000000068119143571762200000000
ex2= 0.000026699841505366500000000000
fx1= -0.002948109163146670000000000000
gx0= 0.044256105789560800000000000000

# cac tham số mô hình trong folder cfg của project cũ
theoretical_draw_wire_length_when_closed = 248 #mm

encoder_length_at_zero_possition = None

model_angle_when_closed = 0.2292
model_correction_gain = 1.8
draw_wire_encoder_steps_per_revolution = 8192 #0x2000 xung
draw_wire_encoder_spool_circumference = 200.0
draw_wire_gain_term = draw_wire_encoder_spool_circumference / float(draw_wire_encoder_steps_per_revolution)

def length_to_angle_polynomial(draw_wire_length):
    x = draw_wire_length
    angle_in_radians = ax6 * x**6 + bx5 * x**5 + cx4 * x**4 + dx3 * x**3 + ex2 * x**2 + fx1 * x + gx0
    scanner_arm_angle_in_radians = model_angle_when_closed + (angle_in_radians - model_angle_when_closed) * model_correction_gain 

    return scanner_arm_angle_in_radians 

import rospy
def convert_draw_wire_length(message_data, draw_wire_gain_term=draw_wire_gain_term):
    """
    Chuyển đổi 4 byte đầu tiên của message.data thành chiều dài dây kéo ra.

    Args:
        message_data (list[int]): Dữ liệu CAN (ít nhất 4 byte).
        draw_wire_gain_term (float): Hệ số chuyển đổi sang mét hoặc mm.

    Returns:
        float: Chiều dài dây kéo ra (đơn vị: mét hoặc mm, tùy gain).
    """
   
    raw_value = (
        float(message_data[0]) +
        float(message_data[1]) * 256.0 +
        float(message_data[2]) * 256.0**2 +
        float(message_data[3]) * 256.0**3
    )

    return (raw_value-cfg.encoder_length_at_zero_possition) * draw_wire_gain_term + 248


def hinge_angle(a1=393.558, a2=309.546,
                        r1=25.815, r2=45.35,
                        theta0=np.radians(0),
                        encoder0=0,
                        encoder_current=None):
    """
    Tính góc mở bản lề từ delta_L dùng scipy.optimize.broyden1,
    hỗ trợ r1=r2=0 (không có buly).
    """
    
    d0 = calc_d(a1,a2,theta0)
    L0 = np.sqrt(d0**2-(r1-r2)**2)
    print('L0',L0,d0)
    measured_length = encoder_current-encoder0
    L = L0 + measured_length

    def f(theta_rad):
        # length opposite to angle theta in the triangle
        d_theta =  calc_d(a1,a2,(theta_rad + theta0)) 
        # L_calc = np.sqrt(d_theta**2-(r1-r2)**2) + r2*(theta_rad - theta0) + r1*np.arcsin((r2-r1)/d_theta)
        L_calc = r2*(theta_rad) + np.sqrt((d_theta-0)**2 - (r2-r1)**2) - L0
        return  L - L_calc
    
    sol = root_scalar(f, bracket=[0, math.pi], method='bisect')
    theta_deg = math.degrees(sol.root)

    return theta_deg


def calc_d(a1, a2, theta):
    """
    Compute the side length opposite to a given angle in a triangle
    given two sides a1, a2 and the included angle in degrees.
    """
    if a1 <= 0 or a2 <= 0:
        raise ValueError("Side lengths must be greater than 0")
    
    # Convert angle to radians
    angle_rad = theta
    
    # Use the law of cosines
    d = math.sqrt(a1**2 + a2**2 - 2 * a1 * a2 * math.cos(angle_rad))
    return d



if __name__ == '__main__':
    import time
    import matplotlib.pyplot as plt

    # print(calc_d(3,4,90))
    data = []
    for i in range(0,500):
        theta = hinge_angle(a1=393.558, a2=309.546,r1=24.815,r2=45.35,theta0=np.deg2rad(90),encoder0=0,encoder_current=i)
        print(f"Hinge angle θ ≈ {theta:.2f}°, {i}")
        data.append(theta)
        time.sleep(0.01)

    # plt.figure(figsize=(8,4))
    # plt.plot(data, marker='o', linestyle='-', color='blue', label='Data')
    # plt.title("Line Plot of Array")
    # plt.xlabel("Index")
    # plt.ylabel("Value")
    # plt.grid(True)
    # plt.legend()
    # plt.show()



    