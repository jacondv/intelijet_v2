import math

# Các hệ số của đa thức bậc 6, bạn thay bằng giá trị thật nếu cần

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

    global encoder_length_at_zero_possition
    rospy.logwarn(raw_value)

    if encoder_length_at_zero_possition is None:
        encoder_length_at_zero_possition = 29910924
        if raw_value < 29910924+100:
            encoder_length_at_zero_possition= raw_value
        else:
            return -1

    return (raw_value-encoder_length_at_zero_possition) * draw_wire_gain_term + 248
