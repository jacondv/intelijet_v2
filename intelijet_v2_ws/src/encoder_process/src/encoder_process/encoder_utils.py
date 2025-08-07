import numpy as np
import scipy.optimize
import rospy


def convert_draw_wire_length(data, gain=1.0):
    """Chuyển đổi 4 byte từ CAN message thành giá trị float."""
    if len(data) < 4:
        raise ValueError("Data len is < 4 byte")
    raw_value = int.from_bytes(data[0:4], byteorder='little', signed=False)
    return raw_value * gain



def cosLaw(s1, s2, s3):
    '''
    cosine law - returns angle between s1 and s2)
                           Given s1,s2,s3
    '''
    # Added the min() here to constrain it to 1 and stop acos errors  
    z = min((s1**2 + s2**2 - s3**2)/(2*s1*s2),1.0)  # Added min to try to remove acos errors
    # z = (s1**2 + s2**2 - s3**2)/(2*s1*s2)  # Original
    try:
        acos = np.arccos(z)
        last_valid_acos = acos
    except ValueError as ve:
        print("ValueError in cosLaw: z = {}, s1 = {}, s2 = {}, s3 = {}}".format(z, s1, s2, s3))
        acos = last_valid_acos
    except RuntimeWarning as rw:
        print("RuntimeWarning in cosLaw: z = {}, s1 = {}, s2 = {}, s3 = {}}".format(z, s1, s2, s3))
        acos = last_valid_acos

    # print("Arcos output is: {}".format(acos))

    return acos


def safe_sqrt(value):
    """Tránh sqrt của số âm."""
    return np.sqrt(value) if value >= 0 else 0.0


def get_initial_estimate(theoretical_length, r1, r2):
    """Ước lượng ban đầu cho p dựa trên chiều dài dây lý thuyết."""
    return np.sqrt(theoretical_length**2 + (r2 - r1)**2)


def make_F(draw_wire_length, r1, r2, b, c, d, e, f):
    absdr = abs(r2 - r1)
    eq3 = (r1 / r2 * (3 * np.pi / 2 - np.arccos(r1 / b) - cosLaw(b, c, d))
           + 3 * np.pi / 2 - np.arccos(r2 / e) - cosLaw(d, e, f))
    sqrtb2r1 = safe_sqrt(b**2 - r1**2)
    sqrte2r2 = safe_sqrt(e**2 - r2**2)


    def F(p):
         return ( - np.sqrt(p**2 - absdr**2)/r2
            + r1/r2 * ( cosLaw(p,c,d) + np.arcsin(absdr/p) )
            + cosLaw(p,d,c) + np.arccos(absdr/p)
            - ( eq3 - (draw_wire_length - sqrtb2r1 - sqrte2r2)/r2))
        
    return F


def length_to_angle_polynomial(length, coeffs, model_angle_when_closed, correction_gain):
    angle = np.polyval(coeffs, length)
    return angle * correction_gain + model_angle_when_closed


def length_to_angle_kinematic(draw_wire_length, F, estimated_p1_p2_length, solution_tolerance,
                              c, d, model_angle_when_closed=0.0, model_correction_gain=1.0,
                              theoretical_draw_wire_length_when_closed=15, r1=24.30385, r2=45.27786):
    """Chuyển đổi từ chiều dài dây sang góc bằng kinematic solver."""
    if estimated_p1_p2_length is None:
        estimated_p1_p2_length = get_initial_estimate(theoretical_draw_wire_length_when_closed, r1, r2)

    try:
        p = scipy.optimize.broyden1(F, estimated_p1_p2_length, f_tol=solution_tolerance)
    except scipy.optimize.nonlin.NoConvergence:
        rospy.logwarn("Kinematic solver: Unable to converge to a solution.")
        return None
    except ValueError as ve:
        rospy.logwarn(f"Kinematic solver: Overflow - Needs calibration: {ve}, at {draw_wire_length}")
        return None

    angle_in_radians = cosLaw(c, d, p)
    return angle_in_radians#model_angle_when_closed + (angle_in_radians - model_angle_when_closed) * model_correction_gain

