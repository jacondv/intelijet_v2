#!/usr/bin/env python3

#command to autorun : gnome-sessiion-properties
import time, math, queue, threading
from evdev import InputDevice, ecodes
from select import select
# import pyautogui
from Xlib import X, display, XK
from Xlib.ext import xtest
import time



# --- Cấu hình ---
TOUCH_DEV = "/dev/input/event5"  # Thay bằng event của bạn
SMOOTH_ALPHA = 0.4
VELOCITY_THRESHOLD = 30
SCALE_FACTOR = 0.02  # velocity -> wheel units

# --- Biến theo dõi ---
slots = {0: None, 1: None}
tracking = {0: -1, 1: -1}
cur_slot = 0
last_dist = None
last_time = None
smoothed_velocity = 0.0
velocity_event = threading.Event()  # sự kiện báo wheel_thread

# --- Queue để giao tiếp 2 thread ---
velocity_queue = queue.Queue(maxsize=2)
time.sleep(0.1)  # đảm bảo device ready

d = display.Display()
# ctrl_keycode = d.keysym_to_keycode(XK.XK_Control_L)
# xtest.fake_input(d, X.KeyPress, ctrl_keycode)
# xtest.fake_input(d, X.KeyRelease, ctrl_keycode)
# d.sync()

root = d.screen().root
NET_ACTIVE_WINDOW = d.intern_atom('_NET_ACTIVE_WINDOW')

def get_active_window_name():
    try:
        window_id = root.get_full_property(NET_ACTIVE_WINDOW, X.AnyPropertyType).value[0]
        window = d.create_resource_object('window', window_id)
        name = window.get_wm_name()
        return name
    except:
        return None

target_window_name = "Jacon Intelijet"  # tên Qt window của bạn

def send_zoom(units):
    """Ctrl + wheel"""
    #ctrl_keycode = d.keysym_to_keycode(XK.XK_Control_L)

    # Ctrl press
    #xtest.fake_input(d, X.KeyPress, ctrl_keycode)
    #d.sync()

    # wheel
    button = 4 if units > 0 else 5
    for _ in range(abs(units)):
        xtest.fake_input(d, X.ButtonPress, button)
        xtest.fake_input(d, X.ButtonRelease, button)
    d.sync()

    # Ctrl release
    #xtest.fake_input(d, X.KeyRelease, ctrl_keycode)
    #d.sync()

# --- Thread đo vận tốc ---
def velocity_monitor_thread():
    global cur_slot, last_dist, last_time, smoothed_velocity
    dev = InputDevice(TOUCH_DEV)
    print("👉 Velocity monitor running. Dùng 2 ngón tay. Ctrl+C để thoát.")

    while True:
        r, _, _ = select([dev], [], [], 0.01)
        if not r:
            smoothed_velocity = 0.0
            continue

        for event in dev.read():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_MT_SLOT:
                    cur_slot = event.value
                elif event.code == ecodes.ABS_MT_TRACKING_ID:
                    tracking[cur_slot] = event.value
                    if event.value == -1:
                        slots[cur_slot] = None
                    else:
                        slots[cur_slot] = [0,0]
                elif event.code == ecodes.ABS_MT_POSITION_X:
                    if tracking[cur_slot] != -1 and slots[cur_slot] is not None:
                        slots[cur_slot][0] = event.value
                elif event.code == ecodes.ABS_MT_POSITION_Y:
                    if tracking[cur_slot] != -1 and slots[cur_slot] is not None:
                        slots[cur_slot][1] = event.value

            elif event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
                now = time.time()
                if slots[0] and slots[1]:
                    dx = slots[0][0] - slots[1][0]
                    dy = slots[0][1] - slots[1][1]
                    dist = math.hypot(dx, dy)

                    if last_dist is not None and last_time is not None:
                        dt = now - last_time
                        if dt > 0:
                            velocity = (dist - last_dist) / dt

                            smoothed_velocity = SMOOTH_ALPHA * velocity + (1 - SMOOTH_ALPHA) * smoothed_velocity
                            if smoothed_velocity < -400:
                                smoothed_velocity = -400.0
                            if smoothed_velocity > 400.0:
                                smoothed_velocity = 400.0


                            # Áp dụng ngưỡng
                            if abs(smoothed_velocity) < VELOCITY_THRESHOLD:
                                smoothed_velocity = 0.0

                            # push vào queue nếu khác 0
                            if smoothed_velocity != 0.0:
                                velocity_queue.put(smoothed_velocity)
                                velocity_event.set()
                                # print(smoothed_velocity)

                    last_dist = dist
                    last_time = now
                else:
                    last_dist = None
                    last_time = None
                    smoothed_velocity = 0.0

def wheel_thread():
    min_dt = 0.01  # tối thiểu 20ms giữa các wheel event
    last_time = 0.0

    while True:
        try:
            # Lấy 1 giá trị velocity từ queue, timeout 0.1s nếu không có
            velocity = velocity_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        now = time.time()
        dt = now - last_time
        if dt < min_dt:
            continue
        last_time = now

        wheel_units = int(velocity * SCALE_FACTOR)

        # clamp
        if wheel_units > 0:
            wheel_units = 1
        elif wheel_units < 0:
            wheel_units = -1

        if wheel_units != 0:
            active_window = get_active_window_name()
            if not active_window or not active_window.startswith(target_window_name):
                continue  # bỏ qua nếu không phải app target
            send_zoom(wheel_units)

            
            #print(f"Wheel scroll: {wheel_units} (velocity: {velocity:.2f})")
# def wheel_thread():
#     global smoothed_velocity
#     last_time = 0.0
#     min_dt = 0.2  # tối thiểu 15ms giữa các wheel event

#     while True:
#         velocity_event.wait()
#         velocity_event.clear()  # reset ngay để nhận event mới trong khi chờ

#         now = time.time()
#         dt = now - last_time
#         if dt < min_dt:
#             # quá nhanh, bỏ qua
#             continue
#         last_time = now

#         velocity = smoothed_velocity
#         wheel_units = int(velocity * SCALE_FACTOR)

#         if wheel_units>0:
#             wheel_units=4

#         if wheel_units<0:
#             wheel_units=-4

#         if wheel_units != 0:
#             send_zoom(wheel_units)
#             #print(f"Wheel scroll: {wheel_units} (velocity: {velocity:.2f})")

# --- Main ---
if __name__ == "__main__":
    t1 = threading.Thread(target=velocity_monitor_thread, daemon=True)
    t2 = threading.Thread(target=wheel_thread, daemon=True)
    t1.start()
    t2.start()



    # giữ main thread sống
    while True:
        time.sleep(1)

