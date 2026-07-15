# 给上层同学看的接口说明


from hand_interface import init, do_gesture, get_status, cleanup

# 1️⃣ 初始化（连接 + 标定 + 回中）
init()                     # 自动用 config_safe.yaml

# 2️⃣ 执行手势
do_gesture("hand_open")    # 张开
do_gesture("hand_close")   # 握拳
do_gesture("pinch_grasp")  # 捏取

# 带参数版本：
do_gesture("hand_open", hold_time_sec=3.0, return_to_neutral=True)

# 3️⃣ 读状态
status = get_status()
# → {"connected": True, "calibrated": True, "motor_currents": {...}, ...}

# 4️⃣ 关闭
cleanup()