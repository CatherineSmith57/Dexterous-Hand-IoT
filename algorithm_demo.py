"""
algorithm_demo.py — 算法组接入示例（开箱即用）

使用方法：
  1. 确保 hand_node 已启动（终端1）:
     source /opt/ros/jazzy/setup.bash
     source ~/dexterous-hand/ros2_ws/install/setup.bash
     ros2 run hand_middle hand_node --ros-args -p auto_initialize:=false

  2. 运行本脚本（终端2）:
     source /opt/ros/jazzy/setup.bash
     source ~/dexterous-hand/ros2_ws/install/setup.bash
     python3 algorithm_demo.py

输入格式（算法组只需要填这三个字段）：
  - command_type: "gesture"
  - gesture_name: "hand_close" | "hand_open" | "pinch_grasp" | "two_finger_pose" | "point_pose"
  - amplitude:     0.0 = 全开, 1.0 = 全闭, 中间值按比例插值

返回格式：
  - success:           bool   (是否执行成功)
  - execution_status:  str    ("completed" | "failed" | "aborted")
  - error_code:        int    (0 = 正常, 1001 = 未连接, 1003 = 无效指令...)
  - error_message:     str    (错误描述)
  - timestamp:         str    (ISO 8601 北京时间)
"""

import rclpy
from rclpy.node import Node
from hand_middle_interfaces.srv import HandCommand


# ══════════════════════════════════════════════════════════════
#  算法组只需要用这个类，三个步骤搞定
# ══════════════════════════════════════════════════════════════

class HandClient(Node):

    def __init__(self):
        super().__init__('algorithm_client')
        self.client = self.create_client(HandCommand, '/hand_middle/command')
        if self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("✅ 已连接到灵巧手服务")
        else:
            self.get_logger().error("❌ 服务不可用，请先启动 hand_node")

    # ── 算法组只需调这一个方法 ──────────────────────────
    def send_gesture(self, gesture_name: str, amplitude: float = 1.0):
        """
        发送手势指令。

        Args:
            gesture_name:  手势名
            amplitude:     开合幅度 (0.0 = 全开, 1.0 = 全闭)
        """
        req = HandCommand.Request()
        req.command_type    = "gesture"
        req.gesture_name    = gesture_name
        req.amplitude       = float(amplitude)
        req.hold_time_sec   = 2.0
        req.return_to_neutral = True

        self.get_logger().info(f"发送指令: {gesture_name}, amplitude={amplitude}")
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            resp = future.result()
            self.get_logger().info(
                f"返回: success={resp.success}, "
                f"status={resp.execution_status}, "
                f"error_code={resp.error_code}, "
                f"msg={resp.error_message}" if resp.error_message else ""
            )
            return resp
        else:
            self.get_logger().error("调用超时或失败")
            return None


# ══════════════════════════════════════════════════════════════
#  使用示例
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    rclpy.init()
    client = HandClient()

    # 算法识别出什么手势，就调对应的 send_gesture
    # 三个参数都是固定映射，不需要知道硬件细节：
    #
    #   open_palm   →  client.send_gesture("hand_open")
    #   fist        →  client.send_gesture("hand_close")
    #   pinch       →  client.send_gesture("pinch_grasp")
    #   two_finger  →  client.send_gesture("two_finger_pose")
    #   point       →  client.send_gesture("point_pose")

    client.send_gesture("hand_open")
    client.send_gesture("hand_close")
    client.send_gesture("pinch_grasp")
    client.send_gesture("hand_close", amplitude=0.3)  # 微合

    client.destroy_node()
    rclpy.shutdown()
