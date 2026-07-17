"""
test_service_client.py — hand_middle 服务测试客户端

功能：
1. 等待 /hand_middle/command 服务上线
2. 依次测试所有指令类型：
   - enable:   使能电机
   - gesture:  预定义手势（hand_close, hand_open, pinch_grasp, two_finger_pose, point_pose）
   - gesture + amplitude: 开合幅度插值（0.3, 0.7）
   - joint_control: 精细关节角度控制
   - disable:  去使能电机
   - reset:    硬件复位
3. 打印每个请求的完整响应（success/status/error_code/error_message/timestamp）
4. 测试异常场景：非法command_type、ROM越界、非法关节名

使用方式:
    # 终端1: 启动服务端
    ros2 launch hand_middle hand_middle.launch.py

    # 终端2: 运行测试客户端
    ros2 run hand_middle test_service_client

    # 仅测试部分场景
    ros2 run hand_middle test_service_client --ros-args -p test_scenario:=gesture_only
    ros2 run hand_middle test_service_client --ros-args -p test_scenario:=joint_only
    ros2 run hand_middle test_service_client --ros-args -p test_scenario:=error_cases
"""

import sys
import time
import logging

import rclpy
from rclpy.node import Node

# ── 自定义Service消息 ──────────────────────────────────────
try:
    from hand_middle_interfaces.srv import HandCommand
    _SRV_AVAILABLE = True
except ModuleNotFoundError:
    HandCommand = None  # type: ignore
    _SRV_AVAILABLE = False

from .hand_bridge import JOINT_ORDER

logger = logging.getLogger(__name__)


def _empty_names():
    """返回长度匹配 JOINT_ORDER 的全空关节名数组。"""
    return [""] * len(JOINT_ORDER)


def _empty_targets():
    """返回长度匹配 JOINT_ORDER 的全零目标角度数组。"""
    return [0.0] * len(JOINT_ORDER)


def _make_arrays(specified: dict = None):
    """从指定关节字典构建 (joint_names, joint_targets) 数组对。

    未指定的关节填充 "" / 0.0。
    """
    specified = specified or {}
    names, targets = [], []
    for j in JOINT_ORDER:
        if j in specified:
            names.append(j)
            targets.append(float(specified[j]))
        else:
            names.append("")
            targets.append(0.0)
    return names, targets


class TestServiceClient(Node):
    """
    hand_middle 服务测试客户端节点。

    测试覆盖:
    1. 正常流程: enable → gesture → joint_control → disable
    2. 手势全枚举: hand_close, hand_open, pinch_grasp, two_finger_pose, point_pose
    3. 开合幅度插值: 0.3 (微张), 0.7 (大半合)
    4. 复位流程: reset → enable → gesture
    5. 异常场景: 非法指令/非法关节/ROM越界/非法手势
    """

    def __init__(self):
        super().__init__("test_service_client")

        # ── 声明参数 ─────────────────────────────────────────
        self.declare_parameter("test_scenario", "all")
        self.declare_parameter("service_timeout_sec", 15.0)
        self.declare_parameter("service_name", "/hand_middle/command")
        self.declare_parameter("inter_command_delay_sec", 1.5)

        self._scenario = (
            self.get_parameter("test_scenario")
            .get_parameter_value()
            .string_value
        )
        self._timeout = (
            self.get_parameter("service_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self._service_name = (
            self.get_parameter("service_name")
            .get_parameter_value()
            .string_value
        )
        self._delay = (
            self.get_parameter("inter_command_delay_sec")
            .get_parameter_value()
            .double_value
        )

        # ── 校验Service消息是否已编译 ────────────────────────
        if not _SRV_AVAILABLE or HandCommand is None:
            self.get_logger().fatal(
                "HandCommand.srv NOT compiled! "
                "Run 'colcon build' before using the test client."
            )
            sys.exit(1)

        # ── 等待服务上线 ─────────────────────────────────────
        self.get_logger().info(
            f"Waiting for service '{self._service_name}' "
            f"(timeout={self._timeout}s)..."
        )
        self._client = self.create_client(
            HandCommand,
            self._service_name,
        )

        if not self._client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().fatal(
                f"Service '{self._service_name}' NOT available "
                f"after {self._timeout}s! "
                f"Is hand_node running? "
                f"Try: ros2 launch hand_middle hand_middle.launch.py"
            )
            sys.exit(1)

        self.get_logger().info(
            f"Service '{self._service_name}' is available. "
            f"Running test scenario: '{self._scenario}'"
        )

        # ── 初始化测试序列 ───────────────────────────────────
        self._test_index = -1  # 测试步骤索引
        self._test_cases = self._build_test_cases()
        self.get_logger().info(f"Total test cases: {len(self._test_cases)}")

    def _build_test_cases(self) -> list:
        """
        构建测试用例列表。

        每个测试用例是一个 (label, request_dict) 元组。
        """
        # ── 通用请求模板 ─────────────────────────────────────
        all_cases = []

        # ═══════════════════════════════════════════════════════
        # 场景A: 正常流程 (enable → gesture → joint_control → disable → reset)
        # ═══════════════════════════════════════════════════════
        normal_flow = [
            ("[1/5] ENABLE motor", {
                "command_type": "enable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[2/5] GESTURE: hand_close (fist)", {
                "command_type": "gesture", "gesture_name": "hand_close",
                "amplitude": 1.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.5, "return_to_neutral": True,
            }),
            ("[3/5] GESTURE: pinch_grasp", {
                "command_type": "gesture", "gesture_name": "pinch_grasp",
                "amplitude": 1.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.5, "return_to_neutral": True,
            }),
            ("[4/5] JOINT_CONTROL: index only", {
                "command_type": "joint_control", "gesture_name": "",
                "amplitude": 0.0,
                "joint_spec": {"index_mcp": 45.0, "index_pip": 60.0},
                "hold_time_sec": 1.5, "return_to_neutral": True,
            }),
            ("[5/5] DISABLE motor", {
                "command_type": "disable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
        ]

        # ═══════════════════════════════════════════════════════
        # 场景B: 全手势枚举 + 幅度插值
        # ═══════════════════════════════════════════════════════
        gesture_tests = [
            ("[G1] ENABLE", {
                "command_type": "enable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            # 全部5种预定义手势
            *[
                (f"[G2-{i+1}] GESTURE: {g}", {
                    "command_type": "gesture", "gesture_name": g,
                    "amplitude": 1.0, "joint_names": _empty_names(),
                    "joint_targets": _empty_targets(),
                    "hold_time_sec": 1.0, "return_to_neutral": True,
                })
                for i, g in enumerate([
                    "hand_open", "hand_close", "pinch_grasp",
                    "two_finger_pose", "point_pose",
                ])
            ],
            # 开合幅度插值
            ("[G3] GESTURE: hand_close amplitude=0.3 (slightly closed)", {
                "command_type": "gesture", "gesture_name": "hand_close",
                "amplitude": 0.3, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            ("[G4] GESTURE: hand_close amplitude=0.7 (mostly closed)", {
                "command_type": "gesture", "gesture_name": "hand_close",
                "amplitude": 0.7, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            # 上层手势标签映射
            ("[G5] GESTURE: fist (label → hand_close)", {
                "command_type": "gesture", "gesture_name": "fist",
                "amplitude": 1.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            ("[G6] DISABLE", {
                "command_type": "disable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
        ]

        # ═══════════════════════════════════════════════════════
        # 场景C: 精细关节控制
        # ═══════════════════════════════════════════════════════
        joint_tests = [
            ("[J1] ENABLE", {
                "command_type": "enable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[J2] JOINT: thumb only", {
                "command_type": "joint_control", "gesture_name": "",
                "amplitude": 0.0,
                "joint_spec": {
                    "thumb_cmc": 15.0, "thumb_abd": 25.0,
                    "thumb_mcp": 30.0, "thumb_dip": 45.0,
                },
                "hold_time_sec": 1.5, "return_to_neutral": True,
            }),
            ("[J3] JOINT: all fingers", {
                "command_type": "joint_control", "gesture_name": "",
                "amplitude": 0.0,
                "joint_spec": {
                    "wrist": 0.0,
                    "thumb_cmc": 15.0, "thumb_abd": 25.0, "thumb_mcp": 30.0, "thumb_dip": 50.0,
                    "index_abd": 0.0, "index_mcp": 45.0, "index_pip": 60.0,
                    "middle_abd": 0.0, "middle_mcp": 45.0, "middle_pip": 60.0,
                    "ring_abd": 0.0, "ring_mcp": 45.0, "ring_pip": 60.0,
                    "pinky_abd": 0.0, "pinky_mcp": 45.0, "pinky_pip": 60.0,
                },
                "hold_time_sec": 1.5, "return_to_neutral": True,
            }),
            ("[J4] DISABLE", {
                "command_type": "disable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
        ]

        # ═══════════════════════════════════════════════════════
        # 场景D: 异常场景（预期返回失败）
        # ═══════════════════════════════════════════════════════
        error_tests = [
            ("[E1] Unknown command_type (expect fail)", {
                "command_type": "do_backflip", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[E2] Invalid gesture name (expect fail)", {
                "command_type": "gesture", "gesture_name": "jazz_hands",
                "amplitude": 1.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            ("[E3] ROM exceeded: thumb_mcp=999° (expect fail)", {
                "command_type": "joint_control", "gesture_name": "",
                "amplitude": 0.0,
                "joint_spec": {"thumb_mcp": 999.0},
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            ("[E4] Invalid joint: elbow_joint (expect fail)", {
                "command_type": "joint_control", "gesture_name": "",
                "amplitude": 0.0,
                "joint_spec": {"elbow_joint": 45.0},
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
            ("[E5] Amplitude out of range: 1.5 (expect fail)", {
                "command_type": "gesture", "gesture_name": "hand_close",
                "amplitude": 1.5, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
        ]

        # ═══════════════════════════════════════════════════════
        # 场景E: 复位流程 (disable → reset → enable → gesture)
        # ═══════════════════════════════════════════════════════
        reset_tests = [
            ("[R1] DISABLE first", {
                "command_type": "disable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[R2] RESET hardware", {
                "command_type": "reset", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[R3] ENABLE after reset", {
                "command_type": "enable", "gesture_name": "",
                "amplitude": 0.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 0.0, "return_to_neutral": True,
            }),
            ("[R4] GESTURE after reset", {
                "command_type": "gesture", "gesture_name": "hand_open",
                "amplitude": 1.0, "joint_names": _empty_names(),
                "joint_targets": _empty_targets(),
                "hold_time_sec": 1.0, "return_to_neutral": True,
            }),
        ]

        # ── 根据 scenario 参数选择测试子集 ────────────────────
        if self._scenario == "all":
            all_cases = normal_flow + gesture_tests + joint_tests + reset_tests + error_tests
        elif self._scenario == "normal":
            all_cases = normal_flow
        elif self._scenario == "gesture_only":
            all_cases = gesture_tests
        elif self._scenario == "joint_only":
            all_cases = joint_tests
        elif self._scenario == "error_cases":
            all_cases = error_tests
        elif self._scenario == "reset_flow":
            all_cases = reset_tests
        else:
            self.get_logger().warn(
                f"Unknown scenario '{self._scenario}', running all tests"
            )
            all_cases = normal_flow + gesture_tests + joint_tests + reset_tests + error_tests

        return all_cases

    # ── 测试执行 ─────────────────────────────────────────────

    def _run_next_test(self):
        """定时器回调：按序执行下一个测试用例。"""
        self._test_index += 1

        if self._test_index >= len(self._test_cases):
            self.get_logger().info(
                f"\n{'='*60}\n"
                f"  ALL TEST CASES COMPLETE ({len(self._test_cases)} total)\n"
                f"{'='*60}"
            )
            self.destroy_timer(self._timer)
            rclpy.shutdown()
            return

        label, req_dict = self._test_cases[self._test_index]
        self._send_command(label, req_dict)

    def _send_command(self, label: str, req_dict: dict):
        """
        发送单条指令并等待响应。

        Parameters
        ----------
        label : str
            测试用例标签（用于日志输出）
        req_dict : dict
            请求参数字典
        """
        # ── 组装请求 ─────────────────────────────────────────
        req = HandCommand.Request()
        req.command_type = req_dict["command_type"]
        req.gesture_name = req_dict["gesture_name"]
        req.amplitude = float(req_dict["amplitude"])
        req.hold_time_sec = float(req_dict["hold_time_sec"])
        req.return_to_neutral = bool(req_dict["return_to_neutral"])

        # 填充动态数组：支持 "joint_spec" 简写或完整的 "joint_names"+"joint_targets"
        if "joint_spec" in req_dict:
            joint_names, joint_targets = _make_arrays(req_dict["joint_spec"])
        else:
            joint_names = req_dict["joint_names"]
            joint_targets = req_dict["joint_targets"]
        for i in range(len(joint_names)):
            req.joint_names.append(joint_names[i])
            req.joint_targets.append(float(joint_targets[i]))

        # ── 打印请求 ─────────────────────────────────────────
        self.get_logger().info(
            f"\n{'─'*60}\n"
            f"  {label}\n"
            f"  command_type:    {req.command_type}\n"
            f"  gesture_name:    {req.gesture_name or '(n/a)'}\n"
            f"  amplitude:       {req.amplitude:.2f}\n"
            f"  hold_time_sec:   {req.hold_time_sec:.1f}s\n"
            f"  return_neutral:  {req.return_to_neutral}\n"
            f"{'─'*60}"
        )

        # 打印非空关节目标
        non_empty = [
            f"{req.joint_names[i]}={req.joint_targets[i]:.1f}°"
            for i in range(len(req.joint_names))
            if req.joint_names[i].strip()
        ]
        if non_empty:
            self.get_logger().info(f"  Joints: {', '.join(non_empty)}")

        # ── 异步调用服务 ─────────────────────────────────────
        future = self._client.call_async(req)

        # 同步等待结果（便于调试输出）
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)

        if future.done() and future.result() is not None:
            resp = future.result()
            self._print_response(label, resp)
        else:
            self.get_logger().error(
                f"  {label}: Service call FAILED or timed out (20s)!"
            )

    @staticmethod
    def _print_response(label: str, resp):
        """
        格式化打印服务响应。

        Parameters
        ----------
        label : str
            测试标签
        resp : HandCommand.Response
            服务响应
        """
        # 根据成功/失败选择图标
        icon = "✅" if resp.success else "❌"

        lines = [
            f"  {icon} Response for {label}:",
            f"    success:           {resp.success}",
            f"    execution_status:  {resp.execution_status}",
            f"    error_code:        {resp.error_code}",
            f"    error_message:     {resp.error_message or '(none)'}",
            f"    timestamp:         {resp.timestamp}",
        ]
        print("\n".join(lines))


# ── 入口点 ──────────────────────────────────────────────────

def main(args=None):
    """ROS2测试客户端入口点 (console_scripts)。"""
    rclpy.init(args=args)

    node = None
    try:
        node = TestServiceClient()
        # 顺序驱动测试，避免 rclpy.spin() + spin_until_future_complete() 嵌套冲突
        while node._test_index + 1 < len(node._test_cases):
            if not node._run_next_test():
                break
            time.sleep(node._delay)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt received — test aborted")
    except Exception as e:
        logger.exception(f"TestServiceClient crashed: {e}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
