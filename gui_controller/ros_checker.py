#!/usr/bin/env python3
# ros_checker.py

import os
import signal
import subprocess
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile

from action_msgs.msg import GoalStatus, GoalStatusArray
from actionlib_msgs.msg import GoalStatus as NavigationGoalStatus
from arms_interfaces.srv import MoveArms
from clips_interfaces.srv import String as ClipsString
from executor_interfaces.action import ExecuteTask
from executor_interfaces.msg import TaskResult
from geometry_msgs.msg import PoseStamped
from grippers_interfaces.srv import MoveGrippers
from head_interfaces.srv import MoveHead
from listener_interfaces.srv import Listen
from rcl_interfaces.msg import Log
from rclpy.time import Time
from semantic_parser_interfaces.srv import Parser
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from talker_interfaces.srv import Talk
from torso_interfaces.srv import MoveTorso

try:
    from ros2node.api import get_node_names
    USE_ROS2NODE_API = True
except ImportError:
    USE_ROS2NODE_API = False
    print("No se pudo importar ros2node.api. Se usará método alternativo.")

class ROS2NodeChecker(Node):
    """Nodo ROS 2 para monitoreo y controles de demostración del GUI."""

    def __init__(self, signals):
        super().__init__('gui_controller')
        self.signals = signals
        self.running_nodes = []
        self.plan_active = None
        self.forward_goal_pending = False
        self.forward_goal_saw_active = False
        self.navigation_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.task_client = ActionClient(
            self, ExecuteTask, '/executor/execute_task')
        self.create_subscription(
            GoalStatusArray,
            '/executor/execute_task/_action/status',
            self._task_status_callback,
            10,
        )
        self.create_subscription(
            Log,
            '/rosout',
            self._rosout_callback,
            100,
        )
        self.active_goal_handle = None
        self.pending_goal_future = None
        self.pending_goal_action = None
        self.pending_speech_future = None
        self.cancel_requested = False
        self.active_test_goal_id = None
        self.test_goal_terminal_status_reported = False
        self.recovered_test_goal_ids = set()
        self.torso_client = self.create_client(
            MoveTorso, '/hardware/torso/move_torso')
        self.head_client = self.create_client(
            MoveHead, '/hardware/head/move_head')
        self.left_arm_client = self.create_client(
            MoveArms, '/hardware/arms/move_left_arm')
        self.right_arm_client = self.create_client(
            MoveArms, '/hardware/arms/move_right_arm')
        self.left_gripper_client = self.create_client(
            MoveGrippers, '/hardware/grippers/move_left_gripper')
        self.right_gripper_client = self.create_client(
            MoveGrippers, '/hardware/grippers/move_right_gripper')
        self.gpsr_listen_client = self.create_client(
            Parser, '/planning/semantic_parser/listen_gpsr')
        self.talker_client = self.create_client(
            Talk, '/hri/talker/talk')
        self.listener_client = self.create_client(
            Listen, '/hri/listener/listen')
        self.goals_to_clips_client = self.create_client(
            ClipsString, '/planning/goals_to_clips/convert')
        self.forward_goal_publisher = self.create_publisher(
            PoseStamped, '/nav_control/goal', 10)
        self.navigation_stop_publisher = self.create_publisher(
            Bool, '/navigation/navigation_ctrl/stop', 10)
        self.create_subscription(
            Bool,
            '/supervisor/supervisor_plan/active',
            self._plan_active_callback,
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            NavigationGoalStatus,
            '/navigation/status',
            self._navigation_status_callback,
            10,
        )

    def _plan_active_callback(self, message):
        self.plan_active = message.data

    def send_forward_goal(self, distance):
        self.send_relative_goal(distance, 0.0, 'forward')

    def send_return_goal(self, distance):
        self.send_relative_goal(distance, math.pi, 'return')

    def send_relative_goal(self, distance, yaw_delta, description):
        if self.forward_goal_pending:
            self.signals.forward_navigation_status.emit(
                'There is already a demonstration navigation in progress.',
                True, False)
            return
        if self.navigation_active:
            self.signals.forward_navigation_status.emit(
                'Navigation is already active. Wait for it to finish before '
                'sending another goal.',
                True, False,
            )
            return
        if self.forward_goal_publisher.get_subscription_count() == 0:
            self.signals.forward_navigation_status.emit(
                'No receiver for /nav_control/goal. Verify Surge and '
                'navigation are running.',
                True, False,
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except TransformException as error:
            self.get_logger().error(
                f'No se pudo obtener la pose map -> base_link: {error}')
            self.signals.forward_navigation_status.emit(
                'Could not find the map -> base_link transform. Wait for '
                'localization and try again.',
                True, False,
            )
            return

        position = transform.transform.translation
        orientation = transform.transform.rotation
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        target_yaw = yaw + yaw_delta

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = position.x + distance * math.cos(yaw)
        goal.pose.position.y = position.y + distance * math.sin(yaw)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.forward_goal_pending = True
        self.forward_goal_saw_active = False
        self.forward_goal_publisher.publish(goal)
        self.signals.forward_navigation_status.emit(
            f'{description.capitalize()} goal published in map: '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}), '
            f'{distance:.2f} m.',
            False, False,
        )

    def stop_forward_navigation(self):
        if not self.forward_goal_pending:
            return
        stop = Bool()
        stop.data = True
        self.navigation_stop_publisher.publish(stop)
        self.signals.forward_navigation_status.emit(
            'Navigation stop requested.', False, False)

    def _navigation_status_callback(self, message):
        self.navigation_active = (
            message.status == NavigationGoalStatus.ACTIVE)
        if not self.forward_goal_pending:
            return
        if self.navigation_active:
            self.forward_goal_saw_active = True
            self.signals.forward_navigation_status.emit(
                'Navigation active toward the demonstration goal.',
                False, False)
            return

        terminal_statuses = {
            NavigationGoalStatus.SUCCEEDED: ('SUCCEEDED', True),
            NavigationGoalStatus.PREEMPTED: ('PREEMPTED', False),
            NavigationGoalStatus.ABORTED: ('ABORTED', False),
            NavigationGoalStatus.REJECTED: ('REJECTED', False),
            NavigationGoalStatus.RECALLED: ('RECALLED', False),
            NavigationGoalStatus.LOST: ('LOST', False),
        }
        if (
            message.status in terminal_statuses
            and (
                self.forward_goal_saw_active
                or message.status in {
                    NavigationGoalStatus.ABORTED,
                    NavigationGoalStatus.REJECTED,
                }
            )
        ):
            self.forward_goal_pending = False
            self.forward_goal_saw_active = False
            status_name, success = terminal_statuses[message.status]
            detail = f": {message.text}" if message.text else ""
            self.signals.forward_navigation_status.emit(
                f"Navigation {status_name.lower()}{detail}",
                True,
                success,
            )

    def send_task(self, action, argument=''):
        self.cancel_requested = False
        if action == 'say':
            if not self.talker_client.service_is_ready():
                self.signals.task_finished.emit(
                    False, 'The speech service /hri/talker/talk is unavailable.')
                return
            request = Talk.Request()
            request.text = argument
            self.pending_speech_future = self.talker_client.call_async(request)
            self.pending_speech_future.add_done_callback(
                self._task_speech_result_callback)
            return

        self.pending_goal_action = action
        if not self.task_client.wait_for_server(timeout_sec=1.0):
            self.pending_goal_action = None
            self.signals.task_finished.emit(
                False, 'El servidor /executor/execute_task no está disponible.')
            return

        goal = ExecuteTask.Goal()
        goal.task.step_id = 1
        goal.task.agent = 'Justina'
        goal.task.action = action
        if argument.strip():
            goal.task.args = [argument.strip()]

        future = self.task_client.send_goal_async(
            goal, feedback_callback=self._task_feedback_callback)
        self.pending_goal_future = future
        future.add_done_callback(self._task_goal_response_callback)

    def _task_feedback_callback(self, feedback):
        self.signals.task_feedback.emit(feedback.feedback.message)

    def _task_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'No se pudo enviar la acción: {error}')
            self.pending_goal_future = None
            self.pending_goal_action = None
            self.cancel_requested = False
            self.signals.task_finished.emit(False, str(error))
            return

        if not goal_handle.accepted:
            self.pending_goal_future = None
            self.pending_goal_action = None
            self.cancel_requested = False
            self.signals.task_finished.emit(
                False, 'El ejecutor rechazó la acción.')
            return

        self.pending_goal_future = None
        task_action = self.pending_goal_action
        self.pending_goal_action = None
        self.active_goal_handle = goal_handle
        goal_id = tuple(goal_handle.goal_id.uuid)
        if task_action in {'test_head', 'test_torso', 'test_arms'}:
            self.active_test_goal_id = goal_id
            self.test_goal_terminal_status_reported = False
        goal_handle.get_result_async().add_done_callback(
            lambda completed, task_goal_id=goal_id:
                self._task_result_callback(completed, task_goal_id))
        if self.cancel_requested:
            self._send_task_cancel(goal_handle)

    def _task_status_callback(self, message):
        if (
            self.active_test_goal_id is None
            or self.test_goal_terminal_status_reported
        ):
            return
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        }
        for status in message.status_list:
            if (
                tuple(status.goal_info.goal_id.uuid) == self.active_test_goal_id
                and status.status in terminal_statuses
            ):
                self._report_test_terminal(
                    status.status == GoalStatus.STATUS_SUCCEEDED,
                    f'Executor action status is terminal ({status.status}).',
                )
                return

    def _rosout_callback(self, message):
        if (
            self.active_test_goal_id is None
            or self.test_goal_terminal_status_reported
            or 'executor_worker_node' not in message.name
        ):
            return
        test_actions = ('test_head', 'test_torso', 'test_arms')
        for action in test_actions:
            if (
                f'[{action}] SUCCESS:' in message.msg
                or f'[{action}] FAILED:' in message.msg
            ):
                succeeded = f'[{action}] SUCCESS:' in message.msg
                self._report_test_terminal(
                    succeeded,
                    f'{message.msg} (received from /rosout).',
                )
                return

    def _report_test_terminal(self, succeeded, message):
        self.test_goal_terminal_status_reported = True
        self.signals.task_terminal_status.emit(succeeded, message)

    def cancel_task(self):
        self.cancel_requested = True
        if self.active_goal_handle is not None:
            self._send_task_cancel(self.active_goal_handle)
        elif self.pending_speech_future is not None:
            self.signals.task_cancel_feedback.emit(
                True,
                'Speech cannot be interrupted; queued actions are canceled and '
                'the controls will be restored when speech ends.',
            )
        elif self.pending_goal_future is None:
            self.cancel_requested = False
            self.signals.task_cancel_feedback.emit(
                False, 'No hay una acción del executor que cancelar.')

    def _task_speech_result_callback(self, future):
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message or (
                'Speech completed.' if success else 'Speech failed.')
        except Exception as error:
            self.get_logger().error(f'Error while speaking task text: {error}')
            success = False
            message = str(error)
        self.pending_speech_future = None
        self.signals.task_finished.emit(success, message)

    def _send_task_cancel(self, goal_handle):
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._task_cancel_callback)

    def _task_cancel_callback(self, future):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
            message = (
                'Cancelación aceptada; esperando a que termine la acción actual.'
                if accepted else 'El executor rechazó la solicitud de cancelación.')
        except Exception as error:
            self.get_logger().error(
                f'Error al cancelar la acción del executor: {error}')
            accepted = False
            message = f'No se pudo solicitar la cancelación: {error}'
        if not accepted:
            self.cancel_requested = False
        self.signals.task_cancel_feedback.emit(accepted, message)

    def _task_result_callback(self, future, goal_id):
        if goal_id in self.recovered_test_goal_ids:
            self.recovered_test_goal_ids.remove(goal_id)
            return
        try:
            wrapped_result = future.result()
            task_result = wrapped_result.result.result
            success = (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and task_result.status == TaskResult.SUCCESS
            )
            message = task_result.message or (
                'Acción completada.' if success else 'La acción falló.')
        except Exception as error:
            self.get_logger().error(f'Error al recibir resultado de acción: {error}')
            success = False
            message = str(error)
        self.active_goal_handle = None
        if goal_id == self.active_test_goal_id:
            self.active_test_goal_id = None
            self.test_goal_terminal_status_reported = False
        self.signals.task_finished.emit(success, message)

    def mark_test_result_recovered(self):
        if self.active_test_goal_id is not None:
            goal_id = self.active_test_goal_id
            self.recovered_test_goal_ids.add(goal_id)
            if (
                self.active_goal_handle is not None
                and tuple(self.active_goal_handle.goal_id.uuid) == goal_id
            ):
                self.active_goal_handle = None
            self.active_test_goal_id = None
            self.test_goal_terminal_status_reported = False

    def move_torso(self, position):
        request = MoveTorso.Request()
        request.position = position
        self._send_service_request(
            self.torso_client, request, 'Torso')

    def move_head(self, positions):
        request = MoveHead.Request()
        request.positions = positions
        self._send_service_request(
            self.head_client, request, 'Cabeza')

    def move_arm(self, side, positions):
        request = MoveArms.Request()
        request.positions = positions
        request.speed = 50
        client = (
            self.left_arm_client if side == 'izquierdo'
            else self.right_arm_client
        )
        self._send_service_request(
            client, request, f'Brazo {side}')

    def move_gripper(self, side, command):
        request = MoveGrippers.Request()
        request.cmd = command
        client = (
            self.left_gripper_client if side == 'izquierdo'
            else self.right_gripper_client
        )
        self._send_service_request(
            client, request, f'Gripper {side}')

    def _send_service_request(self, client, request, label):
        if not client.service_is_ready():
            self.signals.hardware_finished.emit(
                label, False, 'Servicio no disponible.')
            return

        future = client.call_async(request)
        future.add_done_callback(
            lambda completed: self._service_result_callback(
                completed, label))

    def _service_result_callback(self, future, label):
        try:
            response = future.result()
            success = response.success
            message = response.message
        except Exception as error:
            self.get_logger().error(
                f'Error en el servicio de hardware ({label}): {error}')
            success = False
            message = str(error)
        self.signals.hardware_finished.emit(label, success, message)

    def request_gpsr_command(self):
        if not self.talker_client.service_is_ready():
            self.signals.gpsr_listen_finished.emit(
                False, 'Servicio de voz /hri/talker/talk no disponible.')
            return
        request = Talk.Request()
        request.text = 'Please give me a command.'
        future = self.talker_client.call_async(request)
        future.add_done_callback(self._gpsr_prompt_callback)

    def listen_for_name(self):
        if not self.listener_client.service_is_ready():
            self.signals.name_listen_finished.emit(
                False, 'Speech recognition service /hri/listener/listen is unavailable.')
            return
        request = Listen.Request()
        request.enable = True
        request.grammar = ''
        future = self.listener_client.call_async(request)
        future.add_done_callback(self._name_listen_callback)

    def _name_listen_callback(self, future):
        try:
            response = future.result()
            self.signals.name_listen_finished.emit(
                bool(response.success), response.recognized_speech)
        except Exception as error:
            self.get_logger().error(
                f'Error while listening for a visitor name: {error}')
            self.signals.name_listen_finished.emit(False, str(error))

    def _gpsr_prompt_callback(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Error al pedir la instrucción GPSR: {error}')
            self.signals.gpsr_listen_finished.emit(False, str(error))
            return
        if not response.success:
            self.signals.gpsr_listen_finished.emit(
                False, response.message or 'No se pudo emitir el mensaje de voz.')
            return
        if not self.gpsr_listen_client.service_is_ready():
            self.signals.gpsr_listen_finished.emit(
                False, 'Servicio GPSR/listen_gpsr no disponible.')
            return
        request = Parser.Request()
        request.command = ''
        future = self.gpsr_listen_client.call_async(request)
        future.add_done_callback(self._gpsr_listen_callback)

    def _gpsr_listen_callback(self, future):
        try:
            response = future.result()
            self.signals.gpsr_listen_finished.emit(
                bool(response.response), response.response)
        except Exception as error:
            self.get_logger().error(f'Error al escuchar el comando GPSR: {error}')
            self.signals.gpsr_listen_finished.emit(False, str(error))

    def submit_gpsr_goals(self, goals_json):
        if not self.goals_to_clips_client.service_is_ready():
            self.signals.gpsr_plan_finished.emit(
                False, 'Servicio goals_to_clips no disponible.')
            return
        request = ClipsString.Request()
        request.data = goals_json
        future = self.goals_to_clips_client.call_async(request)
        future.add_done_callback(self._gpsr_plan_callback)

    def _gpsr_plan_callback(self, future):
        try:
            response = future.result()
            success = response.data == 'OK'
            message = (
                'Plan GPSR enviado a CLIPS.'
                if success else response.data or 'Falló la creación del plan.')
        except Exception as error:
            self.get_logger().error(f'Error al crear el plan GPSR: {error}')
            success = False
            message = str(error)
        self.signals.gpsr_plan_finished.emit(success, message)

    def refresh_node_list(self):
        try:
            if USE_ROS2NODE_API:
                node_list = get_node_names(node=self, include_hidden_nodes=False)
                self.running_nodes = [full_name for (_, _, full_name) in node_list]
            else:
                self.running_nodes = self.get_node_names()
        except Exception as e:
            self.get_logger().error(f"Error al obtener nodos: {e}")
            self.running_nodes = []

        # Excluir nuestro propio nodo
        our_name = '/' + self.get_name()
        if our_name in self.running_nodes:
            self.running_nodes.remove(our_name)

    def kill_node_by_name(self, node_name: str):
        """Mata un nodo ROS2 usando pkill con el patrón __node:=<nombre>."""
        short_name = node_name.split('/')[-1]
        pattern = f"__node:={node_name}|__node:={short_name}"
        try:
            pgrep_cmd = f"pgrep -f '{pattern}'"
            result = subprocess.run(pgrep_cmd, shell=True, capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                pids = result.stdout.strip().split()
                for pid in pids:
                    ps_info = subprocess.run(['ps', '-p', pid, '-o', 'args='], 
                                             capture_output=True, text=True, timeout=2)
                    if 'gui_controller' in ps_info.stdout:
                        self.get_logger().info(f"  Omitiendo PID {pid} (es el propio GUI controller)")
                        continue
                    os.kill(int(pid), signal.SIGKILL)
                    self.get_logger().info(f"  Nodo {node_name} eliminado (PID {pid})")
                return True
            else:
                self.get_logger().info(f"  No se encontraron procesos para {node_name}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error al matar {node_name}: {e}")
            return False

    def kill_orphan_nodes(self, keep_nodes: list = None):
        """Elimina todos los nodos ROS excepto los que se quieren conservar."""
        self.refresh_node_list()
        current_nodes = set(self.running_nodes)
        our_name = '/' + self.get_name()
        nodes_to_kill = current_nodes - {our_name}
        if keep_nodes is not None:
            nodes_to_kill -= set(keep_nodes)
        if not nodes_to_kill:
            return
        self.get_logger().info(f"Eliminando {len(nodes_to_kill)} nodos residuales...")
        for node_name in nodes_to_kill:
            self.kill_node_by_name(node_name)
        self.get_logger().info("Limpieza completada.")