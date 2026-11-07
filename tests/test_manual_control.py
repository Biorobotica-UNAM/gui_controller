"""A loaded plan must not reserve manual control before Supervisor starts."""
import os
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtWidgets import QApplication, QMessageBox
from gui_controller.gui_controller import GuiSignals, NodeControlPanel


@pytest.fixture
def panel(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(NodeControlPanel, 'load_known_locations', lambda self: None)
    ros = Mock(plan_active=True, navigation_active=False, running_nodes=[], cancel_requested=False)
    window = NodeControlPanel(ros, GuiSignals())
    window.timer.stop()
    window.process_monitor_timer.stop()
    yield window
    window.deleteLater()
    app.processEvents()


@pytest.mark.parametrize('state', [None, False, True])
def test_loaded_plan_without_manager_allows_manual_control(panel, state):
    panel.ros_node.plan_active = state
    assert panel._can_control_robot()
    panel.refresh_node_list()
    assert 'Supervisor sin iniciar' in panel.plannerStatusLabel.text()
    assert panel.gpsrButton.isEnabled()


@pytest.mark.parametrize('external', [False, True])
def test_running_manager_with_loaded_plan_blocks_control(panel, monkeypatch, external):
    monkeypatch.setattr(QMessageBox, 'warning', Mock())
    if external:
        panel.ros_node.running_nodes = ['/supervisor/supervisor_manager_node']
    else:
        panel.processes['supervisor'] = Mock()
        panel.processes['supervisor'].command_finished.return_value = False
    assert not panel._can_control_robot()
    panel.refresh_node_list()
    assert not panel.gpsrButton.isEnabled()


@pytest.mark.parametrize('busy', ['task_busy', 'hardware_busy', 'gpsr_busy', 'forward_navigation_busy'])
def test_start_supervisor_during_manual_operation_is_rejected(panel, monkeypatch, busy):
    monkeypatch.setattr(QMessageBox, 'information', Mock())
    launch = Mock()
    monkeypatch.setattr(panel, '_launch_in_terminal', launch)
    setattr(panel, busy, True)
    panel.start_supervisor()
    launch.assert_not_called()


def test_start_supervisor_with_loaded_plan_and_no_manual_operation(panel, monkeypatch):
    launch = Mock()
    launch.return_value.command_finished.return_value = False
    monkeypatch.setattr(panel, '_launch_in_terminal', launch)
    panel.config['aliases']['justina_ws'] = '/tmp/justina-test'
    panel.start_supervisor()
    launch.assert_called_once()
    assert not panel.startSupervisorButton.isEnabled()
    assert panel._planner_blocks_manual_control()


@pytest.mark.parametrize('index,action', [(0, 'test_head'), (1, 'test_torso'), (2, 'test_arms')])
def test_hardware_test_buttons_send_named_action(panel, index, action):
    panel.hardwareTestButtons[index].click()
    panel.ros_node.send_task.assert_called_once_with(action, '')


def test_say_without_text_stays_local(panel, monkeypatch):
    warning = Mock()
    monkeypatch.setattr(QMessageBox, 'warning', warning)
    panel.actionComboBox.setCurrentText('say')
    panel.actionArgumentEdit.clear()
    panel.runActionButton.click()
    panel.ros_node.send_task.assert_not_called()
    warning.assert_called_once()


def test_say_preserves_user_text(panel):
    panel.actionComboBox.setCurrentText('say')
    panel.actionArgumentEdit.setText('Hello Justina')
    panel.runActionButton.click()
    panel.ros_node.send_task.assert_called_once_with('say', 'Hello Justina')


def test_feedback_uses_ros_action_wrapper():
    from executor_interfaces.action import ExecuteTask
    from gui_controller.ros_checker import ROS2NodeChecker
    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.get_logger = Mock()
    message = ExecuteTask.Impl.FeedbackMessage()
    message.feedback.feedback.message = '[test_head] LOOKING_LEFT: left'
    checker._task_feedback_callback(message)
    checker.signals.task_feedback.emit.assert_called_once_with('[test_head] LOOKING_LEFT: left')


@pytest.mark.parametrize('action,argument', [('', ''), ('say', ''), ('say', '   ')])
def test_empty_requests_never_reach_ros(action, argument):
    from gui_controller.ros_checker import ROS2NodeChecker
    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.get_logger = Mock()
    checker.task_client = Mock()
    checker.talker_client = Mock()
    checker.send_task(action, argument)
    checker.task_client.send_goal_async.assert_not_called()
    checker.talker_client.call_async.assert_not_called()
    assert checker.signals.task_finished.emit.call_args.args[0] is False


# Every action uses the same ROS wrapper; exercise actual generated messages.
ACTIONS = [
    'say', 'listen', 'go_to', 'check', 'point', 'give', 'find_person',
    'align', 'drop', 'find_space', 'find_object', 'focus', 'receive',
    'follow', 'approach', 'approach_partial', 'take', 'verify_take',
    'find_pose', 'analyze_objects', 'analyze_person', 'save_loc', 'fold',
    'test_head', 'test_torso', 'test_arms',
]


@pytest.mark.parametrize('action', ACTIONS)
def test_every_action_preserves_name_arguments_and_ros_feedback(panel, action):
    from concurrent.futures import Future
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus
    from executor_interfaces.action import ExecuteTask
    from executor_interfaces.msg import TaskResult
    from gui_controller.ros_checker import ROS2NodeChecker
    from rclpy.serialization import serialize_message, deserialize_message
    from talker_interfaces.srv import Talk

    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.get_logger = Mock()
    checker.task_client = Mock()
    checker.talker_client = Mock()
    checker.task_client.send_goal_async.return_value = Future()
    checker.talker_client.call_async.return_value = Future()
    panel.ros_node.send_task.side_effect = checker.send_task
    panel.actionComboBox.setCurrentText(action)
    panel.actionArgumentEdit.setText('sample argument with spaces')
    panel.runActionButton.click()
    # Complete the spoken announcement to release the requested action.
    if action not in {'say', 'test_head', 'test_torso', 'test_arms'}:
        announcement = checker.talker_client.call_async.call_args.args[0]
        assert announcement.text.strip()
        panel._on_task_finished(True, 'announcement completed')

    if action == 'say':
        request = checker.talker_client.call_async.call_args.args[0]
        decoded = deserialize_message(serialize_message(request), Talk.Request)
        assert decoded.text == 'sample argument with spaces'
        checker.task_client.send_goal_async.assert_not_called()
        return

    if action == 'listen':
        panel.ros_node.listen_and_repeat.assert_called_once_with()
        checker.task_client.send_goal_async.assert_not_called()
        return

    goal = checker.task_client.send_goal_async.call_args.args[0]
    decoded = deserialize_message(serialize_message(goal), ExecuteTask.Goal)
    assert decoded.task.action == action
    assert decoded.task.agent == 'Justina'
    assert list(decoded.task.args) == ['sample argument with spaces']
    feedback = ExecuteTask.Impl.FeedbackMessage()
    feedback.feedback.feedback.message = f'[{action}] progress'
    checker._task_feedback_callback(feedback)
    checker.signals.task_feedback.emit.assert_called_once_with(f'[{action}] progress')
    result = ExecuteTask.Result()
    result.result.status = TaskResult.SUCCESS
    result.result.message = f'{action} completed'
    future = Future()
    future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result))
    checker.recovered_test_goal_ids = set()
    checker.active_test_goal_id = None
    checker._task_result_callback(future, (1,))
    checker.signals.task_finished.emit.assert_called_once_with(True, f'{action} completed')


def test_all_listed_actions_are_registered_in_worker(panel):
    import re
    from pathlib import Path
    source = (Path(__file__).resolve().parents[2] / 'executor_worker/src/executor_worker_node.cpp').read_text()
    registered = set(re.findall(r'actionMap_\["([^"]+)"\]', source))
    listed = {panel.actionComboBox.itemText(i) for i in range(panel.actionComboBox.count())}
    assert listed == set(ACTIONS)
    assert listed <= registered


@pytest.mark.parametrize('component', ['torso', 'head', 'left_arm', 'right_arm', 'left_gripper', 'right_gripper'])
def test_manual_hardware_services_preserve_payload_and_result(component):
    from concurrent.futures import Future
    from gui_controller.ros_checker import ROS2NodeChecker
    from rclpy.serialization import serialize_message, deserialize_message
    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.get_logger = Mock()
    client = Mock()
    future = Future()
    client.call_async.return_value = future
    setattr(checker, component + '_client', client)
    if component == 'torso':
        checker.move_torso(0.2)
        field, expected = 'position', 0.2
    elif component == 'head':
        checker.move_head([0.3, -0.2])
        field, expected = 'positions', [0.3, -0.2]
    elif component.endswith('arm'):
        checker.move_arm('izquierdo' if component.startswith('left') else 'derecho', [0.2] * 6)
        field, expected = 'positions', [0.2] * 6
    else:
        checker.move_gripper('izquierdo' if component.startswith('left') else 'derecho', 'open')
        field, expected = 'cmd', 'open'
    request = client.call_async.call_args.args[0]
    decoded = deserialize_message(serialize_message(request), type(request))
    value = getattr(decoded, field)
    assert (list(value) if field == 'positions' else value) == expected
    from types import SimpleNamespace
    future.set_result(SimpleNamespace(success=True, message='completed'))
    assert checker.signals.hardware_finished.emit.call_args.args[1:] == (True, 'completed')


@pytest.mark.parametrize('button,method,args', [
    ('moveTorsoButton', 'move_torso', (0.0,)),
    ('moveHeadButton', 'move_head', ([0.0, 0.0],)),
    ('moveArmButton', 'move_arm', ('izquierdo', [0.0] * 6)),
    (0, 'move_gripper', ('izquierdo', 'open')),
    (1, 'move_gripper', ('izquierdo', 'close')),
    (2, 'move_gripper', ('derecho', 'open')),
    (3, 'move_gripper', ('derecho', 'close')),
])
def test_manual_buttons_announce_then_send_correct_command(panel, button, method, args):
    widget = panel.gripperButtons[button] if isinstance(button, int) else getattr(panel, button)
    widget.click()
    assert panel.ros_node.send_task.call_args.args[0] == 'say'
    assert panel.ros_node.send_task.call_args.args[1].strip()
    getattr(panel.ros_node, method).assert_not_called()
    panel._on_task_finished(True, 'announcement completed')
    getattr(panel.ros_node, method).assert_called_once_with(*args)
    panel._on_hardware_finished('test', True, 'completed')
    assert not panel.hardware_busy
    assert panel.runActionButton.isEnabled()


def test_ros_environment_is_set_before_starting_nodes(monkeypatch, tmp_path):
    from gui_controller.gui_controller import configure_ros_environment
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    config = tmp_path / 'config.yaml'
    config.write_text('ros_environment:\n  RMW_IMPLEMENTATION: rmw_cyclonedds_cpp\n  ROS_DOMAIN_ID: "0"\n')
    configure_ros_environment(config)
    assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert os.environ['ROS_DOMAIN_ID'] == '0'


def test_launch_commands_inherit_gui_ros_environment(panel):
    for section in ('surge', 'surge_simulation', 'supervisor'):
        assert not any('export RMW_IMPLEMENTATION' in command
                       for command in panel.config[section]['commands'])
    assert panel.config['ros_environment']['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'


def test_gui_import_does_not_load_wrong_middleware():
    import subprocess
    import sys
    code = '''
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
from gui_controller.gui_controller import configure_ros_environment
configure_ros_environment()
from gui_controller.ros_checker import ROS2NodeChecker
from rclpy.utilities import get_rmw_implementation_identifier
assert get_rmw_implementation_identifier() == 'rmw_cyclonedds_cpp'
'''
    subprocess.run([sys.executable, '-c', code], check=True, capture_output=True, text=True)


@pytest.mark.parametrize('recognized', ['Yes.', 'Hello, my name is Sergio.'])
def test_listen_demo_repeats_raw_speech_without_executor_or_parser(recognized):
    from concurrent.futures import Future
    from listener_interfaces.srv import Listen
    from talker_interfaces.srv import Talk
    from gui_controller.ros_checker import ROS2NodeChecker
    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.listener_client = Mock()
    checker.talker_client = Mock()
    checker.task_client = Mock()
    heard = Future()
    spoken = Future()
    checker.listener_client.call_async.return_value = heard
    checker.talker_client.call_async.return_value = spoken
    checker.listen_and_repeat()
    request = checker.listener_client.call_async.call_args.args[0]
    assert request.enable and request.grammar == ''
    heard.set_result(Listen.Response(success=True, recognized_speech=recognized))
    speech_request = checker.talker_client.call_async.call_args.args[0]
    assert speech_request.text == f'I heard this: {recognized}'
    checker.task_client.send_goal_async.assert_not_called()
    checker.signals.task_finished.emit.assert_not_called()
    spoken.set_result(Talk.Response(success=True, message='spoken'))
    checker.signals.task_finished.emit.assert_called_once_with(True, 'spoken')


@pytest.mark.parametrize('cancel,success,text', [
    (True, True, 'Yes.'), (False, False, ''), (False, True, '   '),
])
def test_listen_demo_does_not_repeat_canceled_or_empty_speech(cancel, success, text):
    from concurrent.futures import Future
    from listener_interfaces.srv import Listen
    from gui_controller.ros_checker import ROS2NodeChecker
    checker = ROS2NodeChecker.__new__(ROS2NodeChecker)
    checker.signals = Mock()
    checker.listener_client = Mock()
    checker.talker_client = Mock()
    checker.active_goal_handle = None
    future = Future()
    checker.listener_client.call_async.return_value = future
    checker.listen_and_repeat()
    if cancel:
        checker.cancel_task()
    future.set_result(Listen.Response(success=success, recognized_speech=text))
    checker.talker_client.call_async.assert_not_called()
    assert checker.signals.task_finished.emit.call_args.args[0] is False
