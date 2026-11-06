#!/usr/bin/env python3
# gui_controller.py

import os
import re
import sys
import json
import math
import yaml
import threading
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from PyQt5 import sip, uic
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QMainWindow,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

import rclpy
from rclpy.executors import SingleThreadedExecutor

from gui_controller.launcher import ProcessLauncher
from gui_controller.ros_checker import ROS2NodeChecker

SCRIPT_DIR = Path(__file__).parent.resolve()
CONFIG_PATH = SCRIPT_DIR / "config.yaml"
NODES_PATH = SCRIPT_DIR / "nodes.yaml"
ALIAS_PATTERN = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")
ENV_VAR_PATTERN = re.compile(r"\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))")


class GuiSignals(QObject):
    task_feedback = pyqtSignal(str)
    task_terminal_status = pyqtSignal(bool, str)
    task_finished = pyqtSignal(bool, str)
    task_cancel_feedback = pyqtSignal(bool, str)
    hardware_finished = pyqtSignal(str, bool, str)
    name_listen_finished = pyqtSignal(bool, str)
    gpsr_listen_finished = pyqtSignal(bool, str)
    gpsr_plan_finished = pyqtSignal(bool, str)
    forward_navigation_status = pyqtSignal(str, bool, bool)


class NodeControlPanel(QMainWindow):
    def __init__(self, ros_node, signals):
        super().__init__()
        uic.loadUi(os.path.join(SCRIPT_DIR, 'node_control.ui'), self)

        self.ros_node = ros_node
        self.signals = signals
        self.processes = {'surge': None, 'supervisor': None}
        self.finished_terminals = {'surge': None, 'supervisor': None}
        self.GPSR = False
        self.task_sequence = []
        self.task_sequence_finished = None
        self.presentation_sequence = None
        self.presentation_cancel_requested = False
        self.interactive_introduction = False
        self.base_mobility_sequence = None
        self.base_mobility_waiting_for_speech = False
        self.base_mobility_cancel_requested = False
        self.task_busy = False
        self.hardware_busy = False
        self.gpsr_busy = False
        self.forward_navigation_busy = False
        self.active_task_action = None
        self.test_feedback_terminal_action = None
        self.test_feedback_recovery_message = ""
        self.test_feedback_recovery_timer = QTimer(self)
        self.test_feedback_recovery_timer.setSingleShot(True)
        self.test_feedback_recovery_timer.setInterval(2000)
        self.test_feedback_recovery_timer.timeout.connect(
            self._recover_from_missing_test_result)
        self._install_demo_tabs()
        self._connect_demo_signals()

        # Cargar configuración de comandos y perfiles
        self.config = self.load_config()
        self.load_planning_profiles()
        self.load_known_locations()

        # Cargar lista de nodos esperados
        self.expected_nodes = self.load_expected_nodes()
        if self.expected_nodes:
            self.ros_node.kill_orphan_nodes(self.expected_nodes)

        # Conectar botones
        self.startSurgeButton.clicked.connect(self.start_surge)
        self.startSupervisorButton.clicked.connect(self.start_supervisor)

        # Timers para refrescar nodos y monitorear procesos
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_node_list)
        self.timer.start(1000)

        self.process_monitor_timer = QTimer()
        self.process_monitor_timer.timeout.connect(self.check_processes_status)
        self.process_monitor_timer.start(2000)

        self.refresh_node_list()

    def _install_demo_tabs(self):
        launch_layout = self.centralwidget.layout()
        launch_page = QWidget()
        launch_page_layout = QVBoxLayout(launch_page)
        while launch_layout.count():
            item = launch_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(launch_page)
                launch_page_layout.addWidget(widget)
        sip.delete(launch_layout)

        self.blankMapCheckBox = QCheckBox(
            "Iniciar Surge con el mapa en blanco (AMCL)"
        )
        self.blankMapCheckBox.setToolTip(
            "Carga el mapa en blanco existente mediante la navegación AMCL normal. "
            "El perfil Restaurant también usa este mapa."
        )
        launch_page_layout.addWidget(self.blankMapCheckBox)

        self.demoTabs = QTabWidget()
        self.demoTabs.addTab(launch_page, "Lanzamiento")
        self.demoTabs.addTab(self._create_actions_tab(), "Acciones")
        self.demoTabs.addTab(self._create_hardware_tab(), "Hardware")
        self.demoTabs.addTab(self._create_demos_tab(), "Demostraciones")

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.demoTabs)
        self.centralwidget.setLayout(main_layout)
        self.resize(1000, 850)

    def _create_actions_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        instructions = QLabel(
            "Ejecuta una acción del executor. Los argumentos se envían como "
            "un solo texto, igual que en los planes."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        action_group = QGroupBox("Acciones disponibles en executor_worker")
        action_form = QFormLayout(action_group)
        self.actionComboBox = QComboBox()
        self.actionComboBox.addItems([
            "say", "listen", "go_to", "check", "point", "give",
            "find_person", "align", "drop", "find_space", "find_object",
            "focus", "receive", "follow", "approach", "approach_partial",
            "take", "verify_take", "find_pose", "analyze_objects",
            "analyze_person", "save_loc", "fold", "test_head",
            "test_torso", "test_arms",
        ])
        self.actionArgumentEdit = QLineEdit()
        self.actionArgumentEdit.setPlaceholderText(
            "Ejemplos: kitchen | apple | hello Justina"
        )
        action_form.addRow("Acción:", self.actionComboBox)
        action_form.addRow("Argumento:", self.actionArgumentEdit)
        layout.addWidget(action_group)

        buttons = QHBoxLayout()
        self.runActionButton = QPushButton("Ejecutar acción")
        self.cancelActionButton = QPushButton("Cancelar acción")
        self.cancelActionButton.setEnabled(False)
        buttons.addWidget(self.runActionButton)
        buttons.addWidget(self.cancelActionButton)
        layout.addLayout(buttons)

        self.actionLog = QTextEdit()
        self.actionLog.setReadOnly(True)
        self.actionLog.setPlaceholderText("Estado y feedback de las acciones.")
        layout.addWidget(self.actionLog)
        self.runActionButton.clicked.connect(self._run_selected_action)
        self.cancelActionButton.clicked.connect(self._cancel_action)
        return page

    def _create_hardware_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        intro = QLabel(
            "Los movimientos predefinidos reutilizan las rutinas test_head, "
            "test_torso y test_arms del executor. La pose manual de brazos "
            "solo ofrece reposo y saludo ya definidos en el proyecto."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        tests_group = QGroupBox("Pruebas predefinidas")
        tests_layout = QHBoxLayout(tests_group)
        self.hardwareTestButtons = []
        for label, action in (
            ("Probar cabeza", "test_head"),
            ("Probar torso", "test_torso"),
            ("Probar brazos y saludo", "test_arms"),
        ):
            button = QPushButton(label)
            self.hardwareTestButtons.append(button)
            button.clicked.connect(
                lambda checked=False, task=action, title=label:
                    self._start_task_sequence([(task, "")], title))
            tests_layout.addWidget(button)
        layout.addWidget(tests_group)

        torso_group = QGroupBox("Torso")
        torso_form = QFormLayout(torso_group)
        self.torsoPosition = QDoubleSpinBox()
        self.torsoPosition.setRange(0.0, 0.30)
        self.torsoPosition.setSingleStep(0.05)
        self.torsoPosition.setDecimals(2)
        self.torsoPosition.setSuffix(" m")
        self.moveTorsoButton = QPushButton("Mover torso")
        torso_form.addRow("Altura (0–0.30 m):", self.torsoPosition)
        torso_form.addRow(self.moveTorsoButton)
        layout.addWidget(torso_group)

        head_group = QGroupBox("Cabeza")
        head_form = QFormLayout(head_group)
        self.headPan = QDoubleSpinBox()
        self.headPan.setRange(-0.60, 0.60)
        self.headPan.setSingleStep(0.10)
        self.headPan.setDecimals(2)
        self.headPan.setSuffix(" rad")
        self.headTilt = QDoubleSpinBox()
        self.headTilt.setRange(-0.70, 0.40)
        self.headTilt.setSingleStep(0.10)
        self.headTilt.setDecimals(2)
        self.headTilt.setSuffix(" rad")
        self.moveHeadButton = QPushButton("Mover cabeza")
        head_form.addRow("Pan:", self.headPan)
        head_form.addRow("Tilt:", self.headTilt)
        head_form.addRow(self.moveHeadButton)
        layout.addWidget(head_group)

        arms_group = QGroupBox("Brazo individual")
        arms_form = QFormLayout(arms_group)
        self.armSideComboBox = QComboBox()
        self.armSideComboBox.addItems(["izquierdo", "derecho"])
        self.armPoseComboBox = QComboBox()
        self.armPoseComboBox.addItem("Reposo")
        self.armPoseComboBox.addItem("Saludo")
        self.moveArmButton = QPushButton("Mover brazo seleccionado")
        arms_form.addRow("Lado:", self.armSideComboBox)
        arms_form.addRow("Pose conocida:", self.armPoseComboBox)
        arms_form.addRow(self.moveArmButton)
        layout.addWidget(arms_group)

        gripper_group = QGroupBox("Grippers")
        gripper_layout = QHBoxLayout(gripper_group)
        self.gripperButtons = []
        for side in ("izquierdo", "derecho"):
            side_layout = QVBoxLayout()
            side_layout.addWidget(QLabel(f"Gripper {side}"))
            for label, command in (("Abrir", "open"), ("Cerrar", "close")):
                button = QPushButton(label)
                self.gripperButtons.append(button)
                button.clicked.connect(
                    lambda checked=False, selected_side=side, cmd=command:
                        self._move_gripper(selected_side, cmd))
                side_layout.addWidget(button)
            gripper_layout.addLayout(side_layout)
        layout.addWidget(gripper_group)

        self.hardwareLog = QTextEdit()
        self.hardwareLog.setReadOnly(True)
        self.hardwareLog.setMaximumHeight(120)
        self.hardwareLog.setPlaceholderText("Resultado de los movimientos.")
        layout.addWidget(self.hardwareLog)

        self.moveTorsoButton.clicked.connect(self._move_torso)
        self.moveHeadButton.clicked.connect(self._move_head)
        self.moveArmButton.clicked.connect(self._move_selected_arm)
        return page

    def _create_demos_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        presentation = QGroupBox("Presentación de Justina")
        presentation_layout = QVBoxLayout(presentation)
        presentation_layout.addWidget(QLabel(
            "Describe cada movimiento antes de elevar el torso, girar la cabeza "
            "y levantar/bajar cada brazo."
        ))
        self.presentationButton = QPushButton("Iniciar presentación")
        presentation_layout.addWidget(self.presentationButton)
        layout.addWidget(presentation)

        choreo = QGroupBox("Coreografía breve")
        choreo_layout = QVBoxLayout(choreo)
        choreo_layout.addWidget(QLabel(
            "Anuncia cada movimiento individual de cabeza y brazos antes de ejecutarlo."
        ))
        self.choreoButton = QPushButton("Iniciar coreografía")
        choreo_layout.addWidget(self.choreoButton)
        layout.addWidget(choreo)

        introduction = QGroupBox("Conoce a Justina")
        introduction_layout = QVBoxLayout(introduction)
        introduction_layout.addWidget(QLabel(
            "Justina se presentará, saludará con una mano, preguntará tu "
            "nombre y te dirá que es un gusto conocerte."
        ))
        self.introductionButton = QPushButton("Presentarse y saludar")
        introduction_layout.addWidget(self.introductionButton)
        layout.addWidget(introduction)

        navigation = QGroupBox("Navegación a destino")
        navigation_form = QFormLayout(navigation)
        navigation_form.addRow(QLabel(
            "Usa la acción go_to y una ubicación conocida por el robot."
        ))
        self.destinationEdit = QComboBox()
        self.destinationEdit.setEditable(True)
        self.destinationEdit.setInsertPolicy(QComboBox.NoInsert)
        self.destinationEdit.setPlaceholderText("Selecciona o escribe una ubicación")
        self.goToButton = QPushButton("Ir a ubicación")
        navigation_form.addRow("Destino:", self.destinationEdit)
        navigation_form.addRow(self.goToButton)
        navigation_form.addRow(QLabel(
            "Avanzar y regresar usan la navegación normal de Surge. Regresar "
            "gira a Justina y envía el objetivo a la distancia seleccionada "
            "detrás de su pose actual."
        ))
        self.forwardDistance = QDoubleSpinBox()
        self.forwardDistance.setRange(0.5, 5.0)
        self.forwardDistance.setSingleStep(0.25)
        self.forwardDistance.setDecimals(2)
        self.forwardDistance.setValue(2.0)
        self.forwardDistance.setSuffix(" m")
        self.forwardGoalButton = QPushButton("Enviar objetivo al frente")
        self.returnGoalButton = QPushButton("Regresar esa distancia")
        self.stopForwardGoalButton = QPushButton("Detener navegación")
        self.stopForwardGoalButton.setEnabled(False)
        navigation_form.addRow("Distancia:", self.forwardDistance)
        forward_buttons = QHBoxLayout()
        forward_buttons.addWidget(self.forwardGoalButton)
        forward_buttons.addWidget(self.returnGoalButton)
        forward_buttons.addWidget(self.stopForwardGoalButton)
        navigation_form.addRow(forward_buttons)

        mobility_group = QGroupBox("Demostración de movilidad de la base")
        mobility_layout = QVBoxLayout(mobility_group)
        mobility_layout.addWidget(QLabel(
            "Justina anunciará y ejecutará giros de 90 grados, y después "
            "avanzará y retrocederá medio metro."
        ))
        self.baseMobilityButton = QPushButton("Demostrar movilidad de la base")
        mobility_layout.addWidget(self.baseMobilityButton)
        navigation_form.addRow(mobility_group)
        layout.addWidget(navigation)

        gpsr_group = QGroupBox("GPSR de un comando, sin «continue»")
        gpsr_layout = QVBoxLayout(gpsr_group)
        gpsr_layout.addWidget(QLabel(
            "Escucha una instrucción, muestra la traducción semántica y pide "
            "confirmación antes de enviarla al planificador CLIPS."
        ))
        self.gpsrButton = QPushButton("Escuchar comando GPSR")
        self.gpsrButton.setEnabled(False)
        gpsr_layout.addWidget(self.gpsrButton)
        layout.addWidget(gpsr_group)

        self.demoLog = QTextEdit()
        self.demoLog.setReadOnly(True)
        self.demoLog.setPlaceholderText("Progreso de las demostraciones.")
        layout.addWidget(self.demoLog)

        self.presentationButton.clicked.connect(self._run_presentation)
        self.choreoButton.clicked.connect(self._run_choreography)
        self.introductionButton.clicked.connect(
            self._run_interactive_introduction)
        self.goToButton.clicked.connect(self._run_go_to)
        self.forwardGoalButton.clicked.connect(self._run_forward_navigation)
        self.returnGoalButton.clicked.connect(self._run_return_navigation)
        self.baseMobilityButton.clicked.connect(self._run_base_mobility_demo)
        self.stopForwardGoalButton.clicked.connect(
            self._stop_forward_navigation)
        self.gpsrButton.clicked.connect(self._request_gpsr_command)
        return page

    def _connect_demo_signals(self):
        self.signals.task_feedback.connect(self._on_task_feedback)
        self.signals.task_terminal_status.connect(
            self._on_task_terminal_status)
        self.signals.task_finished.connect(self._on_task_finished)
        self.signals.task_cancel_feedback.connect(
            self._on_task_cancel_feedback)
        self.signals.hardware_finished.connect(self._on_hardware_finished)
        self.signals.name_listen_finished.connect(
            self._on_introduction_name_listened)
        self.signals.gpsr_listen_finished.connect(self._on_gpsr_listen_finished)
        self.signals.gpsr_plan_finished.connect(self._on_gpsr_plan_finished)
        self.signals.forward_navigation_status.connect(
            self._on_forward_navigation_status)

    def _can_control_robot(self):
        if (
            self.task_busy
            or self.hardware_busy
            or self.gpsr_busy
            or self.forward_navigation_busy
            or self.ros_node.navigation_active
        ):
            QMessageBox.information(
                self, "Acción en curso",
                "Espera a que termine el control actual antes de iniciar otro.")
            return False
        if self.ros_node.plan_active is not False:
            QMessageBox.warning(
                self, "Planificador ocupado",
                "No se puede iniciar una demostración mientras el estado del "
                "planificador no confirme que está inactivo.")
            return False
        return True

    def _start_task_sequence(self, tasks, title, on_complete=None):
        if not self._can_control_robot():
            return
        self.task_sequence = []
        for action, argument in tasks:
            if action not in {"say", "test_head", "test_torso", "test_arms"}:
                self.task_sequence.append(
                    ("say", self._action_announcement(action, argument)))
            self.task_sequence.append((action, argument))
        self.task_sequence_finished = on_complete
        self.task_busy = True
        self.actionLog.append(f"Iniciando: {title}")
        self.demoLog.append(f"Iniciando: {title}")
        self._set_demo_controls_enabled(False)
        self.cancelActionButton.setEnabled(True)
        self._start_next_task()

    def _start_next_task(self):
        if not self.task_sequence:
            callback = self.task_sequence_finished
            self._finish_task_sequence()
            self.actionLog.append("Secuencia completada.")
            self.demoLog.append("Secuencia completada.")
            if callback is not None:
                callback()
            return
        action, argument = self.task_sequence[0]
        self.active_task_action = action
        self.actionLog.append(f"→ {action} {argument}".rstrip())
        self.ros_node.send_task(action, argument)

    def _set_demo_controls_enabled(self, enabled):
        self.runActionButton.setEnabled(enabled)
        self.presentationButton.setEnabled(enabled)
        self.choreoButton.setEnabled(enabled)
        self.introductionButton.setEnabled(enabled)
        self.goToButton.setEnabled(enabled)
        self.gpsrButton.setEnabled(enabled)
        self.forwardGoalButton.setEnabled(enabled)
        self.returnGoalButton.setEnabled(enabled)
        self.baseMobilityButton.setEnabled(enabled)
        self.stopForwardGoalButton.setEnabled(
            not enabled and self.forward_navigation_busy)
        self.moveTorsoButton.setEnabled(enabled)
        self.moveHeadButton.setEnabled(enabled)
        self.moveArmButton.setEnabled(enabled)
        for button in self.hardwareTestButtons:
            button.setEnabled(enabled)
        for button in self.gripperButtons:
            button.setEnabled(enabled)

    def _run_selected_action(self):
        action = self.actionComboBox.currentText()
        self._start_task_sequence(
            [(action, self.actionArgumentEdit.text())], action)

    def _cancel_action(self):
        if self.base_mobility_sequence is not None:
            self.base_mobility_sequence.clear()
            self.base_mobility_cancel_requested = True
            self.cancelActionButton.setEnabled(False)
            if self.forward_navigation_busy:
                self.ros_node.stop_forward_navigation()
            elif not self.base_mobility_waiting_for_speech:
                self._finish_base_mobility_demo(
                    "Base mobility demo stopped.")
            self.demoLog.append(
                "Base mobility demo will stop after the current operation.")
            return
        if self.task_busy:
            if self.presentation_sequence is not None:
                self.presentation_sequence.clear()
                self.presentation_cancel_requested = True
                self.cancelActionButton.setEnabled(False)
                self.demoLog.append(
                    "Presentation will stop after the current operation finishes.")
                self.actionLog.append(
                    "Presentation will stop after the current operation finishes.")
                return
            self.task_sequence.clear()
            self.task_sequence_finished = None
            self.cancelActionButton.setEnabled(False)
            self.ros_node.cancel_task()
            self.demoLog.append("Solicitando cancelar la acción actual...")
            self.actionLog.append("Solicitando cancelar la acción actual...")

    def _on_task_cancel_feedback(self, accepted, message):
        prefix = "Información:" if accepted else "Advertencia:"
        self.demoLog.append(f"{prefix} {message}")
        self.actionLog.append(f"{prefix} {message}")

    def _on_task_feedback(self, message):
        self.actionLog.append(message)
        self.demoLog.append(message)
        test_actions = {"test_head", "test_torso", "test_arms"}
        if (
            self.task_busy
            and self.active_task_action in test_actions
            and f"[{self.active_task_action}]" in message
            and any(state in message for state in ("] SUCCESS", "] FAILED"))
        ):
            self.test_feedback_terminal_action = self.active_task_action
            self.test_feedback_recovery_message = (
                "The test's final feedback was received, but its result was not.")
            self.test_feedback_recovery_timer.start()

    def _on_task_terminal_status(self, success, message):
        test_actions = {"test_head", "test_torso", "test_arms"}
        if (
            self.task_busy
            and self.active_task_action in test_actions
        ):
            self.test_feedback_terminal_action = self.active_task_action
            self.test_feedback_recovery_message = message
            self.test_feedback_recovery_timer.start()

    def _on_task_finished(self, success, message):
        self.test_feedback_recovery_timer.stop()
        self.test_feedback_terminal_action = None
        self.actionLog.append(
            f"{'Completado' if success else 'Error'}: {message}")
        self.demoLog.append(
            f"{'Completado' if success else 'Error'}: {message}")
        if self.presentation_sequence is not None:
            self._advance_presentation(success, message)
            return
        if self.base_mobility_waiting_for_speech:
            self.base_mobility_waiting_for_speech = False
            if self.base_mobility_cancel_requested:
                self._finish_base_mobility_demo(
                    "Base mobility demo stopped.")
                return
            if not success:
                self._finish_base_mobility_demo(
                    f"Speech failed; mobility demo stopped: {message}")
                return
            self._send_next_base_mobility_goal()
            return
        if not self.task_busy:
            return
        if self.ros_node.cancel_requested or not self.task_sequence:
            was_cancel_requested = self.ros_node.cancel_requested
            self.ros_node.cancel_requested = False
            self._finish_task_sequence()
            if was_cancel_requested:
                self.demoLog.append("Acción detenida; controles habilitados.")
                self.actionLog.append("Acción detenida; controles habilitados.")
            return
        if not success:
            self._finish_task_sequence()
            return
        if self.task_sequence:
            self.task_sequence.pop(0)
        self._start_next_task()

    def _finish_task_sequence(self):
        self.test_feedback_recovery_timer.stop()
        self.test_feedback_terminal_action = None
        self.active_task_action = None
        self.task_sequence.clear()
        self.task_sequence_finished = None
        self.task_busy = False
        self.hardware_busy = False
        self.cancelActionButton.setEnabled(False)
        self._set_demo_controls_enabled(True)

    def _recover_from_missing_test_result(self):
        if not self.task_busy or self.test_feedback_terminal_action is None:
            return
        action = self.test_feedback_terminal_action
        self.demoLog.append(
            f"Advertencia: {action}: {self.test_feedback_recovery_message} "
            "Restoring the controls.")
        self.actionLog.append(
            f"Advertencia: {action}: {self.test_feedback_recovery_message} "
            "Restoring the controls.")
        self.ros_node.mark_test_result_recovered()
        self._finish_task_sequence()

    def _move_torso(self):
        height = self.torsoPosition.value()
        self._start_hardware_action(
            f"I am going to raise my torso to {height:.2f} meters.",
            lambda: self.ros_node.move_torso(height))

    def _move_head(self):
        pan, tilt = self.headPan.value(), self.headTilt.value()
        self._start_hardware_action(
            f"I am going to move my head. Pan {pan:.2f}, tilt {tilt:.2f} radians.",
            lambda: self.ros_node.move_head([pan, tilt]))

    def _move_selected_arm(self):
        poses = {
            "Reposo": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "Saludo": [1.2, 0.0, 0.0, 1.5, 0.0, 0.0],
        }
        side = self.armSideComboBox.currentText()
        pose_name = self.armPoseComboBox.currentText()
        pose = poses[pose_name]
        side_name = "left" if side == "izquierdo" else "right"
        pose_label = "rest" if pose_name == "Reposo" else "wave"
        self._start_hardware_action(
            f"I am going to move my {side_name} arm to the {pose_label} pose.",
            lambda: self.ros_node.move_arm(side, pose))

    def _move_gripper(self, side, command):
        verb = "open" if command == "open" else "close"
        side_name = "left" if side == "izquierdo" else "right"
        self._start_hardware_action(
            f"I am going to {verb} my {side_name} gripper.",
            lambda: self.ros_node.move_gripper(side, command))

    def _start_hardware_action(self, announcement, execute):
        if not self._can_control_robot():
            return
        self._start_task_sequence(
            [("say", announcement)],
            "Anuncio de movimiento",
            on_complete=lambda: self._execute_hardware_action(execute))

    def _execute_hardware_action(self, execute):
        self.hardware_busy = True
        self._set_demo_controls_enabled(False)
        execute()

    @staticmethod
    def _action_announcement(action, argument):
        announcements = {
            "listen": "I am going to listen for a command.",
            "go_to": "I am going to go to the requested location.",
            "check": "I am going to check the selected item.",
            "point": "I am going to point at the selected target.",
            "give": "I am going to hand over the requested item.",
            "find_person": "I am going to look for a person.",
            "align": "I am going to align myself with the target.",
            "drop": "I am going to put down the requested item.",
            "find_space": "I am going to look for an available space.",
            "find_object": "I am going to look for the requested object.",
            "focus": "I am going to focus on the selected target.",
            "receive": "I am ready to receive an item.",
            "follow": "I am going to follow the person.",
            "approach": "I am going to approach the target.",
            "approach_partial": "I am going to move partway toward the target.",
            "take": "I am going to pick up the requested item.",
            "verify_take": "I am going to check whether I picked up the item.",
            "find_pose": "I am going to look for the requested pose.",
            "analyze_objects": "I am going to analyze the objects.",
            "analyze_person": "I am going to analyze the person.",
            "save_loc": "I am going to save the current location.",
            "fold": "I am going to fold the requested item.",
        }
        return announcements.get(
            action, "I am going to perform the requested action.")

    def _on_hardware_finished(self, component, success, message):
        self.hardware_busy = False
        if self.presentation_sequence is not None:
            self.hardwareLog.append(
                f"{'Completado' if success else 'Error'}: {component}: {message}")
            self.demoLog.append(
                f"{'Completado' if success else 'Error'}: {component}: {message}")
            self._advance_presentation(success, message)
            return
        self._set_demo_controls_enabled(True)
        line = f"{'Completado' if success else 'Error'}: {component}: {message}"
        self.hardwareLog.append(line)
        self.demoLog.append(line)

    def _run_presentation(self):
        if not self._can_control_robot():
            return
        self.presentation_sequence = [
            ("say", "First, I will raise my torso to thirty centimeters."),
            ("torso", 0.30),
            ("say", "I am turning my head left, thirty-four degrees."),
            ("head", [0.60, 0.0]),
            ("say", "I am turning my head to the right, thirty-four degrees."),
            ("head", [-0.60, 0.0]),
            ("say", "I am centering my head."),
            ("head", [0.0, 0.0]),
            ("say", "I am raising my left arm into the wave position."),
            ("arm", ("izquierdo", "Saludo")),
            ("say", "I am lowering my left arm to its resting position."),
            ("arm", ("izquierdo", "Reposo")),
            ("say", "I am raising my right arm into the wave position."),
            ("arm", ("derecho", "Saludo")),
            ("say", "I am lowering my right arm to its resting position."),
            ("arm", ("derecho", "Reposo")),
            ("say", "Hello, I am Justina. I have tested my components and I am ready."),
        ]
        self.presentation_cancel_requested = False
        self.task_busy = True
        self.actionLog.append("Iniciando presentación de Justina")
        self.demoLog.append("Iniciando presentación de Justina")
        self._set_demo_controls_enabled(False)
        self.cancelActionButton.setEnabled(True)
        self._run_next_presentation_step()

    def _run_next_presentation_step(self):
        if not self.presentation_sequence:
            was_cancelled = self.presentation_cancel_requested
            was_introduction = self.interactive_introduction
            self.presentation_sequence = None
            self.presentation_cancel_requested = False
            self.interactive_introduction = False
            self.task_busy = False
            self.hardware_busy = False
            self.cancelActionButton.setEnabled(False)
            self._set_demo_controls_enabled(True)
            message = (
                "Introduction stopped."
                if was_cancelled and was_introduction else
                "Presentation stopped."
                if was_cancelled else
                "Meet Justina demo completed."
                if was_introduction else
                "Presentation completed.")
            self.actionLog.append(message)
            self.demoLog.append(message)
            return

        kind, payload = self.presentation_sequence.pop(0)
        if kind == "say":
            self.ros_node.send_task("say", payload)
            return
        if kind == "listen_name":
            self.hardware_busy = True
            self.demoLog.append("Listening for the visitor's name...")
            self.ros_node.listen_for_name()
            return
        self.hardware_busy = True
        if kind == "torso":
            self.ros_node.move_torso(payload)
        elif kind == "head":
            self.ros_node.move_head(payload)
        elif kind == "arm":
            side, pose_name = payload
            poses = {
                "Reposo": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "Saludo": [1.2, 0.0, 0.0, 1.5, 0.0, 0.0],
            }
            self.ros_node.move_arm(side, poses[pose_name])
        elif kind == "arm_pose":
            side, pose = payload
            self.ros_node.move_arm(side, pose)

    def _advance_presentation(self, success, message):
        if not success:
            was_introduction = self.interactive_introduction
            self.presentation_sequence = None
            self.presentation_cancel_requested = False
            self.interactive_introduction = False
            self.task_busy = False
            self.hardware_busy = False
            self.cancelActionButton.setEnabled(False)
            self._set_demo_controls_enabled(True)
            self.demoLog.append(
                f"{'Introduction' if was_introduction else 'Presentation'} "
                f"stopped after an operation failed: {message}")
            return
        self._run_next_presentation_step()

    def _on_introduction_name_listened(self, success, recognized_text):
        if (
            not self.interactive_introduction
            or self.presentation_sequence is None
        ):
            return
        self.hardware_busy = False
        if self.presentation_cancel_requested:
            self._run_next_presentation_step()
            return
        name = self._extract_visitor_name(recognized_text)
        if not success or not name:
            self._advance_presentation(
                False,
                recognized_text or "I could not hear a name.")
            return
        self.demoLog.append(f"Visitor's name recognized: {name}")
        self.presentation_sequence.extend([
            ("say", f"Nice to meet you, {name}."),
        ])
        self._run_next_presentation_step()

    @staticmethod
    def _extract_visitor_name(recognized_text):
        name = recognized_text.strip()
        name = re.sub(
            r"^(?:my name is|my name's|i am|i'm|this is|it's|it is)\s+",
            "",
            name,
            flags=re.IGNORECASE,
        )
        return name.strip(" \t\r\n.,!?;:")[:60]

    def _run_choreography(self):
        if not self._can_control_robot():
            return
        self.presentation_sequence = [
            ("say", "I will now perform a short dance."),
            ("say", "I am raising my left arm into the wave position."),
            ("arm", ("izquierdo", "Saludo")),
            ("say", "I am lowering my left arm to rest."),
            ("arm", ("izquierdo", "Reposo")),
            ("say", "I am raising my right arm into the wave position."),
            ("arm", ("derecho", "Saludo")),
            ("say", "I am lowering my right arm to rest."),
            ("arm", ("derecho", "Reposo")),
            ("say", "I am turning my head to the left."),
            ("head", [0.60, 0.0]),
            ("say", "I am turning my head to the right."),
            ("head", [-0.60, 0.0]),
            ("say", "I am centering my head."),
            ("head", [0.0, 0.0]),
            ("say", "Thank you for watching my dance."),
        ]
        self.presentation_cancel_requested = False
        self.task_busy = True
        self.actionLog.append("Iniciando coreografía de Justina")
        self.demoLog.append("Iniciando coreografía de Justina")
        self._set_demo_controls_enabled(False)
        self.cancelActionButton.setEnabled(True)
        self._run_next_presentation_step()

    def _run_interactive_introduction(self):
        if not self._can_control_robot():
            return
        raised_arm = [1.2, 0.0, 0.0, 1.5, 0.0, 0.0]
        wave_left = raised_arm.copy()
        wave_left[1] = -0.30
        wave_right = raised_arm.copy()
        wave_right[1] = 0.30
        self.presentation_sequence = [
            ("say", "Hello! I am Justina, a robot created by the BioRobotics Laboratory at UNAM."),
            ("say", "I am raising my right arm to greet you."),
            ("arm_pose", ("derecho", raised_arm)),
            ("arm_pose", ("derecho", wave_left)),
            ("arm_pose", ("derecho", wave_right)),
            ("arm_pose", ("derecho", wave_left)),
            ("say", "I am returning my right arm to its resting position."),
            ("arm_pose", ("derecho", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
            ("say", "What is your name?"),
            ("listen_name", None),
        ]
        self.interactive_introduction = True
        self.presentation_cancel_requested = False
        self.task_busy = True
        self.actionLog.append("Iniciando presentación interactiva de Justina")
        self.demoLog.append("Iniciando presentación interactiva de Justina")
        self._set_demo_controls_enabled(False)
        self.cancelActionButton.setEnabled(True)
        self._run_next_presentation_step()

    def _run_go_to(self):
        destination = self.destinationEdit.currentText().strip()
        if not destination:
            QMessageBox.warning(
                self, "Destino requerido",
                "Escribe el nombre de una ubicación conocida.")
            return
        self._start_task_sequence(
            [("go_to", destination)], f"Navegación a {destination}")

    def _run_forward_navigation(self):
        if not self._can_control_robot():
            return
        distance = self.forwardDistance.value()
        self._start_task_sequence(
            [("say", f"I am going to move forward {distance:.1f} meters.")],
            "Anuncio de navegación",
            on_complete=lambda: self._send_forward_goal(
                distance, 0.0, "Move forward"))

    def _run_return_navigation(self):
        if not self._can_control_robot():
            return
        distance = self.forwardDistance.value()
        self._start_task_sequence(
            [("say", f"I will turn around and return {distance:.1f} meters.")],
            "Announcing return navigation",
            on_complete=lambda: self._send_forward_goal(
                -distance, math.pi, "Return"))

    def _send_forward_goal(self, distance, yaw_delta, description):
        self.forward_navigation_busy = True
        self._set_demo_controls_enabled(False)
        self.stopForwardGoalButton.setEnabled(True)
        self.demoLog.append(
            f"Normal Surge navigation: {description.lower()} "
            f"{abs(distance):.2f} m.")
        self.ros_node.send_relative_goal(distance, yaw_delta, description)

    def _run_base_mobility_demo(self):
        if not self._can_control_robot():
            return
        self.base_mobility_sequence = [
            ("I am moving forward a short distance while turning ninety degrees to the right.", 0.30, -math.pi / 2.0, "Turn right"),
            ("I am moving forward a short distance while turning ninety degrees to the left.", 0.30, math.pi / 2.0, "Turn left"),
            ("I am moving forward half a meter.", 0.5, 0.0, "Move forward"),
            ("I am moving backward half a meter.", -0.5, 0.0, "Move backward"),
        ]
        self.base_mobility_cancel_requested = False
        self.task_busy = True
        self.demoLog.append("Iniciando demostración de movilidad de la base")
        self._set_demo_controls_enabled(False)
        self.cancelActionButton.setEnabled(True)
        self._announce_next_base_mobility_step()

    def _announce_next_base_mobility_step(self):
        if not self.base_mobility_sequence:
            self._finish_base_mobility_demo("Base mobility demo completed.")
            return
        phrase, _, _, _ = self.base_mobility_sequence[0]
        self.base_mobility_waiting_for_speech = True
        self.ros_node.send_task("say", phrase)

    def _send_next_base_mobility_goal(self):
        _, distance, yaw_delta, description = self.base_mobility_sequence[0]
        self.forward_navigation_busy = True
        self.stopForwardGoalButton.setEnabled(True)
        self.ros_node.send_relative_goal(distance, yaw_delta, description)

    def _finish_base_mobility_demo(self, message):
        self.base_mobility_sequence = None
        self.base_mobility_waiting_for_speech = False
        self.base_mobility_cancel_requested = False
        self.forward_navigation_busy = False
        self.task_busy = False
        self.cancelActionButton.setEnabled(False)
        self._set_demo_controls_enabled(True)
        self.demoLog.append(message)

    def _stop_forward_navigation(self):
        self.ros_node.stop_forward_navigation()

    def _on_forward_navigation_status(self, message, finished, success):
        self.demoLog.append(
            f"{'Completado' if success else 'Error' if finished else 'Estado'}: "
            f"{message}")
        if self.base_mobility_sequence is not None and finished:
            self.forward_navigation_busy = False
            if self.base_mobility_cancel_requested:
                self._finish_base_mobility_demo(
                    "Base mobility demo stopped.")
                return
            if not success:
                self._finish_base_mobility_demo(
                    f"Base mobility demo stopped: {message}")
                return
            self.base_mobility_sequence.pop(0)
            self._announce_next_base_mobility_step()
            return
        if finished:
            self.forward_navigation_busy = False
            self._set_demo_controls_enabled(True)

    def _request_gpsr_command(self):
        if self.GPSR:
            QMessageBox.warning(
                self, "GPSR ya activo",
                "El perfil GPSR actual inicia su propio flujo de voz. Para "
                "evitar dos escuchas y despachos simultáneos, usa esta demo "
                "con un perfil de Surge distinto de GPSR.")
            return
        if not self._can_control_robot():
            return
        self.gpsr_busy = True
        self._set_demo_controls_enabled(False)
        self.demoLog.append("Listening for a command...")
        self.ros_node.request_gpsr_command()

    def _on_gpsr_listen_finished(self, success, response_text):
        if not success:
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append(f"Error en GPSR: {response_text}")
            return
        try:
            payload = json.loads(response_text)
        except json.JSONDecodeError as error:
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append(f"Respuesta GPSR inválida: {error}")
            return

        if not isinstance(payload, dict):
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append("Respuesta GPSR sin formato de metas.")
            return
        goals = payload.get("goals", [])
        heard = payload.get("input", "")
        if payload.get("status") == "aborted" or not goals:
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append(
                f"No se obtuvo un plan válido: {payload.get('reason', response_text)}")
            return

        prompt = (
            f"Instrucción reconocida: {heard or '(sin texto)'}\n\n"
            f"Metas que se enviarán:\n" + "\n".join(map(str, goals)) +
            "\n\nEl servicio goals_to_clips reinicia el contexto de CLIPS y "
            "dispara el plan. ¿Confirmas que el robot y el planificador están "
            "libres para ejecutarlo?"
        )
        answer = QMessageBox.question(
            self, "Confirmar ejecución GPSR", prompt,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append("GPSR cancelado antes de enviar el plan.")
            return
        if self.ros_node.plan_active is not False:
            self.gpsr_busy = False
            self._set_demo_controls_enabled(True)
            self.demoLog.append(
                "GPSR cancelado: el planificador ya no confirma estado inactivo.")
            return
        self.gpsr_busy = False
        self._set_demo_controls_enabled(True)
        self._start_task_sequence(
            [("say", "I am going to execute the command I just heard.")],
            "Anuncio de instrucción GPSR",
            on_complete=lambda: self._submit_gpsr_plan(response_text, goals))

    def _submit_gpsr_plan(self, response_text, goals):
        self.gpsr_busy = True
        self._set_demo_controls_enabled(False)
        self.demoLog.append(f"Ejecutando metas: {', '.join(map(str, goals))}")
        self.ros_node.submit_gpsr_goals(response_text)

    def _on_gpsr_plan_finished(self, success, message):
        self.gpsr_busy = False
        self._set_demo_controls_enabled(True)
        self.demoLog.append(
            f"{'Completado' if success else 'Error'} en GPSR: {message}")

    def load_config(self):
        try:
            with open(CONFIG_PATH, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo cargar config.yaml:\n{e}")
            sys.exit(1)

    def load_planning_profiles(self):
        profiles = self.config.get('planning_profiles', [])
        self.planningComboBox.clear()
        for profile in profiles:
            self.planningComboBox.addItem(profile['name'])

    def load_known_locations(self):
        try:
            locations_path = (
                Path(get_package_share_directory('locations_manager'))
                / 'config'
                / 'known_locations.yaml'
            )
            with locations_path.open('r', encoding='utf-8') as locations_file:
                locations = yaml.safe_load(locations_file) or {}
            if not isinstance(locations, dict) or not locations:
                raise ValueError(f"No locations found in {locations_path}")
            self.destinationEdit.addItems(sorted(locations))
        except (OSError, ValueError, LookupError, yaml.YAMLError) as error:
            self.demoLog.append(f"Could not load navigation locations: {error}")
            QMessageBox.warning(
                self,
                "Navigation locations unavailable",
                "Could not load the known locations from locations_manager. "
                f"You can still type a location name.\n\n{error}",
            )

    def load_expected_nodes(self):
        try:
            with open(NODES_PATH, 'r') as f:
                data = yaml.safe_load(f)
                nodes = data.get('expected_nodes', [])
            self.statusLabel.setText(f"Configuración cargada: {len(nodes)} nodos esperados")
            return nodes
        except FileNotFoundError:
            self.statusLabel.setText(f"ERROR: No se encontró {NODES_PATH}")
            return []
        except Exception as e:
            self.statusLabel.setText(f"ERROR al leer configuración: {e}")
            return []

    def _build_command(self, config_section):
        """Construye un comando único a partir de la lista 'commands' o str 'command' (legacy)."""
        if 'commands' in config_section:
            command = " && ".join(config_section['commands'])
        elif 'command' in config_section:
            command = config_section['command']
        else:
            return None
        return self._expand_config_aliases(command)

    def _expand_config_aliases(self, command):
        aliases = self.config.get('aliases', {})
        resolved_aliases = {}
        resolving = set()

        def resolve_alias(name):
            if name in resolved_aliases:
                return resolved_aliases[name]
            if name in resolving:
                raise ValueError(f"Alias circular en config.yaml: {name}")
            if name not in aliases:
                raise ValueError(f"No existe el alias {{{name}}} en config.yaml")

            resolving.add(name)
            value = str(aliases[name])
            value = ALIAS_PATTERN.sub(lambda match: resolve_alias(match.group(1)), value)
            value = os.path.expanduser(os.path.expandvars(value))
            missing = [
                var_name
                for var_name in (
                    match.group(1) or match.group(2)
                    for match in ENV_VAR_PATTERN.finditer(value)
                )
                if var_name
            ]
            resolving.remove(name)

            if missing:
                env_name = missing[0]
                raise ValueError(
                    f"Falta definir la variable de entorno {env_name} para el alias {name}.\n"
                    f"Agrega esto a tu ~/.bashrc:\nexport {env_name}=/ruta/a/Justina"
                )

            resolved_aliases[name] = value
            return value

        return ALIAS_PATTERN.sub(lambda match: resolve_alias(match.group(1)), command)

    def _close_finished_terminal(self, key):
        terminal = self.finished_terminals.get(key)
        if terminal is None:
            return
        terminal.close_terminal()
        self.finished_terminals[key] = None

    def _launch_in_terminal(self, key, command):
        self._close_finished_terminal(key)
        keep_open = not self.autoCloseTerminalCheckBox.isChecked()
        return ProcessLauncher.launch_in_terminal(command, self, keep_open=keep_open)

    def start_surge(self):
        current_surge = self.processes['surge']
        if current_surge is not None:
            if current_surge.command_finished():
                self._mark_surge_stopped(current_surge)
            else:
                QMessageBox.information(
                    self, "Ya en ejecución", "Surge ya está en ejecución.")
                return

        idx = self.planningComboBox.currentIndex()
        if idx < 0:
            QMessageBox.warning(self, "Aviso", "Por favor selecciona un plan primero.")
            return

        profiles = self.config.get('planning_profiles', [])
        if idx >= len(profiles):
            return

        plan_name = profiles[idx]['name'].lower()
        surge_config = self.config.get('surge', {})
        try:
            command = self._build_command(surge_config)
        except ValueError as e:
            QMessageBox.critical(self, "Error en config.yaml", str(e))
            return

        if not command:
            QMessageBox.critical(self, "Error", "No hay comandos definidos para Surge en config.yaml")
            return
        
        use_blank_map = self.blankMapCheckBox.isChecked()
        if plan_name == "restaurant":
            use_blank_map = True
            command += f" plan_name:=\"{plan_name}\""
        elif plan_name == "gpsr":
            command += " plan_name:=goals"
        else:
            command += f" plan_name:=\"{plan_name}\""
        command += f" blank_map:={str(use_blank_map).lower()}"
        self.GPSR = plan_name == "gpsr"

        try:
            self.processes['surge'] = self._launch_in_terminal('surge', command)
            map_mode = "AMCL con mapa en blanco" if use_blank_map else "AMCL con mapa"
            self.statusLabel.setText(
                f"Surge et Ambula lanzado (Plan: {profiles[idx]['name']}, "
                f"{map_mode}).")
            self.startSurgeButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Surge:\n{e}")
            self.processes['surge'] = None

    def start_supervisor(self):
        if self.processes['supervisor'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Supervisor Manager ya está en ejecución.")
            return
        sup_config = self.config.get('supervisor', {})
        try:
            command = self._build_command(sup_config)
        except ValueError as e:
            QMessageBox.critical(self, "Error en config.yaml", str(e))
            return
        if not command:
            QMessageBox.critical(self, "Error", "No hay comandos definidos para Supervisor en config.yaml")
            return
        try:
            if self.GPSR:
                command += f" natural_language_planning:={str(self.GPSR).lower()}"
            self.processes['supervisor'] = self._launch_in_terminal('supervisor', command)
            self.statusLabel.setText("👁 Supervisor Manager lanzado.")
            self.startSupervisorButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Supervisor:\n{e}")
            self.processes['supervisor'] = None

    def check_processes_status(self):
        for key, proc in self.processes.items():
            if proc is not None and proc.command_finished():
                if key == 'surge':
                    self._mark_surge_stopped(proc)
                    continue
                self.processes[key] = None
                if proc.keep_open and proc.is_terminal_running():
                    self.finished_terminals[key] = proc
                else:
                    proc.cleanup()
                if key == 'supervisor':
                    self.startSupervisorButton.setEnabled(True)
                    self.statusLabel.setText("Supervisor Manager finalizado.")

    def _mark_surge_stopped(self, process):
        if self.processes['surge'] is not process:
            return
        self.processes['surge'] = None
        if process.keep_open and process.is_terminal_running():
            self.finished_terminals['surge'] = process
        else:
            process.cleanup()

        self.test_feedback_recovery_timer.stop()
        self.test_feedback_terminal_action = None
        self.test_feedback_recovery_message = ""
        self.task_sequence.clear()
        self.task_sequence_finished = None
        self.presentation_sequence = None
        self.presentation_cancel_requested = False
        self.interactive_introduction = False
        self.base_mobility_sequence = None
        self.base_mobility_waiting_for_speech = False
        self.base_mobility_cancel_requested = False
        self.task_busy = False
        self.hardware_busy = False
        self.gpsr_busy = False
        self.forward_navigation_busy = False
        self.active_task_action = None
        self.GPSR = False
        self.ros_node.forward_goal_pending = False
        self.ros_node.forward_goal_saw_active = False
        self.ros_node.navigation_active = False
        self.cancelActionButton.setEnabled(False)
        self._set_demo_controls_enabled(True)
        self.startSurgeButton.setEnabled(True)
        self.statusLabel.setText(
            "Surge et Ambula finalizado. La interfaz está lista para relanzarlo.")
        self.demoLog.append(
            "Surge is no longer running. Demo controls were reset; you can "
            "launch Surge again.")
        self.actionLog.append(
            "Surge is no longer running. Action controls were reset.")

    def refresh_node_list(self):
        self.ros_node.refresh_node_list()
        running = set(self.ros_node.running_nodes)
        expected_nodes = list(self.expected_nodes)

        self.gpsrButton.setEnabled(
            self.ros_node.plan_active is False
            and not self.GPSR
            and not self.ros_node.navigation_active
            and not self.task_busy
            and not self.hardware_busy
            and not self.gpsr_busy
        )

        self.nodeListWidget.clear()
        all_ok = True
        for node_name in expected_nodes:
            item = QListWidgetItem(node_name)
            if node_name in running:
                item.setCheckState(Qt.Checked)
                item.setForeground(Qt.darkGreen)
                item.setToolTip("Nodo en ejecución")
            else:
                item.setCheckState(Qt.Unchecked)
                item.setForeground(Qt.red)
                item.setToolTip("Nodo NO detectado")
                all_ok = False
            item.setFlags(item.flags() & ~Qt.ItemIsUserCheckable)
            self.nodeListWidget.addItem(item)

        total = len(expected_nodes)
        running_count = len([n for n in expected_nodes if n in running])
        if all_ok:
            self.statusLabel.setText(f"Todos los {total} nodos están funcionando correctamente.")
        else:
            self.statusLabel.setText(f"{running_count} de {total} nodos en ejecución.")

    def closeEvent(self, event):
        self.timer.stop()
        self.process_monitor_timer.stop()

        # Matar procesos lanzados (terminales)
        for proc in self.processes.values():
            if proc is not None:
                proc.close_terminal()

        for proc in self.finished_terminals.values():
            if proc is not None:
                proc.close_terminal()

        # Matar nodos ROS residuales (excluyendo los esperados)
        self.ros_node.kill_orphan_nodes(keep_nodes=self.expected_nodes)
        event.accept()


def ros_spin(executor):
    try:
        executor.spin()
    except Exception as e:
        print(f"Error en spin de ROS: {e}")


def main():
    rclpy.init()
    signals = GuiSignals()
    ros_node = ROS2NodeChecker(signals)
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = NodeControlPanel(ros_node, signals)
    window.show()

    exit_code = app.exec_()

    executor.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
