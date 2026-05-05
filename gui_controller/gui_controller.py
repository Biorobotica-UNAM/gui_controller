#!/usr/bin/env python3
# gui_controller.py

import os
import sys
import yaml
import threading
import signal
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QMessageBox
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.executors import SingleThreadedExecutor

from gui_controller.launcher import ProcessLauncher
from gui_controller.ros_checker import ROS2NodeChecker

SCRIPT_DIR = Path(__file__).parent.resolve()
CONFIG_PATH = SCRIPT_DIR / "config.yaml"
NODES_PATH = SCRIPT_DIR / "nodes.yaml"


class NodeControlPanel(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        # Cargar UI (asegúrate que node_control.ui esté en el mismo directorio)
        uic.loadUi(os.path.join(SCRIPT_DIR, 'node_control.ui'), self)

        self.ros_node = ros_node
        self.processes = {'surge': None, 'supervisor': None, 'planning': None}

        # Cargar configuración de comandos y perfiles
        self.config = self.load_config()
        self.load_planning_profiles()

        # Cargar lista de nodos esperados
        self.expected_nodes = self.load_expected_nodes()
        if self.expected_nodes:
            self.ros_node.kill_orphan_nodes(self.expected_nodes)

        # Conectar botones
        self.startSurgeButton.clicked.connect(self.start_surge)
        self.startSupervisorButton.clicked.connect(self.start_supervisor)
        self.startPlanningButton.clicked.connect(self.start_planning)

        # Timers para refrescar nodos y monitorear procesos
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_node_list)
        self.timer.start(1000)

        self.process_monitor_timer = QTimer()
        self.process_monitor_timer.timeout.connect(self.check_processes_status)
        self.process_monitor_timer.start(2000)

        self.refresh_node_list()

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
            return " && ".join(config_section['commands'])
        elif 'command' in config_section:
            return config_section['command']
        else:
            return None

    def start_surge(self):
        if self.processes['surge'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Surge et Ambula ya está en ejecución.")
            return
        surge_config = self.config.get('surge', {})
        command = self._build_command(surge_config)
        if not command:
            QMessageBox.critical(self, "Error", "No hay comandos definidos para Surge en config.yaml")
            return
        try:
            self.processes['surge'] = ProcessLauncher.launch_in_terminal(command, self)
            self.statusLabel.setText("🚀 Surge et Ambula lanzado.")
            self.startSurgeButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Surge:\n{e}")
            self.processes['surge'] = None

    def start_supervisor(self):
        if self.processes['supervisor'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Supervisor Manager ya está en ejecución.")
            return
        sup_config = self.config.get('supervisor', {})
        command = self._build_command(sup_config)
        if not command:
            QMessageBox.critical(self, "Error", "No hay comandos definidos para Supervisor en config.yaml")
            return
        try:
            self.processes['supervisor'] = ProcessLauncher.launch_in_terminal(command, self)
            self.statusLabel.setText("👁 Supervisor Manager lanzado.")
            self.startSupervisorButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Supervisor:\n{e}")
            self.processes['supervisor'] = None

    def start_planning(self):
        if self.processes['planning'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Planning Tools ya está en ejecución.")
            return
        idx = self.planningComboBox.currentIndex()
        if idx < 0:
            return
        profiles = self.config.get('planning_profiles', [])
        if idx >= len(profiles):
            return
        profile = profiles[idx]
        command = self._build_command(profile)
        if not command:
            QMessageBox.critical(self, "Error", f"No hay comandos definidos para el perfil {profile.get('name')}")
            return
        try:
            self.processes['planning'] = ProcessLauncher.launch_in_terminal(command, self)
            self.statusLabel.setText(f"🗺 Planning Tools lanzado ({profile['name']}).")
            self.startPlanningButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Planning:\n{e}")
            self.processes['planning'] = None

    def check_processes_status(self):
        for key, proc in self.processes.items():
            if proc is not None and proc.poll() is not None:
                self.processes[key] = None
                if key == 'surge':
                    self.startSurgeButton.setEnabled(True)
                    self.statusLabel.setText("Surge et Ambula finalizado.")
                elif key == 'supervisor':
                    self.startSupervisorButton.setEnabled(True)
                    self.statusLabel.setText("Supervisor Manager finalizado.")
                elif key == 'planning':
                    self.startPlanningButton.setEnabled(True)
                    self.statusLabel.setText("Planning Tools finalizado.")

    def refresh_node_list(self):
        self.ros_node.refresh_node_list()
        running = set(self.ros_node.running_nodes)

        self.nodeListWidget.clear()
        all_ok = True
        for node_name in self.expected_nodes:
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

        total = len(self.expected_nodes)
        running_count = len([n for n in self.expected_nodes if n in running])
        if all_ok:
            self.statusLabel.setText(f"✅ Todos los {total} nodos están funcionando correctamente.")
        else:
            self.statusLabel.setText(f"⚠️ {running_count} de {total} nodos en ejecución.")

    def closeEvent(self, event):
        self.timer.stop()
        self.process_monitor_timer.stop()

        # Matar procesos lanzados (terminales)
        for proc in self.processes.values():
            if proc is not None and proc.poll() is None:
                try:
                    if os.name == 'nt':
                        proc.kill()
                    else:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

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
    ros_node = ROS2NodeChecker()
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = NodeControlPanel(ros_node)
    window.show()

    exit_code = app.exec_()

    executor.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()