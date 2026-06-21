#!/usr/bin/env python3
# gui_controller.py

import os
import re
import sys
import yaml
import threading
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
ALIAS_PATTERN = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")
ENV_VAR_PATTERN = re.compile(r"\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))")

class NodeControlPanel(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        # Cargar UI (asegúrate que node_control.ui esté en el mismo directorio)
        uic.loadUi(os.path.join(SCRIPT_DIR, 'node_control.ui'), self)

        self.ros_node = ros_node
        self.processes = {'surge': None, 'supervisor': None}
        self.finished_terminals = {'surge': None, 'supervisor': None}
        self.GPSR = False
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
        if self.processes['surge'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Plan ya está en ejecución.")
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
        
        if plan_name == "restaurant":
            command += f" plan_name:=\"{plan_name}\" mapless:=true"
        elif plan_name == "gpsr":
            command += f" plan_name:=\"{plan_name}\""
            self.GPSR = True
        else:
            command += f" plan_name:=\"{plan_name}\""

        try:
            self.processes['surge'] = self._launch_in_terminal('surge', command)
            self.statusLabel.setText(f"🚀 Surge et Ambula lanzado (Plan: {profiles[idx]['name']}).")
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
                self.processes[key] = None
                if proc.keep_open and proc.is_terminal_running():
                    self.finished_terminals[key] = proc
                else:
                    proc.cleanup()
                if key == 'surge':
                    self.startSurgeButton.setEnabled(True)
                    self.statusLabel.setText("Surge et Ambula finalizado.")
                elif key == 'supervisor':
                    self.startSupervisorButton.setEnabled(True)
                    self.statusLabel.setText("Supervisor Manager finalizado.")

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
