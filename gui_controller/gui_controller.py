#!/usr/bin/env python3

import os
import sys
import yaml
import threading
import subprocess
import signal
import shlex
from datetime import datetime
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QMessageBox
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

SCRIPT_DIR = Path(__file__).parent.resolve()

ADAPTER_PATH = (SCRIPT_DIR / "nodes.yaml").resolve()

# Intentar importar API robusta de nodos
try:
    from ros2node.api import get_node_names
    USE_ROS2NODE_API = True
except ImportError:
    USE_ROS2NODE_API = False
    print("⚠️ No se pudo importar ros2node.api. Se usará método alternativo.")


class ROS2NodeChecker(Node):
    """Nodo ROS 2 mínimo para obtener lista de nodos."""

    def __init__(self):
        super().__init__('gui_controller')
        self.running_nodes = []

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
        """
        Mata un nodo ROS 2 usando pkill con el patrón __node:=<nombre>.
        Se asegura de no matar la propia GUI.
        """
        import subprocess
        # Patrón seguro: busca el argumento exacto __node:=/planning/clips_node
        # o __node:=clips_node (con o sin namespace)
        short_name = node_name.split('/')[-1]
        # Construimos dos patrones y los combinamos con -e (regex extendida)
        # Por ejemplo: pkill -9 -f "__node:=/planning/clips_node|__node:=clips_node"
        pattern = f"__node:={node_name}|__node:={short_name}"
        try:
            # Excluimos explícitamente procesos que contengan gui_controller (por si acaso)
            cmd = f"pkill -9 -f '{pattern}'"
            # Realizamos un pgrep primero para ver qué procesos coinciden y no incluir el nuestro
            pgrep_cmd = f"pgrep -f '{pattern}'"
            result = subprocess.run(pgrep_cmd, shell=True, capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                pids = result.stdout.strip().split()
                for pid in pids:
                    # Antes de matar, verificamos que no sea el proceso de la GUI
                    # Inspeccionar la línea completa del proceso
                    ps_info = subprocess.run(['ps', '-p', pid, '-o', 'args='], 
                                             capture_output=True, text=True, timeout=2)
                    if 'gui_controller' in ps_info.stdout:
                        self.get_logger().info(f"  Omitiendo PID {pid} (es el propio GUI controller)")
                        continue
                    # Matamos con pkill individualmente o con kill
                    os.kill(int(pid), signal.SIGKILL)
                    self.get_logger().info(f"  Nodo {node_name} eliminado (PID {pid})")
                return True
            else:
                self.get_logger().info(f"  No se encontraron procesos para {node_name}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error al matar {node_name} con pkill: {e}")
            return False

    def kill_orphan_nodes(self, keep_nodes: list = None):
        self.get_logger().info("Iniciando limpieza de nodos residuales...")
        self.refresh_node_list()
        current_nodes = set(self.running_nodes)
        our_name = '/' + self.get_name()
        nodes_to_kill = current_nodes - {our_name}
        if keep_nodes is not None:
            nodes_to_kill -= set(keep_nodes)
        if not nodes_to_kill:
            self.get_logger().info("No se encontraron nodos residuales.")
            return
        self.get_logger().info(f"Eliminando {len(nodes_to_kill)} nodos residuales...")
        for node_name in nodes_to_kill:
            self.kill_node_by_name(node_name)
        self.get_logger().info("Limpieza completada.")


class NodeControlPanel(QMainWindow):
    def __init__(self, ros_node, expected_nodes_file=ADAPTER_PATH):
        super().__init__()
        # Cargar UI
        uic.loadUi(os.path.join(os.path.dirname(__file__), 'node_control.ui'), self)

        self.ros_node = ros_node
        self.expected_nodes_file = expected_nodes_file
        self.expected_nodes = []

        # Diccionarios para almacenar procesos de cada sección
        self.processes = {
            'surge': None,
            'supervisor': None,
            'planning': None
        }

        # Configurar opciones del combobox de Planning
        self.planning_configs = [
            ("HRI", "cd ~/ros2_clips && source install/setup.bash && ros2 launch planning_tools planning.launch.xml plan_name:=\"hri\""),
            ("PNP", "cd ~/ros2_clips && source install/setup.bash && ros2 launch planning_tools planning.launch.xml plan_name:=\"pnp\""),
            ("Laundry", "cd ~/ros2_clips && source install/setup.bash && ros2 launch planning_tools planning.launch.xml plan_name:=\"laundry\""),
            ("Restaurant", "cd ~/ros2_clips && source install/setup.bash && ros2 launch planning_tools planning.launch.xml plan_name:=\"restaurant\""),
            ("Robot Inspection", "cd ~/ros2_clips && source install/setup.bash && ros2 launch planning_tools planning.launch.xml plan_name:=\"robot_inspection\""),
            # Añade aquí más opciones según necesites
        ]
        for name, _ in self.planning_configs:
            self.planningComboBox.addItem(name)

        # Cargar nodos esperados
        self.load_expected_nodes()

        # --- NUEVO: Limpiar nodos basura al inicio ---
        if self.expected_nodes:
            self.ros_node.kill_orphan_nodes(self.expected_nodes)
        # --------------------------------------------

        # Conectar señales
        self.startSurgeButton.clicked.connect(self.start_surge)
        self.startSupervisorButton.clicked.connect(self.start_supervisor)
        self.startPlanningButton.clicked.connect(self.start_planning)

        # Timer para refresco automático de la lista de nodos
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_node_list)
        self.timer.start(1000)

        # Timer para monitorear el estado de los procesos (si ya terminaron)
        self.process_monitor_timer = QTimer()
        self.process_monitor_timer.timeout.connect(self.check_processes_status)
        self.process_monitor_timer.start(2000)  # verificar cada 2 segundos

        self.refresh_node_list()

    def load_expected_nodes(self):
        try:
            with open(self.expected_nodes_file, 'r') as f:
                config = yaml.safe_load(f)
                self.expected_nodes = config.get('expected_nodes', [])
            self.statusLabel.setText(f"Configuración cargada: {len(self.expected_nodes)} nodos esperados")
        except FileNotFoundError:
            self.statusLabel.setText(f"ERROR: No se encontró {self.expected_nodes_file}")
            self.expected_nodes = []
        except Exception as e:
            self.statusLabel.setText(f"ERROR al leer configuración: {e}")
            self.expected_nodes = []

    def check_processes_status(self):
        """Verifica si los procesos han terminado (poll() != None) y re-habilita los botones."""
        for key, proc in self.processes.items():
            if proc is not None and proc.poll() is not None:
                # El proceso ya terminó
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

    # ---------- Métodos auxiliares para lanzar procesos ----------
    def _launch_command(self, command):
        """Lanza un comando siempre en una terminal."""
        return self._launch_in_terminal(command)

    def _launch_in_terminal(self, command):
        """Abre una terminal que ejecuta el comando. La terminal se cerrará al terminar el proceso."""
        terminals = [
            ["gnome-terminal", "--wait", "--", "bash", "-c", f"{command}"],
            ["xterm", "-e", f"bash -c '{command}'"],
            ["konsole", "-e", f"bash -c '{command}'"],
            ["terminator", "-e", f"bash -c '{command}'"],
        ]
        for term_cmd in terminals:
            try:
                if subprocess.run(["which", term_cmd[0]], capture_output=True).returncode == 0:
                    return subprocess.Popen(
                        term_cmd,
                        preexec_fn=os.setsid if os.name != 'nt' else None
                    )
            except Exception:
                continue
        # Fallback
        QMessageBox.warning(self, "Terminal no encontrada",
                            "No se encontró emulador de terminal. Ejecutando en segundo plano.")
        return subprocess.Popen(
            shlex.split(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid if os.name != 'nt' else None
        )

    # ---------- Control de Surge et Ambula ----------
    def start_surge(self):
        if self.processes['surge'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Surge et Ambula ya está en ejecución.")
            return
       
        # Variables configurables
        workspace_path = "~/Justina"
        setup_file = "install/setup.bash"
        package = "surge_et_ambula"
        launch_file = "justina.launch.xml"

        # Construir comando en partes
        cmd_parts = [
            f"cd {workspace_path}",
            f"source {setup_file}",
            f"ros2 launch {package} {launch_file}"
        ]
        cmd = " && ".join(cmd_parts)

        try:
            self.processes['surge'] = self._launch_command(cmd)
            self.statusLabel.setText("🚀 Surge et Ambula lanzado.")
            self.startSurgeButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Surge et Ambula:\n{e}")
            self.processes['surge'] = None

    # ---------- Control de Supervisor Manager ----------
    def start_supervisor(self):
        if self.processes['supervisor'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Supervisor Manager ya está en ejecución.")
            return
        cmd = "ros2 run supervisor_manager supervisor_manager_node"
        try:
            self.processes['supervisor'] = self._launch_command(cmd)
            self.statusLabel.setText("👁 Supervisor Manager lanzado.")
            self.startSupervisorButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Supervisor Manager:\n{e}")
            self.processes['supervisor'] = None

    # ---------- Control de Planning Tools ----------
    def start_planning(self):
        if self.processes['planning'] is not None:
            QMessageBox.information(self, "Ya en ejecución", "Planning Tools ya está en ejecución.")
            return
        idx = self.planningComboBox.currentIndex()
        if idx < 0:
            return
        config_name, cmd = self.planning_configs[idx]
        try:
            self.processes['planning'] = self._launch_command(cmd)
            self.statusLabel.setText(f"🗺 Planning Tools lanzado ({config_name}).")
            self.startPlanningButton.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Planning Tools:\n{e}")
            self.processes['planning'] = None

    def closeEvent(self, event):
        """Al cerrar la ventana, detener todos los procesos lanzados y nodos ROS."""
        self.timer.stop()
        if hasattr(self, 'process_monitor_timer'):
            self.process_monitor_timer.stop()

        # Primero matar los procesos de terminal (si aún existen)
        for key, proc in self.processes.items():
            if proc is not None and proc.poll() is None:
                try:
                    if os.name == 'nt':
                        proc.kill()
                    else:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

        # Luego eliminar todos los nodos ROS lanzados con el nuevo método
        # Excluimos nuestro propio nodo y los que estén en expected_nodes (que no se lanzaron desde aquí)
        self.ros_node.kill_orphan_nodes(keep_nodes=self.expected_nodes)

        event.accept()


def ros_spin_thread(executor):
    try:
        executor.spin()
    except Exception as e:
        print(f"Error en spin de ROS: {e}")


def main():
    rclpy.init()
    ros_node = ROS2NodeChecker()
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
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