#!/usr/bin/env python3
# ros_checker.py

import os
import signal
import subprocess
from rclpy.node import Node

try:
    from ros2node.api import get_node_names
    USE_ROS2NODE_API = True
except ImportError:
    USE_ROS2NODE_API = False
    print("⚠️ No se pudo importar ros2node.api. Se usará método alternativo.")

class ROS2NodeChecker(Node):
    """Nodo ROS 2 mínimo para obtener y matar nodos."""

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