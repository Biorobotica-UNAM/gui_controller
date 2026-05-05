#!/usr/bin/env python3
# launcher.py

import os
import subprocess
import shlex
from PyQt5.QtWidgets import QMessageBox

class ProcessLauncher:
    """Lanza comandos en una terminal y rastrea el proceso."""

    @staticmethod
    def launch_in_terminal(command, parent_widget=None):
        """
        Abre una terminal que ejecuta el comando.
        Retorna el objeto Popen o None si falla.
        """
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
        # Fallback si no hay terminal
        if parent_widget:
            QMessageBox.warning(parent_widget, "Terminal no encontrada",
                                "No se encontró emulador de terminal. Ejecutando en segundo plano.")
        return subprocess.Popen(
            shlex.split(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid if os.name != 'nt' else None
        )