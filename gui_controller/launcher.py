#!/usr/bin/env python3
# launcher.py

import os
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from PyQt5.QtWidgets import QMessageBox


class TerminalProcess:
    """Proceso de terminal con una marca separada para saber cuando termina el comando."""

    def __init__(self, process, done_file=None, close_file=None, script_file=None, keep_open=False):
        self.process = process
        self.done_file = done_file
        self.close_file = close_file
        self.script_file = script_file
        self.keep_open = keep_open

    def poll(self):
        return self.process.poll()

    def command_finished(self):
        if self.done_file is not None and self.done_file.exists():
            return True
        return self.process.poll() is not None

    def is_terminal_running(self):
        return self.process.poll() is None

    def close_terminal(self):
        if self.process.poll() is not None:
            self.cleanup()
            return
        if self.close_file is not None:
            try:
                self.close_file.touch()
                for _ in range(20):
                    if self.process.poll() is not None:
                        self.cleanup()
                        return
                    time.sleep(0.1)
            except Exception:
                pass
        try:
            if os.name == 'nt':
                self.process.kill()
            else:
                os.killpg(os.getpgid(self.process.pid), 9)
        except Exception:
            pass
        self.cleanup()

    def cleanup(self):
        for path in (self.done_file, self.close_file, self.script_file):
            if path is None:
                continue
            try:
                path.unlink(missing_ok=True)
            except Exception:
                pass


class ProcessLauncher:
    """Lanza comandos en una terminal y rastrea el proceso."""

    @staticmethod
    def _make_terminal_script(command, keep_open):
        tmp_dir = Path(tempfile.gettempdir())
        token = uuid.uuid4().hex
        done_file = tmp_dir / f"gui_controller_{token}.done"
        close_file = tmp_dir / f"gui_controller_{token}.close"
        script_file = tmp_dir / f"gui_controller_{token}.sh"
        keep_open_value = "1" if keep_open else "0"
        script_file.write_text(
            "#!/usr/bin/env bash\n"
            "set +e\n"
            f"{command}\n"
            "status=$?\n"
            f"printf '%s\\n' \"$status\" > \"{done_file}\"\n"
            "echo\n"
            "echo \"Proceso terminado con codigo: $status\"\n"
            f"if [ \"{keep_open_value}\" = \"1\" ]; then\n"
            "  echo \"La terminal queda abierta. Presiona Enter, cierra la ventana o relanza desde el GUI.\"\n"
            f"  while [ ! -f \"{close_file}\" ]; do\n"
            "    read -r -t 1 _line && break\n"
            "  done\n"
            "fi\n"
            "exit \"$status\"\n"
        )
        script_file.chmod(0o700)
        return script_file, done_file, close_file

    @staticmethod
    def launch_in_terminal(command, parent_widget=None, keep_open=False):
        """
        Abre una terminal que ejecuta el comando.
        Retorna un TerminalProcess o None si falla.
        """
        script_file, done_file, close_file = ProcessLauncher._make_terminal_script(command, keep_open)
        terminals = [
            ["gnome-terminal", "--wait", "--", str(script_file)],
            ["xterm", "-e", str(script_file)],
            ["konsole", "-e", str(script_file)],
            ["terminator", "-e", str(script_file)],
        ]
        for term_cmd in terminals:
            try:
                if subprocess.run(["which", term_cmd[0]], capture_output=True).returncode == 0:
                    proc = subprocess.Popen(
                        term_cmd,
                        preexec_fn=os.setsid if os.name != 'nt' else None,
                    )
                    return TerminalProcess(
                        proc,
                        done_file=done_file,
                        close_file=close_file,
                        script_file=script_file,
                        keep_open=keep_open,
                    )
            except Exception:
                continue
        # Fallback si no hay terminal
        if parent_widget:
            QMessageBox.warning(parent_widget, "Terminal no encontrada",
                                "No se encontró emulador de terminal. Ejecutando en segundo plano.")
        if keep_open:
            script_file, done_file, close_file = ProcessLauncher._make_terminal_script(command, keep_open=False)
        proc = subprocess.Popen(
            [str(script_file)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid if os.name != 'nt' else None
        )
        return TerminalProcess(proc, done_file=done_file, close_file=close_file, script_file=script_file)
