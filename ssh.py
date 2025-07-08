import sys
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QTextEdit
)
import paramiko
import re

class SSHApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Raspberry Pi Script Launcher")
        self.setGeometry(200, 200, 550, 500)
        self.config_path = os.path.expanduser("~/.pi_ssh_config.json")

        self.client = None
        self.sftp = None
        self.shell = None
        self.conda_activated = False

        # Input fields
        self.ip_label = QLabel("IP Address:")
        self.ip_input = QLineEdit()
        self.user_label = QLabel("Username:")
        self.user_input = QLineEdit()
        self.pass_label = QLabel("Password:")
        self.pass_input = QLineEdit()
        self.pass_input.setEchoMode(QLineEdit.Password)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_to_pi)

        self.output_box = QTextEdit()
        self.output_box.setReadOnly(True)


        self.scripts = {
            "Heartbeat": "connect_heartbeat.py",
            "Mavlink": "pymavlink_connect.py",
            "Arm": "set_mode_arm.py",
            "Auto": "set_mode_auto.py",
            "Guided": "guided.py"

        }

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.ip_label)
        self.layout.addWidget(self.ip_input)
        self.layout.addWidget(self.user_label)
        self.layout.addWidget(self.user_input)
        self.layout.addWidget(self.pass_label)
        self.layout.addWidget(self.pass_input)
        self.layout.addWidget(self.connect_btn)
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter shell command and press Enter")
        self.command_input.returnPressed.connect(self.send_user_command)
        self.command_input.setVisible(False)
        self.layout.addWidget(self.command_input)

        self.conda_btn = QPushButton("Activate Conda Environment (drone)")
        self.conda_btn.clicked.connect(self.activate_conda)
        self.conda_btn.setVisible(False)
        self.layout.addWidget(self.conda_btn)

        self.script_buttons = []
        for label, path in self.scripts.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, p=path: self.run_script(p))
            btn.setVisible(False)
            self.script_buttons.append(btn)
            self.layout.addWidget(btn)

        self.layout.addWidget(QLabel("Output:"))
        self.layout.addWidget(self.output_box)
        self.setLayout(self.layout)

        self.load_saved_credentials()

    def load_saved_credentials(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    creds = json.load(f)
                    self.ip_input.setText(creds.get("ip", ""))
                    self.user_input.setText(creds.get("username", ""))
                    self.pass_input.setText(creds.get("password", ""))
            except Exception as e:
                self.output_box.append(f"⚠️ Failed to load saved credentials: {e}")
    def strip_ansi_codes(text):
        return re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', text)
    def save_credentials(self):
        creds = {
            "ip": self.ip_input.text(),
            "username": self.user_input.text(),
            "password": self.pass_input.text()
        }
        try:
            with open(self.config_path, "w") as f:
                json.dump(creds, f)
        except Exception as e:
            self.output_box.append(f"⚠️ Failed to save credentials: {e}")
    def send_user_command(self):
        if not self.shell:
            clean_output = strip_ansi_codes(output)
            lines = clean_output.splitlines()
            filtered = [line for line in lines if not line.strip().endswith('$') and line.strip()]
            self.output_box.append('\n'.join(filtered))
            self.output_box.append("❌ Not connected to shell.")
            return

        cmd = self.command_input.text().strip()
        if not cmd:
            return

        self.output_box.append(f"> {cmd}")
        self.shell.send(cmd + "\n")
        self.command_input.clear()

        output = ""
        try:
            while True:
                part = self.shell.recv(1024).decode("utf-8")
                output += part
                if not part:
                    break
        except Exception:
            pass
        self.output_box.append(output)
     

    def connect_to_pi(self):
        ip = self.ip_input.text()
        user = self.user_input.text()
        passwd = self.pass_input.text()
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(ip, username=user, password=passwd)
            self.sftp = self.client.open_sftp()
            self.shell = self.client.invoke_shell()
            self.shell.settimeout(2)
            self.output_box.append("✅ Connected to Raspberry Pi with persistent shell.")
            self.save_credentials()

            # Hide login widgets
            self.ip_label.setVisible(False)
            self.ip_input.setVisible(False)
            self.user_label.setVisible(False)
            self.user_input.setVisible(False)
            self.pass_label.setVisible(False)
            self.pass_input.setVisible(False)
            self.connect_btn.setVisible(False)

            for btn in self.script_buttons:
                btn.setVisible(True)
            self.conda_btn.setVisible(True)
            self.command_input.setVisible(True)


        except Exception as e:
            self.output_box.append(f"❌ Connection failed: {str(e)}")

    def activate_conda(self):
        if not self.shell:
            self.output_box.append("❌ Not connected to Raspberry Pi.")
            return
        self.output_box.append("📦 Activating Conda environment 'drone'...")
        self.shell.send("conda activate drone\n")
        self.conda_activated = True
        self.output_box.append("✅ Conda will be used for future script runs.")

    def run_script(self, local_path):
        if not self.shell or not self.sftp:
            self.output_box.append("❌ Not connected.")
            return
        try:
            if not os.path.exists(local_path):
                self.output_box.append(f"❌ Local script not found: {local_path}")
                return

            filename = os.path.basename(local_path)
            remote_dir = "/home/pi/scripts/"
            remote_path = remote_dir + filename

            try:
                self.sftp.stat(remote_dir)
            except IOError:
                self.sftp.mkdir(remote_dir)

            upload_needed = True
            local_size = os.path.getsize(local_path)
            try:
                remote_attr = self.sftp.stat(remote_path)
                if remote_attr.st_size == local_size:
                    self.output_box.append(f"📂 {filename} already on Pi. Skipping upload.")
                    upload_needed = False
            except IOError:
                self.output_box.append(f"📁 {filename} not found on Pi. Uploading.")

            if upload_needed:
                self.sftp.put(local_path, remote_path)
                self.sftp.chmod(remote_path, 0o755)
                self.output_box.append(f"✅ Uploaded {filename}.")

            cmd = f"python {remote_path}\n" if self.conda_activated else f"python3 {remote_path}\n"
            self.output_box.append(f"🚀 Running: {cmd.strip()}")
            self.shell.send(cmd)

            output = ""
            try:
                while True:
                    part = self.shell.recv(1024).decode("utf-8")
                    output += part
                    if not part:
                        break
            except Exception:
                pass  # Timeout expected

            self.output_box.append(output)
        except Exception as e:
            self.output_box.append(f"❌ Error: {str(e)}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SSHApp()
    win.show()
    sys.exit(app.exec_())
