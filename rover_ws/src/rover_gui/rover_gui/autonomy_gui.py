import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rover_interfaces.action import AutonomyTask
from rover_interfaces.msg import AutonomyLeg
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QComboBox,
    QLineEdit,
    QDialog,
    QFormLayout,
    QDialogButtonBox,
    QMessageBox,
)
from PyQt5.QtGui import QColor
import sys
import threading
import json


class AddWaypointDialog(QDialog):
    """
    Interactive dialog for adding a waypoint

    :author: Nelson Durrant (w Google Gemini 2.5 Pro)
    :date: Apr 2025
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Waypoint")
        self.layout = QFormLayout()

        self.name_edit = QLineEdit()
        self.type_combo = QComboBox()
        self.type_combo.addItems(["gps", "aruco", "obj"])
        self.latitude_edit = QLineEdit("0.0")
        self.longitude_edit = QLineEdit("0.0")
        self.tag_id_combo = QComboBox()
        self.tag_id_combo.addItems(["1", "2", "3"])
        self.object_combo = QComboBox()
        self.object_combo.addItems(["mallet", "bottle"])

        self.layout.addRow("Name:", self.name_edit)
        self.layout.addRow("Type:", self.type_combo)
        self.layout.addRow("Latitude:", self.latitude_edit)
        self.layout.addRow("Longitude:", self.longitude_edit)
        self.layout.addRow("Tag ID:", self.tag_id_combo)
        self.layout.addRow("Object Name:", self.object_combo)

        self.type_combo.currentIndexChanged.connect(self.update_fields_visibility)
        self.update_fields_visibility()

        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.layout.addWidget(self.buttons)

        self.setLayout(self.layout)

    def update_fields_visibility(self):
        waypoint_type = self.type_combo.currentText()
        self.tag_id_combo.setVisible(waypoint_type == "aruco")
        self.object_combo.setVisible(waypoint_type == "obj")

    def get_waypoint_data(self):
        waypoint = {}
        waypoint["name"] = self.name_edit.text()
        waypoint["type"] = self.type_combo.currentText()
        try:
            waypoint["latitude"] = float(self.latitude_edit.text())
            waypoint["longitude"] = float(self.longitude_edit.text())
        except ValueError:
            QMessageBox.critical(
                self, "Error", "Latitude and Longitude must be numbers."
            )
            return None
        if waypoint["type"] == "aruco":
            tag_id_text = self.tag_id_combo.currentText()
            waypoint["tag_id"] = int(tag_id_text)
        elif waypoint["type"] == "obj":
            waypoint["object"] = self.object_combo.currentText()
        return waypoint


class AutonomyGUI(Node, QWidget):
    """
    GUI for starting, stopping, and monitoring the rover's autonomy task

    :author: Nelson Durrant (w Google Gemini 2.5 Pro)
    :date: Apr 2025

    Clients:
    - /exec_autonomy_task/_action/cancel_goal (action_msgs/CancelGoal)
    Action Clients:
    - exec_autonomy_task (rover_interfaces/AutonomyTask)
    """

    DEFAULT_WAYPOINTS_JSON = """
{
    "legs": [
        {
            "name": "gps1_sim",
            "type": "gps",
            "latitude": 38.162923,
            "longitude": -122.454987
        },
        {
            "name": "aruco1_sim",
            "type": "aruco",
            "latitude": 38.162958,
            "longitude": -122.455412,
            "tag_id": 1
        },
        {
            "name": "mallet_sim",
            "type": "obj",
            "latitude": 38.162635,
            "longitude": -122.454902,
            "object": "mallet"
        }
    ]
}
"""

    def __init__(self):
        Node.__init__(self, "autonomy_gui")
        QWidget.__init__(self)

        self.resize(800, 800)

        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self,
            AutonomyTask,
            "/exec_autonomy_task",
            callback_group=self.callback_group,
        )

        self.waypoints = []
        self.goal_handle = None
        self.close_flag = False

        self.setWindowTitle("BYU Mars Rover Autonomy Task")
        self.layout = QVBoxLayout()

        # Waypoint Display
        self.waypoint_label = QLabel("Waypoints:")
        self.layout.addWidget(self.waypoint_label)
        self.waypoint_list = QListWidget()
        self.layout.addWidget(self.waypoint_list)

        # Buttons Layout
        buttons_layout = QHBoxLayout()
        self.add_button = QPushButton("Add Waypoint")
        self.add_button.clicked.connect(self.open_add_waypoint_dialog)
        buttons_layout.addWidget(self.add_button)

        self.remove_button = QPushButton("Remove Waypoint")
        self.remove_button.clicked.connect(self.remove_waypoint)
        buttons_layout.addWidget(self.remove_button)

        self.clear_button = QPushButton("Clear All")
        self.clear_button.clicked.connect(self.clear_waypoints)
        buttons_layout.addWidget(self.clear_button)

        self.start_button = QPushButton("Start Task")
        self.start_button.clicked.connect(self.start_task)
        buttons_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Task")
        self.stop_button.clicked.connect(self.stop_task)
        buttons_layout.addWidget(self.stop_button)

        self.layout.addLayout(buttons_layout)

        # Feedback Display
        self.feedback_label = QLabel("Task Feedback:")
        self.layout.addWidget(self.feedback_label)
        self.feedback_display = QListWidget()
        self.layout.addWidget(self.feedback_display)

        # Set stretch factors
        self.layout.setStretch(0, 1)  # Waypoint Label
        self.layout.setStretch(1, 6)  # Waypoint List
        self.layout.setStretch(2, 1)  # Buttons Container
        self.layout.setStretch(3, 1)  # Feedback Label
        self.layout.setStretch(4, 20) # Feedback Display

        self.setLayout(self.layout)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.spin_thread.daemon = True
        self.spin_thread.start()

        self._load_default_waypoints()
        self._update_button_states()

    def _load_default_waypoints(self):
        try:
            default_waypoints_data = json.loads(self.DEFAULT_WAYPOINTS_JSON)
            for leg_data in default_waypoints_data.get("legs", []):
                waypoint = {
                    "name": leg_data.get("name", ""),
                    "type": leg_data.get("type", ""),
                    "latitude": leg_data.get("latitude", 0.0),
                    "longitude": leg_data.get("longitude", 0.0),
                }
                if waypoint["type"] == "aruco":
                    waypoint["tag_id"] = leg_data.get("tag_id", 0)
                elif waypoint["type"] == "obj":
                    waypoint["object"] = leg_data.get("object", "")
                self.waypoints.append(waypoint)
            self.update_waypoint_list()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding default waypoints: {e}")
        except Exception as e:
            self.get_logger().error(f"Error loading default waypoints: {e}")

    def _format_feedback_text(self, text):
        item = QListWidgetItem(text)
        if "[SUCCESS]" in text:
            item.setForeground(QColor("green"))
        elif "[FATAL]" in text or "[ERROR]" in text:
            item.setForeground(QColor("red"))
        elif "[WARN]" in text:
            item.setForeground(QColor("orange"))
        self.feedback_display.addItem(item)
        self.feedback_display.scrollToBottom()

    def closeEvent(self, event):
        self.close_flag = True
        self.stop_task()
        self.destroy_node()
        event.accept()

    def _update_button_states(self):
        task_running = (
            self.goal_handle is not None
            and self.goal_handle.status == GoalStatus.STATUS_EXECUTING
        )
        self.start_button.setEnabled(not task_running and len(self.waypoints) > 0)
        self.stop_button.setEnabled(task_running)

    def open_add_waypoint_dialog(self):
        self.get_logger().info("Adding waypoint...")
        dialog = AddWaypointDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            waypoint_data = dialog.get_waypoint_data()
            if waypoint_data:
                self.waypoints.append(waypoint_data)
                self.update_waypoint_list()
                self._update_button_states()

    def remove_waypoint(self):
        self.get_logger().info("Removing waypoint...")
        selected_item = self.waypoint_list.currentItem()
        if selected_item:
            index = self.waypoint_list.row(selected_item)
            del self.waypoints[index]
            self.update_waypoint_list()
            self._update_button_states()

    def clear_waypoints(self):
        self.get_logger().info("Clearing waypoints...")
        self.waypoints = []
        self.update_waypoint_list()
        self._update_button_states()

    def update_waypoint_list(self):
        self.waypoint_list.clear()
        for waypoint in self.waypoints:
            self.waypoint_list.addItem(str(waypoint))
            self.feedback_display.scrollToBottom()

    def start_task(self):
        self.get_logger().info("Starting task...")
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available after 2 seconds!")
            QMessageBox.critical(self, "Error", "Action server not available!")
            return

        if not self.waypoints:
            QMessageBox.warning(self, "Warning", "No waypoints to send.")
            return

        goal_msg = AutonomyTask.Goal()
        goal_msg.legs = []
        for wp in self.waypoints:
            leg_msg = AutonomyLeg()
            leg_msg.name = wp.get("name", "")
            leg_msg.type = wp.get("type", "")
            leg_msg.latitude = float(wp.get("latitude", 0.0))
            leg_msg.longitude = float(wp.get("longitude", 0.0))

            if leg_msg.type == "aruco":
                leg_msg.tag_id = int(wp.get("tag_id", 0))
                leg_msg.object = ""  # Ensure other fields are set to default or empty
            elif leg_msg.type == "obj":
                leg_msg.object = wp.get("object", "")
                leg_msg.tag_id = 0  # Ensure other fields are set to default
            else:  # gps type
                leg_msg.tag_id = 0
                leg_msg.object = ""

            goal_msg.legs.append(leg_msg)

        self.feedback_display.clear()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.goal_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self._update_button_states()

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_display.addItem(
                self._format_feedback_text("[ERROR] [gui] Goal rejected by action server")
            )
        else:
            self.feedback_display.addItem(
                self._format_feedback_text("[gui] Goal accepted by action server:")
            )
            for wp in self.waypoints:
                self.feedback_display.addItem(" - " + str(wp))
            get_result_future = self.goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
            self.feedback_display.scrollToBottom()
        self._update_button_states()

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        formatted_text = self._format_feedback_text(f"{feedback.status}")
        self.feedback_display.addItem(formatted_text)
        self.feedback_display.scrollToBottom()
        self._update_button_states()

    def get_result_callback(self, future):
        result = future.result()
        formatted_text = self._format_feedback_text(f"[gui] {result.result.msg}")
        self.feedback_display.addItem(formatted_text)
        self.feedback_display.scrollToBottom()
        self.goal_handle = None
        self._update_button_states()

    def stop_task(self):
        self.get_logger().info("Stopping task...")
        client = self.create_client(
            CancelGoal,
            "/exec_autonomy_task/_action/cancel_goal",
            callback_group=self.callback_group,
        )

        if not client.wait_for_service(timeout_sec=2.0):
            if not self.close_flag:
                self.get_logger().error("Service not available after 2 seconds!")
                QMessageBox.critical(self, "Error", "Service not available!")
            else:
                self.get_logger().warn("Service not available after 2 seconds!")
            return

        request = CancelGoal.Request()
        future = client.call_async(request)
        future.add_done_callback(self.cancel_response_callback)
        self._update_button_states()

    def cancel_response_callback(self, future):
        response = future.result()
        if response.return_code == CancelGoal.Response.ERROR_NONE:
            self.feedback_display.addItem(
                self._format_feedback_text("[gui] Cancel request sent successfully.")
            )
            self.goal_handle = None
        else:
            self.feedback_display.addItem(
                self._format_feedback_text("[ERROR] [gui] Failed to send cancel request")
            )
        self.feedback_display.scrollToBottom()
        self._update_button_states()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = AutonomyGUI()
    gui.show()
    exit_code = app.exec_()
    gui.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()