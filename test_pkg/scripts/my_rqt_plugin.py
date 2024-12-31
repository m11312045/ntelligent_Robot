import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import tkinter as tk
from tf.transformations import euler_from_quaternion

class Subscriber():
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.window = None

        self.target_points = [
            (-6.32124423370122, -0.36829197778328726, 0.6852967070968377),  # 1
            (-6.169139762420635, 3.394310149718285, 0.020165100435462827),  # 2
            (-2.448408433663198, 0.6563227180899639, -0.08052122888371446), # 3
            (1.1393254699374271, 3.3830700858909513, 0.6106034706582273),  # 4
            (6.012279866058944, 0.996959726134316, -0.6064188066327508),   # 5
            (6.044607089527036, -1.8760686913586067, 0.7018818821602578)    # 6
        ]
        self.init_param()

        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        self.create_window()

    def init_param(self):
        self.target_pose_x = 0.0
        self.target_pose_y = 0.0
        self.target_pose_w = 0.0
        self.now_pose_x = 0.0
        self.now_pose_y = 0.0
        self.now_pose_w = 0.0

    def set_goal(self, idx):
        # 設置目標點
        target = self.target_points[idx]

        # 更新目標位置的數據
        self.target_pose_x = target[0]
        self.target_pose_y = target[1]
        self.target_pose_w = target[2]

        self.publish_goal(target)

    def publish_goal(self, target):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = target[0]
        goal_msg.pose.position.y = target[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = target[2]

        rospy.loginfo(f"Publishing goal: {goal_msg}")
        self.goal_pub.publish(goal_msg)

    def pose_callback(self, msg):
        self.now_pose_x = msg.pose.pose.position.x
        self.now_pose_y = msg.pose.pose.position.y
        self.now_pose_w = msg.pose.pose.orientation.w

        orientation = msg.pose.pose.orientation
        _, _, self.now_pose_w = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def create_window(self):
        self.window = tk.Tk()
        self.window.title("Robot Pose Controller")
        self.window.geometry('220x350+1700+560')
        self.window.minsize(220, 350)

        # 標籤框架
        frame = tk.Frame(self.window)
        frame.pack(pady=10)

        # 創建標籤
        labels_text = [
            "Target Pose", "Robot Pose X:", "Robot Pose Y:", "Robot Pose Orientation:",
            "Now Pose", "Robot Pose X Now:", "Robot Pose Y Now:", "Robot Pose Orientation Now:"
        ]
        self.labels = {}
        for i, text in enumerate(labels_text):
            label = tk.Label(frame, text=text)
            value_label = tk.Label(frame, text="0.0")
            label.grid(row=i, column=0, sticky='w', padx=5)
            value_label.grid(row=i, column=1, sticky='e', padx=5)
            self.labels[text] = value_label

        # 按鈕創建
        button_frame = tk.Frame(self.window)
        button_frame.pack(pady=10)

        button_labels = ['Goal 1', 'Goal 2', 'Goal 3', 'Goal 4', 'Goal 5', 'Goal 6']
        for i, label in enumerate(button_labels):
            button = tk.Button(button_frame, text=label, command=lambda i=i: self.set_goal(i))
            button.grid(row=i, column=0, pady=2)

        self.window.after(50, self.update_window)
        self.window.mainloop()

    def update_window(self):
        update_values = {
            "Robot Pose X:": self.target_pose_x,
            "Robot Pose Y:": self.target_pose_y,
            "Robot Pose Orientation:": self.target_pose_w,
            "Robot Pose X Now:": self.now_pose_x,
            "Robot Pose Y Now:": self.now_pose_y,
            "Robot Pose Orientation Now:": self.now_pose_w,
        }
        for key, value in update_values.items():
            self.labels[key].configure(text=f"{value:.2f}")

        self.window.after(50, self.update_window)

if __name__ == '__main__':
    rospy.init_node('gui')
    subscriber = Subscriber()
    rospy.spin()
