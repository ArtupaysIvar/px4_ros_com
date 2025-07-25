#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
import matplotlib.pyplot as plt
from std_msgs.msg import Empty

class WaypointServer(Node):
    def __init__(self):
        super().__init__('waypoint_server')
        
        # Publisher - continuously publish at 2 Hz
        self.pose_array_pub = self.create_publisher(PoseArray, '/waypoint_array', 10)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/clicked_waypoints', 10)
        self.start_pub = self.create_publisher(Empty, '/start_mission', 10)


        self.publish_timer = self.create_timer(0.5, self.publish_waypoints)  # 2 Hz
        
        self.waypoint_labels = []  # To store plt.text objects

        # Waypoint management
        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.frame_id = "map"
        
        # Add initial hover waypoint at (0, 0, -1)
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = -1.0  # 1 meter altitude
        initial_pose.orientation.w = 1.0
        self.pose_array_msg.poses.append(initial_pose)
        
        # UI state
        self.clicked_points = [(0.0, 0.0)]  # Include initial point
        self.plotted_dots = []
        
        self.get_logger().info("Waypoint Server started. Publishing waypoints at 2 Hz on /waypoint_array")

    def publish_waypoints(self):
        """Continuously publish current waypoint plan at 2 Hz"""
        self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_pub.publish(self.pose_array_msg)
    
    def onclick(self, event):
        if event.xdata is None or event.ydata is None:
            return  # Ignore clicks outside plot
        
        #x, y = event.xdata, event.ydata
        y, x = event.xdata, event.ydata  # <- rotated axes
        
        # Add to PoseArray (at flight altitude -1m)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = -1.0  # Flight altitude
        pose.orientation.w = 1.0
        self.pose_array_msg.poses.append(pose)

        # Publish PoseStamped for visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose  
        self.pose_stamped_pub.publish(pose_stamped)
        
        print(f"Added waypoint: ({x:.2f}, {y:.2f}, -1.0) | Total: {len(self.pose_array_msg.poses)} waypoints")
        
        # Store and plot the point
        self.clicked_points.append((x, y))
        #dot, = plt.plot(x, y, 'bo', markersize=6)
        #label = plt.text(x+0.1, y+0.1, f'{len(self.clicked_points)-1}', fontsize=8)
        dot, = plt.plot(y, x, 'bo', markersize=6)  # <- plot reversed
        label = plt.text(y+0.1, x+0.1, f'{len(self.clicked_points)-1}', fontsize=8)
        self.plotted_dots.append(dot)
        self.waypoint_labels.append(label)
        plt.draw()

    
    def onkey(self, event):
        if event.key == 'z' and len(self.clicked_points) > 1:  # Don't allow removing initial waypoint
            # Undo last point
            last_point = self.clicked_points.pop()
            last_dot = self.plotted_dots.pop()
            removed_label = self.waypoint_labels.pop()
            removed_label.remove()  # Remove text label
            last_dot.remove()
            plt.draw()
            
            if len(self.pose_array_msg.poses) > 1:  # Keep initial waypoint
                self.pose_array_msg.poses.pop()
            print(f"Removed waypoint: ({last_point[0]:.2f}, {last_point[1]:.2f}) | Total: {len(self.pose_array_msg.poses)} waypoints")
        
        elif event.key == 'c':
            # Clear all waypoints except initial
            self.pose_array_msg.poses = [self.pose_array_msg.poses[0]]  # Keep initial hover
            self.clicked_points = [(0.0, 0.0)]

            # Remove all waypoint text labels (except 'HOME')
            for label in self.waypoint_labels:
                label.remove()
            self.waypoint_labels.clear()

            # Remove all plotted dots (except origin)
            for dot in self.plotted_dots:
                dot.remove()
            self.plotted_dots.clear()

            # Re-draw origin dot and HOME label if needed
            plt.plot(0, 0, 'ro', markersize=10)
            home_label = plt.text(0.1, 0.1, 'HOME', fontsize=10, fontweight='bold')
            self.waypoint_labels.append(home_label)

            plt.draw()
            print("Cleared all waypoints except initial hover point")

            
        elif event.key == 'd':
            self.start_pub.publish(Empty())
            print("Mission start signal sent on /start_mission!")


def main():
    rclpy.init()
    server = WaypointServer()
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Waypoint Server | Click=add waypoint | z=undo | c=clear | d=start")
    #ax.set_xlabel("X [m]")
    #ax.set_ylabel("Y [m]")
    ax.set_xlabel("Y [m]")
    ax.set_ylabel("X [m]")
    ax.plot(0, 0, 'ro', markersize=10, label='Initial Hover (0,0,-1m)')
    ax.text(0.1, 0.1, 'HOME', fontsize=10, fontweight='bold')
    
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Add simplified instructions
    """
    instructions = [
        "WAYPOINT SERVER:",
        " Click anywhere to add waypoint",
        " 'z' = undo last waypoint", 
        " 'c' = clear all waypoints",
        "",
        " Publishing continuously on:",
        " /waypoint_array (2 Hz)"
    ]
    
    textstr = '\n'.join(instructions)
    props = dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', bbox=props)
    """
    # Connect callbacks
    fig.canvas.mpl_connect('button_press_event', server.onclick)
    fig.canvas.mpl_connect('key_press_event', server.onkey)
    
    # Run ROS2 spin in background
    import threading
    thread = threading.Thread(target=rclpy.spin, args=(server,), daemon=True)
    thread.start()
    
    print("Waypoint Server Running!")
    print("   - Publishing waypoints continuously on /waypoint_array at 2 Hz")
    print("   - Click on the plot to add waypoints")
    print("   - Your C++ offboard_control node can subscribe to /waypoint_array")
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()