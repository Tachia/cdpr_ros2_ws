#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time

# Set matplotlib backend for WSL2
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for WSL2

class CDPRVisualizer(Node):
    def __init__(self):
        super().__init__('cdpr_visualizer')
        
        # Data storage
        self.position_history = deque(maxlen=200)
        self.tension_history = deque(maxlen=200)
        self.time_history = deque(maxlen=200)
        self.cable_tensions = [0.0] * 8
        self.current_position = [0.0, 0.0, 5.0]
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/platform/pose', self.pose_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Start time
        self.start_time = time.time()
        
        # Setup plots
        self.setup_plots()
        
        # Timer for updating plots
        self.timer = self.create_timer(0.1, self.update_plots)
        
        self.get_logger().info('CDPR Visualizer started')
        
    def setup_plots(self):
        plt.ion()  # Turn on interactive mode
        
        # Create figure with 4 subplots
        self.fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
        
        # Plot 1: Platform trajectory (XY)
        ax1.set_title('Platform Trajectory (XY Plane)', fontsize=12, fontweight='bold')
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        self.trajectory_line, = ax1.plot([], [], 'b-', linewidth=2, label='Actual Path')
        self.desired_circle = plt.Circle((0, 0), 2.0, color='r', fill=False, linestyle='--', 
                                        linewidth=1.5, label='Desired Path')
        ax1.add_patch(self.desired_circle)
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend(loc='upper right')
        ax1.axis('equal')
        ax1.set_xlim(-4, 4)
        ax1.set_ylim(-4, 4)
        
        # Plot 2: Cable tensions over time
        ax2.set_title('Cable Tensions Over Time', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Time (s)', fontsize=10)
        ax2.set_ylabel('Tension (N)', fontsize=10)
        self.tension_lines = []
        colors = ['#FF0000', '#00FF00', '#0000FF', '#FF00FF', 
                 '#00FFFF', '#FFFF00', '#FFA500', '#800080']
        cable_labels = [f'Cable {i+1}' for i in range(8)]
        
        for i in range(8):
            line, = ax2.plot([], [], color=colors[i], linewidth=1.5, label=cable_labels[i])
            self.tension_lines.append(line)
        
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.legend(loc='upper right', fontsize=8)
        ax2.set_ylim(0, 100)
        
        # Plot 3: 3D trajectory
        from mpl_toolkits.mplot3d import Axes3D
        self.ax3 = self.fig.add_subplot(223, projection='3d')
        self.ax3.set_title('3D Trajectory', fontsize=12, fontweight='bold')
        self.ax3.set_xlabel('X (m)', fontsize=10)
        self.ax3.set_ylabel('Y (m)', fontsize=10)
        self.ax3.set_zlabel('Z (m)', fontsize=10)
        self.traj_3d, = self.ax3.plot([], [], [], 'b-', linewidth=2, label='3D Path')
        
        # Plot anchor points (pillars)
        pillar_positions = [
            (-5, -5, 10), (5, -5, 10),
            (-5, 5, 10), (5, 5, 10)
        ]
        for pos in pillar_positions:
            self.ax3.scatter(*pos, c='r', marker='^', s=100, label='Pillar' if pillar_positions.index(pos) == 0 else "")
        
        self.ax3.set_xlim(-6, 6)
        self.ax3.set_ylim(-6, 6)
        self.ax3.set_zlim(0, 12)
        
        # Plot 4: Current tension distribution
        ax4.set_title('Current Cable Tensions', fontsize=12, fontweight='bold')
        ax4.set_xlabel('Cable Number', fontsize=10)
        ax4.set_ylabel('Tension (N)', fontsize=10)
        self.bars = ax4.bar(range(1, 9), self.cable_tensions, 
                          color=colors, alpha=0.7, edgecolor='black')
        ax4.set_xticks(range(1, 9))
        ax4.set_ylim(0, 100)
        ax4.grid(True, axis='y', linestyle='--', alpha=0.6)
        
        # Add tension values on bars
        for bar in self.bars:
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.1f}', ha='center', va='bottom', fontsize=8)
        
        plt.tight_layout()
        self.fig.canvas.draw()
        
    def pose_callback(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.position_history.append(pos)
        self.current_position = pos
        
    def joint_callback(self, msg):
        if len(msg.effort) == 8:
            self.cable_tensions = msg.effort
            current_time = time.time() - self.start_time
            self.time_history.append(current_time)
            self.tension_history.append(list(self.cable_tensions))
            
    def update_plots(self):
        try:
            # Update trajectory plot
            if len(self.position_history) > 0:
                positions = np.array(self.position_history)
                if len(positions) > 1:
                    self.trajectory_line.set_data(positions[:, 0], positions[:, 1])
                    
                    # Update 3D trajectory
                    self.traj_3d.set_data_3d(
                        positions[:, 0], 
                        positions[:, 1], 
                        positions[:, 2]
                    )
            
            # Update tension plots
            if len(self.time_history) > 0 and len(self.tension_history) > 0:
                times = list(self.time_history)
                tensions = np.array(self.tension_history)
                
                for i in range(8):
                    if len(times) > 0:
                        self.tension_lines[i].set_data(times[-100:], tensions[-100:, i])
                
                # Update x-axis limits for time plot
                ax2 = self.fig.axes[1]
                if len(times) > 1:
                    ax2.set_xlim(max(0, times[-1] - 10), max(10, times[-1] + 1))
            
            # Update bar chart
            for bar, tension in zip(self.bars, self.cable_tensions):
                bar.set_height(tension)
            
            # Update tension values on bars
            ax4 = self.fig.axes[3]
            for bar in self.bars:
                height = bar.get_height()
                ax4.texts.clear()  # Clear previous texts
                ax4.text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.1f}', ha='center', va='bottom', fontsize=8)
            
            # Update title with current position
            self.fig.suptitle(
                f'CDPR Simulation - Position: [{self.current_position[0]:.2f}, '
                f'{self.current_position[1]:.2f}, {self.current_position[2]:.2f}]',
                fontsize=14, fontweight='bold'
            )
            
            # Redraw
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f'Error updating plots: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = CDPRVisualizer()
        
        # Run ROS node in a separate thread
        executor_thread = threading.Thread(target=rclpy.spin, args=(visualizer,))
        executor_thread.start()
        
        # Show plot window (blocks until window is closed)
        plt.show(block=True)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        if 'executor_thread' in locals():
            executor_thread.join()
        plt.close('all')

if __name__ == '__main__':
    main()
