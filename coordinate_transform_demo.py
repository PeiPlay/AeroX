import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider
import math
import time

class CoordinateTransformDemo:
    def __init__(self):
        # Create figure and subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))
        self.fig.suptitle('Ground Coordinate System → Body Coordinate System Transform Demo', fontsize=16)
        
        # Initial parameters
        self.robot_x_ground = 2.0      # Robot X position in ground coordinate system
        self.robot_y_ground = 1.5      # Robot Y position in ground coordinate system
        self.robot_yaw = 0.3           # Robot yaw angle (radians)
        
        # Target point position in ground coordinate system
        self.target_x_ground = 4.0
        self.target_y_ground = 3.0
        
        # Keyboard control variables
        self.key_pressed = set()
        self.key_step = 0.1  # Step size for keyboard control
        self.yaw_step = 0.05  # Step size for yaw angle
        
        # Setup axes
        self.setup_axes()
        
        # Create slider controls
        self.create_sliders()
        
        # Setup keyboard events
        self.setup_keyboard_events()
        
        # Initial drawing
        self.update_plot()
        
    def setup_axes(self):
        """Setup coordinate axes"""
        # Ground coordinate system view
        self.ax1.set_xlim(-1, 6)
        self.ax1.set_ylim(-1, 5)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_title('Ground Coordinate System View\n(Origin at power-on point, x-left y-forward z-up)')
        self.ax1.set_xlabel('X (Left positive)')
        self.ax1.set_ylabel('Y (Forward positive)')
        
        # Body coordinate system view
        self.ax2.set_xlim(-3, 3)
        self.ax2.set_ylim(-3, 3)
        self.ax2.set_aspect('equal')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_title('Body Coordinate System View\n(Origin at robot center, x-left y-forward z-up)')
        self.ax2.set_xlabel('X (Robot left positive)')
        self.ax2.set_ylabel('Y (Robot forward positive)')
        
    def create_sliders(self):
        """Create slider controls"""
        plt.subplots_adjust(bottom=0.35)
        
        # Robot position and orientation sliders
        ax_robot_x = plt.axes([0.1, 0.25, 0.3, 0.03])
        ax_robot_y = plt.axes([0.1, 0.20, 0.3, 0.03])
        ax_robot_yaw = plt.axes([0.1, 0.15, 0.3, 0.03])
        
        # Target point position sliders
        ax_target_x = plt.axes([0.6, 0.25, 0.3, 0.03])
        ax_target_y = plt.axes([0.6, 0.20, 0.3, 0.03])
        
        self.slider_robot_x = Slider(ax_robot_x, 'Robot X', -1, 6, valinit=self.robot_x_ground)
        self.slider_robot_y = Slider(ax_robot_y, 'Robot Y', -1, 5, valinit=self.robot_y_ground)
        self.slider_robot_yaw = Slider(ax_robot_yaw, 'Robot Yaw(rad)', -np.pi, np.pi, valinit=self.robot_yaw)
        
        self.slider_target_x = Slider(ax_target_x, 'Target X', -1, 6, valinit=self.target_x_ground)
        self.slider_target_y = Slider(ax_target_y, 'Target Y', -1, 5, valinit=self.target_y_ground)
        
        # Bind slider events
        self.slider_robot_x.on_changed(self.update_robot_x)
        self.slider_robot_y.on_changed(self.update_robot_y)
        self.slider_robot_yaw.on_changed(self.update_robot_yaw)
        self.slider_target_x.on_changed(self.update_target_x)
        self.slider_target_y.on_changed(self.update_target_y)
        
    def setup_keyboard_events(self):
        """Setup keyboard event handlers"""
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        
        # Start keyboard polling timer
        self.timer = self.fig.canvas.new_timer(interval=50)  # 20 FPS
        self.timer.add_callback(self.process_keyboard_input)
        self.timer.start()
        
    def on_key_press(self, event):
        """Handle key press events"""
        if event.key:
            self.key_pressed.add(event.key)
            
    def on_key_release(self, event):
        """Handle key release events"""
        if event.key in self.key_pressed:
            self.key_pressed.remove(event.key)
            
    def process_keyboard_input(self):
        """Process continuous keyboard input"""
        changed = False
        
        # Robot position control (WASD)
        if 'w' in self.key_pressed:  # Move robot forward (Y+)
            self.robot_y_ground = min(5, self.robot_y_ground + self.key_step)
            changed = True
        if 's' in self.key_pressed:  # Move robot backward (Y-)
            self.robot_y_ground = max(-1, self.robot_y_ground - self.key_step)
            changed = True
        if 'a' in self.key_pressed:  # Move robot left (X+)
            self.robot_x_ground = min(6, self.robot_x_ground + self.key_step)
            changed = True
        if 'd' in self.key_pressed:  # Move robot right (X-)
            self.robot_x_ground = max(-1, self.robot_x_ground - self.key_step)
            changed = True
            
        # Robot orientation control (Q/E)
        if 'q' in self.key_pressed:  # Rotate counterclockwise
            self.robot_yaw = min(np.pi, self.robot_yaw + self.yaw_step)
            changed = True
        if 'e' in self.key_pressed:  # Rotate clockwise
            self.robot_yaw = max(-np.pi, self.robot_yaw - self.yaw_step)
            changed = True
            
        # Target position control (Arrow keys)
        if 'up' in self.key_pressed:  # Move target forward (Y+)
            self.target_y_ground = min(5, self.target_y_ground + self.key_step)
            changed = True
        if 'down' in self.key_pressed:  # Move target backward (Y-)
            self.target_y_ground = max(-1, self.target_y_ground - self.key_step)
            changed = True
        if 'left' in self.key_pressed:  # Move target left (X+)
            self.target_x_ground = min(6, self.target_x_ground + self.key_step)
            changed = True
        if 'right' in self.key_pressed:  # Move target right (X-)
            self.target_x_ground = max(-1, self.target_x_ground - self.key_step)
            changed = True
            
        # Update sliders and plot if any value changed
        if changed:
            self.update_sliders()
            self.update_plot()
            
    def update_sliders(self):
        """Update slider values to match current parameters"""
        self.slider_robot_x.set_val(self.robot_x_ground)
        self.slider_robot_y.set_val(self.robot_y_ground)
        self.slider_robot_yaw.set_val(self.robot_yaw)
        self.slider_target_x.set_val(self.target_x_ground)
        self.slider_target_y.set_val(self.target_y_ground)
        
    def ground_to_body_transform(self, x_ground, y_ground, robot_x, robot_y, robot_yaw):
        """
        Transform from ground coordinate system to body coordinate system
        
        Parameters:
        - x_ground, y_ground: Target point position in ground coordinate system
        - robot_x, robot_y: Robot position in ground coordinate system
        - robot_yaw: Robot yaw angle (radians, counterclockwise positive)
            yaw=0 means robot is facing positive y-axis of ground coordinate system
        
        Returns:
        - x_body, y_body: Target point position in body coordinate system
        """
        # 1. Calculate target point position vector relative to robot (ground coordinate system)
        dx_ground = x_ground - robot_x
        dy_ground = y_ground - robot_y
        
        # 2. Rotation transform: Convert ground coordinate vector to body coordinate system
        # Since both coordinate systems have the same axis definitions (x-left, y-forward, z-up),
        # when yaw=0, the coordinate systems are identical.
        # We only need to rotate by -robot_yaw (inverse rotation)
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        
        # Standard 2D rotation matrix for inverse rotation
        # [ cos(-yaw) -sin(-yaw) ] [ dx ]   [ dx*cos(-yaw) - dy*sin(-yaw) ]
        # [ sin(-yaw)  cos(-yaw) ] [ dy ] = [ dx*sin(-yaw) + dy*cos(-yaw) ]
        
        x_body = dx_ground * cos_yaw - dy_ground * sin_yaw  # Robot left positive
        y_body = dx_ground * sin_yaw + dy_ground * cos_yaw  # Robot forward positive
        
        return x_body, y_body
    
    def draw_robot(self, ax, x, y, yaw, color='blue', label=None):
        """Draw robot"""
        # Robot size
        robot_size = 0.3
        
        # Robot body (rectangle)
        # When yaw=0, rectangle should be facing y-axis positive direction
        robot_rect = patches.Rectangle(
            (x - robot_size/2, y - robot_size/2), 
            robot_size, robot_size, 
            angle=math.degrees(yaw + math.pi/2),  # Add 90 degrees so yaw=0 means facing y+ direction
            facecolor=color, 
            alpha=0.6,
            label=label
        )
        ax.add_patch(robot_rect)
        
        # Draw robot orientation arrow
        # When yaw=0, arrow should point to y-axis positive direction
        arrow_length = 0.4
        arrow_x = x + arrow_length * math.sin(yaw)  # Use sin for x to make yaw=0 point to y+
        arrow_y = y + arrow_length * math.cos(yaw)  # Use cos for y to make yaw=0 point to y+
        
        ax.arrow(x, y, arrow_x - x, arrow_y - y, 
                head_width=0.1, head_length=0.1, 
                fc=color, ec=color, linewidth=2)
        
        return robot_rect
    
    def update_plot(self):
        """Update plot"""
        # Clear previous drawings
        self.ax1.clear()
        self.ax2.clear()
        self.setup_axes()
        
        # Calculate target point position in body coordinate system
        target_x_body, target_y_body = self.ground_to_body_transform(
            self.target_x_ground, self.target_y_ground,
            self.robot_x_ground, self.robot_y_ground, self.robot_yaw
        )
        
        # === Ground coordinate system view ===
        # Draw ground coordinate system origin
        self.ax1.plot(0, 0, 'ko', markersize=8, label='Ground Origin')
        
        # Draw ground coordinate system axes
        self.ax1.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='red', ec='red', label='Ground X-axis')
        self.ax1.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='green', ec='green', label='Ground Y-axis')
        
        # Draw robot
        self.draw_robot(self.ax1, self.robot_x_ground, self.robot_y_ground, self.robot_yaw, 'blue', 'Robot')
        
        # Draw target point
        self.ax1.plot(self.target_x_ground, self.target_y_ground, 'ro', markersize=8, label='Target Point')
        
        # Draw connection line
        self.ax1.plot([self.robot_x_ground, self.target_x_ground], 
                     [self.robot_y_ground, self.target_y_ground], 'r--', alpha=0.5)
        
        # Add text annotations
        self.ax1.text(self.target_x_ground + 0.1, self.target_y_ground + 0.1, 
                     f'Target\n({self.target_x_ground:.1f}, {self.target_y_ground:.1f})', 
                     fontsize=10)
        self.ax1.text(self.robot_x_ground + 0.1, self.robot_y_ground - 0.3, 
                     f'Robot\n({self.robot_x_ground:.1f}, {self.robot_y_ground:.1f})\nYaw={math.degrees(self.robot_yaw):.1f}°', 
                     fontsize=10)
        
        self.ax1.legend(loc='upper left', fontsize=8)
        
        # === Body coordinate system view ===
        # Draw body coordinate system origin (robot center)
        self.ax2.plot(0, 0, 'bo', markersize=8, label='Body Origin')
        
        # Draw body coordinate system axes
        self.ax2.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='red', ec='red', label='Body X-axis(left)')
        self.ax2.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='green', ec='green', label='Body Y-axis(forward)')
        
        # Draw robot (fixed at origin, facing Y-axis positive direction)
        self.draw_robot(self.ax2, 0, 0, 0, 'blue', 'Robot')
        
        # Draw target point position in body coordinate system
        self.ax2.plot(target_x_body, target_y_body, 'ro', markersize=8, label='Target Point')
        
        # Draw connection line
        self.ax2.plot([0, target_x_body], [0, target_y_body], 'r--', alpha=0.5)
        
        # Add text annotations
        self.ax2.text(target_x_body + 0.1, target_y_body + 0.1, 
                     f'Target\n({target_x_body:.2f}, {target_y_body:.2f})', 
                     fontsize=10)
        
        # Add distance and angle information
        distance = math.sqrt(target_x_body**2 + target_y_body**2)
        angle_rad = math.atan2(target_y_body, target_x_body)
        self.ax2.text(-2.5, -2.5, 
                     f'Distance: {distance:.2f}m\nAngle: {math.degrees(angle_rad):.1f}°', 
                     fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
        
        self.ax2.legend(loc='upper right', fontsize=8)
        
        # Add transform formula display
        formula_text = (
            "Transform Formula:\n"
            f"dx = {self.target_x_ground:.1f} - {self.robot_x_ground:.1f} = {self.target_x_ground - self.robot_x_ground:.1f}\n"
            f"dy = {self.target_y_ground:.1f} - {self.robot_y_ground:.1f} = {self.target_y_ground - self.robot_y_ground:.1f}\n"
            f"x_body = dx·cos(-yaw) - dy·sin(-yaw) = {target_x_body:.2f}\n"
            f"y_body = dx·sin(-yaw) + dy·cos(-yaw) = {target_y_body:.2f}"
        )
        
        # Add keyboard control instructions
        control_text = (
            "Keyboard Controls:\n"
            "Robot: WASD (move), Q/E (rotate)\n"
            "Target: Arrow keys (move)\n"
            "Hold keys for continuous movement"
        )
        
        self.fig.text(0.25, 0.08, formula_text, fontsize=9, ha='center', 
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow"))
        self.fig.text(0.75, 0.08, control_text, fontsize=9, ha='center',
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen"))
        
        plt.draw()
    
    # Slider event handler functions
    def update_robot_x(self, val):
        self.robot_x_ground = val
        self.update_plot()
    
    def update_robot_y(self, val):
        self.robot_y_ground = val
        self.update_plot()
    
    def update_robot_yaw(self, val):
        self.robot_yaw = val
        self.update_plot()
    
    def update_target_x(self, val):
        self.target_x_ground = val
        self.update_plot()
    
    def update_target_y(self, val):
        self.target_y_ground = val
        self.update_plot()

def main():
    """Main function"""
    print("Coordinate Transform Demo Program Started")
    print("=" * 50)
    print("Coordinate System Description:")
    print("Ground coordinate system: Power-on point as origin, x-right(left) positive, y-forward positive, z-up positive")
    print("Body coordinate system: Robot center as origin, x-left positive, y-forward positive, z-up positive")
    print("=" * 50)
    print("Use sliders to adjust robot position, orientation and target point position")
    print("Left side shows ground coordinate system view, right side shows body coordinate system view")
    print("=" * 50)
    print("Keyboard Controls:")
    print("Robot Movement: W/S (forward/backward), A/D (left/right)")
    print("Robot Rotation: Q (counterclockwise), E (clockwise)")
    print("Target Movement: Arrow keys (up/down/left/right)")
    print("Hold keys for continuous movement")
    print("=" * 50)
    
    # Create demo instance
    demo = CoordinateTransformDemo()
    
    # Show window
    plt.show()

if __name__ == "__main__":
    main()
