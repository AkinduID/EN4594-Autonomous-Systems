import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

# Import our modular components
from .kalman_filter import KalmanFilter1D
from .gazebo_interface import GazeboInterface
from .trajectory_generator import TrajectoryGenerator, NoiseGenerator


class DroneKF(Node):
    def __init__(self):
        super().__init__('drone_kf_node')

        # Publishers for plotting data
        self.est_pub = self.create_publisher(Odometry, '/estimated/state', 10)
        self.sensor_pub = self.create_publisher(Odometry, '/sensor/measurement', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth', 10)

        # Initialize modular components
        self.dt = 0.1
        self.kf = KalmanFilter1D(
            dt=self.dt,
            process_noise=0.01,
            measurement_noise=0.5
        )
        
        self.gazebo = GazeboInterface(self, entity_name='simple_drone')
        self.trajectory = TrajectoryGenerator(trajectory_type='sine')
        self.noise_gen = NoiseGenerator(noise_std=0.5)

        # Start Loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.start_time = time.time()
        self.get_logger().info("Drone Simulation & Filter Started!")

    def control_loop(self):
        # 1. SIMULATE PHYSICS - Get trajectory position
        elapsed = time.time() - self.start_time
        x, y, true_z = self.trajectory.get_position(elapsed)

        # Send command to Gazebo to move the visual box
        self.gazebo.set_position(x=x, y=y, z=true_z)

        # 2. GENERATE NOISY SENSOR DATA
        measured_z = self.noise_gen.add_noise(true_z)

        # 3. KALMAN FILTER UPDATE
        self.kf.predict()
        self.kf.update(measured_z)
        
        estimated_z = self.kf.get_position()

        # 4. PUBLISH RESULTS
        self.publish_data(self.truth_pub, true_z)
        self.publish_data(self.sensor_pub, measured_z)
        self.publish_data(self.est_pub, estimated_z)

    def publish_data(self, pub, z_val):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.z = float(z_val)
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
