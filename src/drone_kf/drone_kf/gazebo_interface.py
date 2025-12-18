"""
Interface for communicating with Gazebo simulation.
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from geometry_msgs.msg import Pose


class GazeboInterface:
    """
    Handles all Gazebo service calls for entity control.
    """
    
    def __init__(self, node: Node, entity_name='simple_drone'):
        """
        Initialize Gazebo interface.
        
        Args:
            node: ROS 2 node instance
            entity_name: Name of the entity in Gazebo
        """
        self.node = node
        self.entity_name = entity_name
        
        # Create service clients
        self.client_set = node.create_client(
            SetEntityState, 
            '/gazebo/set_entity_state'
        )
        
        self.client_get = node.create_client(
            GetEntityState, 
            '/gazebo/get_entity_state'
        )
        
        # Wait for services to be available
        self._wait_for_services()
    
    def _wait_for_services(self, timeout=5.0):
        """Wait for Gazebo services to become available."""
        if not self.client_set.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().warn(
                'SetEntityState service not available, continuing anyway...'
            )
        
        if not self.client_get.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().warn(
                'GetEntityState service not available, continuing anyway...'
            )
    
    def set_position(self, x=0.0, y=0.0, z=0.0):
        """
        Set the position of the entity in Gazebo.
        
        Args:
            x, y, z: Position coordinates
        """
        if not self.client_set.service_is_ready():
            return False
        
        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        
        # Async call (non-blocking)
        self.client_set.call_async(req)
        return True
    
    def set_pose(self, pose: Pose):
        """
        Set the full pose of the entity.
        
        Args:
            pose: Geometry_msgs Pose message
        """
        if not self.client_set.service_is_ready():
            return False
        
        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose = pose
        
        self.client_set.call_async(req)
        return True
    
    def get_position(self):
        """
        Get the current position from Gazebo (synchronous).
        
        Returns:
            Tuple (x, y, z) or None if service unavailable
        """
        if not self.client_get.service_is_ready():
            return None
        
        req = GetEntityState.Request()
        req.name = self.entity_name
        
        try:
            future = self.client_get.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.result() is not None:
                pos = future.result().state.pose.position
                return (pos.x, pos.y, pos.z)
        except Exception as e:
            self.node.get_logger().error(f'Failed to get position: {e}')
        
        return None
    
    def delete_entity(self):
        """Delete the entity from Gazebo."""
        from gazebo_msgs.srv import DeleteEntity
        delete_client = self.node.create_client(DeleteEntity, '/delete_entity')
        
        if delete_client.wait_for_service(timeout_sec=1.0):
            request = DeleteEntity.Request()
            request.name = self.entity_name
            future = delete_client.call_async(request)
            self.node.get_logger().info(f"Deleted entity: {self.entity_name}")
