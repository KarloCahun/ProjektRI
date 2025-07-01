#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
import socket
import time

class TCPBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('listen_host', '192.168.0.216'),
                ('listen_port', 7122),
                ('send_host', '192.168.0.25'),
                ('send_port', 7122),
                ('confirmation_message', 'Message received!'),
                ('receive_timeout', 1.0),
                ('send_timeout', 2.0),
                ('topic_name', 'coordinates_topic')  
            ])
        
        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = self.get_parameter('listen_port').value
        self.send_host = self.get_parameter('send_host').value
        self.send_port = self.get_parameter('send_port').value
        self.confirmation_msg = self.get_parameter('confirmation_message').value.encode()
        self.receive_timeout = self.get_parameter('receive_timeout').value
        self.send_timeout = self.get_parameter('send_timeout').value
        
       
        self.publisher_ = self.create_publisher(
            String, 
            self.get_parameter('topic_name').value, 10)
        
        self.listener_socket = None
        self.active_connection = None
        self.shutdown_requested = False
        self.setup_listener()
        
        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info(f'Node started, listening on {self.listen_host}:{self.listen_port}')
        self.get_logger().info(f'Publishing messages to topic: {self.get_parameter("topic_name").value}')

    def setup_listener(self):
        """Postavi socket za slušanje"""
        try:
            if self.listener_socket:
                self.listener_socket.close()
                
            self.listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener_socket.bind((self.listen_host, self.listen_port))
            self.listener_socket.listen()
            self.listener_socket.settimeout(self.receive_timeout)
            
        except Exception as e:
            self.get_logger().error(f'Error setting up listener: {str(e)}')
            time.sleep(1)
            if not self.shutdown_requested:
                self.setup_listener()

    def send_confirmation(self):
        """Šalje potvrdnu poruku"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(self.send_timeout)
                s.connect((self.send_host, self.send_port))
                s.sendall(self.confirmation_msg)
                self.get_logger().info('Confirmation sent successfully', throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f'Error sending confirmation: {str(e)}')

    def publish_message(self, data):
        """Publisha primljenu poruku na topic"""
        try:
            msg = String()
            msg.data = data.decode() if isinstance(data, bytes) else str(data)
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published to topic: {msg.data}', throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f'Error publishing message: {str(e)}')

    def main_loop(self):
        """Glavna obrada poruka"""
        if self.shutdown_requested:
            return
            
        try:
          
            conn, addr = self.listener_socket.accept()
            self.get_logger().info(f"New connection from {addr[0]}:{addr[1]}")
            self.active_connection = conn
            
            with conn:
                conn.settimeout(self.receive_timeout)
                while not self.shutdown_requested:
                    try:
                        data = conn.recv(4096) 
                        if not data:
                            break
                            
                        self.get_logger().info(f'Received: {data.decode()}')
                        self.send_confirmation()
                        self.publish_message(data)
                        
                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.get_logger().error(f'Connection error: {str(e)}')
                        break
                        
        except socket.timeout:
            pass  
        except Exception as e:
            self.get_logger().error(f'Accept error: {str(e)}')
            if not self.shutdown_requested:
                time.sleep(1)
                self.setup_listener()

    def shutdown(self):
        """Čišćenje pri zatvaranju čvora"""
        self.shutdown_requested = True
        self.get_logger().info("Shutting down node...")
        
        if self.active_connection:
            try:
                self.active_connection.close()
            except:
                pass
                
        if self.listener_socket:
            try:
                self.listener_socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TCPBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        node.get_logger().info("Node shutdown complete")

if __name__ == '__main__':
    main()
