import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from auv_control.srv import InjectFault
import time

class FTCTester(Node):
    def __init__(self):
        super().__init__('ftc_tester')
        self.u_virtual = None
        self.sub = self.create_subscription(
            Float64MultiArray, '/auv/virtual_u', self.cb, 10)
        self.client = self.create_client(InjectFault, '/auv/inject_fault')

    def cb(self, msg):
        self.u_virtual = msg.data

def main():
    rclpy.init()
    tester = FTCTester()
    
    print("Esperando a que el controlador publique datos...")
    while rclpy.ok() and tester.u_virtual is None:
        rclpy.spin_once(tester, timeout_sec=1.0)
    
    u_init = tester.u_virtual[0]
    print(f"Esfuerzo inicial u1: {u_init:.4f}")
    
    print("Inyectando falla del 50%...")
    req = InjectFault.Request()
    req.thruster_id = 1
    req.fault_factor = 0.5
    req.fault_type = 'abrupt'
    
    while not tester.client.wait_for_service(timeout_sec=1.0):
        print("Esperando al servicio de falla...")
    
    future = tester.client.call_async(req)
    rclpy.spin_until_future_complete(tester, future)
    
    print("Esperando adaptación (5s)...")
    time.sleep(5.0)
    
    # Spin multiple times to clear queue and get latest data
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    u_final = tester.u_virtual[0]
    print(f"Esfuerzo final u1: {u_final:.4f}")
    
    if abs(u_final) > abs(u_init) * 1.5:
        print("\nSUCCESS: El controlador aumentó el esfuerzo compensando la falla.")
    else:
        print("\nFAILURE: La adaptación no fue suficiente o no ocurrió.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
