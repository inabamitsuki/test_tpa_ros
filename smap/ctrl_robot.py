#!/usr/bin/env python3
# encoding: utf-8

import sys, select, termios, tty ,time ,threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 # หรือเปลี่ยนตาม Message Type ของ Yahboom
from sensor_msgs.msg import LaserScan  # เพิ่มการใช้งาน Lidar

msg = """
Control Your Yahboom Robot & Servo!
----------------------------------
Moving around:
    u    i    o
    j    k    l
    m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Servo Control:
    p : Press to -40, Release to 0 (Servo ID 1)

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), ',': (-1, 0), '.': (-1, 1), 'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1), 'z': (.9, .9),
    'w': (1.1, 1), 'x': (.9, 1),
    'e': (1, 1.1), 'c': (1, .9),
}

class Yahboom_Keyboard(Node):
    def __init__(self, name):
        super().__init__(name)
        # Publisher สำหรับการเคลื่อนที่
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publisher สำหรับ Servo (ปรับชื่อ Topic ตามหุ่นของคุณ เช่น /servo_angle)
        self.pub_servo = self.create_publisher(Int32, 'servo_s1', 10)
        
        # Subscribe ข้อมูล Lidar
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.declare_parameter("linear_speed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.servo_busy = False # ป้องกันการกดซ้อนขณะทำงาน
        
        # ระยะเริ่มต้น (เมตร)
        self.dist_front = 10.0
        self.dist_back  = 10.0
        self.dist_left  = 10.0
        self.dist_right = 10.0
        self.safety_limit = 0.3 # ระยะหยุด 30 ซม.
        
        self.last_beep_time = 0.0 # สำหรับหน่วงเวลาเสียงเตือน

    def scan_callback(self, msg):
        num_points = len(msg.ranges) # ปกติจะมี 360 หรือ 720 จุด
        
        # --- ลองสลับค่าตรงนี้ตามอาการที่เจอ ---
        # ถ้าหน้า-หลัง สลับกัน ให้สลับเลขระหว่าง f_idx กับ b_idx
        # ถ้าซ้าย-ขวา สลับกัน ให้สลับเลขระหว่าง l_idx กับ r_idx
        
        b_idx = list(range(0, 31)) + list(range(num_points-30, num_points))
        r_idx = list(range(60, 121))
        f_idx = list(range(150, 211))
        l_idx = list(range(240, 301))

        def get_min_dist(indices):
            # กรองค่าที่อ่านได้ (ตัด 0.0 ออก)
            ranges = [msg.ranges[i] for i in indices if i < len(msg.ranges) and 0.05 < msg.ranges[i] < 3.0]
            return min(ranges) if ranges else 10.0

        self.dist_front = get_min_dist(f_idx)
        self.dist_left  = get_min_dist(l_idx)
        self.dist_back  = get_min_dist(b_idx)
        self.dist_right = get_min_dist(r_idx)

        # --- ส่วนสำหรับตรวจสอบ (เปิดไว้ดูตอนทดสอบ) ---
        # ลองเอามือบังแต่ละด้านของ Lidar แล้วดูว่าเลขไหนลดลง
        # print(f"F: {self.dist_front:.1f} | B: {self.dist_back:.1f} | L: {self.dist_left:.1f} | R: {self.dist_right:.1f}")
    def play_warning_sound(self):
        """ ส่งเสียง Beep ผ่าน Terminal """
        current_time = time.time()
        # ส่งเสียงเตือนทุกๆ 0.5 วินาทีเมื่อติดสิ่งกีดขวาง
        if current_time - self.last_beep_time > 0.5:
            sys.stdout.write('\a')
            sys.stdout.flush()
            self.last_beep_time = current_time
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    def run_servo_sequence(self):
        """ ฟังก์ชันรันลำดับ Servo แบบอัตโนมัติ """
        self.servo_busy = True
        
        # 1. ไปที่ -40
        msg_servo = Int32(data=-40)
        self.pub_servo.publish(msg_servo)
        print("\n[Servo] Step 1: Move to -40")
        
        # 2. รอ 1 วินาที
        time.sleep(1.0)
        
        # 3. กลับมาที่ 0
        msg_servo.data = 0
        self.pub_servo.publish(msg_servo)
        print("[Servo] Step 2: Back to 0")
        
        self.servo_busy = False
        
    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

def main():
    rclpy.init()
    node = Yahboom_Keyboard("yahboom_keyboard_ctrl")
    
    (speed, turn) = (0.15, 0.8)
    (x, th) = (0, 0)
    status = 0 # เพิ่มตัวนับสถานะเพื่อพิมพ์ msg ซ้ำ
    count = 0
    

    try:
        print(msg)
        print(node.vels(speed, turn))
        
        while rclpy.ok():
            # --- จุดสำคัญ: ต้องเพิ่มบรรทัดนี้ เพื่อให้ Callback ของ Lidar ทำงาน ---
            rclpy.spin_once(node, timeout_sec=0.01)
            key = node.getKey()

            if key == 'p':
                if not node.servo_busy:
                    threading.Thread(target=node.run_servo_sequence).start()

            # --- Logic สำหรับการเคลื่อนที่ ---
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                # Speed Limit Check
                speed = min(speed, node.linear_speed_limit)
                turn = min(turn, node.angular_speed_limit)
    
                # แสดงความเร็วปัจจุบัน
                print(node.vels(speed, turn))
                
                # --- ส่วนที่เพิ่มเข้ามา: พิมพ์เมนูซ้ำเมื่อครบรอบ ---
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                # -------------------------------------------
                count = 0
            elif key == ' ' or key == 'k':
                x, th = 0, 0
            elif key == '\x03': # CTRL-C
                break
            else:
                # ถ้าไม่มีการกดปุ่มใดๆ ให้หุ่นหยุดนิ่ง (Smooth Stop)
                count += 1
                if count > 4:
                    x, th = 0, 0

            # --- เงื่อนไขการวิ่งแบบแยก 4 ทิศทาง ---
            twist = Twist()
            target_linear = speed * x
            target_angular = turn * th
            is_blocked = False

            # 1. เช็คหน้า: ถ้าจะเดินหน้า (x>0) แต่ข้างหน้าติด
            if x > 0 and node.dist_front < node.safety_limit:
                target_linear, is_blocked = 0.0, True
                print(f"\r[STOP] Front Blocked ({node.dist_front:.2f}m)", end="")
            
            # 2. เช็คหลัง: ถ้าจะถอยหลัง (x<0) แต่ข้างหลังติด
            elif x < 0 and node.dist_back < node.safety_limit:
                target_linear, is_blocked = 0.0, True
                print(f"\r[STOP] Rear Blocked ({node.dist_back:.2f}m) ", end="")

            # 3. เช็คซ้าย: ถ้าจะเลี้ยวซ้าย (th>0) แต่ข้างซ้ายติด
            if th > 0 and node.dist_left < node.safety_limit:
                target_linear, is_blocked = 0.0, True
                print(f"\r[STOP] Left Blocked ({node.dist_left:.2f}m)  ", end="")

            # 4. เช็คขวา: ถ้าจะเลี้ยวขวา (th<0) แต่ข้างขวาติด
            elif th < 0 and node.dist_right < node.safety_limit:
                target_linear, is_blocked = 0.0, True
                print(f"\r[STOP] Right Blocked ({node.dist_right:.2f}m) ", end="")
                
            # ถ้ามีการ Block ทิศทางใดทิศทางหนึ่ง ให้ส่งเสียงเตือน
            if is_blocked:
                node.play_warning_sound()
                
            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
            node.pub.publish(twist)
            
            # บังคับให้ Terminal พิมพ์ข้อความออกมาทันที (ไม่ค้างใน Buffer)
            sys.stdout.flush()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # หยุดหุ่นยนต์และคืนค่า Servo ก่อนปิดโปรแกรม
        node.pub.publish(Twist())
        node.pub_servo.publish(Int32(data=0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
