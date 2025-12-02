#!/usr/bin/env python3
# encoding: utf-8
# stm32 python sdk
import sys
import enum
import time
import queue
import struct
import serial
import threading
import argparse
import termios
import tty
import os

class PacketControllerState(enum.IntEnum):
    # The format of communication protocol
    # 0xAA 0x55 Length Function ID Data Checksum
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1
    PACKET_CONTROLLER_STATE_LENGTH = 2
    PACKET_CONTROLLER_STATE_FUNCTION = 3
    PACKET_CONTROLLER_STATE_ID = 4
    PACKET_CONTROLLER_STATE_DATA = 5
    PACKET_CONTROLLER_STATE_CHECKSUM = 6

class PacketFunction(enum.IntEnum):
    # 可通过串口实现的控制功能(The control function that is able to be implemented via serial port)
    PACKET_FUNC_SYS = 0
    PACKET_FUNC_LED = 1  # LED控制(LED control)
    PACKET_FUNC_BUZZER = 2  # 蜂鸣器控制(Buzzer control)
    PACKET_FUNC_MOTOR = 3  # 电机控制(Motor control)
    PACKET_FUNC_PWM_SERVO = 4  # PWM舵机控制, 板子上从里到外依次为1-4(PWM servo control. The port number is 1 to 0 from inside to outside)
    PACKET_FUNC_BUS_SERVO = 5  # 总线舵机控制(Bus servo control)
    PACKET_FUNC_KEY = 6  # 按键获取(Key retrieval)
    PACKET_FUNC_IMU = 7  # IMU获取(IMU retrieval)
    PACKET_FUNC_GAMEPAD = 8  # 手柄获取(Handle retrieval)
    PACKET_FUNC_SBUS = 9  # 航模遥控获取(RC control retrieval)
    PACKET_FUNC_OLED = 10 # OLED 显示内容设置(OLED display content settings）
    PACKET_FUNC_NONE = 11

class PacketReportKeyEvents(enum.IntEnum):
    # 按键的不同状态（Different status of the keys）
    KEY_EVENT_PRESSED = 0x01
    KEY_EVENT_LONGPRESS = 0x02
    KEY_EVENT_LONGPRESS_REPEAT = 0x04
    KEY_EVENT_RELEASE_FROM_LP = 0x08
    KEY_EVENT_RELEASE_FROM_SP = 0x10
    KEY_EVENT_CLICK = 0x20
    KEY_EVENT_DOUBLE_CLICK= 0x40
    KEY_EVENT_TRIPLE_CLICK = 0x80

crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    # Check
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

class SBusStatus:
    def __init__(self):
        self.channels = [0] * 16
        self.channel_17 = False
        self.channel_18 = False
        self.signal_loss = True
        self.fail_safe = False

class Board:
    buttons_map = {
            'GAMEPAD_BUTTON_MASK_L2':        0x0001,
            'GAMEPAD_BUTTON_MASK_R2':        0x0002,
            'GAMEPAD_BUTTON_MASK_SELECT':    0x0004,
            'GAMEPAD_BUTTON_MASK_START':     0x0008,
            'GAMEPAD_BUTTON_MASK_L3':        0x0020,
            'GAMEPAD_BUTTON_MASK_R3':        0x0040,
            'GAMEPAD_BUTTON_MASK_CROSS':     0x0100,
            'GAMEPAD_BUTTON_MASK_CIRCLE':    0x0200,
            'GAMEPAD_BUTTON_MASK_SQUARE':    0x0800,
            'GAMEPAD_BUTTON_MASK_TRIANGLE':  0x1000,
            'GAMEPAD_BUTTON_MASK_L1':        0x4000,
            'GAMEPAD_BUTTON_MASK_R1':        0x8000
    }

    def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=5):
        self.enable_recv = False
        self.frame = []
        self.recv_count = 0

        self.port = serial.Serial(None, baudrate, timeout=timeout)
        self.port.rts = False
        self.port.dtr = False
        self.port.setPort(device)
        self.port.open()

        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
        self.servo_read_lock = threading.Lock()
        self.pwm_servo_read_lock = threading.Lock()

        # 队列用来存储数据(A queue is used to store data)
        self.sys_queue = queue.Queue(maxsize=1)
        self.bus_servo_queue = queue.Queue(maxsize=1)
        self.pwm_servo_queue = queue.Queue(maxsize=1)
        self.key_queue = queue.Queue(maxsize=1)
        self.imu_queue = queue.Queue(maxsize=1)
        self.gamepad_queue = queue.Queue(maxsize=1)
        self.sbus_queue = queue.Queue(maxsize=1)

        self.parsers = {
            PacketFunction.PACKET_FUNC_SYS: self.packet_report_sys,
            PacketFunction.PACKET_FUNC_KEY: self.packet_report_key,
            PacketFunction.PACKET_FUNC_IMU: self.packet_report_imu,
            PacketFunction.PACKET_FUNC_GAMEPAD: self.packet_report_gamepad,
            PacketFunction.PACKET_FUNC_BUS_SERVO: self.packet_report_serial_servo,
            PacketFunction.PACKET_FUNC_SBUS: self.packet_report_sbus,
            PacketFunction.PACKET_FUNC_PWM_SERVO: self.packet_report_pwm_servo
        }

        time.sleep(0.5)
        threading.Thread(target=self.recv_task, daemon=True).start()


    def packet_report_sys(self, data):
        try:
            self.sys_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_key(self, data):
        try:
            self.key_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_imu(self, data):
        try:
            self.imu_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_gamepad(self, data):
        try:
            self.gamepad_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_serial_servo(self, data):
        try:
            self.bus_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_pwm_servo(self, data):
        try:
            self.pwm_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_sbus(self, data):
        try:
            self.sbus_queue.put_nowait(data)
        except queue.Full:
            pass

    def get_battery(self):
        # 获取电压，单位mAh(Retrieve the voltage, unit mAh)
        if self.enable_recv:
            try:
                data = self.sys_queue.get(block=False)
                if data[0] == 0x04:
                    return struct.unpack('<H', data[1:])[0]
                else:
                    None
            except queue.Empty:
                return None
        else:
            print('get_battery enable reception first!')
            return None

    def get_button(self):
        # 获取按键key1， key2状态，返回按键ID(1表示按键1，2表示按键2)和状态(0表示按下，1表示松开)(Retrieve the status of key 1 and Key 2; return key ID (1 for key 1, 2 for key 2) and status (1 for presses, 2 for released))
        if self.enable_recv:
            try:
                data = self.key_queue.get(block=False)
                key_id = data[0]
                key_event = PacketReportKeyEvents(data[1])
                if key_event == PacketReportKeyEvents.KEY_EVENT_CLICK:
                    return key_id, 0
                elif key_event == PacketReportKeyEvents.KEY_EVENT_PRESSED:
                    return key_id, 1
            except queue.Empty:
                return None
        else:
            print('get_button enable reception first!')
            return None

    def get_imu(self):
        # 获取IMU数据，返回ax, ay, az, gx, gy, gz(Retrieve IMU data, return ax, ay, az, gx, gy, gz)
        if self.enable_recv:
            try:
                # ax, ay, az, gx, gy, gz
                return struct.unpack('<6f', self.imu_queue.get(block=False))
            except queue.Empty:
                return None
        else:
            print('get_imu enable reception first!')
            return None

    def get_gamepad(self):
        # 获取手柄数据(Retrieve the handle data)
        if self.enable_recv:
            try:
                # buttons, hat, lx, ly, rx, ry
                gamepad_data = struct.unpack("<HB4b", self.gamepad_queue.get(block=False))
                # 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
                axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', ''
                buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
                for b in self.buttons_map:
                    if self.buttons_map[b] & gamepad_data[0]:
                        if b == 'GAMEPAD_BUTTON_MASK_R2':
                            axes[4] = 1.0
                        elif b == 'GAMEPAD_BUTTON_MASK_L2':
                            axes[5] = 1.0
                        elif b == 'GAMEPAD_BUTTON_MASK_CROSS':
                            buttons[0] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_CIRCLE':
                            buttons[1] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_SQUARE':
                            buttons[3] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_TRIANGLE':
                            buttons[4] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_L1':
                            buttons[6] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_R1':
                            buttons[7] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_SELECT':
                            buttons[10] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_START':
                            buttons[11] = 1
               
                if gamepad_data[2] > 0:
                    axes[0] = -gamepad_data[2] / 127
                elif gamepad_data[2] < 0:
                    axes[0] = -gamepad_data[2] / 128

                if gamepad_data[3] > 0:
                    axes[1] = gamepad_data[3] / 127
                elif gamepad_data[3] < 0:
                    axes[1] = gamepad_data[3] / 128

                if gamepad_data[4] > 0:
                    axes[2] = -gamepad_data[4] / 127
                elif gamepad_data[4] < 0:
                    axes[2] = -gamepad_data[4] / 128

                if gamepad_data[5] > 0:
                    axes[3] = gamepad_data[5] / 127
                elif gamepad_data[5] < 0:
                    axes[3] = gamepad_data[5] / 128
            
                if gamepad_data[1] == 9:
                    axes[6] = 1.0
                elif gamepad_data[1] == 13:
                    axes[6] = -1.0
                
                if gamepad_data[1] == 11:
                    axes[7] = -1.0
                elif gamepad_data[1] == 15:
                    axes[7] = 1.0
                return axes, buttons
            except queue.Empty:
                return None
        else:
            print('get_gamepad enable reception first!')
            return None

    def get_sbus(self):
        if self.enable_recv:
            try:
                sbus_data = self.sbus_queue.get(block=False)
                status = SBusStatus()
                *status.channels, ch17, ch18, sig_loss, fail_safe = struct.unpack("<16hBBBB", sbus_data)
                status.channel_17 = ch17 != 0
                status.channel_18 = ch18 != 0
                status.signal_loss = sig_loss != 0
                status.fail_safe = fail_safe != 0
                data = []
                if status.signal_loss:
                    data = 16 * [0.5]
                    data[4] = 0
                    data[5] = 0
                    data[6] = 0
                    data[7] = 0
                else:
                    for i in status.channels:
                        data.append(2*(i - 192)/(1792 - 192) - 1)
                return data
            except queue.Empty:
                return None
        else:
            print('get_sbus enable reception first!')
            return None

    def buf_write(self, func, data):
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        self.port.write(buf)

    def set_led(self, on_time, off_time, repeat=1, led_id=1):
        on_time = int(on_time*1000)
        off_time = int(off_time*1000)
        self.buf_write(PacketFunction.PACKET_FUNC_LED, struct.pack("<BHHH", led_id, on_time, off_time, repeat))

    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        on_time = int(on_time*1000)
        off_time = int(off_time*1000)
        self.buf_write(PacketFunction.PACKET_FUNC_BUZZER, struct.pack("<HHHH", freq, on_time, off_time, repeat))

    def set_motor_speed(self, speeds):
        data = [0x01, len(speeds)]
        for i in speeds:
            data.extend(struct.pack("<Bf", int(i[0] - 1), float(i[1])))
        self.buf_write(PacketFunction.PACKET_FUNC_MOTOR, data)

    def set_oled_text(self, line, text):
        data = [line, len(text)] # 子命令为 0x01 设置 SSID, 第二个字节是字符串长度，该长度包含'\0'字符串结束符(The subcommand 0x01 sets SSID. The second byte is the length of string, which includes the terminator null '\0')
        data.extend(bytes(text, encoding='utf-8'))
        self.buf_write(PacketFunction.PACKET_FUNC_OLED, data)

    def pwm_servo_set_position(self, duration, positions):
        duration = int(duration * 1000)
        data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)]
        for i in positions:
            data.extend(struct.pack("<BH", i[0], i[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)
    
    def pwm_servo_set_offset(self, servo_id, offset):
        data = struct.pack("<BBb", 0x07, servo_id, int(offset))
        self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)

    def pwm_servo_read_and_unpack(self, servo_id, cmd, unpack):
        with self.servo_read_lock:
            self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [cmd, servo_id])
            data = self.pwm_servo_queue.get(block=True)
            servo_id, cmd, info = struct.unpack(unpack, data)
            return info

    def pwm_servo_read_offset(self, servo_id):
        return self.pwm_servo_read_and_unpack(servo_id, 0x09, "<BBb")

    def pwm_servo_read_position(self, servo_id):
        return self.pwm_servo_read_and_unpack(servo_id, 0x05, "<BBH")

    def bus_servo_enable_torque(self, servo_id, enable):
        if enable:
            data = struct.pack("<BB", 0x0B, servo_id)
        else:
            data = struct.pack("<BB", 0x0C, servo_id)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_id(self, servo_id_now, servo_id_new):
        data = struct.pack("<BBB", 0x10, servo_id_now, servo_id_new)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_offset(self, servo_id, offset):
        data = struct.pack("<BBb", 0x20, servo_id, int(offset))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_save_offset(self, servo_id):
        data = struct.pack("<BB", 0x24, servo_id)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_angle_limit(self, servo_id, limit):
        data = struct.pack("<BBHH", 0x30, servo_id, int(limit[0]), int(limit[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_vin_limit(self, servo_id, limit):
        data = struct.pack("<BBHH", 0x34, servo_id, int(limit[0]), int(limit[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_temp_limit(self, servo_id, limit):
        data = struct.pack("<BBb", 0x38, servo_id, int(limit))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_stop(self, servo_id):
        data = [0x03, len(servo_id)] 
        data.extend(struct.pack("<"+'B'*len(servo_id), *servo_id))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_set_position(self, duration, positions):
        duration = int(duration * 1000)
        data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)] # 0x00 is bus servo sub command
        for i in positions:
            data.extend(struct.pack("<BH", i[0], i[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
        with self.servo_read_lock:
            self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
            data = self.bus_servo_queue.get(block=True)
            servo_id, cmd, success, *info = struct.unpack(unpack, data)
            if success == 0:
                return info

    def bus_servo_read_id(self, servo_id=254):
        return self.bus_servo_read_and_unpack(servo_id, 0x12, "<BBbB")

    def bus_servo_read_offset(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x22, "<BBbb")
    
    def bus_servo_read_position(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x05, "<BBbh")

    def bus_servo_read_vin(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x07, "<BBbH")
    
    def bus_servo_read_temp(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x09, "<BBbB")

    def bus_servo_read_temp_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x3A, "<BBbB")

    def bus_servo_read_angle_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x32, "<BBb2H")

    def bus_servo_read_vin_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x36, "<BBb2H")

    def bus_servo_read_torque_state(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x0D, "<BBbb")

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def recv_task(self):
        while True:
            if self.enable_recv:
                recv_data = self.port.read()
                if recv_data:
                    for dat in recv_data:
                        # print("%0.2X "%dat)
                        if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                            if dat == 0xAA:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                            continue
                        elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                            if dat == 0x55:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION
                            else:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                            continue
                        elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION:
                            if dat < int(PacketFunction.PACKET_FUNC_NONE):
                                self.frame = [dat, 0]
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                            else:
                                self.frame = []
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                            continue
                        elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                            self.frame[1] = dat
                            self.recv_count = 0
                            if dat == 0:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                            else:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA
                            continue
                        elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                            self.frame.append(dat)
                            self.recv_count += 1
                            if self.recv_count >= self.frame[1]:
                                self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                            continue
                        elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                            crc8 = checksum_crc8(bytes(self.frame))
                            if crc8 == dat:
                                func = PacketFunction(self.frame[0])
                                data = bytes(self.frame[2:])
                                if func in self.parsers:
                                    self.parsers[func](data)
                            else:
                                print("校验失败")
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                            continue
            else:
                time.sleep(0.01)
        self.port.close()
        print("END...")

class KeyboardReader:
    """Thread-safe keyboard reader for non-blocking input."""
    
    def __init__(self):
        self.key_queue = queue.Queue()
        self.running = False
        self.thread = None
        self.old_settings = None
        
    def start(self):
        """Start the keyboard reading thread."""
        self.running = True
        self.thread = threading.Thread(target=self._read_keys, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Stop the keyboard reading thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=0.5)
    
    def _read_keys(self):
        """Background thread that reads keyboard input."""
        fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                # Use os.read with non-blocking mode
                try:
                    ch = os.read(fd, 1).decode('utf-8', errors='ignore')
                    if ch:
                        # Handle escape sequences (arrow keys, etc.)
                        if ch == '\x1b':
                            # Try to read next chars for arrow keys
                            try:
                                ch2 = os.read(fd, 1).decode('utf-8', errors='ignore')
                                if ch2 == '[':
                                    ch3 = os.read(fd, 1).decode('utf-8', errors='ignore')
                                    self.key_queue.put('\x1b[' + ch3)
                                else:
                                    self.key_queue.put(ch)
                            except:
                                self.key_queue.put(ch)
                        else:
                            self.key_queue.put(ch)
                except:
                    time.sleep(0.001)
        finally:
            if self.old_settings:
                termios.tcsetattr(fd, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """Get a key from the queue if available."""
        try:
            return self.key_queue.get_nowait()
        except queue.Empty:
            return None

def bus_servo_test(board):
    """Interactive bus servo control with keyboard."""
    print("=" * 60)
    print("INTERACTIVE BUS SERVO CONTROL")
    print("=" * 60)
    print("Controls:")
    print("  1-8     - Select servo ID")
    print("  ↑/W     - Increase position (+50)")
    print("  ↓/S     - Decrease position (-50)")
    print("  →/D     - Fine increase (+10)")
    print("  ←/A     - Fine decrease (-10)")
    print("  +/=     - Increase duration")
    print("  -/_     - Decrease duration")
    print("  SPACE   - Move to center (500)")
    print("  T       - Toggle torque on/off")
    print("  I       - Read servo info")
    print("  ESC     - Exit (or Ctrl+C)")
    print("=" * 60)
    print()
    
    selected_servo = 1
    target_position = 500
    duration = 1.0  # seconds
    torque_enabled = True
    
    def print_status():
        print(f"\rServo: {selected_servo} | Position: {target_position:4d} | Duration: {duration:.2f}s | Torque: {'ON' if torque_enabled else 'OFF'} ", end='', flush=True)
    
    def move_servo():
        board.bus_servo_set_position(duration, [[selected_servo, target_position]])
    
    def read_servo_info():
        print("\n" + "=" * 60)
        print(f"Reading info for servo {selected_servo}...")
        try:
            pos = board.bus_servo_read_position(selected_servo)
            print(f"  Current position: {pos}")
            vin = board.bus_servo_read_vin(selected_servo)
            print(f"  Voltage: {vin}")
            temp = board.bus_servo_read_temp(selected_servo)
            print(f"  Temperature: {temp}")
            torque = board.bus_servo_read_torque_state(selected_servo)
            print(f"  Torque state: {torque}")
            angle_limit = board.bus_servo_read_angle_limit(selected_servo)
            print(f"  Angle limits: {angle_limit}")
        except Exception as e:
            print(f"  Error reading servo: {e}")
        print("=" * 60)
    
    # Initial position
    move_servo()
    print_status()
    
    # Start keyboard reader
    kb = KeyboardReader()
    kb.start()
    
    try:
        while True:
            key = kb.get_key()
            
            if key:
                # Servo selection
                if key in '12345678':
                    selected_servo = int(key)
                    print_status()
                
                # Position control
                elif key in ['w', 'W', '\x1b[A']:  # Up - large step
                    target_position = min(1000, target_position + 50)
                    move_servo()
                    print_status()
                elif key in ['s', 'S', '\x1b[B']:  # Down - large step
                    target_position = max(0, target_position - 50)
                    move_servo()
                    print_status()
                elif key in ['d', 'D', '\x1b[C']:  # Right - fine step
                    target_position = min(1000, target_position + 10)
                    move_servo()
                    print_status()
                elif key in ['a', 'A', '\x1b[D']:  # Left - fine step
                    target_position = max(0, target_position - 10)
                    move_servo()
                    print_status()
                elif key == ' ':  # Center
                    target_position = 500
                    move_servo()
                    print_status()
                
                # Duration control
                elif key in ['+', '=']:
                    duration = min(5.0, duration + 0.1)
                    print_status()
                elif key in ['-', '_']:
                    duration = max(0.1, duration - 0.1)
                    print_status()
                
                # Torque toggle
                elif key in ['t', 'T']:
                    torque_enabled = not torque_enabled
                    board.bus_servo_enable_torque(selected_servo, torque_enabled)
                    print_status()
                
                # Read info
                elif key in ['i', 'I']:
                    read_servo_info()
                    print_status()
                
                # Exit
                elif key in ['\x1b', '\x03']:  # ESC or Ctrl+C
                    break
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        print("\n\nBus servo test complete!")

def pwm_servo_test(board):
    """Interactive PWM servo control with keyboard."""
    print("=" * 60)
    print("INTERACTIVE PWM SERVO CONTROL")
    print("=" * 60)
    print("Controls:")
    print("  1-4     - Select servo ID (1-4)")
    print("  ↑/W     - Increase position (+100μs)")
    print("  ↓/S     - Decrease position (-100μs)")
    print("  →/D     - Fine increase (+25μs)")
    print("  ←/A     - Fine decrease (-25μs)")
    print("  +/=     - Increase duration")
    print("  -/_     - Decrease duration")
    print("  SPACE   - Move to center (1500μs)")
    print("  0       - Set offset to 0")
    print("  [/]     - Adjust offset")
    print("  I       - Read servo info")
    print("  ESC     - Exit (or Ctrl+C)")
    print("=" * 60)
    print("Note: PWM position range is typically 500-2500μs")
    print("=" * 60)
    print()
    
    selected_servo = 1
    target_position = 1500  # microseconds
    duration = 0.5  # seconds
    current_offset = 0
    
    def print_status():
        print(f"\rServo: {selected_servo} | Position: {target_position:4d}μs | Duration: {duration:.2f}s | Offset: {current_offset:3d} ", end='', flush=True)
    
    def move_servo():
        board.pwm_servo_set_position(duration, [[selected_servo, target_position]])
    
    def read_servo_info():
        print("\n" + "=" * 60)
        print(f"Reading info for PWM servo {selected_servo}...")
        try:
            offset = board.pwm_servo_read_offset(selected_servo)
            print(f"  Offset: {offset}")
            position = board.pwm_servo_read_position(selected_servo)
            print(f"  Position: {position}")
        except Exception as e:
            print(f"  Error reading servo: {e}")
        print("=" * 60)
    
    # Initial position
    move_servo()
    print_status()
    
    # Start keyboard reader
    kb = KeyboardReader()
    kb.start()
    
    try:
        while True:
            key = kb.get_key()
            
            if key:
                # Servo selection
                if key in '1234':
                    selected_servo = int(key)
                    print_status()
                
                # Position control
                elif key in ['w', 'W', '\x1b[A']:  # Up - large step
                    target_position = min(2500, target_position + 100)
                    move_servo()
                    print_status()
                elif key in ['s', 'S', '\x1b[B']:  # Down - large step
                    target_position = max(500, target_position - 100)
                    move_servo()
                    print_status()
                elif key in ['d', 'D', '\x1b[C']:  # Right - fine step
                    target_position = min(2500, target_position + 25)
                    move_servo()
                    print_status()
                elif key in ['a', 'A', '\x1b[D']:  # Left - fine step
                    target_position = max(500, target_position - 25)
                    move_servo()
                    print_status()
                elif key == ' ':  # Center
                    target_position = 1500
                    move_servo()
                    print_status()
                
                # Duration control
                elif key in ['+', '=']:
                    duration = min(5.0, duration + 0.1)
                    print_status()
                elif key in ['-', '_']:
                    duration = max(0.1, duration - 0.1)
                    print_status()
                
                # Offset control
                elif key == '[':
                    current_offset = max(-127, current_offset - 5)
                    board.pwm_servo_set_offset(selected_servo, current_offset)
                    print_status()
                elif key == ']':
                    current_offset = min(127, current_offset + 5)
                    board.pwm_servo_set_offset(selected_servo, current_offset)
                    print_status()
                elif key == '0':
                    current_offset = 0
                    board.pwm_servo_set_offset(selected_servo, current_offset)
                    print_status()
                
                # Read info
                elif key in ['i', 'I']:
                    read_servo_info()
                    print_status()
                
                # Exit
                elif key in ['\x1b', '\x03']:  # ESC or Ctrl+C
                    break
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        print("\n\nPWM servo test complete!")

def test_imu(board, interval=0.01):
    """Test IMU sensor and continuously print readings."""
    print("Testing IMU... (Press Ctrl+C to stop)")
    print("Format: ax, ay, az, gx, gy, gz")
    while True:
        try:
            res = board.get_imu()
            if res is not None:
                print("IMU: ", end='')
                for item in res:
                    print("{: .8f} ".format(item), end='')
                print()
            time.sleep(interval)
        except KeyboardInterrupt:
            break

def test_button(board, interval=0.01):
    """Test button inputs."""
    print("Testing buttons... (Press Ctrl+C to stop)")
    print("Format: (button_id, state) - state: 0=click, 1=pressed")
    while True:
        try:
            res = board.get_button()
            if res is not None:
                print(f"Button: ID={res[0]}, State={res[1]}")
            time.sleep(interval)
        except KeyboardInterrupt:
            break

def test_battery(board, interval=1.0):
    """Test battery voltage reading."""
    print("Testing battery... (Press Ctrl+C to stop)")
    print("Format: voltage in mV")
    while True:
        try:
            res = board.get_battery()
            if res is not None:
                print(f"Battery: {res} mV ({res/1000:.2f} V)")
            time.sleep(interval)
        except KeyboardInterrupt:
            break

def test_gamepad(board, interval=0.01):
    """Test gamepad inputs."""
    print("Testing gamepad... (Press Ctrl+C to stop)")
    print("Format: (axes, buttons)")
    while True:
        try:
            data = board.get_gamepad()
            if data is not None:
                print(f"Axes: {[f'{x:.2f}' for x in data[0]]}")
                print(f"Buttons: {data[1]}")
                print("---")
            time.sleep(interval)
        except KeyboardInterrupt:
            break

def test_sbus(board, interval=0.01):
    """Test SBUS receiver."""
    print("Testing SBUS... (Press Ctrl+C to stop)")
    while True:
        try:
            res = board.get_sbus()
            if res is not None:
                print(f"SBUS: {[f'{x:.2f}' for x in res]}")
            time.sleep(interval)
        except KeyboardInterrupt:
            break

def test_led(board, on_time=0.1, off_time=0.9, repeat=1, led_id=1):
    """Test LED control."""
    print(f"Testing LED {led_id}: on={on_time}s, off={off_time}s, repeat={repeat}")
    board.set_led(on_time, off_time, repeat, led_id)
    print("LED command sent!")

def test_buzzer(board, freq=1900, on_time=0.3, off_time=0.01, repeat=1):
    """Test buzzer control."""
    print(f"Testing buzzer: freq={freq}Hz, on={on_time}s, off={off_time}s, repeat={repeat}")
    board.set_buzzer(freq, on_time, off_time, repeat)
    print("Buzzer command sent!")

def test_motor(board, duration=None, speed=0.6):
    """Interactive motor control with keyboard - Ackerman drive (car-like steering)."""
    print("=" * 60)
    print("INTERACTIVE MOTOR CONTROL (Ackerman Drive)")
    print("=" * 60)
    print("Controls:")
    print("  W/↑     - Increase forward speed")
    print("  S/↓     - Increase backward speed")
    print("  A/←     - Steer left")
    print("  D/→     - Steer right")
    print("  SPACE   - Stop / Reset to center")
    print("  +/=     - Increase max speed limit")
    print("  -/_     - Decrease max speed limit")
    print("  ESC     - Exit (or Ctrl+C)")
    print("=" * 60)
    print()
    
    max_speed = speed
    throttle = 0.0  # Current forward/backward speed (-1.0 to 1.0)
    steering = 0.0  # Current steering (-1.0 left to 1.0 right)
    
    throttle_step = 0.05
    steering_step = 0.1
    
    def update_motors():
        # Ackerman drive: combine throttle and steering
        # Assuming motors 1,2 = left side, motors 3,4 = right side
        # For simple car: reduce speed on inside of turn
        # Note: negating throttle because motors are wired in reverse
        left_speed = -throttle - (steering * abs(throttle))
        right_speed = -throttle + (steering * abs(throttle))
        
        # Clamp speeds
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))
        
        board.set_motor_speed([
            [1, left_speed],
            [2, left_speed],
            [3, right_speed],
            [4, right_speed]
        ])
    
    def print_status():
        direction = "FWD" if throttle > 0 else "REV" if throttle < 0 else "STOP"
        steer_dir = "LEFT" if steering < 0 else "RIGHT" if steering > 0 else "CENTER"
        print(f"\r{direction} Speed: {abs(throttle):.2f} | Steering: {steer_dir:6s} {abs(steering):.2f} | Max: {max_speed:.2f}   ", end='', flush=True)
    
    print_status()
    
    # Start keyboard reader
    kb = KeyboardReader()
    kb.start()
    
    try:
        start_time = time.time()
        while True:
            # Check duration limit if specified
            if duration is not None and (time.time() - start_time) >= duration:
                break
            
            key = kb.get_key()
            
            if key:
                # Throttle controls
                if key in ['w', 'W', '\x1b[A']:  # Increase forward
                    throttle = min(max_speed, throttle + throttle_step)
                    update_motors()
                    print_status()
                elif key in ['s', 'S', '\x1b[B']:  # Increase backward
                    throttle = max(-max_speed, throttle - throttle_step)
                    update_motors()
                    print_status()
                
                # Steering controls
                elif key in ['a', 'A', '\x1b[D']:  # Steer left
                    steering = max(-1.0, steering - steering_step)
                    update_motors()
                    print_status()
                elif key in ['d', 'D', '\x1b[C']:  # Steer right
                    steering = min(1.0, steering + steering_step)
                    update_motors()
                    print_status()
                
                # Stop
                elif key == ' ':  # Stop and center
                    throttle = 0.0
                    steering = 0.0
                    update_motors()
                    print_status()
                
                # Speed limit adjustment
                elif key in ['+', '=']:  # Increase max speed
                    max_speed = min(1.0, max_speed + 0.05)
                    print_status()
                elif key in ['-', '_']:  # Decrease max speed
                    max_speed = max(0.1, max_speed - 0.05)
                    # Also clamp current throttle
                    throttle = max(-max_speed, min(max_speed, throttle))
                    update_motors()
                    print_status()
                
                # Exit
                elif key in ['\x1b', '\x03']:  # ESC or Ctrl+C
                    break
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        # Stop all motors
        board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        print("\n\nMotors stopped!")

def test_oled(board, line, text):
    """Test OLED display."""
    print(f"Setting OLED line {line} to: {text}")
    board.set_oled_text(line, text)
    print("OLED command sent!")

def main():
    parser = argparse.ArgumentParser(
        description='ROS Robot Controller SDK - Test and control STM32 board',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --port /dev/ttyUSB0 --imu              # Test IMU sensor
  %(prog)s --port /dev/rrc --battery              # Test battery reading
  %(prog)s --port /dev/rrc --gamepad --interval 0.05  # Test gamepad at 50ms interval
  %(prog)s --port /dev/rrc --led --led-on 0.5 --led-off 0.5  # Blink LED
  %(prog)s --port /dev/rrc --motor --motor-speed 0.3 --motor-duration 2  # Test motors
        """
    )
    
    # Connection parameters
    parser.add_argument('--port', '-p', type=str, default='/dev/rrc',
                        help='Serial port device (default: /dev/rrc)')
    parser.add_argument('--baudrate', '-b', type=int, default=1000000,
                        help='Serial baudrate (default: 1000000)')
    parser.add_argument('--timeout', '-t', type=float, default=5,
                        help='Serial timeout in seconds (default: 5)')
    
    # Test modes
    test_group = parser.add_argument_group('test modes')
    test_group.add_argument('--imu', action='store_true',
                            help='Test IMU sensor (accelerometer and gyroscope)')
    test_group.add_argument('--button', action='store_true',
                            help='Test button inputs')
    test_group.add_argument('--battery', action='store_true',
                            help='Test battery voltage reading')
    test_group.add_argument('--gamepad', action='store_true',
                            help='Test gamepad controller')
    test_group.add_argument('--sbus', action='store_true',
                            help='Test SBUS receiver')
    test_group.add_argument('--led', action='store_true',
                            help='Test LED control')
    test_group.add_argument('--buzzer', action='store_true',
                            help='Test buzzer')
    test_group.add_argument('--motor', action='store_true',
                            help='Test motor control')
    test_group.add_argument('--bus-servo', action='store_true',
                            help='Test bus servo control')
    test_group.add_argument('--pwm-servo', action='store_true',
                            help='Test PWM servo control')
    test_group.add_argument('--oled', action='store_true',
                            help='Test OLED display')
    
    # General parameters
    parser.add_argument('--interval', type=float, default=0.01,
                        help='Polling interval for sensor tests in seconds (default: 0.01)')
    
    # LED parameters
    led_group = parser.add_argument_group('LED parameters')
    led_group.add_argument('--led-id', type=int, default=1,
                          help='LED ID (default: 1)')
    led_group.add_argument('--led-on', type=float, default=0.1,
                          help='LED on time in seconds (default: 0.1)')
    led_group.add_argument('--led-off', type=float, default=0.9,
                          help='LED off time in seconds (default: 0.9)')
    led_group.add_argument('--led-repeat', type=int, default=1,
                          help='LED repeat count (default: 1)')
    
    # Buzzer parameters
    buzzer_group = parser.add_argument_group('buzzer parameters')
    buzzer_group.add_argument('--buzzer-freq', type=int, default=1900,
                             help='Buzzer frequency in Hz (default: 1900)')
    buzzer_group.add_argument('--buzzer-on', type=float, default=0.3,
                             help='Buzzer on time in seconds (default: 0.3)')
    buzzer_group.add_argument('--buzzer-off', type=float, default=0.01,
                             help='Buzzer off time in seconds (default: 0.01)')
    buzzer_group.add_argument('--buzzer-repeat', type=int, default=1,
                             help='Buzzer repeat count (default: 1)')
    
    # Motor parameters
    motor_group = parser.add_argument_group('motor parameters')
    motor_group.add_argument('--motor-speed', type=float, default=0.6,
                            help='Initial motor speed (default: 0.6)')
    motor_group.add_argument('--motor-duration', type=float, default=None,
                            help='Motor test duration in seconds (default: None = interactive mode, runs until ESC/Ctrl+C)')
    
    # OLED parameters
    oled_group = parser.add_argument_group('OLED parameters')
    oled_group.add_argument('--oled-line', type=int, default=0,
                           help='OLED line number (default: 0)')
    oled_group.add_argument('--oled-text', type=str, default='Hello World',
                           help='OLED text to display (default: "Hello World")')
    
    args = parser.parse_args()
    
    # Check if at least one test mode is specified
    test_modes = [args.imu, args.button, args.battery, args.gamepad, args.sbus,
                  args.led, args.buzzer, args.motor, args.bus_servo, args.pwm_servo, args.oled]
    
    if not any(test_modes):
        parser.print_help()
        print("\n❌ Error: Please specify at least one test mode (e.g., --imu, --battery, etc.)")
        sys.exit(1)
    
    # Initialize board
    print(f"Connecting to board on {args.port} at {args.baudrate} baud...")
    try:
        board = Board(args.port, baudrate=args.baudrate, timeout=args.timeout)
        board.enable_reception()
        time.sleep(0.1)
        print("✓ Connected successfully!")
        print()
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        sys.exit(1)
    
    # Run selected tests
    try:
        if args.imu:
            test_imu(board, args.interval)
        
        if args.button:
            test_button(board, args.interval)
        
        if args.battery:
            test_battery(board, args.interval)
        
        if args.gamepad:
            test_gamepad(board, args.interval)
        
        if args.sbus:
            test_sbus(board, args.interval)
        
        if args.led:
            test_led(board, args.led_on, args.led_off, args.led_repeat, args.led_id)
        
        if args.buzzer:
            test_buzzer(board, args.buzzer_freq, args.buzzer_on, 
                       args.buzzer_off, args.buzzer_repeat)
        
        if args.motor:
            test_motor(board, args.motor_duration, args.motor_speed)
        
        if args.bus_servo:
            bus_servo_test(board)
        
        if args.pwm_servo:
            pwm_servo_test(board)
        
        if args.oled:
            test_oled(board, args.oled_line, args.oled_text)
        
    except KeyboardInterrupt:
        print("\n\n✓ Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        raise
    finally:
        print("\nTest complete!")

if __name__ == "__main__":
    main()
