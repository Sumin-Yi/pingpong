import asyncio
import serial
from collections import deque
from bleak import BleakClient
import time

# BLE 설정
# BLE_DEVICE_ADDRESS = "35:7C:CE:EE:8A:40"  # Ring 디바이스의 MAC 주소
BLE_DEVICE_ADDRESS = "E5F088FF-EDFB-9C73-68E9-83CB9159FA51"  # Ring 디바이스의 MAC 주소
TX_CHARACTERISTIC_UUID = "2101"  # 서버에서 클라이언트로 전송 (FINGER_TAP_DETECTED)

# 시리얼 통신 설정
# SERIAL_PORTS = ["COM8", "COM9"]
SERIAL_PORTS = ["/dev/cu.usbserial-1410", "/dev/cu.usbserial-1420", "/dev/cu.usbserial-1430"]
BAUD_RATE = 115200

# 전역 변수
command_queue = deque()
last_selected_device = None  # 현재 선택된 디바이스
selection_interval = 0.9  # seconds between device selection
last_selected_time = 0


class SerialManager:
    def __init__(self, ports, baud_rate):
        self.serial_conns = [serial.Serial(port, baud_rate, timeout=1) for port in ports]

    def process_serial(self):
        global command_queue
        for conn in self.serial_conns:
            if conn.in_waiting:
                serial_data = conn.readline().decode("utf-8").strip()
                if serial_data == "DETECTED":
                    print(f"Detected signal received from device on {conn.port}")
                    command_queue.append(conn)  # 큐에 디바이스 연결 객체 추가
                else:
                    print(f"Debug message from device on {conn.port}: {serial_data}")

    def send_to_device(self, device, message):
        try:
            device.write(f"{message}\n".encode("utf-8"))
            print(f"Sent '{message}' to device on {device.port}")
        except Exception as e:
            print(f"Failed to send message to device: {e}")

    def close_connections(self):
        for conn in self.serial_conns:
            conn.close()


class BLEClient:
    def __init__(self, device_address):
        self.device_address = device_address
        self.client = BleakClient(device_address)

    async def connect(self):
        try:
            await self.client.connect()
            print(f"Connected to BLE device at {self.device_address}")
            
        except Exception as e:
            print(f"Failed to connect to BLE device: {e}")

    # async def write(self):
    #     try:
    #         await self.client.write_gatt_char(TX_CHARACTERISTIC_UUID, b'PAIRED')
    #         print("Shoot!")
    #     except Exception as e:
    #         print(f"Hing : {e}")
        

    async def listen_notifications(self):
        async def notification_handler(characteristic, data):
            global last_selected_devices
            message = data.decode("utf-8")
            print(f"Received notification: {message}")
            if message == "FINGER_TAP_DETECTED":
                if last_selected_device:
                    serial_manager.send_to_device(last_selected_device, "CONFIRMED")
                    print(f"CONFIRMED sent to device on {last_selected_device.port}")
                else:
                    print("No device currently selected for CONFIRMED")

        try:
            await self.client.start_notify(TX_CHARACTERISTIC_UUID, notification_handler)
            print(f"Listening for notifications on {TX_CHARACTERISTIC_UUID}...")
        except Exception as e:
            print(f"Failed to start notifications: {e}")

    async def stop_notifications(self):
        try:
            await self.client.stop_notify(TX_CHARACTERISTIC_UUID)
            print(f"Stopped notifications on {TX_CHARACTERISTIC_UUID}.")
        except Exception as e:
            print(f"Failed to stop notifications: {e}")

    async def disconnect(self):
        await self.client.disconnect()
        print("Disconnected from BLE device.")


async def main():
    global command_queue, last_selected_time, last_selected_device
    print("Initializing serial connections...")
    try:
        global serial_manager
        serial_manager = SerialManager(SERIAL_PORTS, BAUD_RATE)
        print("Serial connections established.")
    except serial.SerialException as e:
        print(f"Failed to connect to serial ports: {e}")
        return

    # BLE 클라이언트 생성 및 연결
    ble_client = BLEClient(BLE_DEVICE_ADDRESS)
    await ble_client.connect()

    # await ble_client.write()

    # BLE 알림 수신 시작
    await ble_client.listen_notifications()

    try:
        while True:
            # 시리얼 데이터 처리
            serial_manager.process_serial()

            # 큐에서 디바이스 선택
            current_time = time.time()
            if command_queue and (last_selected_device is None or current_time - last_selected_time >= selection_interval):
                last_selected_device = command_queue.popleft()  # 큐에서 디바이스 선택
                last_selected_time = current_time  # 마지막 선택 시간 업데이트
                print(f"Device selected: {last_selected_device.port}")

                try:
                    last_selected_device.write(b"SELECTED\n")  # SELECTED 명령 전송
                    print(f"Sent 'SELECTED' to device on {last_selected_device.port}")
                except Exception as e:
                    print(f"Failed to communicate with device {last_selected_device.port}: {e}")

            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        await ble_client.stop_notifications()
        await ble_client.disconnect()
        serial_manager.close_connections()


if __name__ == "__main__":
    asyncio.run(main())
