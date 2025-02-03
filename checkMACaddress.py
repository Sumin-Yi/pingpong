import serial.tools.list_ports

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    connected_ports = [port.device for port in ports]
    
    if connected_ports:
        print("Connected serial ports:")
        for port in connected_ports:
            print(f"- {port}")
    else:
        print("No serial ports found.")

if __name__ == "__main__":
    list_serial_ports()