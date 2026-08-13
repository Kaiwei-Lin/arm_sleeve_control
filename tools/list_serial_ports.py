from serial.tools import list_ports


def list_serial_ports() -> None:
    ports = list_ports.comports()

    if not ports:
        print("未检测到可用串口")
        return

    print(f"检测到 {len(ports)} 个串口：")

    for port in ports:
        print("-" * 50)
        print(f"设备名：{port.device}")
        print(f"描述：{port.description}")
        print(f"硬件 ID：{port.hwid}")
        print(f"厂商 ID：{port.vid}")
        print(f"产品 ID：{port.pid}")
        print(f"序列号：{port.serial_number}")
        print(f"制造商：{port.manufacturer}")
        print(f"产品名称：{port.product}")
        print(f"接口：{port.interface}")


if __name__ == "__main__":
    list_serial_ports()