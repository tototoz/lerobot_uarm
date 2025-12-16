import serial
import time

# 通过串口实时读取数据并显示
def read_serial_data(port='/dev/ttyACM0', baudrate=115200, timeout=0.12):
    # 打开串口
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        print("开始读取数据...")
        
        while True:
            # 读取最多100个字节的数据
            response = ser.read(100)
            
            if response:
                # 将读取到的数据转化为十六进制并输出
                print(f"接收到的数据: {response.hex()}")
            else:
                print("没有接收到数据...等待中...")
            
            time.sleep(0.5)  # 每次读取后暂停 0.5 秒

# 主函数
def main():
    time.sleep(2)  # 等待设备初始化（如果需要）
    read_serial_data()  # 调用实时读取串口上的数据

# 判断是否为主程序
if __name__ == "__main__":
    main()  # 运行主函数
