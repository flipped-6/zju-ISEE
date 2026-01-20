import serial
import csv
import time
import keyboard
import matplotlib.pyplot as plt
import numpy as np

# 配置串口参数
port = 'COM4'  # 根据实际情况修改串口号
baudrate = 115200  # 根据实际情况修改波特率

# 打开串口
ser = serial.Serial(port, baudrate)

# 创建CSV文件并写入表头
csv_file = open('serial_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Angle', 'Resistance'])

angle_resistance_dict = {}


try:
    while True:
        # 读取串口数据
        data = ser.readline().decode().strip()
        if data:
            # 解析数据
            try:
                angle, resistance = data.split(',')
                angle = int(angle)
                resistance = float(resistance)
                print(f"Received data: Angle={angle}, Resistance={resistance}")
                # 将解析后的数据写入CSV文件
                csv_writer.writerow([angle, resistance])
                if angle not in angle_resistance_dict:
                    angle_resistance_dict[angle] = []
                angle_resistance_dict[angle].append(resistance)
            except ValueError:
                print(f"Invalid data: {data}")

        # 检测回车键
        if keyboard.is_pressed('enter'):
            print("Stopped by user")
            break
except KeyboardInterrupt:
    print("Stopped by user")
finally:
    # 关闭串口和CSV文件
    ser.close()
    csv_file.close()

    # 绘制折线图
    angles = np.linspace(9, 180-9, 19, dtype=int)
    resistances = [np.mean(angle_resistance_dict.get(angle, [0])) / 1000 for angle in angles]  # 转换为kΩ

    plt.plot(angles, resistances, marker='o')
    plt.xlabel('Angle')
    plt.ylabel('Resistance (kΩ)')
    plt.title('Resistance vs Angle')
    plt.xticks(angles)
    plt.ylim(0, 15)

    # 显示每个点的阻值
    for angle, resistance in zip(angles, resistances):
        plt.text(angle, resistance, f'{resistance:.2f} kΩ', ha='center', va='bottom')

    plt.show()
