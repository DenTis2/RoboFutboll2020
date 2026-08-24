#!/usr/bin/env python3
import math
import serial
import time
import queue
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port=port, 
            baudrate=baud, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS, 
            timeout=0.1  # увеличил таймаут для стабильности чтения
        )
        # Буфер для 360 точек: каждая точка представлена [расстояние, качество]
        self.lidarData = [[0, 0] for _ in range(360)]
        self.q = queue.Queue()
        self.running = True  # флаг для остановки потока

    def checksum(self, data):
        # data: список из 20 байт (без контрольной суммы)
        data_list = []
        for t in range(10):
            data_list.append(data[2*t] + (data[2*t+1] << 8))
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d
        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        checksum = checksum & 0x7FFF
        return int(checksum)

    def compute_speed(self, data):
        # Не используется в дальнейшей логике, но можно сохранить значение скорости
        speed_rpm = float(data[0] | (data[1] << 8)) / 64.0
        return speed_rpm

    def update_view(self, angle, data):
        # data: 4 байта, первые 10 бит задают дистанцию, остальные 2 байта – качество
        if len(data) < 4:
            return
        dist_mm = data[0] | ((data[1] & 0x3F) << 8)
        quality = data[2] | (data[3] << 8)
        # Границы проверки: если индекс выходит за диапазон, игнорируем
        if 0 <= angle < 360:
            self.lidarData[angle] = [dist_mm, quality]
        # Если достигли конца оборота, помещаем копию данных в очередь
        if angle >= 359:
            self.q.put(self.lidarData.copy())

    def start_scan(self):
        """
        Читает по 22 байта за раз (1 байт: заголовок, 1 байт: индекс, 2 байта: скорость,
        16 байт: данные по 4 блокам по 4 байта, 2 байта: контрольная сумма)
        """
        while self.running:
            try:
                # Синхронизируемся на заголовке 0xFA
                header = self.ser.read(1)
                if not header or header[0] != 0xFA:
                    continue

                second = self.ser.read(1)
                if not second:
                    continue
                second_val = second[0]
                # Проверяем, что индекс в диапазоне 0xA0-0xF9
                if not (0xA0 <= second_val <= 0xF9):
                    continue
                index = second_val - 0xA0  # индекс блока от 0 до 89

                # Читаем оставшиеся 20 байт
                packet = self.ser.read(20)
                if len(packet) < 20:
                    continue

                # Первые 18 байт от пакета – полезные данные
                data_payload = list(header) + list(second) + list(packet[:18])
                # Последние 2 байта – контрольная сумма
                chk_bytes = packet[18:20]
                incoming_checksum = chk_bytes[0] + (chk_bytes[1] << 8)

                if self.checksum(data_payload) == incoming_checksum:
                    b_speed = list(packet[0:2])
                    # Разбиваем данные на 4 блока по 4 байта
                    b_data0 = list(packet[2:6])
                    b_data1 = list(packet[6:10])
                    b_data2 = list(packet[10:14])
                    b_data3 = list(packet[14:18])
                    # Можно вычислить скорость, если нужно:
                    _ = self.compute_speed(b_speed)
                    # Обновляем данные для 4 углов (каждый блок соответствует 1/4 оборота)
                    self.update_view(index * 4 + 0, b_data0)
                    self.update_view(index * 4 + 1, b_data1)
                    self.update_view(index * 4 + 2, b_data2)
                    self.update_view(index * 4 + 3, b_data3)
                else:
                    # При ошибке контрольной суммы заполняем данные нулями
                    for i in range(4):
                        self.update_view(index * 4 + i, [0, 0x80, 0, 0])
                time.sleep(0.001)  # небольшая задержка для освобождения ЦП
            except Exception as e:
                print(f"Exception in scan thread: {str(e)}")
        # При завершении работы закрываем последовательный порт
        self.ser.close()

    def stop(self):
        self.running = False

class NeatoLidarNode(Node):
    def __init__(self):
        super().__init__('neato_lidar_node')
        port = '/dev/ttyUSB1'
        baud = 115200
        self.lidar = Lidar(port, baud)
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Запуск потока сканирования (daemon-поток позволит завершиться процессу корректно)
        self.scan_thread = threading.Thread(target=self.lidar.start_scan, daemon=True)
        self.scan_thread.start()
        self.get_logger().info('Neato Lidar node started.')

    def timer_callback(self):
        # Обрабатываем все накопленные данные из очереди
        while not self.lidar.q.empty():
            scan_data = self.lidar.q.get()
            if scan_data is None:
                continue
            scan_msg = LaserScan()
            now = self.get_clock().now().to_msg()
            scan_msg.header.stamp = now
            scan_msg.header.frame_id = 'laser_frame'
            # Заполняем угловые параметры
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2 * math.pi
            scan_msg.angle_increment = math.radians(1)  # 1 градус в радианах
            scan_msg.scan_time = 0.1
            num_readings = 360
            scan_msg.time_increment = scan_msg.scan_time / num_readings
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0

            ranges = [float('inf')] * num_readings
            intensities = [0.0] * num_readings
            for angle in range(360):
                dist_mm, quality = scan_data[angle]
                if dist_mm > 0:
                    ranges[angle] = dist_mm / 1000.0  # перевод мм в метры
                    intensities[angle] = float(quality)
                else:
                    ranges[angle] = float('inf')
                    intensities[angle] = 0.0

            scan_msg.ranges = ranges
            scan_msg.intensities = intensities

            self.publisher_.publish(scan_msg)
            self.get_logger().info('Published LaserScan message.')
            time.sleep(0.1)

    def destroy_node(self):
        # Останавливаем сканирование перед разрушением узла
        self.lidar.stop()
        if self.scan_thread.is_alive():
            self.scan_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NeatoLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
