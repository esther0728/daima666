from pyb import UART
import struct
class My_uart:
    def __init__(self,index,baud_rate,bits_s=8,parity_s=None,stop_s=1):
        self.uart = UART(index,baud_rate)
        self.uart.init(baud_rate, bits=bits_s, parity=bits_s, stop=stop_s)
    def send_floats(self, flts):
        buffer = bytearray([0xaa, 0])  # 初始化缓冲区，第一位是0xaa，第二位暂存长度
        # 打包所有浮点数
        for flt in flts:
            float_bytes = struct.pack('>f', flt)  # 将浮点数转换为4字节大端序
            buffer.extend(float_bytes)  # 添加到缓冲区
        # 设置长度字段（总字节数，包括0xaa、长度本身和所有数据）
        buffer[1] = len(buffer) + 1  # +1是为了包含即将添加的校验和字节
        # 计算校验和（所有字节的累加和）
        checksum = sum(buffer) & 0xFF  # 确保校验和是8位
        buffer.append(checksum)  # 添加校验和到末尾
        self.uart.write(buffer)  # 发送完整的缓冲区

