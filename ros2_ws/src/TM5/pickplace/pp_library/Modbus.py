from pymodbus.client import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import rclpy
import time


class ModbusClass:
    def __init__(self, ip = '192.168.44.14'):
        self.host = ip
        self.port_modbus = 502
        self.client = ModbusTcpClient(self.host, port=self.port_modbus)
        self.client.connect()
        
    # Start the TM project via Modbus
    def start_program(self):
        print("Starting project...")
        status = self.client.write_coil(7104, True)
        time.sleep(3)

    # Stop the TM project via Modbus
    def stop_program(self):
        print("Stopping project...")
        status = self.client.write_coil(7105, True, unit=1)
        time.sleep(3)

    # Decode the object obtained from reading the Modbus input registers
    def decode(self, input):
        pos_x = BinaryPayloadDecoder.fromRegisters(input.registers[0:2], Endian.BIG).decode_32bit_float()
        pos_y = BinaryPayloadDecoder.fromRegisters(input.registers[2:4], Endian.BIG).decode_32bit_float()
        pos_z = BinaryPayloadDecoder.fromRegisters(input.registers[4:6], Endian.BIG).decode_32bit_float()
        rot_x = BinaryPayloadDecoder.fromRegisters(input.registers[6:8], Endian.BIG).decode_32bit_float()
        rot_y = BinaryPayloadDecoder.fromRegisters(input.registers[8:10], Endian.BIG).decode_32bit_float()
        rot_z = BinaryPayloadDecoder.fromRegisters(input.registers[10:12], Endian.BIG).decode_32bit_float()

        return [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

    # Get the 12 register addresses (8300 to 8311) which corresponds to the x, y, z, Rx, Ry and Rz 
    # of the CURRENT base (Refer to Modbus Data Table)
    def get_base(self):
        curBaseReg = self.client.read_input_registers(8300, count=12)
        return self.decode(curBaseReg)

    # Returns cartesian coordinates and rotation w.r.t Robot base WITH tool
    def get_pos(self):
        curPosReg = self.client.read_input_registers(7049, count=12)
        return self.decode(curPosReg)
    
    # Read the power draw of the manipulator
    def get_power_draw(self):
        try:
            # Read registers 7344~7345 (1CB0~1CB1 in hex)
            power_registers = self.client.read_input_registers(7344, count=2)
            if not power_registers.isError():
                # Decode the 32-bit float value
                decoder = BinaryPayloadDecoder.fromRegisters(power_registers.registers, Endian.BIG)
                power_draw = decoder.decode_32bit_float()
                return power_draw
            else:
                print("Error reading power draw registers")
                return None
        except Exception as e:
            print(f"Exception while reading power draw: {e}")
            return None

    # Open IO using modbus
    def open_io(self):
        self.client.write_coil(800, 1)
        time.sleep(0.2)
        while (not self.io_status()):
            time.sleep(0.1)

    # Close IO using modbus
    def close_io(self):
        self.client.write_coil(800, 0)
        time.sleep(0.2)
        while (not self.io_status()):
            time.sleep(0.1)

    # Initialise the gripper via IO coupling
    def init_io(self):
        self.client.write_coil(800, 0)
        self.client.write_coil(801, 0)
        time.sleep(0.2)
        self.client.write_coil(800, 1)
        time.sleep(0.2)
        self.client.write_coil(800, 0)
        time.sleep(0.2)
        self.client.write_coil(801, 1)
        time.sleep(0.2)
        self.client.write_coil(801, 0)


    # Get IO status using modbus
    def io_status(self):
        stateReg = self.client.read_discrete_inputs(801, 1)
        #print(stateReg.bits[0])
        return stateReg.bits[0]

    


"""
def decode(input):
        pos_x = BinaryPayloadDecoder.fromRegisters(input.registers[0:2], Endian.Big).decode_32bit_float()
        pos_y = BinaryPayloadDecoder.fromRegisters(input.registers[2:4], Endian.Big).decode_32bit_float()
        pos_z = BinaryPayloadDecoder.fromRegisters(input.registers[4:6], Endian.Big).decode_32bit_float()
        rot_x = BinaryPayloadDecoder.fromRegisters(input.registers[6:8], Endian.Big).decode_32bit_float()
        rot_y = BinaryPayloadDecoder.fromRegisters(input.registers[8:10], Endian.Big).decode_32bit_float()
        rot_z = BinaryPayloadDecoder.fromRegisters(input.registers[10:12], Endian.Big).decode_32bit_float()
        return [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]


if __name__ == '__main__':
    #initialise ROS
    rclpy.init(args=sys.argv)

    #initialise the Modbus client
    host = '192.168.2.10'
    port_modbus = 502
    client = ModbusTcpClient(host, port_modbus)
    client.connect()


    print(get_base())
"""
