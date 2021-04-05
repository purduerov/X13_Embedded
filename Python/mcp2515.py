from spidev import SpiDev
from enum import IntEnum
import time

class MCP2515():

	DEFAULT_SPI_CLOCK_FREQUENCY = 4000000 # 4MHz
	DEFAULT_CLOCK_POLARITY = 0
	DEFAULT_CLOCK_PHASE = 0

	# SPI Command Bytes
	RESET_INSTRUCTION_BYTE = 0xC0
	WRITE_INSTRUCTION_BYTE = 0x02
	READ_INSTRUCTION_BYTE = 0x03
	LOAD_TX_BUFFER_INSTRUCTION_BASE_BYTE = 0x40
	REQUEST_TO_SEND_INSTRUCTION_BYTE = 0x80
	READ_STATUS_INSTRUCTION_BYTE = 0xA0

	BYTE_MASK = 0xFF

	def __init__(self, bus_num, device_num, 
			max_speed_hz=DEFAULT_SPI_CLOCK_FREQUENCY,
			clock_polarity=DEFAULT_CLOCK_POLARITY,
			clock_phase=DEFAULT_CLOCK_PHASE):

		self.spi_instance = SpiDev()

		self.bus_num = bus_num
		self.device_num = device_num

		self.clk_speed = max_speed_hz
		self.clk_polarity = clock_polarity
		self.clk_phase = clock_phase

	def set_spi_max_speed_hz(self, max_speed_hz=DEFAULT_SPI_CLOCK_FREQUENCY):
		self.spi_instance.max_speed_hz = max_speed_hz

	def set_spi_mode(self, clock_polarity=DEFAULT_CLOCK_POLARITY, clock_phase=DEFAULT_CLOCK_PHASE):
		self.spi_instance.mode = (clock_polarity << 1) | clock_phase

	def open_can_connection(self):
		self.spi_instance.open(self.bus_num, self.device_num)
		
		# Cannot set these SPI parameters until SPI connection has been opened
		self.set_spi_max_speed_hz(self.clk_speed)
		self.set_spi_mode(self.clk_polarity, self.clk_phase)

	def close_can_connection(self):
		self.spi_instance.close()

	def write_bytes(self, address, data_bytes):
		if not isinstance(data_bytes, list):
			data_bytes = [data_bytes]
		bytes_to_write = [MCP2515.WRITE_INSTRUCTION_BYTE, address];
		bytes_to_write.extend(data_bytes)
		self.spi_instance.xfer2(bytes_to_write)

	def read_bytes(self, address, num_bytes):
		bytes_to_write = [MCP2515.READ_INSTRUCTION_BYTE, address]
		bytes_to_write.extend([0] * num_bytes)
		bytes_read = self.spi_instance.xfer2(bytes_to_write)
		return bytes_read[2:]

	def reset(self):
		self.spi_instance.xfer2([MCP2515.RESET_INSTRUCTION_BYTE])

	def load_tx_buffer(self, can_message_buffer, can_message_buffer_number):
		data_bytes_to_send = []

		load_tx_buffer_command = MCP2515.LOAD_TX_BUFFER_INSTRUCTION_BASE_BYTE | (can_message_buffer_number << 1)
		data_bytes_to_send.append(load_tx_buffer_command)

		data_byte = (can_message_buffer.get_standard_id() & (MCP2515.BYTE_MASK << 3)) >> 3
		data_bytes_to_send.append(data_byte)

		data_byte = (can_message_buffer.get_standard_id() & 0x7) << 5
		data_byte |= (can_message_buffer.get_id_extension() << 3)

		if (can_message_buffer.get_id_extension()):
			data_byte |= ((can_message_buffer.get_extended_id() & (0x3 << 16)) >> 16)
			data_bytes_to_send.append(data_byte)

			data_byte = (can_message_buffer.get_extended_id() & (0xFF << 8)) >> 8
			data_bytes_to_send.append(data_byte)

			data_byte = (can_message_buffer.get_extended_id() & 0xFF)
			data_bytes_to_send.append(data_byte)
		else:
			data_bytes_to_send.append(data_byte)
			data_bytes_to_send.append(0x00)
			data_bytes_to_send.append(0x00)

		data_byte = can_message_buffer.get_remote_transmission() << 6
		data_byte |= can_message_buffer.get_num_data_bytes_to_send()
		data_bytes_to_send.append(data_byte)

		for i in range(can_message_buffer.get_num_data_bytes_to_send()):
			data_bytes_to_send.append(can_message_buffer.get_data_bytes_to_send()[i])

		self.spi_instance.xfer2(data_bytes_to_send)

	def send_tx_buffers(self, buffer0=False, buffer1=False, buffer2=False):
		buffer_bits = 0
		if buffer0: buffer_bits += 1
		if buffer1: buffer_bits += 2
		if buffer2: buffer_bits += 4
		byte_to_send = MCP2515.REQUEST_TO_SEND_INSTRUCTION_BYTE | buffer_bits
		print(f"RTS Byte = {byte_to_send}")
		self.spi_instance.xfer2([byte_to_send])

	def send_tx_buffer_with_priority(self, buffer_number, priority):
		# Priority [0, 3] with 0 the lowest and 3 the highest
		write_address = self.get_tx_buffer_control_register_address(buffer_number)
		data_byte_to_send = (1 << 3) | priority
		self.write_bytes(write_address, data_byte_to_send)

	def get_tx_buffer_control_register_address(self, buffer_number):
		BASE_TX_BUFFER_ADDRESS = 0x30
		NUM_BYTES_BETWEEN_TX_BUFFERS = 16
		return (BASE_TX_BUFFER_ADDRESS + (buffer_number * NUM_BYTES_BETWEEN_TX_BUFFERS))

	def get_status(self, num_consecutive_times=1):
		bytes_to_write = [MCP2515.READ_STATUS_INSTRUCTION_BYTE]
		bytes_to_write.extend([0] * num_consecutive_times)
		bytes_read = self.spi_instance.xfer2(bytes_to_write)
		return bytes_read[1:]

	def wait_until_tx_message_success(self, buffer_number):
		tx_message_success = False
		while (not tx_message_success):
			byte_read = self.get_status()[0]
			bit_mask = 1 << ((2 * buffer_number) + 2)
			tx_message_success = not (byte_read & bit_mask)
		return tx_message_success

	def configure_bit_timing(self, bit_timing_configuration):
		CONFIGURATION_REGISTER1_ADDRESS = 0x2A
		CONFIGURATION_REGISTER2_ADDRESS = 0x29
		CONFIGURATION_REGISTER3_ADDRESS = 0x28
		bytes_to_write = []
		
		byte_to_write = bit_timing_configuration.get_phase_segment2_length() & 0x07
		bytes_to_write.append(byte_to_write)

		byte_to_write = bit_timing_configuration.get_propagation_segment_length() & 0x07
		byte_to_write |= ((bit_timing_configuration.get_phase_segment1_length() & 0x07) << 3)
		if (bit_timing_configuration.get_num_samples_per_bit() == 3):
			byte_to_write |= (1 << 6)
		byte_to_write |= (1 << 7)
		bytes_to_write.append(byte_to_write)

		byte_to_write = bit_timing_configuration.get_baud_rate_prescalar() & 0x3F
		byte_to_write |= (bit_timing_configuration.get_sjw() - 1) & 0x03
		bytes_to_write.append(byte_to_write)

		self.write_bytes(CONFIGURATION_REGISTER3_ADDRESS, bytes_to_write)

	def switch_operation_modes(self, operating_mode):
		CAN_CONTROL_REGISTER_ADDRESS = 0x0F
		self.write_bytes(CAN_CONTROL_REGISTER_ADDRESS, operating_mode)

	def get_operation_mode(self):
		CAN_STATUS_REGISTER_ADDRESS = 0x0E
		read_byte = self.read_bytes(CAN_STATUS_REGISTER_ADDRESS, 1)[0]
		return OperationModes((read_byte & (0x7 << 5)) >> 5)

	def initialize(self, bit_timing_configuration):
		self.reset()
		time.sleep(.1)
		while (self.get_operation_mode() != OperationModes.CONFIGURATION):
			self.reset()
			time.sleep(.1)
		self.configure_bit_timing(bit_timing_configuration)
		while (self.get_operation_mode() != OperationModes.NORMAL):
			self.switch_operation_modes(OperationModes.NORMAL)
			time.sleep(.1)

class CANMessageBuffer():
	CAN_STANDARD_ID_MASK = 0x7FF # 11 bits
	CAN_EXTENDED_ID_MASK = 0x3FFFF # 18 bits
	CAN_EXTENDED_ID_SHIFT = 11

	def __init__(self):
		pass

	def set_id(self, id):
		self.set_standard_id(id & CANMessageBuffer.CAN_STANDARD_ID_MASK)
		self.set_extended_id((id & (CANMessageBuffer.CAN_EXTENDED_ID_MASK << CANMessageBuffer.CAN_EXTENDED_ID_SHIFT)) >> CANMessageBuffer.CAN_EXTENDED_ID_SHIFT)

	def get_id(self):
		return (self.get_extended_id() << CANMessageBuffer.CAN_EXTENDED_ID_SHIFT) | self.get_standard_id()

	def set_standard_id(self, standard_id):
		self.standard_id = standard_id & CANMessageBuffer.CAN_STANDARD_ID_MASK

	def get_standard_id(self):
		return self.standard_id

	def set_extended_id(self, extended_id):
		self.extended_id = extended_id & CANMessageBuffer.CAN_EXTENDED_ID_MASK

	def get_extended_id(self):
		return self.extended_id

	def set_id_extension(self, id_extension):
		self.ide = 1 if id_extension else 0

	def get_id_extension(self):
		return self.ide

	def set_remote_transmission(self, remote_transmission):
		self.rtr = 1 if remote_transmission else 0

	def get_remote_transmission(self):
		return self.rtr

	def set_data_bytes_to_send(self, data_bytes):
		self.dlc = len(data_bytes)
		self.data_bytes = data_bytes

	def get_data_bytes_to_send(self):
		return self.data_bytes

	def get_num_data_bytes_to_send(self):
		return self.dlc

class BitTimingConfiguration():

	def __init__(self):
		pass

	def set_sjw(self, sjw):
		self.sjw = sjw

	def get_sjw(self):
		return self.sjw

	def set_baud_rate_prescalar(self, brp):
		self.brp = brp

	def get_baud_rate_prescalar(self):
		return self.brp

	def set_propagation_segment_length(self, num_time_quanta):
		self.propseg = num_time_quanta

	def get_propagation_segment_length(self):
		return self.propseg

	def set_phase_segment1_length(self, num_time_quanta):
		self.ps1 = num_time_quanta

	def get_phase_segment1_length(self):
		return self.ps1

	def set_phase_segment2_length(self, num_time_quanta):
		self.ps2 = num_time_quanta

	def get_phase_segment2_length(self):
		return self.ps2

	def set_num_samples_per_bit(self, num_samples):
		# Number of samples can only be 1 or 3
		self.num_samples = num_samples

	def get_num_samples_per_bit(self):
		return self.num_samples

class OperationModes(IntEnum):
	NORMAL = 0
	SLEEP = 1
	LOOPBACK = 2
	LISTEN_ONLY = 3
	CONFIGURATION = 4

# Transmits a single CAN Message
if __name__ == "__main__":
	can_controller = MCP2515(bus_num=0, device_num=0)
	can_controller.open_can_connection()

	bit_timing = BitTimingConfiguration()
	bit_timing.set_sjw(1)
	bit_timing.set_baud_rate_prescalar(1)
	bit_timing.set_propagation_segment_length(2 - 1)
	bit_timing.set_phase_segment1_length(8 - 1)
	bit_timing.set_phase_segment2_length(5 - 1)
	bit_timing.set_num_samples_per_bit(1)	
	can_controller.initialize(bit_timing)
	print("Initialization Finished")
	
	# Clear Interrupt Enable Register
	can_controller.write_bytes(0x2B, 0x00)

	TX_BUFFER_NUMBER = 0
	tx_buffer = CANMessageBuffer()
	tx_buffer.set_id(0x201)
	tx_buffer.set_id_extension(False)
	tx_buffer.set_remote_transmission(False)
	tx_buffer.set_data_bytes_to_send([0x01, 0x04, 0x09, 0x10])
	can_controller.load_tx_buffer(tx_buffer, can_message_buffer_number=TX_BUFFER_NUMBER)

	#can_controller.send_tx_buffers(buffer2=True)
	can_controller.send_tx_buffer_with_priority(buffer_number=TX_BUFFER_NUMBER, priority=3)

	while True:
		read_byte = can_controller.read_bytes(0x30, 1)
		print(f"TX CNTRL = {read_byte}")
		read_byte = can_controller.read_bytes(0x2C, 1)
		print(f"CANINTF = {read_byte}")
		read_byte = can_controller.read_bytes(0x0E, 1)
		print(f"CAN Status Register = {read_byte}")
		read_byte = can_controller.read_bytes(0x2D, 1)
		print(f"Error Status Register = {read_byte}")
		time.sleep(.1)

	can_controller.wait_until_tx_message_success(TX_BUFFER_NUMBER)
	print("Message Transmission Success")
