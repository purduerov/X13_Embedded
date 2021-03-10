from spidev import SpiDev

class MCP2515():

	DEFAULT_SPI_CLOCK_FREQUENCY = 4000000 # 4MHz
	DEFAULT_CLOCK_POLARITY = 0
	DEFAULT_CLOCK_PHASE = 0

	# SPI Command Bytes
	RESET_INSTRUCTION_BYTE = 0xC0
	WRITE_INSTRUCTION_BYTE = 0x02
	READ_INSTRUCTION_BYTE = 0x03

	def __init__(self, bus_num, device_num, 
			max_speed_hz=DEFAULT_SPI_CLOCK_FREQUENCY,
			clock_polarity=DEFAULT_CLOCK_POLARITY,
			clock_phase=DEFAULT_CLOCK_PHASE):

		self.spi_instance = SpiDev()

		self.bus_num = bus_num
		self.device_num = device_num

		self.set_spi_max_speed_hz(max_speed_hz)
		self.set_spi_mode(clock_polarity, clock_phase)

	def set_spi_max_speed_hz(self, max_speed_hz):
		self.spi_instance.max_speed_hz = max_speed_hz

	def set_spi_mode(self, clock_polarity=DEFAULT_CLOCK_POLARITY, clock_phase=DEFAULT_CLOCK_PHASE):
		self.spi_instance.mode = (clock_polarity << 1) | clock_phase

	def open_can_connection(self):
		self.spi_instance.open(self.bus_num, self.device_num)

	def close_can_connection(self):
		self.spi_instance.close()

	def write_bytes(self, address, data_bytes):
		if not isinstance(data_bytes, list):
			data_bytes = [data_bytes]
		bytes_to_write = [WRITE_INSTRUCTION_BYTE, address];
		bytes_to_write.extend(data_bytes)
		self.spi_instance.xfer2(bytes_to_write)

	def read_bytes(self, address, num_bytes):
		bytes_to_write = [READ_INSTRUCTION_BYTE, address]
		bytes_to_write.extend([0] * num_bytes)
		bytes_read = self.spi_instance.xfer2(bytes_to_write)
		return bytes_read[2:]

	def reset(self):
		self.spi_instance.xfer2([RESET_INSTRUCTION_BYTE])

	def load_tx_buffer(self, can_message_buffer, can_message_buffer_number):
		data_bytes_to_send = []

		load_tx_buffer_command = 0x40 | (can_message_buffer_number << 1)
		data_bytes_to_send.append(load_tx_buffer_command)

		data_byte = (can_message_buffer.get_standard_id() & (0xFF << 3)) >> 3
		data_bytes_to_send.append(data_byte)

		data_byte = (can_message_buffer.get_standard_id() & 0x7) << 5
		data_byte |= (can_message_buffer.get_id_extension() << 3)
		data_byte |= ((can_message_buffer.get_extended_id() & (0x3 << 16)) >> 16)
		data_bytes_to_send.append(data_byte)

		data_byte = (can_message_buffer.get_extended_id() & (0xFF << 8)) >> 8
		data_bytes_to_send.append(data_byte)

		data_byte = (can_message_buffer.get_extended_id() & 0xFF)
		data_bytes_to_send.append(data_byte)

		data_byte = can_message_buffer.get_remote_transmission() << 6
		data_byte |= can_message_buffer.get_num_data_bytes_to_send()
		data_bytes_to_send.append(data_byte)

		for i in range(can_message_buffer.get_num_data_bytes_to_send()):
			data_bytes_to_send.append(can_message_buffer.get_data_bytes_to_send()[i])

		self.spi_instance.xfer2(data_bytes_to_send)

	def send_tx_buffer(self, buffer0=False, buffer1=False, buffer2=False):
		buffer_bits = 0
		if buffer0: buffer_bits += 1
		if buffer1: buffer_bits += 2
		if buffer2: buffer_bits += 4
		byte_to_send = 0xC0 | buffer_bits
		self.spi_instance.xfer2([byte_to_send])

	def initialize(self):
		self.reset()

class CANMessageBuffer():
	CAN_STANDARD_ID_MASK = 0x7FF # 11 bits
	CAN_EXTENDED_ID_MASK = 0x3FFFF # 18 bits
	CAN_EXTENDED_ID_SHIFT = 11

	def __init__():
		pass

	def set_id(self, id):
		self.set_standard_id(id & CAN_STANDARD_ID_MASK)
		self.set_extended_id((id & (CAN_EXTENDED_ID_MASK << CAN_EXTENDED_ID_SHIFT)) >> CAN_EXTENDED_ID_SHIFT)

	def get_id(self):
		return (self.get_extended_id() << CAN_EXTENDED_ID_SHIFT) | self.get_standard_id()

	def set_standard_id(self, standard_id):
		self.standard_id = standard_id & CAN_STANDARD_ID_MASK

	def get_standard_id(self):
		return self.standard_id

	def set_extended_id(self, extended_id):
		self.extended_id = extended_id & CAN_EXTENDED_ID_MASK

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


if __name__ == "__main__":
	mcp2515 = MCP2515(0, 0)