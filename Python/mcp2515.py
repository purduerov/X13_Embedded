from spidev import SpiDev
from enum import IntEnum
import time

from can_data_structures import *

class OperationModes(IntEnum):
	NORMAL = 0
	SLEEP = 1
	LOOPBACK = 2
	LISTEN_ONLY = 3
	CONFIGURATION = 4

class MCP2515():

	DEFAULT_SPI_CLOCK_FREQUENCY = 4000000 # 4MHz
	DEFAULT_CLOCK_POLARITY = 0
	DEFAULT_CLOCK_PHASE = 0

	# SPI Command Bytes
	RESET_INSTRUCTION_BYTE = 0xC0
	WRITE_INSTRUCTION_BYTE = 0x02
	READ_INSTRUCTION_BYTE = 0x03
	LOAD_TX_BUFFER_INSTRUCTION_BASE_BYTE = 0x40
	READ_RX_BUFFER_INSTRUCTION_BASE_BYTE = 0x90
	REQUEST_TO_SEND_INSTRUCTION_BYTE = 0x80
	READ_STATUS_INSTRUCTION_BYTE = 0xA0
	BIT_MODIFY_INSTRUCTION_BYTE = 0x05

	# CAN RX Filter Number to Address Map
	CAN_BUFFER_NUMBER_TO_CR_ADDRESS = [0x60, 0x70]
	CAN_FILTER_NUMBER_TO_STARTING_ADDRESS = [
		0x00,
		0x04,
		0x08,
		0x10,
		0x14,
		0x18
	]
	CAN_MASK_NUMBER_TO_STARTING_ADDRESS = [0x20, 0x24]

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

			data_byte = (can_message_buffer.get_extended_id() & (MCP2515.BYTE_MASK  << 8)) >> 8
			data_bytes_to_send.append(data_byte)

			data_byte = (can_message_buffer.get_extended_id() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)
		else:
			data_bytes_to_send.append(data_byte)
			data_bytes_to_send.append(0x00)
			data_bytes_to_send.append(0x00)

		data_byte = can_message_buffer.get_remote_transmission() << 6
		data_byte |= can_message_buffer.get_num_data_bytes()
		data_bytes_to_send.append(data_byte)

		for i in range(can_message_buffer.get_num_data_bytes()):
			data_bytes_to_send.append(can_message_buffer.get_data_bytes()[i])

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

	def set_can_rx_filter(self, can_filter_number, can_filter):
		# Filter Number = [0, 5]
		data_bytes_to_send = []
		base_filter_address = MCP2515.CAN_FILTER_NUMBER_TO_STARTING_ADDRESS[can_filter_number]

		data_byte = (can_filter.get_standard_id() & (MCP2515.BYTE_MASK << 3)) >> 3
		data_bytes_to_send.append(data_byte)

		data_byte = (can_filter.get_standard_id() & 0x7) << 5
		data_byte |= (can_filter.get_id_extension() << 3)

		if (can_filter.get_id_extension()):
			data_byte |= ((can_filter.get_extended_id() & (0x3 << 16)) >> 16)
			data_bytes_to_send.append(data_byte)

			data_byte = (can_filter.get_extended_id() & (MCP2515.BYTE_MASK  << 8)) >> 8
			data_bytes_to_send.append(data_byte)

			data_byte = (can_filter.get_extended_id() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)
		else:
			data_bytes_to_send.append(data_byte)

			data_byte = (can_filter.get_data_byte0() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)

			data_byte = (can_filter.get_data_byte1() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)

		self.write_bytes(base_filter_address, data_bytes_to_send)

	def set_can_rx_mask(self, can_mask_number, can_mask):
		# Mask Number = [0, 1]
		data_bytes_to_send = []
		base_mask_address = MCP2515.CAN_MASK_NUMBER_TO_STARTING_ADDRESS[can_mask_number]

		data_byte = (can_mask.get_standard_id() & (MCP2515.BYTE_MASK << 3)) >> 3
		data_bytes_to_send.append(data_byte)

		data_byte = (can_mask.get_standard_id() & 0x7) << 5

		if (can_mask.get_id_extension()):
			data_byte |= ((can_mask.get_extended_id() & (0x3 << 16)) >> 16)
			data_bytes_to_send.append(data_byte)

			data_byte = (can_mask.get_extended_id() & (MCP2515.BYTE_MASK  << 8)) >> 8
			data_bytes_to_send.append(data_byte)

			data_byte = (can_mask.get_extended_id() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)
		else:
			data_bytes_to_send.append(data_byte)

			data_byte = (can_mask.get_data_byte0() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)

			data_byte = (can_mask.get_data_byte1() & MCP2515.BYTE_MASK)
			data_bytes_to_send.append(data_byte)

		self.write_bytes(base_mask_address, data_bytes_to_send)

	def read_rx_control_register(self, buffer_number):
		rx_control_register_address = 0x60 if (buffer_number == 0) else 0x70
		return self.read_bytes(rx_control_register_address, num_bytes=1)

	def read_rx_buffer(self, buffer_number, expected_num_data_bytes=8):
		# RX Buffer Number = [0, 1]
		bytes_to_write = []
		num_bytes_to_receive = 5 + expected_num_data_bytes

		# RXnIF Flag automatically cleared when using READ RX BUFFER Command
		read_rx_command = MCP2515.READ_RX_BUFFER_INSTRUCTION_BASE_BYTE | (buffer_number << 2)
		bytes_to_write.append(read_rx_command)
		bytes_to_write.extend([0] * num_bytes_to_receive)

		bytes_read = self.spi_instance.xfer2(bytes_to_write)
		return bytes_read[1:]

	def interpret_rx_buffer(self, received_buffer_bytes):
		can_rx_message = CANMessageBuffer()

		standard_id = (received_buffer_bytes[0] << 3) | ((received_buffer_bytes[1] & (0x07 << 5)) >> 5)
		can_rx_message.set_standard_id(standard_id)

		ide = (received_buffer_bytes[1] & (0x01 << 3)) >> 3
		can_rx_message.set_id_extension(ide)

		if ide:
			extended_id = (received_buffer_bytes[1] & 0x03) << 16
			extended_id |= (received_buffer_bytes[2] & MCP2515.BYTE_MASK) << 8
			extended_id |= (received_buffer_bytes[3] & MCP2515.BYTE_MASK)
			can_rx_message.set_extended_id(extended_id)

			rtr = (received_buffer_bytes[4] & (0x01 << 6)) >> 6
			can_rx_message.set_remote_transmission(rtr)
		else:
			can_rx_message.set_extended_id(0)
			rtr = (received_buffer_bytes[1] & (0x01 << 4)) >> 4
			can_rx_message.set_remote_transmission(rtr)

		num_data_bytes = received_buffer_bytes[4] & 0x0F
		expected_num_data_bytes = len(received_buffer_bytes) - 5
		num_data_bytes_to_read = min(num_data_bytes, expected_num_data_bytes)
		starting_data_index = 5
		ending_data_index = starting_data_index + num_data_bytes_to_read
		can_rx_message.set_data_bytes(received_buffer_bytes[starting_data_index:ending_data_index])

		return can_rx_message

	def wait_until_message_received(self, rx_buffer_number):
		rx_message_received = False
		while (not rx_message_received):
			byte_read = self.get_status()[0]
			bit_mask = (0x01 << rx_buffer_number)
			rx_message_received = True if (byte_read & bit_mask) else False
		return rx_message_received

	def configure_rx_buffer(self, rx_buffer_number, is_filters_enabled=True, buffer_rollover=False):
		# Only RX Buffer 0 has the Buffer Overflow Option
		# Add additional bit to bit mask to allow buffer rollover control
		if rx_buffer_number == 0:
			mask_byte = 0x64
		else:
			mask_byte = 0x60
		
		data_byte = 0x00 if is_filters_enabled else (0x03 << 5)
		data_byte |= (0x01 << 2) if buffer_rollover else 0x00 

		# Modify Individual Bits of Control Register (CR)
		bytes_to_write = [
			MCP2515.BIT_MODIFY_INSTRUCTION_BYTE,
			MCP2515.CAN_BUFFER_NUMBER_TO_CR_ADDRESS[rx_buffer_number],
			mask_byte,
			data_byte
		]
		self.spi_instance.xfer2(bytes_to_write)

