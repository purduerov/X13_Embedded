import time

from can_data_structures import *
from mcp2515 import *

# Transmits a single CAN Message
if __name__ == "__main__":
	can_controller = MCP2515(bus_num=0, device_num=0)
	can_controller.open_can_connection()

	initialize_mcp2515(can_controller, rx=0)
	print("Initialization Finished")
	
	# Clear Interrupt Enable Register
	can_controller.write_bytes(0x2B, 0x00)

def send_can_message(mcp2515_instance):
	TX_BUFFER_NUMBER = 0
	tx_buffer = CANMessageBuffer()
	tx_buffer.set_id(0x201)
	tx_buffer.set_id_extension(False)
	tx_buffer.set_remote_transmission(False)
	tx_buffer.set_data_bytes([0x01, 0x04, 0x09, 0x10])
	mcp2515_instance.load_tx_buffer(tx_buffer, can_message_buffer_number=TX_BUFFER_NUMBER)

	#mcp2515_instance.send_tx_buffers(buffer2=True)
	mcp2515_instance.send_tx_buffer_with_priority(buffer_number=TX_BUFFER_NUMBER, priority=3)

	while True:
		read_byte = mcp2515_instance.read_bytes(0x30, 1)
		print(f"TX CNTRL = {read_byte}")
		read_byte = mcp2515_instance.read_bytes(0x2C, 1)
		print(f"CANINTF = {read_byte}")
		read_byte = mcp2515_instance.read_bytes(0x0E, 1)
		print(f"CAN Status Register = {read_byte}")
		read_byte = mcp2515_instance.read_bytes(0x2D, 1)
		print(f"Error Status Register = {read_byte}")
		time.sleep(.1)

	mcp2515_instance.wait_until_tx_message_success(TX_BUFFER_NUMBER)
	print("Message Transmission Success")

def receive_can_message(mcp2515_instance):
	rx_buffer_number = 0
	print("Waiting on Message Reception...")
	mcp2515_instance.wait_until_message_received(rx_buffer_number)

def initialize_mcp2515(mcp2515_instance, rx):
	# Switch to Configuration Mode by resetting the device
	mcp2515_instance.reset()
	time.sleep(.1)
	while (mcp2515_instance.get_operation_mode() != OperationModes.CONFIGURATION):
		mcp2515_instance.reset()
		time.sleep(.1)

	configure_bit_timing(mcp2515_instance)
	if rx:
		configure_rx_message_mask(0)
		configure_rx_message_filter(0x211, 0)

	# Switch to Normal Mode
	mcp2515_instance.configure_bit_timing(bit_timing_configuration)
	while (mcp2515_instance.get_operation_mode() != OperationModes.NORMAL):
		mcp2515_instance.switch_operation_modes(OperationModes.NORMAL)
		time.sleep(.1)

def configure_bit_timing(mcp2515_instance):
	bit_timing = BitTimingConfiguration()
	bit_timing.set_sjw(1)
	bit_timing.set_baud_rate_prescalar(1)
	bit_timing.set_propagation_segment_length(2 - 1)
	bit_timing.set_phase_segment1_length(8 - 1)
	bit_timing.set_phase_segment2_length(5 - 1)
	bit_timing.set_num_samples_per_bit(1)
	mcp2515_instance.configure_bit_timing(bit_timing)

def configure_rx_message_filter(mcp2515_instance, can_stdid, filter_number):
	can_rx_message_filter = CANMessageFilter()
	can_rx_message_filter.set_id(can_stdid & 0x7FF)
	can_rx_message_filter.set_id_extension(False)
	can_rx_message_filter.set_data_byte0(0x00)
	can_rx_message_filter.set_data_byte1(0x00)
	mcp2515_instance.set_can_rx_filter(filter_number, can_rx_message_filter)

def configure_rx_message_mask(mcp2515_instance, mask_number):
	can_rx_message_mask = CANMessageMask()
	can_rx_message_mask.set_standard_id(0x7FF)
	can_rx_message_mask.set_extended_id(0)
	can_rx_message_mask.set_id_extension(True)
	can_rx_message_mask.set_data_byte0(0x00)
	can_rx_message_mask.set_data_byte1(0x00)
	mcp2515_instance.set_can_rx_mask(mask_number, can_rx_message_mask)