class CANMessageID():

	CAN_STANDARD_ID_MASK = 0x7FF # 11 bits
	CAN_EXTENDED_ID_MASK = 0x3FFFF # 18 bits
	CAN_EXTENDED_ID_SHIFT = 11

	def __init__(self):
		pass

	def set_id(self, id):
		self.set_standard_id(id & CANMessageID.CAN_STANDARD_ID_MASK)
		self.set_extended_id((id & (CANMessageID.CAN_EXTENDED_ID_MASK << CANMessageID.CAN_EXTENDED_ID_SHIFT)) >> CANMessageID.CAN_EXTENDED_ID_SHIFT)

	def get_id(self):
		return (self.get_extended_id() << CANMessageID.CAN_EXTENDED_ID_SHIFT) | self.get_standard_id()

	def set_standard_id(self, standard_id):
		self.standard_id = standard_id & CANMessageID.CAN_STANDARD_ID_MASK

	def get_standard_id(self):
		return self.standard_id

	def set_extended_id(self, extended_id):
		self.extended_id = extended_id & CANMessageID.CAN_EXTENDED_ID_MASK

	def get_extended_id(self):
		return self.extended_id

	def set_id_extension(self, id_extension):
		self.ide = 1 if id_extension else 0

	def get_id_extension(self):
		return self.ide

class CANMessageFilter(CANMessageID):

	BYTE_MASK = 0xFF

	def __init__(self):
		super().__init__()

	def set_data_byte0(self, data_byte0):
		self.data_byte0 = data_bytes0 & CANMessageFilter.BYTE_MASK

	def get_data_byte0(self):
		return self.data_byte0

	def set_data_byte1(self, data_byte1):
		self.data_byte1 = data_byte1 & CANMessageFilter.BYTE_MASK

	def get_data_byte1(self):
		return self.data_byte1

class CANMessageMask(CANMessageFilter):

	def __init__(self):
		super().__init__()


class CANMessageBuffer(CANMessageID):

	def __init__(self):
		super().__init__()

	def set_remote_transmission(self, remote_transmission):
		self.rtr = 1 if remote_transmission else 0

	def get_remote_transmission(self):
		return self.rtr

	def set_data_bytes(self, data_bytes):
		self.dlc = len(data_bytes)
		self.data_bytes = data_bytes

	def get_data_bytes(self):
		return self.data_bytes

	def get_num_data_bytes(self):
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