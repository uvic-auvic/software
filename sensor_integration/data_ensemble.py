import pprint
import serial
import io

DATA_ID                  = 0x7F7F
FIXED_LEADER_ID          = 0x0000
VARIABLE_LEADER_ID       = 0x8000
VELOCITY_ID              = 0x0001
CORRELATION_MAGNITUDE_ID = 0x0002
ECHO_INTENSITY_ID        = 0x0003
PERCENT_GOOD             = 0x0004
BOTTOM_TRACK_ID          = 0x0600

def read_int(f, b=1):
    return int.from_bytes(f.read(b),byteorder="little")

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

class DataEnsemble:
    def __init__(self, bs):
        self.bs = bs.rstrip()
        with io.BytesIO(bs) as f:
            # HEADER
            self.header_id      = read_int(f)
            self.data_source_id = read_int(f)
            self.num_bytes      = read_int(f, 2)
            self.spare          = read_int(f)
            self.num_types      = read_int(f)
            self.offsets = []
            for i in range(self.num_types):
                self.offsets.append(read_int(f, 2))

            # FIXED LEADER DATA
            self.fixed_leader_id        = read_int(f, 2)
            self.cpu_fw_version         = read_int(f)
            self.cpu_fw_revision        = read_int(f)
            self.system_configuration   = read_int(f, 2) # TODO: see manual
            self.real_sim_flag          = read_int(f)
            self.lag_length             = read_int(f)
            self.num_beams              = read_int(f)
            self.num_cells              = read_int(f)
            self.pings_per_ensemble     = read_int(f, 2)
            self.depth_cell_length      = read_int(f, 2)
            self.blank_after_trasmit    = read_int(f, 2)
            self.profiling_mode         = read_int(f)
            self.low_corr_thresh        = read_int(f)
            self.num_code_reps          = read_int(f)
            read_int(f)                 # blank?
            self.error_velocity_maximum = read_int(f, 2)
            self.tpp_minutes            = read_int(f)
            self.tpp_seconds            = read_int(f)
            self.tpp_hundredths         = read_int(f)
            self.coordinate_transform   = read_int(f) # TODO: see manual
            self.heading_alignment      = read_int(f, 2)
            self.heading_bias           = read_int(f, 2)
            self.sensor_source          = read_int(f) # TODO: see manual
            self.sensors_available      = read_int(f) # TODO: see manual
            self.bin_1_distance         = read_int(f, 2) # no LSB, MSB in manual?
            self.xmt_pulse_length       = read_int(f, 2)
            self.wp_ref_layer_average   = read_int(f, 2)
            self.false_target_thresh    = read_int(f)
            self.spare_1                = read_int(f)
            self.transmit_lag_distance  = read_int(f, 2)
            self.spare_2                = read_int(f, 3)
            self.system_bandwidth       = read_int(f, 2)
            self.spare_3                = read_int(f)
            self.spare_4                = read_int(f)
            self.system_serial_number   = read_int(f, 3)

            # VARIABLE LEADER DATA
            self.variable_leader_id       = read_int(f, 2)
            self.ensemble_number          = read_int(f, 2) # TODO: see manual
            self.rtc_year                 = read_int(f)
            self.rtc_month                = read_int(f)
            self.rtc_day                  = read_int(f)
            self.rtc_hour                 = read_int(f)
            self.rtc_minute               = read_int(f)
            self.rtc_second               = read_int(f)
            self.rtc_hudredths            = read_int(f)
            self.ensemble_number_msb      = read_int(f)
            self.bit_result               = read_int(f, 2) # TODO: see manual
            self.speed_of_sound           = read_int(f, 2)
            self.depth_of_transducer      = read_int(f, 2)
            self.heading                  = read_int(f, 2)
            self.pitch                    = read_int(f, 2)
            self.roll                     = read_int(f, 2)
            self.salinity                 = read_int(f, 2)
            self.temperature              = read_int(f, 2)
            self.mpt_minutes              = read_int(f)
            self.mpt_seconds              = read_int(f)
            self.mpt_hundredths           = read_int(f)
            self.hdg_std_dev              = read_int(f)
            self.pitch_std_dev            = read_int(f)
            self.roll_std_dev             = read_int(f) # vvv outputs of the Analog=to-Digital Converter (ADC)..
            self.adc_channel_0            = read_int(f) # Not Used
            self.adc_channel_1            = read_int(f) # XMIT VOLTAGE
            self.adc_channel_2            = read_int(f) # Not Used
            self.adc_channel_3            = read_int(f) # Not Used
            self.adc_channel_4            = read_int(f) # Not Used
            self.adc_channel_5            = read_int(f) # Not Used
            self.adc_channel_6            = read_int(f) # Not Used
            self.adc_channel_7            = read_int(f) # Not Used
            self.error_status_word        = read_int(f, 4) # 4 spearate values, reserved for TRDI use
            self.spare_5                  = read_int(f, 2) # reserved for TRDI use
            self.pressure                 = read_int(f, 4)
            self.pressure_sensor_variance = read_int(f, 4)
            self.spare_6                  = read_int(f, 4)
            
            # VELOCITY DATA
            self.velocity_id = read_int(f, 2)
            self.velocities = []
            for i in range(self.num_cells):
                self.velocities.append(twos_comp(read_int(f, 2), 16))
            
            # CORRELATION MAGNITUDE
            # are the rest of the data types even sent?
            # 3 data types including fixed and variable leader
            # If WP (number of pings to average) = zero the ExplorerDVL does not collect water-profile data.
            # WD sets which data types are collected
            self.correlation_magnitude_id = read_int(f, 2)
            self.correlation_magnitudes = []
            for i in range(self.num_cells):
                self.correlation_magnitudes.append(read_int(f))

            # ECHO INTENSITY
            self.echo_intensity_id = read_int(f, 2)
            self.echo_intensities = []
            for i in range(self.num_cells):
                self.echo_intensities.append(read_int(f))

            # PERCENT GOOD
            self.percent_good_id = read_int(f, 2)
            self.percent_goods = []
            for i in range(self.num_cells):
                self.percent_goods.append(read_int(f))

            # BOTTOM TRACK DATA
            # TODO

            # RESERVED
            # TODO

            # CHECKSUM
            # TODO

    def __str__(self):
        return f"ensemble size: {len(self.bs)} bytes\n{self.velocities}" # pprint.pformat(self.__dict__,width=1)



if __name__=="__main__":
    with serial.Serial('/dev/ttyUSB0') as s:
        s.reset_input_buffer() # doesn't work?
        s.send_break() # allows us to interact with the DVL
        while (c:=s.read()) != b">": pass # print(c)
        print("done reading DVL intro")
        # s.write(b"CF01110\r\n") # changes to binary output, doesn't work?
        while True:
            input("ping? ")
            s.write(b"\t") # initiates a ping
            print(s.readline())
            data = s.readline()
            de = DataEnsemble(data)
            print(de)
