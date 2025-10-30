# Sr14TdFrame.msg
# Designed for IMST sR-14MPc radar (fixed 4-channel system)

std_msgs/Header header      # Timestamp

float32[] ch1               # Channel 1 time-domain samples
float32[] ch2               # Channel 2 time-domain samples
float32[] ch3               # Channel 3 time-domain samples
float32[] ch4               # Channel 4 time-domain samples

