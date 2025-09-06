# 0 for acoustic modem, 1 for radio
uint8 connection_type

# id of vehicle connections
int64[] vehicle_ids

# connection status
bool[] connections

# time since last ping response
int64[] last_ping