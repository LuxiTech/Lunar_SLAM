# Dock transfer message for quadruped/biped mode switching
# Corresponds to device_bridge::DockTransferMessage

std_msgs/Header header

# Message type: 0=ACK, 1=COMMAND, 15=E_STOP
uint8 MSG_TYPE_ACK = 0
uint8 MSG_TYPE_COMMAND = 1
uint8 MSG_TYPE_E_STOP = 15
uint8 type

# Mode flags (order must match SendDockTransfer.srv)
bool is_slave      # true=Slave robot, false=Master robot
bool is_quadruped  # true=Quadruped mode, false=Biped mode
bool is_balance    # true=Balance mode, false=Cart mode
bool error         # true=Error message
