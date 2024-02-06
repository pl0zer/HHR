
import os
import time 


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_RED                = 65
    LEN_LED_RED                 = 1         # Data Byte Length
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4       # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4       # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 3800        # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4000     # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 3000000
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_LED_RED                = 563       # R.G.B Address: 563 (red), 564 (green), 565 (blue)
    LEN_LED_RED                 = 1         # Data Byte Length
    ADDR_GOAL_POSITION          = 596
    LEN_GOAL_POSITION           = 4
    ADDR_PRESENT_POSITION       = 611
    LEN_PRESENT_POSITION        = 4
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512       # Control table address is different in DYNAMIXEL model
    ADDR_LED_RED                = 513       # R.G.B Address: 513 (red), 544 (green), 515 (blue)
    LEN_LED_RED                 = 1         # Data Byte Length
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM10'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]        # Goal position
dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

#Enable Torque on any motor
def enableTorque(ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID)

def disableTorque(ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def addParamStorage(ID):
    dxl_addparam_result = groupBulkRead.addParam(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % ID)
        quit()

def addGoalPosition(ID):
    dxl_addparam_result = groupBulkWrite.addParam(ID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % ID)
        quit()

#Checks if bulkread data is available 
def checkBulkRead(ID):
    dxl_getdata_result = groupBulkRead.isAvailable(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % ID)
        quit() 

def getPresentPosition(ID):
    groupBulkRead.getData(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    



#Enables Torque for ID1 and ID2
enableTorque(1)
enableTorque(2)

param_goal_position = [
    DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), 
    DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), 
    DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), 
    DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))
    ]

#Add Parameter storage for present position for ID1 and ID2
addParamStorage(1)
addParamStorage(2)

while 1: 
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    disableTorque(1)
    disableTorque(2)

      # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()


    #Wait 10 seconds
    time.sleep(10)

    enableTorque(1)
    enableTorque(2)

    dxl_goal_position = [3700, 4000]

    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), 
        DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), 
        DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), 
        DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))
        ]
    
    addGoalPosition(1)
    addGoalPosition(2)

        # Bulkread present position and LED status
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result)) 

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    time.sleep(10)


    disableTorque(1)
    disableTorque(2)

    groupBulkRead.clearParam()

    # Bulk read present position and LED status
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    checkBulkRead(1)
    checkBulkRead(2)

    dxl1_present_position = getPresentPosition(1)
    dxl2_present_position = getPresentPosition(2)


    print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
        
    if not (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
        break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear bulkread parameter storage
groupBulkRead.clearParam()

# Close port
portHandler.closePort()



