
from dynamixel_sdk import *

PORTS = [
    "/dev/ttyUSB3",
    "/dev/ttyUSB0"
]

BAUDRATE = 4_000_000
PROTOCOL_VERSION = 2.0

# IDs you expect to have connected
# ids = [1, 3, 5, 6, 7, 8]
ids = [2, 4]

packet = PacketHandler(PROTOCOL_VERSION)


# ---------------------------------------------------------
# Helper function to read a value from a Dynamixel
# ---------------------------------------------------------
def read_value(port, dxl_id, address, size):

    if size == 1:
        value, result, error = packet.read1ByteTxRx(
            port, dxl_id, address
        )

    elif size == 2:
        value, result, error = packet.read2ByteTxRx(
            port, dxl_id, address
        )

    elif size == 4:
        value, result, error = packet.read4ByteTxRx(
            port, dxl_id, address
        )

    else:
        raise ValueError("Unsupported size")

    if result != COMM_SUCCESS:
        return None

    if error != 0:
        return None

    return value


# ---------------------------------------------------------
# Addresses for XM430-W350
# ---------------------------------------------------------
# address : (name, size)

REGISTERS = {
    11:  ("Operating Mode",       1),
    64:  ("Torque Enable",        1),

    80:  ("Position D Gain",      2),
    82:  ("Position I Gain",      2),
    84:  ("Position P Gain",      2),

    104: ("Goal Velocity",        4),
    108: ("Profile Acceleration", 4),
    112: ("Profile Velocity",     4),

    126: ("Present Current",      2),
    132: ("Present Position",     4),

    70:  ("Hardware Error",       1),
}


# ---------------------------------------------------------
# Open all ports
# ---------------------------------------------------------
port_handlers = []

for port_name in PORTS:

    port = PortHandler(port_name)

    if not port.openPort():
        print(f"Could not open port: {port_name}")
        continue

    if not port.setBaudRate(BAUDRATE):
        print(f"Could not set baudrate on: {port_name}")
        continue

    print(f"Opened: {port_name}")

    port_handlers.append((port_name, port))


# ---------------------------------------------------------
# Scan all motors
# ---------------------------------------------------------
for port_name, port in port_handlers:

    print()
    print("=" * 70)
    print(f"PORT: {port_name}")
    print("=" * 70)

    for dxl_id in ids:

        # -------------------------------------------------
        # Ping
        # -------------------------------------------------
        model_number, result, error = packet.ping(
            port,
            dxl_id
        )

        if result != COMM_SUCCESS:
            print(f"ID {dxl_id}: communication error")
            continue

        if error != 0:
            print(
                f"ID {dxl_id}: "
                f"Dynamixel error {error}"
            )
            continue

        # -------------------------------------------------
        # Firmware version
        # Firmware Version is address 6
        # -------------------------------------------------
        firmware = read_value(
            port,
            dxl_id,
            6,
            1
        )

        print()
        print(f"ID {dxl_id}")
        print(f"  Model number : {model_number}")
        print(f"  Firmware     : {firmware}")

        # -------------------------------------------------
        # Read configuration
        # -------------------------------------------------
        for address, (name, size) in REGISTERS.items():

            value = read_value(
                port,
                dxl_id,
                address,
                size
            )

            if value is None:
                print(f"  {name:22}: ERROR")
            else:
                print(f"  {name:22}: {value}")


# ---------------------------------------------------------
# Close ports
# ---------------------------------------------------------
for port_name, port in port_handlers:
    port.closePort()

print()
print("Done.")
