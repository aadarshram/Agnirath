import struct

def create_can_message(can_address, float1, float2):
    # Verify that the input CAN address is a valid integer
    if not isinstance(can_address, int):
        raise ValueError("Invalid CAN address. Please provide an integer value.")

    # Pack the two floats into bytes (little-endian format)
    data_bytes = struct.pack('<ff', float1, float2)

    # Convert the packed bytes to hexadecimal
    data_hex = data_bytes.hex()

    # Ensure that the data_hex string is exactly 8 characters long (4 bytes in little-endian)
    if len(data_hex) != 8:
        raise ValueError("Invalid data format. Please provide two floats.")

    # Create the CAN message string
    can_message = f"{can_address:X}#{data_hex.upper()}"

    return can_message

# Example usage:
can_address = 0x6AA
float1 = 12.34
float2 = 56.78
message = create_can_message(can_address, float1, float2)
print(f"Generated CAN Message: {message}")
