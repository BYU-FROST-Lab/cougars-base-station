from digi.xbee.devices import XBeeDevice
import time
import threading

PORT = "/dev/ttyUSB1"  # Update with your port
BAUD_RATE = 9600
MESSAGE = "STATUS"
TRANSMIT_INTERVAL = 3  # Changed to 3 seconds between transmissions

# Shared flag to signal when a response is received
response_received = False


def transmit_data(device):
    """Thread function for periodic data transmission"""
    global response_received

    try:
        while not response_received:
            # print(f"Broadcasting: {MESSAGE}")
            device.send_data_broadcast(MESSAGE)

            # Check for response every second instead of sleeping for the full interval
            for i in range(TRANSMIT_INTERVAL):
                if response_received:
                    print("Response received. Stopping transmission.")
                    break
                time.sleep(1)

    except KeyboardInterrupt:
        return
    print("Transmitter stopped")


def main():
    global response_received_
    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        # Receive callback handler
        def data_received_callback(message):
            global response_received
            addr = message.remote_device.get_64bit_addr()
            data = message.data.decode()
            print(f"Received from {addr}: {data}")

            # Set the flag to stop transmitting
            if (
                data and not data.strip() == MESSAGE
            ):  # Ensure we're not just seeing our own message
                response_received = True
                print("Response detected. Will stop sending requests.")

        device.add_data_received_callback(data_received_callback)

        # Start transmit thread
        transmitter = threading.Thread(target=transmit_data, args=(device,))
        transmitter.daemon = True
        transmitter.start()

        print("Listening for data + transmitting every 3 seconds until response...")
        input("Press Enter to exit\n")  # Keep main thread alive

    finally:
        if device.is_open():
            device.close()
        print("Device closed")


if __name__ == "__main__":
    main()
