import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "XIAO_Sense"
TX_UUID = "12345678-1234-5678-1234-56789abcdef1"  # Notify
RX_UUID = "12345678-1234-5678-1234-56789abcdef2"  # Write

expected_hashes = 4
received_hashes = []
client = None
ack_lock = asyncio.Lock()

async def send_ack():
    """Send 0x06 acknowledgment to MCU after each hash."""
    async with ack_lock:
        await asyncio.sleep(0.1)  # Small delay (ensures MCU ready)
        await client.write_gatt_char(RX_UUID, bytes([0x06]))
        print("📤 Sent ACK (0x06)")

def handle_notify(sender, data: bytearray):
    """Handle incoming BLE notifications (32-byte hashes)."""
    idx = len(received_hashes) + 1
    hex_str = data.hex().upper()
    print(f"[📡 Notification {idx}] {hex_str}")
    received_hashes.append(hex_str)

    if len(received_hashes) < expected_hashes:
        asyncio.create_task(send_ack())
    else:
        print("✅ All hashes received!")

async def main():
    global client
    print("🔍 Scanning for device...")
    devices = await BleakScanner.discover(timeout=6)

    target = next((d for d in devices if DEVICE_NAME in (d.name or "")), None)
    if not target:
        print("❌ Device not found.")
        return

    print(f"✅ Found {target.name} ({target.address})")

    client = BleakClient(target)

    async with client:
        print("🔗 Connected to device.")

        await client.start_notify(TX_UUID, handle_notify)

        # Step 1: Ask MCU to send first hash
        await asyncio.sleep(0.3)
        await client.write_gatt_char(RX_UUID, bytes([0x05]))
        print("📤 Sent 0x05 (request first hash)")
        print("🔁 Waiting for hashes...")

        # Wait enough time for all 4 exchanges (4x ~0.3s)
        await asyncio.sleep(5)

        await client.stop_notify(TX_UUID)
        print("🛑 Notifications stopped.")
        print("🔌 Disconnecting...")

    # Display results
    print("\n=========================")
    print("🧠 HASHES RECEIVED (HEX):")
    print("=========================")
    for i, h in enumerate(received_hashes, 1):
        print(f"{i}. {h}")
    print("=========================")
    print("✅ Done.")

if __name__ == "__main__":
    asyncio.run(main())