import asyncio
from bleak import BleakClient, BleakScanner

# BLE UUIDs
DEVICE_NAME = "XIAO_Sense"
TX_UUID = "12345678-1234-5678-1234-56789abcdef1"  # Notify characteristic
RX_UUID = "12345678-1234-5678-1234-56789abcdef2"  # Write characteristic

expected_hashes = 4
received_hashes = []
client = None
ack_lock = asyncio.Lock()

async def send_ack():
    """Send 0x06 acknowledgment to MCU after each hash."""
    async with ack_lock:
        await asyncio.sleep(0.15)  # Small delay so MCU is ready for next
        await client.write_gatt_char(RX_UUID, bytes([0x06]))
        print("📤 Sent ACK (0x06)")

def handle_notify(sender, data: bytearray):
    """Handle incoming BLE notifications (hash data)."""
    idx = len(received_hashes) + 1
    hex_str = data.hex().upper()
    print(f"[📡 Notification {idx}] {hex_str}")
    received_hashes.append(hex_str)

    # Send ACK for next hash if not done
    if len(received_hashes) < expected_hashes:
        asyncio.create_task(send_ack())
    else:
        print("✅ All hashes received successfully!")

async def main():
    global client
    print("🔍 Scanning for device...")
    devices = await BleakScanner.discover(timeout=6)

    # Find XIAO Sense device
    target = next((d for d in devices if DEVICE_NAME in (d.name or "")), None)
    if not target:
        print("❌ Device not found.")
        return

    print(f"✅ Found {target.name} ({target.address})")

    client = BleakClient(target)

    async with client:
        print("🔗 Connected to device.")

        # Subscribe to notifications
        await client.start_notify(TX_UUID, handle_notify)

        # Step 1: Send 0x01 to enable sending
        await client.write_gatt_char(RX_UUID, bytes([0x01]))
        print("📤 Sent 0x01 (enable sending)")
        await asyncio.sleep(0.3)

        # Step 2: Send 0x05 to request first hash
        await client.write_gatt_char(RX_UUID, bytes([0x05]))
        print("📤 Sent 0x05 (request first hash)")
        print("🔁 Waiting for 4 hashes...")

        # Wait enough for MCU to send all 4 (2s each, 8s total)
        await asyncio.sleep(10)

        await client.stop_notify(TX_UUID)
        print("🛑 Notifications stopped.")
        print("🔌 Disconnecting...")

    # Show results
    print("\n=========================")
    print("🧠 HASHES RECEIVED (HEX):")
    print("=========================")
    for i, h in enumerate(received_hashes, 1):
        print(f"{i}. {h}")
    print("=========================")
    print("✅ Done.")

if __name__ == "__main__":
    asyncio.run(main())