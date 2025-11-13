import asyncio
from bleak import BleakClient, BleakScanner
import spotipy
from spotipy.oauth2 import SpotifyOAuth
import requests
import time
import logging
#import aioconsole

expected_hashes = 4
received_hashes = []
client = None
ack_lock = asyncio.Lock()
MOOD_MAP = {}

CLIENT_ID = "5be1bfcb26f048ccb5ba5eec281b214a"
CLIENT_SECRET = "d12d00fad73f438d8bb0dd2b3f5630ed"
REDIRECT_URI = "http://127.0.0.1:8000/callback"

RAPIDAPI_URL = "https://track-analysis.p.rapidapi.com/pktx/analysis"
RAPIDAPI_KEY = "acecc382edmshfa46558ae5ad043p19adf6jsnbb00f063b71a"
RAPIDAPI_HEADERS = {
    "x-rapidapi-key": RAPIDAPI_KEY,
    "x-rapidapi-host": "track-analysis.p.rapidapi.com"
}

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
TX_UUID = "12345678-1234-5678-1234-56789abcdef1"
RX_UUID = "12345678-1234-5678-1234-56789abcdef2"

logging.basicConfig(
    filename="mood_ble_spotify.log",
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s"
)

sp = spotipy.Spotify(auth_manager=SpotifyOAuth(
    client_id=CLIENT_ID,
    client_secret=CLIENT_SECRET,
    redirect_uri=REDIRECT_URI,
    scope="user-read-playback-state user-modify-playback-state playlist-read-private user-library-read",
    cache_path="token.txt"
))

print("\nAuthenticating to Spotify...")
user = sp.current_user()
print(f"Logged in as: {user['display_name']}\n")
logging.info(f"Logged into Spotify as {user['display_name']}")

async def send_ack():
    """Send ACK (0x06) to device with small delay."""
    global client
    async with ack_lock:
        await asyncio.sleep(2.1)
        try:
            if client and client.is_connected:
                await client.write_gatt_char(RX_UUID, bytes([0x06]))
                print("Sent ACK (0x06)")
        except Exception as e:
            logging.warning(f"Failed to send ACK: {e}")

def debug_print_hash_list():
    print("Received hashes (short):")
    for i, h in enumerate(received_hashes, start=1):
        print(f"  [{i}] {h[:16]}...")
    print()

def handle_hash_notify(sender, data: bytearray):
    """Receive and collect 32-byte hashes from device."""
    try:
        if len(received_hashes) >= expected_hashes:
            return
        hex_str = data.hex().upper()
        if hex_str not in received_hashes:
            received_hashes.append(hex_str)
            print(f"[Hash {len(received_hashes)}] {hex_str}")
            logging.info(f"Received hash #{len(received_hashes)}: {hex_str[:32]}...")
            asyncio.create_task(send_ack())
    except Exception as e:
        logging.error(f"Error in handle_hash_notify: {e}")

def extract_playlist_id(url_or_id: str) -> str:
    """Extract playlist ID from URL or raw ID."""
    if "playlist/" in url_or_id:
        return url_or_id.split("playlist/")[-1].split("?")[0].strip()
    return url_or_id.strip()

tracks = []
while True:
    playlist_url = input("Enter Spotify playlist URL or ID: ").strip()
    playlist_id = extract_playlist_id(playlist_url)

    try:
        results = sp.playlist_items(playlist_id, additional_types=["track"])
        if not results or "items" not in results:
            raise ValueError("No tracks found or invalid playlist data.")

        tracks.clear()
        while results:
            for item in results.get("items", []):
                track = item.get("track")
                if track and track.get("id"):
                    tracks.append({
                        "id": track["id"],
                        "name": track["name"],
                        "artist": track["artists"][0]["name"]
                    })
            results = sp.next(results) if results.get("next") else None

        print(f"\nLoaded {len(tracks)} tracks from playlist.")
        logging.info(f"Loaded {len(tracks)} tracks from playlist {playlist_id}")
        break

    except Exception as e:
        print("\nInvalid or inaccessible playlist.")
        print("Please try again with a valid Spotify playlist link or ID.\n")
        logging.warning(f"Playlist input failed: {e}")
        time.sleep(1.5)

moods = {"Happy": [], "Sad": [], "Energetic": [], "Calm": []}
print("Analyzing songs for mood...\n(This may take a few minutes.)\n")
logging.info("Started mood analysis using Track Analysis API")

request_count = 0
for t in tracks[:15]:
    request_count += 1
    params = {"song": t["name"], "artist": t["artist"]}
    start_time = time.time()
    try:
        response = requests.get(RAPIDAPI_URL, headers=RAPIDAPI_HEADERS, params=params)
        elapsed = time.time() - start_time
        logging.info(f"Request #{request_count} | '{t['name']}' by {t['artist']} | Status: {response.status_code} | Time: {elapsed:.2f}s")

        if response.status_code == 429:
            continue
        if response.status_code == 200:
            data = response.json()
            happiness = data.get("happiness", 0)
            energy = data.get("energy", 0)
            if happiness > 60 and energy > 60:
                mood = "Happy"
            elif happiness < 40 and energy < 40:
                mood = "Sad"
            elif energy > 75:
                mood = "Energetic"
            else:
                mood = "Calm"
            moods[mood].append(t)
    except Exception as e:
        logging.error(f"Error analyzing '{t['name']}' — {e}")
    time.sleep(6)

print("\nFinished mood classification.")
print("=" * 40)
for mood_name, songs in moods.items():
    print(f"\n{mood_name} songs ({len(songs)}):")
    if songs:
        for s in songs[:5]:
            print(f"   • {s['name']} — {s['artist']}")
    else:
        print("   (none)")
print("\nReady for BLE mood detection...\n")

async def send_mode_and_handle_calibration(client_obj, mode_byte):
    """Handles calibration send + wait for confirmation."""
    calib_event = asyncio.Event()

    def calib_notify(sender, data: bytearray):
        if len(data) == 1 and data[0] == 0x0C:
            calib_event.set()
        else:
            try:
                txt = data.decode(errors="ignore").upper()
                if "CALIBR" in txt:
                    print("Calibrating...   ", end="\r")
            except Exception:
                pass

    await client_obj.start_notify(TX_UUID, calib_notify)

    try:
        await client_obj.write_gatt_char(RX_UUID, bytes([mode_byte]))
        print(f"Sent mode command 0x{mode_byte:02X}")
    except Exception as e:
        print(f"Failed to send mode command: {e}")
        await client_obj.stop_notify(TX_UUID)
        return False

    if mode_byte == 0x0C:
        print("Waiting for calibration to complete...")
        try:
            await asyncio.wait_for(calib_event.wait(), timeout=60.0)
            print("\nCalibration complete!")
            await client_obj.stop_notify(TX_UUID)
            return True
        except asyncio.TimeoutError:
            print("\nCalibration timeout.")
            await client_obj.stop_notify(TX_UUID)
            return False
    else:
        await asyncio.sleep(0.1)
        await client_obj.stop_notify(TX_UUID)
        return True

async def show_mood_songs(client_param, mood):
    mood = mood.capitalize()
    target_mood = mood
    if mood == "Sad":
        if moods["Happy"]: target_mood = "Happy"
        elif moods["Energetic"]: target_mood = "Energetic"
        else: target_mood = "Calm"

    if not moods.get(target_mood):
        print(f"\nNo songs found for mood '{target_mood}'.")
    else:
        print(f"\nSuggested '{target_mood}' songs:")
        for s in moods[target_mood][:5]:
            print(f"   • {s['name']} — {s['artist']}")

mood_received_once = False

async def play_songs_for_mood(client_param, mood):
    """Play up to 5 songs for the given mood using Spotify (simple version)."""

    mood = mood.capitalize()
    
    if mood == "Sad":
        print("\nMood is Sad → Playing Calm songs instead.")
        mood = "Calm"

    print(f"\nStarting playback for mood: {mood}")

    if not moods.get(mood) or len(moods[mood]) == 0:
        print(f"No songs for mood '{mood}'. Skipping playback.")
        return

    selected_tracks = moods[mood][:5]
    uris = [f"spotify:track:{t['id']}" for t in selected_tracks]

    try:
        sp.start_playback(uris=uris)
        print(f"Playing {len(uris)} songs for mood '{mood}'...")

        if client_param and client_param.is_connected:
            await asyncio.sleep(0.3)
            await client_param.write_gatt_char(RX_UUID, bytes([0x01]))
            print("Sent 0x01 to MCU to request new mood hash.")

    except Exception as e:
        print(f"Error playing songs: {e}")

async def handle_mood_notify(client_param, sender, data):
    global mood_received_once
    if mood_received_once:
        return

    try:
        hex_str = data.hex().upper()

        if hex_str in MOOD_MAP:
            mood = MOOD_MAP[hex_str]
            print(f"\nBLE mood detected: {mood}")

            await show_mood_songs(client_param, mood)

            await play_songs_for_mood(client_param, mood)

            mood_received_once = True

            try:
                if client_param and client_param.is_connected:
                    await asyncio.sleep(0.3)
                    await client_param.write_gatt_char(RX_UUID, bytearray([0x04]))
                    print("Sent 0x04 (Stop notifications).")
            except Exception as e:
                print(f"Failed to send 0x04: {e}")

        else:
            print(f"\nUnknown mood hash: {hex_str[:16]}...")

    except Exception as e:
        logging.error(f"Error in handle_mood_notify: {e}")

async def main():
    global client, received_hashes, MOOD_MAP

    print("\nScanning for device 'XIAO_Sense'...")
    device = await BleakScanner.find_device_by_name("XIAO_Sense", timeout=10.0)
    if not device:
        print("Could not find 'XIAO_Sense'. Make sure it is advertising and in range.")
        return

    client = BleakClient(device)
    try:
        async with client:
            if not client.is_connected:
                print("Failed to connect.")
                return
            print(f"Connected to device: {device.address}")

            while True:
                print("\nSelect mode:")
                print("1. Calibration")
                print("2. Start")
                user_choice = input("Enter choice (1/2): ").strip()

                if user_choice == "1":
                    await send_mode_and_handle_calibration(client, 0x0C)
                    continue

                elif user_choice == "2":
                    await client.write_gatt_char(RX_UUID, bytes([0x01]))
                    print("Sent Start (0x01). Proceeding to hash sync...")
                    break
                else:
                    print("Invalid choice. Try again.")

            received_hashes = []
            await client.start_notify(TX_UUID, handle_hash_notify)
            await client.write_gatt_char(RX_UUID, bytes([0x05]))  # request hashes

            start_ts = time.time()
            while len(received_hashes) < expected_hashes and (time.time() - start_ts) < 20:
                await asyncio.sleep(0.2)

            await client.stop_notify(TX_UUID)

            if len(received_hashes) < expected_hashes:
                print(f"Timeout while waiting for hashes. Got {len(received_hashes)} / {expected_hashes}.")
                return

            print("\nHash sync complete — mapping hashes to moods...\n")
            MOOD_MAP = {
                received_hashes[0].upper(): "Happy",
                received_hashes[1].upper(): "Sad",
                received_hashes[2].upper(): "Calm",
                received_hashes[3].upper(): "Energetic"
            }
            debug_print_hash_list()

            await client.start_notify(TX_UUID, lambda s, d: asyncio.create_task(handle_mood_notify(client, s, d)))
            await client.write_gatt_char(RX_UUID, bytes([0x01]))  # start mood detection

            # Keep alive
            while True:
                await asyncio.sleep(1)

    except Exception as e:
        logging.error(f"Main BLE loop exception: {e}")
        print(f"BLE error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected — sending 0x0A (Sleep)...")
        
        try:
            sp.pause_playback()
            print("Spotify playback paused.")
        except Exception as e:
            print(f"Could not pause Spotify playback: {e}")

        async def send_sleep():
            try:
                # Reuse connected client if available
                if 'client' in globals() and client and client.is_connected:
                    await client.write_gatt_char(RX_UUID, bytes([0x0A]))
                    print("Sent 0x0A (Sleep) to connected device.")
                    return

                # Otherwise try to find device again
                print("Device not connected, scanning to send sleep...")
                device = await BleakScanner.find_device_by_name("XIAO_Sense", timeout=5.0)
                if not device:
                    print("Device not found — skipping sleep command.")
                    return

                async with BleakClient(device) as temp_client:
                    if temp_client.is_connected:
                        await temp_client.write_gatt_char(RX_UUID, bytes([0x0A]))
                        print("Sent 0x0A (Sleep) to device.")
                        await asyncio.sleep(0.3)
                    else:
                        print("Could not connect to device to send sleep.")
            except Exception as e:
                print(f"Could not send sleep command: {e}")

        # Create a new short-lived event loop just for the sleep send
        import asyncio
        asyncio.run(send_sleep())

        print("Device sleeping. Program terminated.")