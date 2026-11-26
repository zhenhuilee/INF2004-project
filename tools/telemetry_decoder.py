import struct
import sys

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Missing paho-mqtt.")
    print("Install with: python3 -m pip install paho-mqtt")
    sys.exit(1)


# ============================================================
# INF2004 Buddy1 MQTT Telemetry Decoder
# ============================================================

BROKER = "10.82.83.75"
PORT = 1883
TOPIC = "buddy1/test"


MOTION_MODES = {
    0: "IDLE",
    1: "MOVING",
    2: "TURNING",
    3: "DRIVING",
}

INSTRUCTIONS = {
    0: "NONE",
    1: "LEFT",
    2: "RIGHT",
    3: "STRAIGHT",
    4: "UTURN",
}

TERRAINS = {
    0: "FLAT",
    1: "UPHILL",
    2: "PEAK",
    3: "DOWNHILL",
}


def decode_motor(data):
    # 02 | mode | left speed | right speed | left enc | right enc
    if len(data) != 18:
        return f"[MOTOR] invalid packet length={len(data)}"

    mode = data[1]

    left_speed = struct.unpack_from("<i", data, 2)[0]
    right_speed = struct.unpack_from("<i", data, 6)[0]

    left_encoder = struct.unpack_from("<i", data, 10)[0]
    right_encoder = struct.unpack_from("<i", data, 14)[0]

    mode_name = MOTION_MODES.get(mode, f"UNKNOWN({mode})")

    return (
        f"[MOTOR] mode={mode_name} "
        f"left={left_speed}mm/s "
        f"right={right_speed}mm/s "
        f"encL={left_encoder} "
        f"encR={right_encoder}"
    )

def decode_line(data):
    # 03 | line_status
    #
    # bit 0 = left IR
    # bit 1 = center IR
    # bit 2 = right IR

    if len(data) != 2:
        return f"[LINE] invalid packet length={len(data)}"

    status = data[1]

    left = 1 if (status & 0x01) else 0
    center = 1 if (status & 0x02) else 0
    right = 1 if (status & 0x04) else 0

    return (
        f"[LINE] "
        f"L={left} "
        f"C={center} "
        f"R={right}"
    )

def decode_barcode(data):
    # 04 | decoded ASCII character | action
    #
    # decoded:
    #   A-Z = successfully decoded Code39 character
    #   0   = no currently stored decoded barcode
    #
    # action:
    #   0 = NONE
    #   1 = LEFT
    #   2 = RIGHT
    #   3 = STRAIGHT
    #   4 = UTURN

    if len(data) != 3:
        return f"[BARCODE] invalid packet length={len(data)}"

    decoded = data[1]
    action = data[2]

    action_name = INSTRUCTIONS.get(
        action,
        f"UNKNOWN({action})"
    )

    if ord("A") <= decoded <= ord("Z"):
        character = chr(decoded)
    else:
        character = "NONE"

    return (
        f"[BARCODE] decoded={character} "
        f"action={action_name}"
    )


def decode_hump(data):
    # 05 | status | current height | highest height
    if len(data) != 4:
        return f"[HUMP] invalid packet length={len(data)}"

    status = data[1]
    current_height = data[2]
    highest_height = data[3]

    terrain = status & 0x03
    fault = bool(status & 0x04)
    accelerating = bool(status & 0x08)
    turning = bool(status & 0x10)
    lifted = bool(status & 0x20)

    terrain_name = TERRAINS.get(
        terrain,
        f"UNKNOWN({terrain})"
    )

    return (
        f"[HUMP] terrain={terrain_name} "
        f"current={current_height}mm "
        f"highest={highest_height}mm "
        f"fault={'YES' if fault else 'NO'} "
        f"accelerating={'YES' if accelerating else 'NO'} "
        f"turning={'YES' if turning else 'NO'} "
        f"lifted={'YES' if lifted else 'NO'}"
    )


def decode_obstacle(data):
    # 06 | status | distance u16 | width u16 | span u16 | bypass u16
    if len(data) != 10:
        return f"[OBSTACLE] invalid packet length={len(data)}"

    status = data[1]

    blocked = bool(status & 0x01)
    valid = bool(status & 0x02)

    distance_raw = struct.unpack_from("<H", data, 2)[0]
    width_raw = struct.unpack_from("<H", data, 4)[0]
    span_raw = struct.unpack_from("<H", data, 6)[0]
    bypass_raw = struct.unpack_from("<H", data, 8)[0]

    distance = distance_raw / 10.0
    width = width_raw / 10.0
    span = span_raw / 10.0
    bypass = bypass_raw / 10.0

    return (
        f"[OBSTACLE] detected={'YES' if blocked else 'NO'} "
        f"valid={'YES' if valid else 'NO'} "
        f"distance={distance:.1f}cm "
        f"width={width:.1f}cm "
        f"span={span:.1f}deg "
        f"bypass={bypass:.1f}cm"
    )


def decode_distance(data):
    # 07 | odometer int32
    if len(data) != 5:
        return f"[DISTANCE] invalid packet length={len(data)}"

    distance_mm = struct.unpack_from("<i", data, 1)[0]

    return (
        f"[DISTANCE] {distance_mm} mm "
        f"({distance_mm / 1000.0:.3f} m)"
    )


def decode_packet(data):
    if not data:
        return "[UNKNOWN] empty packet"

    # Heartbeat is ASCII: "hb 0", "hb 1", etc.
    if data.startswith(b"hb "):
        try:
            return f"[HEARTBEAT] {data.decode('ascii')}"
        except UnicodeDecodeError:
            return f"[HEARTBEAT] {data.hex(' ')}"

    packet_type = data[0]

    if packet_type == 0x02:
        return decode_motor(data)

    if packet_type == 0x03:
        return decode_line(data)

    if packet_type == 0x04:
        return decode_barcode(data)

    if packet_type == 0x05:
        return decode_hump(data)

    if packet_type == 0x06:
        return decode_obstacle(data)

    if packet_type == 0x07:
        return decode_distance(data)

    return f"[UNKNOWN 0x{packet_type:02X}] {data.hex(' ')}"


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker")
        print(f"Subscribed to: {TOPIC}")
        print("-" * 80)
        client.subscribe(TOPIC)
    else:
        print(f"MQTT connection failed: {reason_code}")


def on_message(client, userdata, msg):
    print(decode_packet(msg.payload))


client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2
)

client.on_connect = on_connect
client.on_message = on_message

print("INF2004 Telemetry Decoder")
print(f"Broker: {BROKER}:{PORT}")
print(f"Topic:  {TOPIC}")
print()

try:
    client.connect(BROKER, PORT, 60)
    client.loop_forever()

except KeyboardInterrupt:
    print("\nStopped.")

except Exception as e:
    print(f"\nERROR: {e}")

