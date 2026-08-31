import struct
import glob
import os
import sys

mcap_files = glob.glob("/home/rp/bag/*.mcap")
if not mcap_files:
    print("No MCAP files found!")
    sys.exit(1)
latest_file = max(mcap_files, key=os.path.getmtime)
print(f"Analyzing latest MCAP file: {latest_file} (Size: {os.path.getsize(latest_file)/1024/1024:.2f} MB)")

channels = {}
msg_counts = {}
first_time = {}
last_time = {}

with open(latest_file, "rb") as f:
    # Read magic
    magic = f.read(8)
    if magic != b"\x89MCAP0\r\n":
        print("Invalid MCAP magic!")
        sys.exit(1)
    
    while True:
        header = f.read(9)
        if len(header) < 9:
            break
        opcode, length = struct.unpack("<BQ", header)
        
        if opcode == 2: # Channel
            data = f.read(length)
            if len(data) < length:
                break
            # channel record layout:
            # - id (2 bytes)
            # - schema_id (2 bytes)
            # - topic_len (4 bytes)
            # - topic (topic_len bytes)
            chan_id, schema_id, topic_len = struct.unpack("<HHI", data[:8])
            topic = data[8:8+topic_len].decode()
            channels[chan_id] = topic
            msg_counts[chan_id] = 0
        elif opcode == 3: # Message
            if length < 14:
                print(f"Error: message length {length} too small!")
                break
            msg_header = f.read(14)
            if len(msg_header) < 14:
                break
            chan_id, seq, log_time = struct.unpack("<HIQ", msg_header)
            
            if chan_id not in msg_counts:
                msg_counts[chan_id] = 0
            msg_counts[chan_id] += 1
            if chan_id not in first_time:
                first_time[chan_id] = log_time
            last_time[chan_id] = log_time
            
            # Skip the remaining data for this message
            f.seek(length - 14, 1)
        else:
            # Skip other record types (e.g. Schema, Chunk, Index, Summary)
            f.seek(length, 1)

print("\nChannel message counts and frequencies:")
for chan_id, topic in channels.items():
    count = msg_counts.get(chan_id, 0)
    if count > 1:
        duration_sec = (last_time[chan_id] - first_time[chan_id]) / 1e9
        fps = count / duration_sec if duration_sec > 0 else 0
        print(f"  Topic: {topic:<15} | Count: {count:<6} | Duration: {duration_sec:.2f}s | Freq: {fps:.2f} Hz")
    else:
        print(f"  Topic: {topic:<15} | Count: {count:<6} | Not enough data")
