import sys
from mcap.reader import make_reader

with open("bag/2026_7_14_21_46_10.mcap", "rb") as f:
    reader = make_reader(f)
    print("Topics:")
    summary = reader.get_summary()
    for id, channel in summary.channels.items():
        print(f"  {channel.topic} ({channel.message_encoding})")
