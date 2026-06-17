#!/usr/bin/env python3
import sys
import os
import sqlite3
import struct
import csv
from pathlib import Path

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 decode_bag.py <path_to_bag_dir_or_db3_file> [output_dir]")
        sys.exit(1)
        
    input_path = Path(sys.argv[1]).resolve()
    if input_path.is_dir():
        db3_files = list(input_path.glob("*.db3"))
        if not db3_files:
            zstd_files = list(input_path.glob("*.db3.zstd"))
            if zstd_files:
                zstd_file = zstd_files[0]
                decompressed_file = input_path / zstd_file.name[:-5]  # Remove .zstd
                print(f"Decompressing {zstd_file.name} using zstd...")
                ret = os.system(f"zstd -d '{zstd_file}' -o '{decompressed_file}'")
                if ret != 0:
                    print("Error decompressing using zstd.")
                    sys.exit(1)
                db3_file = decompressed_file
            else:
                print(f"No .db3 or .db3.zstd files found in {input_path}")
                sys.exit(1)
        else:
            db3_file = db3_files[0]
    else:
        db3_file = input_path

    if len(sys.argv) > 2:
        out_dir = Path(sys.argv[2]).resolve()
    else:
        out_dir = db3_file.parent
        
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_csv_path = out_dir / "raw_messages.csv"
    
    print(f"Connecting to SQLite database: {db3_file}")
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()
    
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}
    print(f"Found {len(topics)} topics:")
    for tid, (name, ttype) in topics.items():
        print(f"  ID {tid}: {name} ({ttype})")
        
    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC")
    rows = cursor.fetchall()
    print(f"Reading {len(rows)} messages...")
    
    decoded_count = 0
    raw_data = []
    
    for topic_id, timestamp, data in rows:
        if topic_id not in topics:
            continue
        topic_name, topic_type = topics[topic_id]
        
        # We only support std_msgs/msg/Float64 or general float64 CDR serialization
        if len(data) == 12:
            val, = struct.unpack("<d", data[4:])
            raw_data.append((timestamp, topic_name, val))
            decoded_count += 1
            
    print(f"Decoded {decoded_count} Float64 messages.")
    
    print(f"Saving raw messages to {raw_csv_path}")
    with open(raw_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "topic", "value"])
        writer.writerows(raw_data)
        
    try:
        import pandas as pd
        print("Pandas is available. Creating aligned pivoted CSV...")
        df = pd.DataFrame(raw_data, columns=["timestamp_ns", "topic", "value"])
        df["topic"] = df["topic"].apply(lambda x: x.split('/')[-1])
        
        pivoted = df.pivot_table(index="timestamp_ns", columns="topic", values="value")
        pivoted = pivoted.sort_index()
        pivoted_aligned = pivoted.ffill().bfill()
        
        start_time = pivoted_aligned.index[0]
        pivoted_aligned.insert(0, 'time_s', (pivoted_aligned.index - start_time) / 1e9)
        
        pivoted_csv_path = out_dir / "pivoted_messages.csv"
        pivoted_aligned.to_csv(pivoted_csv_path)
        print(f"Saved pivoted aligned messages to {pivoted_csv_path}")
    except ImportError:
        print("Pandas is not installed. Aligned pivoted CSV skipped.")
        
    conn.close()
    print("Done!")

if __name__ == "__main__":
    main()
