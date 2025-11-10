#!/usr/bin/env python3
"""Test timestamp parsing"""

line = "[00:39:21.219180043] (+?.?????????) enzo anytime:anytime_server_init: { cpu_id = 16 }"

ts_start = line.find('[')
ts_end = line.find(']')
timestamp_str = line[ts_start+1:ts_end]
print(f'Timestamp string: "{timestamp_str}"')

# Try to convert
try:
    timestamp = float(timestamp_str.replace(':', ''))
    print(f'Timestamp float: {timestamp}')
except Exception as e:
    print(f'Error: {e}')

# Check what we get
no_colons = timestamp_str.replace(':', '')
print(f'After removing colons: "{no_colons}"')

# The correct way - convert HH:MM:SS.nanos to seconds
parts = timestamp_str.split(':')
if len(parts) == 3:
    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2])
    total_seconds = hours * 3600 + minutes * 60 + seconds
    print(f'Correctly converted to seconds: {total_seconds}')
