"""
Shared trace parsing utilities for YOLO experiment analysis scripts.

Provides:
- TraceEvent: Represents a single LTTng trace event
- parse_fields(): Parses trace event field strings into dictionaries
- parse_trace_directory(): Parses a trace directory using babeltrace

Event names always include the 'anytime:' prefix for consistency with raw trace output.
"""

import subprocess


class TraceEvent:
    """Represents a single trace event"""

    def __init__(self, timestamp, event_name, fields):
        self.timestamp = timestamp
        self.event_name = event_name
        self.fields = fields

    def __repr__(self):
        return f"TraceEvent({self.timestamp}, {self.event_name}, {self.fields})"


def parse_fields(fields_str):
    """Parse the fields string into a dictionary"""
    fields = {}

    parts = []
    current = []
    in_string = False

    for char in fields_str + ',':
        if char == '"':
            in_string = not in_string
            current.append(char)
        elif char == ',' and not in_string:
            parts.append(''.join(current).strip())
            current = []
        else:
            current.append(char)

    for part in parts:
        if '=' not in part:
            continue

        key, value = part.split('=', 1)
        key = key.strip()
        value = value.strip()

        # Remove quotes from strings
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        # Try to convert to appropriate type
        else:
            try:
                if '.' in value:
                    value = float(value)
                else:
                    value = int(value)
            except ValueError:
                pass

        fields[key] = value

    return fields


def parse_trace_directory(trace_dir):
    """
    Parse a single trace directory using babeltrace.

    Returns a list of TraceEvent objects. Event names always include the
    'anytime:' prefix (e.g., 'anytime:yolo_layer_start').
    """
    print(f"  Parsing trace: {trace_dir.name}")

    try:
        result = subprocess.run(
            ['babeltrace', str(trace_dir)],
            capture_output=True,
            text=True,
            check=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"    ERROR: babeltrace not found or failed. Please install lttng-tools.")
        return []

    # Pre-filter lines to only process anytime events
    anytime_lines = [line for line in result.stdout.split(
        '\n') if 'anytime:' in line]

    events = []
    skipped = 0
    for line in anytime_lines:
        if not line.strip() or not 'anytime:' in line:
            continue

        try:
            # Extract timestamp (in brackets) - format: [HH:MM:SS.nanoseconds]
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            # Convert HH:MM:SS.nanoseconds to nanoseconds (int/float)
            parts = timestamp_str.split(':')
            try:
                if len(parts) == 3:
                    hh = int(parts[0])
                    mm = int(parts[1])
                    ss = float(parts[2])
                    seconds_total = hh * 3600 + mm * 60 + ss
                    timestamp = seconds_total * 1e9  # nanoseconds
                else:
                    timestamp = float(timestamp_str) * 1e9
            except ValueError:
                continue

            # Extract event name - everything after "anytime:" until the next colon
            event_start = line.find('anytime:')
            if event_start == -1:
                continue

            event_name_part = line[event_start:]
            event_name_end = event_name_part.find(
                ':', 8)  # Find colon after 'anytime:'
            if event_name_end == -1:
                continue

            event_name = 'anytime:' + event_name_part[8:event_name_end]

            # Extract fields (everything after the event name)
            fields_start = line.find('{')
            fields_end = line.rfind('}')
            if fields_start == -1 or fields_end == -1:
                fields = {}
            else:
                fields_str = line[fields_start+1:fields_end].strip()
                fields = parse_fields(fields_str)

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception:
            skipped += 1
            continue

    if skipped > 0:
        print(f"    WARNING: Skipped {skipped} unparseable lines")
    print(f"    Parsed {len(events)} events")
    return events
