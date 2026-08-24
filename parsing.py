import re
from typing import Dict, List, Optional, Tuple

from models import Connection, Graph, LineType, ParseError, Zone

NB_DRONES = re.compile(
    r"^nb_drones:\s*(\d+)\s*$"
)

METADATA = re.compile(
    r"^\["
    r"([A-Za-z_]\w*=[^\s\]]+)"
    r"(?:\s+[A-Za-z_]\w*=[^\s\]]+)*"
    r"\]$"
)


class Parser:
    def __init__(self, arg: List[str]) -> None:
        self.arg = arg
        self.nb_drones: Optional[int] = None
        self.graph = Graph()

    def classify(self, line: str) -> Optional[LineType]:
        for line_type in LineType:
            if line.startswith(line_type.value):
                return line_type
        return None

    def metadata_validation(self, line_number: int, line_type: LineType, metadata: Dict[str, str]) -> Dict[str, object]:
        result: Dict[str, object] = {"zone": "normal","color": None,"max_drones": 1}
        allowed_zone_types = {"normal", "blocked", "restricted", "priority"}
        allowed_keys = {"zone", "color", "max_drones"}
        is_start_or_end = line_type in (LineType.START_HUB, LineType.END_HUB)

        for key, value in metadata.items():
            if key not in allowed_keys:
                raise ParseError(line_number, f"unknown metadata key: '{key}'")

            if key == "zone":
                if value not in allowed_zone_types:
                    valid = ", ".join(sorted(allowed_zone_types))
                    raise ParseError(line_number, f"invalid zone type '{value}', expected one of: {valid}")
                result[key] = value

            elif key == "color":
                result[key] = value

            elif key == "max_drones":
                if is_start_or_end:
                    continue
                try:
                    parsed = int(value)
                except ValueError:
                    raise ParseError(line_number, f"max_drones must be a positive integer, got '{value}'")
                if parsed <= 0:
                    raise ParseError(line_number, f"max_drones must be a positive integer, got '{value}'")
                result[key] = parsed

        return result

    def metadata_validation_connection(self, line_number: int, metadata: Dict[str, str]) -> Dict[str, int]:
        result = {"max_link_capacity": 1}
        allowed_keys = {"max_link_capacity"}

        for key, value in metadata.items():
            if key not in allowed_keys:
                raise ParseError(line_number, f"unknown metadata key: '{key}'")
            try:
                parsed = int(value)
            except ValueError:
                raise ParseError(line_number, f"max_link_capacity must be a positive integer, got '{value}'")
            if parsed <= 0:
                raise ParseError(line_number, f"max_link_capacity must be a positive integer, got '{value}'")
            result[key] = parsed

        return result

    def parse_metadata(self, line_number: int, text: str) -> Dict[str, str]:
        metadata: Dict[str, str] = {}
        if "[" not in text:
            return metadata
        start = text.rfind("[")
        end = text.rfind("]") + 1
        block = text[start:end]
        if not re.fullmatch(METADATA, block):
            raise ParseError(line_number, "invalid metadata format")
        block = block[1:-1]
        for token in block.split():
            if "=" not in token:
                raise ParseError(line_number, f"malformed metadata entry: '{token}'")
            key, value = token.split("=", 1)
            if not key or not value:
                raise ParseError(line_number, f"malformed metadata entry: '{token}'")
            if key in metadata:
                raise ParseError(line_number, f"duplicate metadata key: '{key}'")
            metadata[key] = value
        return metadata

    def parse_drone_count(self, line_number: int, text: str) -> int:
        match = re.fullmatch(NB_DRONES, text)
        if not match:
            raise ParseError(line_number, "invalid nb_drones line")
        drones = int(match.group(1))
        if drones <= 0:
            raise ParseError(line_number, "the drone count must be positive")
        return drones

    def parse_zone(self, line_number: int, line_type: LineType, text: str) -> List:
        metadata_raw = self.parse_metadata(line_number, text)
        metadata = self.metadata_validation(line_number, line_type, metadata_raw)

        if "[" in text:
            text = text[: text.rfind("[")].strip()
        else:
            text = text.strip()

        parts = text.split()
        if len(parts) != 4:
            raise ParseError(line_number, "missing or extra value")

        prefix = parts[0].split(":")[0].strip()
        if not re.fullmatch(r"(start_hub|hub|end_hub)", prefix):
            raise ParseError(line_number, "invalid format")

        name = parts[1].strip()
        if not re.fullmatch(r"[^\s-]+", name):
            raise ParseError(line_number, "invalid zone name")

        coords = []
        for raw_value in parts[2:4]:
            raw_value = raw_value.strip()
            if not re.fullmatch(r"-?\d+", raw_value):
                raise ParseError(line_number, "invalid position")
            coords.append(int(raw_value))

        return [prefix, name, coords[0], coords[1], metadata]

    def parse_connection(self, line_number: int, text: str) -> List:
        metadata_raw = self.parse_metadata(line_number, text)
        metadata = self.metadata_validation_connection(line_number, metadata_raw)

        if "[" in text:
            text = text[: text.rfind("[")].strip()
        else:
            text = text.strip()

        parts = text.split()
        if len(parts) != 2:
            raise ParseError(line_number, "missing or extra value")

        prefix = parts[0].split(":")[0].strip()
        if prefix != "connection":
            raise ParseError(line_number, "invalid format")

        if not re.fullmatch(r"[^\s-]+-[^\s-]+", parts[1]):
            raise ParseError(line_number, "the connection must be exactly in this format: zone1-zone2")
        name1, name2 = parts[1].split("-")

        return ["connection", name1, name2, metadata]

    def parsing(self) -> None:
        if len(self.arg) != 2:
            raise ParseError(None, "usage: python3 main.py <map_file>")

        file_path = self.arg[1]
        connection_lines: List[Tuple[int, str]] = []
        seen_first_line = False

        try:
            handle = open(file_path, "r")
        except OSError as exc:
            raise ParseError(None, f"could not read file '{file_path}': {exc}")

        with handle as f:
            for line_number, raw_line in enumerate(f, start=1):
                content = raw_line.strip()
                if not content or content.startswith("#"):
                    continue

                line_type = self.classify(content)
                if line_type is None:
                    raise ParseError(line_number, f"unrecognized line: '{content}'")

                if not seen_first_line:
                    if line_type != LineType.DRONE_COUNT:
                        raise ParseError(line_number, "the first line must declare 'nb_drones'.")
                    seen_first_line = True

                if line_type == LineType.DRONE_COUNT:
                    if self.nb_drones is not None:
                        raise ParseError(line_number, "nb_drones must be declared exactly once.")
                    self.nb_drones = self.parse_drone_count(line_number, content)

                elif line_type in (LineType.START_HUB, LineType.HUB, LineType.END_HUB):
                    zone_data = self.parse_zone(line_number, line_type, content)
                    zone = Zone(zone_data[1], (zone_data[2], zone_data[3]), line_type, zone_data[4])
                    self.graph.add_zone(zone, line_number, line_type)

                elif line_type == LineType.CONNECTION:
                    connection_lines.append((line_number, content))

        if self.nb_drones is None:
            raise ParseError(None, "missing required 'nb_drones' declaration.")

        for line_number, content in connection_lines:
            connection_data = self.parse_connection(line_number, content)
            name1, name2, metadata = (connection_data[1], connection_data[2], connection_data[3])

            zone1 = self.graph.zones.get(name1)
            zone2 = self.graph.zones.get(name2)
            if zone1 is None:
                raise ParseError(line_number, f'Unknown zone "{name1}".')
            if zone2 is None:
                raise ParseError(line_number, f'Unknown zone "{name2}".')
            self.graph.add_connection(zone1, zone2, line_number, metadata["max_link_capacity"])

        self.graph.validate()
