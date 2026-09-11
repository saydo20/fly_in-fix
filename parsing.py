import re
from typing import Dict, List, Optional, Tuple
from models import Graph, LineType, ParseError, Zone

NB_DRONES = re.compile(
    r"^nb_drones:\s*(\d+)\s*$"
)

METADATA = re.compile(
    r"^\["
    r"(?:\s*[A-Za-z_]\w*=[^\s\]]+)*"
    r"\]$"
)


class Parser:
    """Parse and validate map files into graph and simulation structures."""

    def __init__(self, arg: List[str]) -> None:
        """Initialize parser with command line arguments.

        Args:
            arg: List of command-line arguments (sys.argv).
        """
        self.arg = arg
        self.nb_drones: int = -1
        self.graph = Graph()

    def classify(self, line: str) -> Optional[LineType]:
        """Classify line prefix into corresponding LineType enum.

        Args:
            line: Raw text line to check.

        Returns:
            Matching LineType enum or None if unrecognized.
        """
        for line_type in LineType:
            if line.startswith(line_type.value):
                return line_type
        return None

    def metadata_validation(
            self, line_number: int,
            line_type: LineType,
            metadata: Dict[str, str]
    ) -> Dict[str, object]:
        """Validate zone metadata keys and values against allowed rules.

        Args:
            line_number: Line number for error reporting.
            line_type: Zone category prefix.
            metadata: Dictionary of raw parsed key-value pairs.

        Returns:
            Dictionary with validated and typed metadata attributes.

        Raises:
            ParseError: If unknown key, invalid zone type, or bad max_drones.
        """
        result: Dict[str, object] = {"zone": "normal",
                                     "color": None,
                                     "max_drones": 1}
        allowed_zone_types = {"normal", "blocked", "restricted", "priority"}
        allowed_keys = {"zone", "color", "max_drones"}
        is_start_or_end = line_type in (LineType.START_HUB, LineType.END_HUB)

        for key, value in metadata.items():
            if key not in allowed_keys:
                raise ParseError(line_number, f"unknown metadata key: '{key}'")

            if key == "zone":
                if value not in allowed_zone_types:
                    valid = ", ".join(allowed_zone_types)
                    raise ParseError(line_number, "invalid zone type"
                                     f"'{value}', expected one of: {valid}")
                result[key] = value

            elif key == "color":
                result[key] = value

            elif key == "max_drones":
                if is_start_or_end:
                    continue
                try:
                    parsed = int(value)
                except ValueError:
                    raise ParseError(
                        line_number, "max_drones must be an integer,"
                        f" got '{value}'")
                if parsed <= 0:
                    raise ParseError(line_number, "max_drones must be a "
                                     f"positive integer, got '{value}'")
                result[key] = parsed

        return result

    def metadata_validation_connection(
            self,
            line_number: int,
            metadata: Dict[str, str]
    ) -> Dict[str, int]:
        """Validate connection metadata keys and capacity values.

        Args:
            line_number: Line number for error reporting.
            metadata: Dictionary of raw parsed key-value pairs.

        Returns:
            Dictionary with validated max_link_capacity.

        Raises:
            ParseError: If unknown key or non-positive capacity.
        """
        result = {"max_link_capacity": 1}
        allowed_keys = {"max_link_capacity"}

        for key, value in metadata.items():
            if key not in allowed_keys:
                raise ParseError(line_number, f"unknown metadata key: '{key}'")
            try:
                parsed = int(value)
            except ValueError:
                raise ParseError(line_number,
                                 "max_link_capacity must be a "
                                 f"positive integer, got '{value}'")
            if parsed <= 0:
                raise ParseError(line_number, "max_link_capacity must be a "
                                 f"positive integer, got '{value}'")
            result[key] = parsed

        return result

    def parse_metadata(self,
                       line_number: int,
                       text: str
                       ) -> Tuple[Dict[str, str], str]:
        """Extract and parse optional bracketed metadata from line text.

        Args:
            line_number: Line number for error reporting.
            text: Line content potentially containing metadata brackets.

        Returns:
            Tuple containing parsed metadata dict and remaining line text.

        Raises:
            ParseError: If bracket format is malformed or duplicate keys found.
        """
        metadata: Dict[str, str] = {}
        text_stripped = text.strip()

        if text_stripped.endswith("]") and "[" in text_stripped:
            start_idx = text_stripped.rfind("[")
            if start_idx != -1:
                block = text_stripped[start_idx:]
                inner = block[1:-1].strip()

                if not re.fullmatch(METADATA, block):
                    raise ParseError(line_number, "invalid metadata format")

                for token in inner.split():
                    key, value = token.split("=", 1)
                    if key in metadata:
                        raise ParseError(line_number,
                                         f"duplicate metadata key: '{key}'")
                    metadata[key] = value

                return metadata, text_stripped[:start_idx].strip()

        return metadata, text_stripped

    def parse_drone_count(self, line_number: int, text: str) -> int:
        """Parse the drone count from the nb_drones declaration line.

        Args:
            line_number: Line number for error reporting.
            text: Raw text line declaring drone count.

        Returns:
            Positive integer number of drones.

        Raises:
            ParseError: If line format is invalid or count is non-positive.
        """
        match = re.fullmatch(NB_DRONES, text)
        if not match:
            raise ParseError(line_number, "invalid nb_drones line")
        drones = int(match.group(1))
        if drones <= 0:
            raise ParseError(line_number, "the drone count must be positive")
        return drones

    def parse_zone(self,
                   line_number: int,
                   line_type: LineType,
                   text: str
                   ) -> List:
        """Parse a zone declaration line into its structural components.

        Args:
            line_number: Line number for error reporting.
            line_type: Category prefix of the zone.
            text: Raw line text describing the zone.

        Returns:
            List containing prefix, name, x, y, and validated metadata.

        Raises:
            ParseError: If syntax, coordinates, or name are invalid.
        """
        metadata_raw, text = self.parse_metadata(line_number, text)
        metadata = self.metadata_validation(
            line_number, line_type, metadata_raw
        )

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
        """Parse a connection declaration line linking two zones.

        Args:
            line_number: Line number for error reporting.
            text: Raw line text describing the connection.

        Returns:
            List containing prefix, zone1 name, zone2 name, and metadata.

        Raises:
            ParseError: If syntax or connection format is invalid.
        """
        metadata_raw, text = self.parse_metadata(line_number, text)
        metadata = self.metadata_validation_connection(
            line_number, metadata_raw
        )

        parts = text.split()
        if len(parts) != 2:
            raise ParseError(line_number, "missing or extra value")

        prefix = parts[0].split(":")[0].strip()
        if prefix != "connection":
            raise ParseError(line_number, "invalid format")

        if not re.fullmatch(r"[^\s-]+-[^\s-]+$", parts[1]):
            raise ParseError(line_number,
                             "the connection must be exactly in"
                             " this format: zone1-zone2")
        name1, name2 = parts[1].split("-")

        return ["connection", name1, name2, metadata]

    def parsing(self) -> None:
        """Read and parse the input map file, constructing the full graph.

        Raises:
            ParseError: If arguments are invalid, file cannot be read,
                or syntax errors are present.
        """
        if len(self.arg) != 2:
            raise ParseError(None, "usage: python3 main.py <map_file>")

        file_path = self.arg[1]
        connection_lines: List[Tuple[int, str]] = []
        seen_first_line = False

        try:
            with open(file_path, "r") as f:
                for line_number, raw_line in enumerate(f, start=1):
                    content = raw_line.strip()
                    if not content or content.startswith("#"):
                        continue

                    line_type = self.classify(content)
                    if line_type is None:
                        raise ParseError(line_number,
                                         f"unrecognized line: '{content}'")

                    if not seen_first_line:
                        if line_type != LineType.DRONE_COUNT:
                            raise ParseError(
                                line_number, "the first line must declare "
                                "'nb_drones'.")
                        seen_first_line = True

                    if line_type == LineType.DRONE_COUNT:
                        if self.nb_drones != -1:
                            raise ParseError(
                                line_number, "nb_drones must be declared"
                                " exactly once.")
                        self.nb_drones = self.parse_drone_count(
                            line_number, content
                        )

                    elif line_type in (
                        LineType.START_HUB, LineType.HUB, LineType.END_HUB
                    ):
                        zone_data = self.parse_zone(
                            line_number, line_type, content
                        )
                        zone = Zone(
                            zone_data[1],
                            (zone_data[2],
                             zone_data[3]),
                            line_type,
                            zone_data[4])
                        self.graph.add_zone(zone, line_number, line_type)

                    elif line_type == LineType.CONNECTION:
                        connection_lines.append((line_number, content))
        except OSError as exc:
            raise ParseError(None, f"could not read file '{file_path}': {exc}")

        if self.nb_drones is None:
            raise ParseError(None, "missing required 'nb_drones' declaration.")

        for line_number, content in connection_lines:
            connection_data = self.parse_connection(line_number, content)
            name1, name2, metadata = (
                connection_data[1], connection_data[2], connection_data[3]
            )

            zone1 = self.graph.zones.get(name1)
            zone2 = self.graph.zones.get(name2)
            if zone1 is None:
                raise ParseError(line_number, f'Unknown zone "{name1}".')
            if zone2 is None:
                raise ParseError(line_number, f'Unknown zone "{name2}".')
            self.graph.add_connection(
                zone1, zone2, line_number, metadata["max_link_capacity"]
            )

        self.graph.validate()
