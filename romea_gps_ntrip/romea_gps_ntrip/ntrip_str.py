from dataclasses import dataclass
# flake8: noqa Q000

@dataclass
class NtripSTR:
  mountpoint:     str
  identifier:     str
  format:         str
  format_details: str
  carrier:        int
  nav_system:     str
  network:        str
  country:        str
  latitude:       float
  longitude:      float
  nmea:           bool
  solution:       int
  version:        str

  def from_string(data):
    fields = data.split(';')[1:]
    return NtripSTR(
      mountpoint=fields[0],
      identifier=fields[1],
      format=fields[2],
      format_details=fields[3],
      carrier=int(fields[4]),
      nav_system=fields[5],
      network=fields[6],
      country=fields[7],
      latitude=float(fields[8]),
      longitude=float(fields[9]),
      nmea=bool(fields[10]),
      solution=int(fields[11]),
      version=fields[12],
    )

  def dist(self, pos):
    dlat = self.latitude - pos[0]
    dlon = self.longitude - pos[1]
    return dlat * dlat + dlon * dlon
