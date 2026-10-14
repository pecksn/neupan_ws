"""PVTSLNA, N4 protocol table 7-81 (header excluded from field indexes)."""

import math


def crc32(data):
    """Unicore CRC32: reflected polynomial, initial zero, no final XOR."""
    crc = 0
    for byte in data.encode('ascii'):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def parse_pvtslna(sentence):
    body, checksum = sentence.strip().rsplit('*', 1)
    if not body.startswith('#PVTSLNA,') or len(checksum) != 8:
        raise ValueError('Invalid PVTSLNA framing')
    if crc32(body[1:]) != int(checksum, 16):
        raise ValueError('Invalid PVTSLNA CRC32')
    header, payload = body[1:].split(';', 1)
    fields = payload.split(',')
    if len(header.split(',')) != 10 or len(fields) < 35:
        raise ValueError('Truncated PVTSLNA')
    position_type, heading_type = fields[0], fields[20]
    length, heading, pitch = map(float, fields[21:24])
    latitude, longitude = map(float, fields[2:4])
    satellites = int(fields[25])
    if not all(math.isfinite(v) for v in (length, heading, pitch, latitude, longitude)):
        raise ValueError('Non-finite PVTSLNA data')
    if not (-90 <= latitude <= 90 and -180 <= longitude <= 180
            and 0 <= length <= 3000 and 0 <= heading <= 360
            and -90 <= pitch <= 90 and satellites >= 0):
        raise ValueError('Out-of-range PVTSLNA data')
    heading_fixed = heading_type == 'NARROW_INT'
    return {
        'source': 'PVTSLNA',
        'position_type': position_type,
        'position_fixed': position_type == 'NARROW_INT',
        'latitude': latitude, 'longitude': longitude,
        'heading_type': heading_type, 'heading_fixed': heading_fixed,
        'heading_valid': heading_fixed and length > 0 and satellites > 0,
        'baseline_m': length, 'heading_deg': heading,
        'pitch_deg': pitch, 'heading_satellites': satellites,
    }
