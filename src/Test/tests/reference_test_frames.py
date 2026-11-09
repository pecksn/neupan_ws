"""Explicit acquisition identities for synthetic successive road observations."""
def stamp_reference_frame(node, sequence):
    for item in node.cache_geometry.values():
        item['capture_timestamp_ns'] = 1000000000000+int(sequence)*1000000
