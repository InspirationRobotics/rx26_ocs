"""rx_bridge — the OCS half of the RoboCommand link.

Import order matters exactly once: `from .proto import ...` puts the generated
protobuf tree on sys.path, so anything importing pb2 modules must go through
rx_bridge.proto rather than reaching into gen/ directly.
"""
__all__ = ["bridge", "config", "governor", "proto", "runstate", "seqstore",
           "validate", "wirelog"]
