# Technical Summary

Date: 2026-03-27

## Decision

Meshtastic was chosen as the better default fit for VSCP over MeshCore.

Reasons:

- More mature ecosystem for ESP32-based mesh deployments.
- Better tooling, documentation, and operational support.
- Lower integration and maintenance risk for a VSCP gateway.
- Faster path to a practical deployment.

MeshCore remains a valid option only if strict low-level control of transport framing and routing behavior is more important than ecosystem maturity and implementation cost.

## Transport Design

The recommended approach is to run VSCP over a compact application envelope on top of a mesh carrier.

Key design points:

- Compact binary mesh header.
- Fragmentation for larger VSCP events.
- Reassembly for inbound fragments.
- Duplicate suppression.
- QoS derived from VSCP priority.
- Conservative fragment size suitable for mesh transport.

Suggested mapping:

- High-priority VSCP traffic uses stronger delivery behavior.
- Lower-priority traffic uses best-effort delivery to limit airtime cost.

## Firmware Changes

Implemented a new VSCP mesh transport layer in the firmware.

Added:

- Mesh header encode/decode.
- Outbound fragmentation for VSCP event frames.
- Inbound reassembly with timeout-based slot management.
- Duplicate message suppression using source nickname and message id.
- VSCP-priority-to-QoS mapping.
- Integration with existing binary callback flow.

Behavioral integration:

- Outbound VSCP eventex traffic is routed through the mesh transport.
- Received VSCP events are converted to eventex and routed through the same path.
- The binary event handler was updated so it no longer returns success without forwarding the event.

Safety behavior:

- If no actual mesh transmit callback is registered, the code preserves previous behavior and does not fail the existing callback path.

## Files Changed

- [main/vscp-mesh.h](main/vscp-mesh.h)
- [main/vscp-mesh.c](main/vscp-mesh.c)
- [main/callbacks-binary.c](main/callbacks-binary.c)
- [main/vscp-binary.c](main/vscp-binary.c)
- [main/CMakeLists.txt](main/CMakeLists.txt)

## Validation

- The ESP32 firmware build completed successfully after the changes.

## Current Limitations

- No concrete Meshtastic TX adapter is wired yet.
- No concrete Meshtastic RX adapter is wired yet.
- ACK and retry scheduling are represented in the transport design, but a full retry scheduler is not yet implemented.

## Recommended Next Step

Connect the transport layer to a real Meshtastic send/receive interface so outbound packets go on-air and inbound mesh packets feed the reassembly path.