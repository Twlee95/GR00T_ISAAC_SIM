# GR00T ISAAC SIM

Isaac Sim setup for GR00T G1 simulation, SONIC whole-body control, and VLA inference.

## Components

- `groot.g1.bridge`: Isaac Sim G1 bridge extension
- SONIC G1 deploy integration
- DDS-ZMQ bridge for LowState / LowCmd communication
- VLA camera and action communication

## Main communication ports

- `5555`: Isaac Sim camera output to VLA
- `5558`: LowCmd input to Isaac Sim
- `5559`: Isaac Sim state output to SONIC
- `5556`: VLA action output to SONIC
- `5557`: SONIC debug/state output
