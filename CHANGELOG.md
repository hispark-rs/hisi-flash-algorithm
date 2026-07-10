# Changelog

All notable changes to this project are documented here.

## [Unreleased]

### Changed

- Batch WS63 probe-rs programming in 64 KiB transfers and use the SFC bus-DMA
  engine instead of thousands of 64-byte register-mode commands.

## [0.2.0]

### Fixed

- **SFC controller state corruption**: restore SFC idle state (SR1 register) after
  flash programming/erase.
- **WIP poll bounding**: bounded write-in-progress wait loop.
