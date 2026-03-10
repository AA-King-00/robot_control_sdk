# robot_control_sdk

A personal robotics motion control SDK for learning, implementing, and validating robot control algorithms on UR5.

## Goals

- Build an industrial-style C++ robotics SDK
- Implement core robotics modules from scratch
- Validate algorithms in simulation and on real hardware
- Build a strong engineering portfolio for robotics / embodied AI roles

## Current Focus

Month 1: engineering foundation + forward kinematics

Current tasks:
- Set up repository structure and coding rules
- Implement UR5 forward kinematics in C++
- Add unit tests with Google Test
- Add CI build checks with GitHub Actions
- Validate FK in simulation and on the real robot

## Project Structure

robot_control_sdk/
├─ include/        # public headers
├─ src/            # source files
├─ tests/          # unit tests
├─ examples/       # runnable examples
├─ config/         # robot parameters and yaml files
├─ docs/           # notes and design docs
├─ scripts/        # helper scripts
└─ .github/        # CI workflows

## Build

```bash
mkdir -p build
cd build
cmake ..
make -j  
