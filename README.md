# Robot Kinematics Library

A TypeScript library for robot kinematics calculations and visualization.

## Features

- Forward and inverse kinematics calculations using DH parameters
- Support for multiple robot configurations 
- 3D visualization using Three.js
- Interactive joint control with sliders and number inputs
- Real-time position and orientation display
- Camera controls for better visualization

## Installation

```bash
npm install
```

## Development

```bash
npm run dev
```

## Testing

```bash
npm run test
```

## Project Structure

- `src/`
  - `RobotKinematics.ts` - Core kinematics calculations
  - `visualizer.ts` - 3D visualization using Three.js
  - `configs/` - Robot configuration files
  - `examples/` - Example usage and tests

## Visualization Features

- Real-time 3D visualization of robot arm
- Interactive joint control with sliders and number inputs
- Position and orientation display
- Camera controls:
  - Rotate: Left mouse button
  - Pan: Right mouse button
  - Zoom: Mouse wheel
- Visual elements:
  - Gray cylinders for links
  - Green spheres for joint positions
  - Red sphere at origin
  - Colored tripod at end-effector showing orientation

## Robot Configurations

The library includes configurations for several industrial robots.

Each configuration includes:
- DH parameters
- Joint limits
- Joint types
- Base and tool offsets

## License

MIT 