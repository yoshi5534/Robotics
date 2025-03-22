# Robot Kinematics Calculator

This TypeScript library provides forward kinematics calculations for industrial robots using Denavit-Hartenberg (DH) parameters.

## Features

- Configurable robot description using DH parameters
- Forward kinematics calculation for end-effector position and orientation
- TypeScript support with type safety
- Comprehensive test suite

## Installation

```bash
npm install
```

## Usage

```typescript
import { RobotKinematics } from './src/RobotKinematics';

// Define DH parameters for your robot
const dhParams = [
    { theta: 0, d: 0.1, a: 0.2, alpha: 0 },      // Base joint
    { theta: 0, d: 0, a: 0.3, alpha: Math.PI/2 }, // Shoulder joint
    { theta: 0, d: 0, a: 0.2, alpha: 0 }          // Elbow joint
];

// Create a new robot instance
const robot = new RobotKinematics(dhParams);

// Calculate forward kinematics for given joint angles
const jointAngles = [0, Math.PI/4, Math.PI/4]; // in radians
const result = robot.calculateForwardKinematics(jointAngles);

console.log('End-effector position:', result.position);
console.log('End-effector orientation:', result.orientation);
```

## DH Parameters

Each joint is described by four DH parameters:
- `theta`: Joint angle (in radians)
- `d`: Link offset (distance along z-axis)
- `a`: Link length (distance along x-axis)
- `alpha`: Link twist (angle around x-axis)

## Development

```bash
# Build the project
npm run build

# Run tests
npm test

# Run the example
npm start
```

## License

MIT 