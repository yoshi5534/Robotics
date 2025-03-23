import { RobotKinematics, RobotConfig } from '../RobotKinematics';
import * as fs from 'fs';
import * as path from 'path';

// Create a configuration for the Stäubli RX90B robot
const config: RobotConfig = {
    name: "Stäubli RX90B",
    description: "6-DOF industrial robot arm",
    dhParameters: [
        { theta: 0, d: 0.325, a: 0.225, alpha: -Math.PI/2 },  // Base
        { theta: 0, d: 0, a: 0.735, alpha: 0 },               // Shoulder
        { theta: 0, d: 0, a: 0.175, alpha: 0 },               // Elbow
        { theta: 0, d: 0.775, a: 0, alpha: -Math.PI/2 },      // Wrist 1
        { theta: 0, d: 0, a: 0, alpha: Math.PI/2 },           // Wrist 2
        { theta: 0, d: 0.1, a: 0, alpha: 0 }                  // Wrist 3
    ],
    jointLimits: [
        { min: -Math.PI, max: Math.PI },  // Base
        { min: -Math.PI, max: Math.PI },  // Shoulder
        { min: -Math.PI, max: Math.PI },  // Elbow
        { min: -Math.PI, max: Math.PI },  // Wrist 1
        { min: -Math.PI, max: Math.PI },  // Wrist 2
        { min: -Math.PI, max: Math.PI }   // Wrist 3
    ],
    jointTypes: ['revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute'],
    baseOffset: { x: 0, y: 0, z: 0 },
    toolOffset: { x: 0, y: 0, z: 0 }
};

const robot = new RobotKinematics(config);

// Example 1: Home position
console.log('Example 1: Home position');
const homeAngles = [0, 0, 0, 0, 0, 0];
console.log('Joint angles:', homeAngles);
const homeResult = robot.calculateForwardKinematics(homeAngles);
console.log('End-effector position:', homeResult.position);
console.log('End-effector orientation:', homeResult.orientation);

// Example 2: Base joint at 90 degrees
console.log('\nExample 2: Base joint at 90 degrees');
const base90Angles = [Math.PI/2, 0, 0, 0, 0, 0];
console.log('Joint angles:', base90Angles);
const base90Result = robot.calculateForwardKinematics(base90Angles);
console.log('End-effector position:', base90Result.position);
console.log('End-effector orientation:', base90Result.orientation);

// Example 3: Shoulder joint at 90 degrees
console.log('\nExample 3: Shoulder joint at 90 degrees');
const shoulder90Angles = [0, Math.PI/2, 0, 0, 0, 0];
console.log('Joint angles:', shoulder90Angles);
const shoulder90Result = robot.calculateForwardKinematics(shoulder90Angles);
console.log('End-effector position:', shoulder90Result.position);
console.log('End-effector orientation:', shoulder90Result.orientation);

// Example 4: Elbow joint at 90 degrees
console.log('\nExample 4: Elbow joint at 90 degrees');
const elbow90Angles = [0, 0, Math.PI/2, 0, 0, 0];
console.log('Joint angles:', elbow90Angles);
const elbow90Result = robot.calculateForwardKinematics(elbow90Angles);
console.log('End-effector position:', elbow90Result.position);
console.log('End-effector orientation:', elbow90Result.orientation);

// Example 5: Wrist roll at 90 degrees
console.log('\nExample 5: Wrist roll at 90 degrees');
const wrist1_90Angles = [0, 0, 0, Math.PI/2, 0, 0];
console.log('Joint angles:', wrist1_90Angles);
const wrist1_90Result = robot.calculateForwardKinematics(wrist1_90Angles);
console.log('End-effector position:', wrist1_90Result.position);
console.log('End-effector orientation:', wrist1_90Result.orientation);

// Example 6: Wrist pitch at 90 degrees
console.log('\nExample 6: Wrist pitch at 90 degrees');
const wrist2_90Angles = [0, 0, 0, 0, Math.PI/2, 0];
console.log('Joint angles:', wrist2_90Angles);
const wrist2_90Result = robot.calculateForwardKinematics(wrist2_90Angles);
console.log('End-effector position:', wrist2_90Result.position);
console.log('End-effector orientation:', wrist2_90Result.orientation);

// Example 7: Final wrist roll at 90 degrees
console.log('\nExample 7: Final wrist roll at 90 degrees');
const wrist3_90Angles = [0, 0, 0, 0, 0, Math.PI/2];
console.log('Joint angles:', wrist3_90Angles);
const wrist3_90Result = robot.calculateForwardKinematics(wrist3_90Angles);
console.log('End-effector position:', wrist3_90Result.position);
console.log('End-effector orientation:', wrist3_90Result.orientation);

// Example 8: Complex pose (all joints at 45 degrees)
console.log('\nExample 8: Complex pose (all joints at 45 degrees)');
const complexAngles = [Math.PI/4, Math.PI/4, Math.PI/4, Math.PI/4, Math.PI/4, Math.PI/4];
console.log('Joint angles:', complexAngles);
const complexResult = robot.calculateForwardKinematics(complexAngles);
console.log('End-effector position:', complexResult.position);
console.log('End-effector orientation:', complexResult.orientation);

// Print workspace information
console.log('\nWorkspace Information:');
console.log('Number of joints:', robot.getNumJoints());
console.log('Maximum reach:', robot.getWorkspaceRadius(), 'meters');

// Example of inverse kinematics (3-DOF)
console.log('\nInverse Kinematics Example (3-DOF):');
const targetPosition: [number, number, number] = [0.5, 0.2, 0.3];
console.log('Target position:', targetPosition);

// Create a 3-DOF robot for inverse kinematics
const threeDOFConfig: RobotConfig = {
    name: "3-DOF Test Robot",
    description: "3-DOF robot for inverse kinematics testing",
    dhParameters: [
        { theta: 0, d: 0.325, a: 0.225, alpha: -Math.PI/2 },  // Base
        { theta: 0, d: 0, a: 0.735, alpha: 0 },               // Shoulder
        { theta: 0, d: 0, a: 0.175, alpha: 0 }                // Elbow
    ],
    jointLimits: [
        { min: -Math.PI, max: Math.PI },  // Base
        { min: -Math.PI, max: Math.PI },  // Shoulder
        { min: -Math.PI, max: Math.PI }   // Elbow
    ],
    jointTypes: ['revolute', 'revolute', 'revolute'],
    baseOffset: { x: 0, y: 0, z: 0 },
    toolOffset: { x: 0, y: 0, z: 0 }
};

const threeDOFRobot = new RobotKinematics(threeDOFConfig);
const ikResult = threeDOFRobot.calculateInverseKinematics(targetPosition);
console.log('Inverse kinematics result:', ikResult);

if (ikResult.valid) {
    console.log('Found solutions:', ikResult.solutions.length);
    // Verify the first solution
    const solution = ikResult.solutions[0];
    const fkResult = threeDOFRobot.calculateForwardKinematics(solution);
    console.log('Verification - Forward kinematics result:', fkResult.position);
    
    // Calculate error
    const error = Math.sqrt(
        Math.pow(fkResult.position[0] - targetPosition[0], 2) +
        Math.pow(fkResult.position[1] - targetPosition[1], 2) +
        Math.pow(fkResult.position[2] - targetPosition[2], 2)
    );
    console.log('Position error:', error);
} else {
    console.log('No valid solutions found');
} 