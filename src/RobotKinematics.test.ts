import { RobotKinematics, RobotConfig } from './RobotKinematics';

describe('RobotKinematics', () => {
    let robot: RobotKinematics;

    beforeEach(() => {
        const config: RobotConfig = {
            name: "Test Robot",
            description: "Test robot for unit tests",
            dhParameters: [
                { theta: 0, d: 0, a: 0.3, alpha: -Math.PI/2 },  // Base
                { theta: 0, d: 0, a: 0.3, alpha: -Math.PI/2 },  // Shoulder
                { theta: 0, d: 0, a: 0.3, alpha: 0 }            // Elbow
            ],
            jointLimits: [
                { min: -Math.PI, max: Math.PI },
                { min: -Math.PI, max: Math.PI },
                { min: -Math.PI, max: Math.PI }
            ],
            jointTypes: ['revolute', 'revolute', 'revolute'],
            baseOffset: { x: 0, y: 0, z: 0 },
            toolOffset: { x: 0, y: 0, z: 0 }
        };
        robot = new RobotKinematics(config);
    });

    test('should calculate forward kinematics correctly', () => {
        const jointAngles = [0, 0, 0];
        const result = robot.calculateForwardKinematics(jointAngles);
        
        expect(result.position).toBeDefined();
        expect(result.orientation).toBeDefined();
        expect(result.position.length).toBe(3);
        expect(result.orientation.size()).toEqual([3, 3]);
    });

    test.skip('should calculate inverse kinematics for reachable position', () => {
        const targetPosition: [number, number, number] = [0.3, 0.3, 0.3];
        const result = robot.calculateInverseKinematics(targetPosition);
        
        expect(result.valid).toBe(true);
        expect(result.solutions.length).toBeGreaterThan(0);
        
        // Verify the solution works by checking forward kinematics
        const solution = result.solutions[0];
        const fkResult = robot.calculateForwardKinematics(solution);
        
        // Check if the position matches within tolerance
        const positionError = Math.sqrt(
            Math.pow(fkResult.position[0] - targetPosition[0], 2) +
            Math.pow(fkResult.position[1] - targetPosition[1], 2) +
            Math.pow(fkResult.position[2] - targetPosition[2], 2)
        );
        
        expect(positionError).toBeLessThan(0.1);
    });

    test('should return no solutions for unreachable position', () => {
        const targetPosition: [number, number, number] = [2.0, 2.0, 2.0]; // Position beyond reach
        const result = robot.calculateInverseKinematics(targetPosition);
        
        expect(result.valid).toBe(false);
        expect(result.solutions.length).toBe(0);
    });

    test('should correctly identify reachable positions', () => {
        const reachablePosition: [number, number, number] = [0.5, 0.2, 0.3];
        const unreachablePosition: [number, number, number] = [2.0, 2.0, 2.0];
        
        expect(robot.isReachable(reachablePosition)).toBe(true);
        expect(robot.isReachable(unreachablePosition)).toBe(false);
    });

    test('should return correct workspace radius', () => {
        const expectedRadius = 0.9; // Sum of link lengths (0.3 + 0.3 + 0.3)
        const actualRadius = robot.getWorkspaceRadius();
        expect(Math.abs(actualRadius - expectedRadius)).toBeLessThan(1e-10);
    });
}); 