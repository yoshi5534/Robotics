import { RobotKinematics } from './RobotKinematics';

describe('RobotKinematics', () => {
    // Example DH parameters for a simple 3-DOF robot arm
    const dhParams = [
        { theta: 0, d: 0.1, a: 0.2, alpha: 0 },      // Base joint
        { theta: 0, d: 0, a: 0.3, alpha: Math.PI/2 }, // Shoulder joint
        { theta: 0, d: 0, a: 0.2, alpha: 0 }          // Elbow joint
    ];

    let robot: RobotKinematics;

    beforeEach(() => {
        robot = new RobotKinematics(dhParams);
    });

    test('should create robot with correct number of joints', () => {
        expect(robot.getNumJoints()).toBe(3);
    });

    test('should calculate forward kinematics for zero angles', () => {
        const jointAngles = [0, 0, 0];
        const result = robot.calculateForwardKinematics(jointAngles);

        // For zero angles, the end-effector should be at:
        // x = 0.2 + 0.3 + 0.2 = 0.7
        // y = 0
        // z = 0.1
        expect(result.position[0]).toBeCloseTo(0.7, 6);
        expect(result.position[1]).toBeCloseTo(0, 6);
        expect(result.position[2]).toBeCloseTo(0.1, 6);
    });

    test('should throw error for incorrect number of joint angles', () => {
        const jointAngles = [0, 0]; // Only 2 angles for 3 joints
        expect(() => robot.calculateForwardKinematics(jointAngles))
            .toThrow('Expected 3 joint angles, got 2');
    });

    test('should calculate forward kinematics for non-zero angles', () => {
        const jointAngles = [Math.PI/4, Math.PI/4, Math.PI/4];
        const result = robot.calculateForwardKinematics(jointAngles);

        // The position should be different from the zero position
        expect(result.position[0]).not.toBeCloseTo(0.7, 6);
        expect(result.position[1]).not.toBeCloseTo(0, 6);
        expect(result.position[2]).not.toBeCloseTo(0.1, 6);
    });
}); 