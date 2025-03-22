import * as math from 'mathjs';

interface DHParameters {
    theta: number;  // Joint angle (radians)
    d: number;      // Link offset
    a: number;      // Link length
    alpha: number;  // Link twist
}

export class RobotKinematics {
    private dhParams: DHParameters[];
    private numJoints: number;

    constructor(dhParameters: DHParameters[]) {
        this.dhParams = dhParameters;
        this.numJoints = dhParameters.length;
    }

    /**
     * Calculate the transformation matrix for a single joint using DH parameters
     */
    private calculateJointTransform(params: DHParameters): math.Matrix {
        const { theta, d, a, alpha } = params;
        const ct = Math.cos(theta);
        const st = Math.sin(theta);
        const ca = Math.cos(alpha);
        const sa = Math.sin(alpha);

        return math.matrix([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ]);
    }

    /**
     * Calculate forward kinematics for all joints
     * @param jointAngles Array of joint angles in radians
     * @returns The end-effector position [x, y, z] and orientation matrix
     */
    public calculateForwardKinematics(jointAngles: number[]): {
        position: [number, number, number];
        orientation: math.Matrix;
    } {
        if (jointAngles.length !== this.numJoints) {
            throw new Error(`Expected ${this.numJoints} joint angles, got ${jointAngles.length}`);
        }

        // Create DH parameters with current joint angles
        const currentDHParams = this.dhParams.map((params, index) => ({
            ...params,
            theta: jointAngles[index]
        }));

        // Calculate the transformation matrix for each joint
        const transforms = currentDHParams.map(params => this.calculateJointTransform(params));

        // Multiply all transformation matrices to get the final transformation
        let finalTransform = transforms[0];
        for (let i = 1; i < transforms.length; i++) {
            finalTransform = math.multiply(finalTransform, transforms[i]);
        }

        // Extract position and orientation
        const position: [number, number, number] = [
            finalTransform.get([0, 3]),
            finalTransform.get([1, 3]),
            finalTransform.get([2, 3])
        ];

        const orientation = math.subset(finalTransform, math.index([0, 1, 2], [0, 1, 2]));

        return { position, orientation };
    }

    /**
     * Get the number of joints in the robot
     */
    public getNumJoints(): number {
        return this.numJoints;
    }
} 