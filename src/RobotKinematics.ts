import * as math from 'mathjs';

type JointType = 'revolute' | 'prismatic';

interface Point3D {
    x: number;
    y: number;
    z: number;
}

interface DHParameters {
    theta: number;  // Joint angle (radians)
    d: number;      // Link offset
    a: number;      // Link length
    alpha: number;  // Link twist
}

export interface RobotConfig {
    name: string;
    description: string;
    dhParameters: DHParameters[];
    jointLimits: {
        min: number;
        max: number;
    }[];
    jointTypes: JointType[];
    baseOffset: Point3D;
    toolOffset: Point3D;
}

interface IKResult {
    solutions: number[][];  // Array of joint angle solutions
    valid: boolean;         // Whether valid solutions exist
}

export class RobotKinematics {
    public readonly config: RobotConfig;
    private readonly dhMatrices: math.Matrix[];
    private numJoints: number;

    constructor(config: RobotConfig) {
        this.config = config;
        this.dhMatrices = this.config.dhParameters.map(params => 
            this.calculateDHTransform(params, 0)
        );
        this.numJoints = config.dhParameters.length;
    }

    /**
     * Create a RobotKinematics instance from a JSON configuration
     */
    public static fromJSON(json: string): RobotKinematics {
        const config = JSON.parse(json) as RobotConfig;
        return new RobotKinematics(config);
    }

    /**
     * Convert the robot configuration to JSON
     */
    public toJSON(): string {
        return JSON.stringify(this.config, null, 2);
    }

    /**
     * Calculate the transformation matrix for a single joint using DH parameters
     */
    public calculateDHTransform(params: DHParameters, jointAngle: number): math.Matrix {
        const { theta, d, a, alpha } = params;
        const ct = Math.cos(theta + jointAngle);
        const st = Math.sin(theta + jointAngle);
        const ca = Math.cos(alpha);
        const sa = Math.sin(alpha);

        // Standard DH transformation matrix
        // [cos(θ),            -sin(θ)cos(α),      sin(θ)sin(α),       acos(θ)]
        // [sin(θ),             cos(θ)cos(α),     -cos(θ)sin(α),       asin(θ)]
        // [0,                  sin(α),            cos(α),              d      ]
        // [0,                  0,                 0,                   1      ]
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

        // Calculate the transformation matrix for each joint
        const transforms = this.config.dhParameters.map((params, i) => 
            this.calculateDHTransform(params, jointAngles[i])
        );

        // Multiply all transforms to get the final transformation
        const finalTransform = transforms.reduce((acc, curr) => 
            math.multiply(acc, curr)
        );

        // Extract position and orientation
        const position: [number, number, number] = [
            finalTransform.get([0, 3]) as number,
            finalTransform.get([1, 3]) as number,
            finalTransform.get([2, 3]) as number
        ];

        // Extract orientation (3x3 rotation matrix)
        const orientation = math.subset(finalTransform, math.index(
            [0, 1, 2],
            [0, 1, 2]
        ));

        return { position, orientation };
    }

    /**
     * Get the number of joints in the robot
     */
    public getNumJoints(): number {
        return this.numJoints;
    }

    /**
     * Calculate inverse kinematics for a given target position
     * @param targetPosition Target [x, y, z] position
     * @returns Array of possible joint angle solutions
     */
    public calculateInverseKinematics(targetPosition: [number, number, number], targetOrientation?: number[][]): IKResult {
        // Only support 3-DOF robots for now
        if (this.numJoints !== 3) {
            console.log(`Inverse kinematics is currently only supported for 3-DOF robots. This robot has ${this.numJoints} DOF.`);
            return { solutions: [], valid: false };
        }

        const [x, y, z] = targetPosition;
        const solutions: number[][] = [];

        // Extract link lengths and offsets from DH parameters
        const a1 = this.config.dhParameters[0].a;  // Base link length
        const a2 = this.config.dhParameters[1].a;  // Shoulder link length
        const a3 = this.config.dhParameters[2].a;  // Elbow link length
        const d1 = this.config.dhParameters[0].d;  // Base vertical offset
        const d2 = this.config.dhParameters[1].d;  // Shoulder vertical offset
        const d3 = this.config.dhParameters[2].d;  // Elbow vertical offset

        console.log('Target position:', targetPosition);
        console.log('Link parameters:', { a1, a2, a3, d1, d2, d3 });

        // Calculate target height relative to base
        const h = z - d1;

        // Calculate distance from base to target in x-y plane
        const r = Math.sqrt(x * x + y * y);
        console.log('Distances:', { r, h });

        // Check if target is reachable at all
        const maxReach = a1 + a2 + a3;
        if (r > maxReach) {
            console.log('Target is unreachable: distance exceeds maximum reach');
            return { solutions: [], valid: false };
        }

        // Try different base angles to find the best configuration
        let bestSolution: { solution: number[], error: number } | null = null;
        const stepSize = Math.PI / 180;  // 2-degree resolution

        for (let theta1 = 0; theta1 < 2 * Math.PI; theta1 += stepSize) {
            // Calculate shoulder position based on base angle
            const shoulderX = a1 * Math.cos(theta1);
            const shoulderY = a1 * Math.sin(theta1);
            const shoulderZ = d1 + d2;

            // Calculate vector from shoulder to target
            const dx = x - shoulderX;
            const dy = y - shoulderY;
            const dz = z - shoulderZ;
            const shoulderToTarget = Math.sqrt(dx * dx + dy * dy + dz * dz);

            // Check if target is reachable from this shoulder position
            if (shoulderToTarget > a2 + a3) {
                continue;  // Try next base angle
            }

            // Calculate elbow angle (theta3) using law of cosines
            const cosTheta3 = (a2 * a2 + a3 * a3 - shoulderToTarget * shoulderToTarget) / (2 * a2 * a3);
            if (Math.abs(cosTheta3) > 1) {
                continue;  // Try next base angle
            }
            const theta3 = Math.acos(cosTheta3);

            // Calculate shoulder angle (theta2)
            // First, calculate the angle between shoulder link and line to target
            const cosTheta2 = (a2 * a2 + shoulderToTarget * shoulderToTarget - a3 * a3) / (2 * a2 * shoulderToTarget);
            if (Math.abs(cosTheta2) > 1) {
                continue;  // Try next base angle
            }
            const alpha = Math.acos(cosTheta2);

            // Then, calculate the angle from horizontal to line to target
            const beta = Math.atan2(dz, Math.sqrt(dx * dx + dy * dy));

            // The shoulder angle is the difference between these angles
            const theta2 = beta - alpha;

            // Verify this solution
            const solution = [theta1, theta2, theta3];
            const fkResult = this.calculateForwardKinematics(solution);
            const error = Math.sqrt(
                Math.pow(fkResult.position[0] - x, 2) +
                Math.pow(fkResult.position[1] - y, 2) +
                Math.pow(fkResult.position[2] - z, 2)
            );

            // Keep track of the best solution
            if (bestSolution === null || error < bestSolution.error) {
                bestSolution = { solution, error };
            }
        }

        if (bestSolution === null) {
            console.log('No valid solution found');
            return { solutions: [], valid: false };
        }

        // Add the elbow-down solution
        solutions.push(bestSolution.solution);
        console.log('Elbow-down solution:', bestSolution.solution);

        // For the elbow-up solution:
        // 1. Invert the elbow angle
        // 2. Find a new base angle that places the shoulder on the circle
        const [_, theta2, theta3] = bestSolution.solution;
        const invertedTheta3 = -theta3;

        // Try different base angles for the elbow-up configuration
        let elbowUpSolution: { solution: number[], error: number } | null = null;
        for (let theta1 = 0; theta1 < 2 * Math.PI; theta1 += stepSize) {
            // Calculate shoulder position based on base angle
            const shoulderX = a1 * Math.cos(theta1);
            const shoulderY = a1 * Math.sin(theta1);
            const shoulderZ = d1 + d2;  // Total vertical offset to shoulder

            // Calculate vector from shoulder to target
            const dx = x - shoulderX;
            const dy = y - shoulderY;
            const dz = z - shoulderZ;
            const shoulderToTarget = Math.sqrt(dx * dx + dy * dy + dz * dz);

            // Check if target is reachable from this shoulder position
            if (shoulderToTarget > a2 + a3) {
                continue;
            }

            // Calculate shoulder angle for this configuration
            const cosTheta2 = (a2 * a2 + shoulderToTarget * shoulderToTarget - a3 * a3) / (2 * a2 * shoulderToTarget);
            if (Math.abs(cosTheta2) > 1) {
                continue;
            }
            const alpha = Math.acos(cosTheta2);
            const beta = Math.atan2(dz, Math.sqrt(dx * dx + dy * dy));
            const newTheta2 = beta - alpha;

            // Verify this solution
            const solution = [theta1, newTheta2, invertedTheta3];
            const fkResult = this.calculateForwardKinematics(solution);
            const error = Math.sqrt(
                Math.pow(fkResult.position[0] - x, 2) +
                Math.pow(fkResult.position[1] - y, 2) +
                Math.pow(fkResult.position[2] - z, 2)
            );

            if (error < 0.1 && (elbowUpSolution === null || error < elbowUpSolution.error)) {
                elbowUpSolution = { solution, error };
            }
        }

        if (elbowUpSolution !== null) {
            solutions.push(elbowUpSolution.solution);
            console.log('Elbow-up solution:', elbowUpSolution.solution);
        }

        return { solutions, valid: true };
    }

    /**
     * Check if a target position is reachable
     * @param targetPosition Target [x, y, z] position
     * @returns boolean indicating if the position is reachable
     */
    public isReachable(targetPosition: [number, number, number]): boolean {
        const result = this.calculateInverseKinematics(targetPosition);
        return result.valid;
    }

    /**
     * Get the workspace radius (maximum reachable distance)
     */
    public getWorkspaceRadius(): number {
        return this.config.dhParameters.reduce((sum, params) => sum + params.a, 0);
    }
} 