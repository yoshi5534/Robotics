import * as THREE from 'three';
import * as math from 'mathjs';
import { RobotKinematics } from './RobotKinematics';
import { loadRobotConfig } from './configs/robotConfigs';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

interface DHParameters {
    theta: number;
    d: number;
    a: number;
    alpha: number;
}

class RobotVisualizer {
    private scene: THREE.Scene;
    private camera: THREE.PerspectiveCamera;
    private renderer: THREE.WebGLRenderer;
    private controls: OrbitControls;
    private robot: RobotKinematics | null;
    private robotModel: THREE.Group | null;
    private jointAngles: number[];

    constructor() {
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.robot = null;
        this.robotModel = null;
        this.jointAngles = [];
        
        this.init();
    }

    private init(): void {
        // Setup renderer
        this.renderer.setSize(window.innerWidth - 300, window.innerHeight);
        const container = document.getElementById('canvas-container');
        if (container) {
            container.appendChild(this.renderer.domElement);
        }

        // Setup camera
        this.camera.position.set(2, 2, 2);
        this.camera.lookAt(0, 0, 0);

        // Setup camera controls
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;

        // Add lights
        const ambientLight = new THREE.AmbientLight(0x404040);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(1, 1, 1);
        this.scene.add(ambientLight, directionalLight);

        // Add grid helper
        const gridHelper = new THREE.GridHelper(2, 20);
        this.scene.add(gridHelper);

        // Setup controls
        this.setupControls();

        // Start animation loop
        this.animate();

        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize(), false);
    }

    private setupControls(): void {
        const robotSelect = document.getElementById('robot-select') as HTMLSelectElement;
        if (robotSelect) {
            robotSelect.addEventListener('change', () => this.loadRobot(robotSelect.value));
            // Load initial robot
            this.loadRobot(robotSelect.value);
        }
    }

    private async loadRobot(robotName: string): Promise<void> {
        const config = await loadRobotConfig(robotName);
        this.robot = new RobotKinematics(config);
        this.jointAngles = new Array(config.dhParameters.length).fill(0);
        this.updateRobotModel();
        this.updateJointControls();
    }

    private updateJointControls(): void {
        const container = document.getElementById('joint-controls');
        if (!container) return;

        container.innerHTML = '';

        this.jointAngles.forEach((angle, index) => {
            const control = document.createElement('div');
            control.className = 'joint-control';
            
            const label = document.createElement('label');
            label.textContent = `Joint ${index + 1} (${(angle * 180 / Math.PI).toFixed(1)}°)`;
            
            const slider = document.createElement('input');
            slider.type = 'range';
            slider.min = '-180';
            slider.max = '180';
            slider.value = (angle * 180 / Math.PI).toString();
            slider.addEventListener('input', (e: Event) => {
                const target = e.target as HTMLInputElement;
                this.jointAngles[index] = parseFloat(target.value) * Math.PI / 180;
                label.textContent = `Joint ${index + 1} (${target.value}°)`;
                numberInput.value = target.value;
                this.updateRobotModel();
            });

            const numberInput = document.createElement('input');
            numberInput.type = 'number';
            numberInput.value = (angle * 180 / Math.PI).toFixed(1);
            numberInput.step = '0.1';
            numberInput.addEventListener('change', (e: Event) => {
                const target = e.target as HTMLInputElement;
                this.jointAngles[index] = parseFloat(target.value) * Math.PI / 180;
                slider.value = target.value;
                label.textContent = `Joint ${index + 1} (${target.value}°)`;
                this.updateRobotModel();
            });

            control.appendChild(label);
            control.appendChild(slider);
            control.appendChild(numberInput);
            container.appendChild(control);
        });
    }

    private updateRobotModel(): void {
        if (!this.robot) return;

        // Remove existing robot model
        if (this.robotModel) {
            this.scene.remove(this.robotModel);
        }

        // Calculate forward kinematics
        const result = this.robot.calculateForwardKinematics(this.jointAngles);
        
        // Update position and orientation display
        const positionElement = document.getElementById('position');
        if (positionElement) {
            positionElement.textContent = 
                `[${result.position[0].toFixed(3)}, ${result.position[1].toFixed(3)}, ${result.position[2].toFixed(3)}]`;
        }
        
        // Convert rotation matrix to Euler angles
        const orientationArray = result.orientation.toArray() as number[][];
        const matrix = new THREE.Matrix4().fromArray(orientationArray.flat());
        const euler = new THREE.Euler().setFromRotationMatrix(matrix);
        
        const orientationElement = document.getElementById('orientation');
        if (orientationElement) {
            orientationElement.textContent = 
                `[${(euler.x * 180 / Math.PI).toFixed(1)}°, ${(euler.y * 180 / Math.PI).toFixed(1)}°, ${(euler.z * 180 / Math.PI).toFixed(1)}°]`;
        }

        // Create new robot model
        this.robotModel = new THREE.Group();
        
        // Add base
        const baseGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.05, 32);
        const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x808080 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        this.robotModel.add(base);

        // Calculate cumulative transformations for each joint
        const transforms: math.Matrix[] = [];
        for (let i = 0; i < this.robot.config.dhParameters.length; i++) {
            // Calculate transform for this joint
            const transform = this.robot!.calculateDHTransform(this.robot.config.dhParameters[i], this.jointAngles[i]);
            // Multiply with previous transforms to get cumulative transform
            if (i === 0) {
                transforms.push(transform);
            } else {
                transforms.push(math.multiply(transforms[i - 1], transform));
            }
        }

        // Draw cylinders between consecutive points
        let previousPoint = new THREE.Vector3(0, 0, 0);
        
        // Add a point at the origin for debugging
        const originGeometry = new THREE.SphereGeometry(0.02, 16, 16);
        const originMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000 });
        const originPoint = new THREE.Mesh(originGeometry, originMaterial);
        this.robotModel.add(originPoint);
        
        transforms.forEach((transform, index) => {
            // Get the position from the transform matrix
            const transformArray = transform.toArray() as number[][];
            const currentPoint = new THREE.Vector3(
                transformArray[0][3],
                transformArray[1][3],
                transformArray[2][3]
            );

            // Add a point at each joint for debugging
            const jointGeometry = new THREE.SphereGeometry(0.02, 16, 16);
            const jointMaterial = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
            const jointPoint = new THREE.Mesh(jointGeometry, jointMaterial);
            jointPoint.position.copy(currentPoint);
            this.robotModel!.add(jointPoint);

            // Create cylinder between previous and current point
            const direction = currentPoint.clone().sub(previousPoint);
            const length = direction.length();
            direction.normalize();

            const cylinderGeometry = new THREE.CylinderGeometry(0.02, 0.02, length, 32);
            const cylinderMaterial = new THREE.MeshPhongMaterial({ color: 0x808080 });
            const cylinder = new THREE.Mesh(cylinderGeometry, cylinderMaterial);

            // Position cylinder at midpoint
            cylinder.position.copy(previousPoint.clone().add(currentPoint).multiplyScalar(0.5));

            // Calculate rotation to align cylinder with direction
            // Create a quaternion that rotates from the cylinder's default orientation (along Y)
            // to the desired direction
            const quaternion = new THREE.Quaternion();
            quaternion.setFromUnitVectors(
                new THREE.Vector3(0, 1, 0), // Default cylinder orientation (along Y)
                direction
            );
            cylinder.setRotationFromQuaternion(quaternion);

            this.robotModel!.add(cylinder);
            previousPoint.copy(currentPoint);

            // If this is the last point, add a tripod to show orientation
            if (index === transforms.length - 1) {
                const tripod = new THREE.Group();
                
                // Create three axes (red for X, green for Y, blue for Z)
                const axisLength = 0.1;
                const axisRadius = 0.005;
                
                const xAxis = new THREE.Mesh(
                    new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 32),
                    new THREE.MeshPhongMaterial({ color: 0xff0000 })
                );
                xAxis.rotation.z = -Math.PI / 2;
                xAxis.position.x = axisLength / 2;
                
                const yAxis = new THREE.Mesh(
                    new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 32),
                    new THREE.MeshPhongMaterial({ color: 0x00ff00 })
                );
                yAxis.position.y = axisLength / 2;
                
                const zAxis = new THREE.Mesh(
                    new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 32),
                    new THREE.MeshPhongMaterial({ color: 0x0000ff })
                );
                zAxis.rotation.x = Math.PI / 2;
                zAxis.position.z = axisLength / 2;
                
                tripod.add(xAxis, yAxis, zAxis);
                tripod.position.copy(currentPoint);
                
                // Apply the final orientation to the tripod
                const finalRotation = new THREE.Matrix4().fromArray(transformArray.flat());
                tripod.setRotationFromMatrix(finalRotation);
                
                this.robotModel!.add(tripod);
            }
        });

        if (this.robotModel) {
            this.scene.add(this.robotModel);
        }
    }

    private onWindowResize(): void {
        this.camera.aspect = (window.innerWidth - 300) / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth - 300, window.innerHeight);
    }

    private animate(): void {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Initialize the visualizer when the page loads
window.addEventListener('load', () => {
    new RobotVisualizer();
}); 