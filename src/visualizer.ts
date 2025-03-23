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

        // Add links
        let currentPosition = [0, 0, 0];
        let currentRotation = new THREE.Matrix4();
        
        this.robot.config.dhParameters.forEach((params: DHParameters, index: number) => {
            const linkGeometry = new THREE.CylinderGeometry(0.02, 0.02, params.a, 32);
            const linkMaterial = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
            const link = new THREE.Mesh(linkGeometry, linkMaterial);
            
            // Position and rotate link
            link.position.set(currentPosition[0], currentPosition[1], currentPosition[2]);
            link.rotation.x = params.alpha;
            link.rotation.z = this.jointAngles[index];
            
            this.robotModel?.add(link);
            
            // Update position and rotation for next link
            if (!this.robot) return;
            const transform = this.robot.calculateDHTransform(params, this.jointAngles[index]);
            const transformArray = transform.toArray() as number[][];
            currentPosition = [transformArray[0][3], transformArray[1][3], transformArray[2][3]];
            currentRotation = new THREE.Matrix4().fromArray(transformArray.flat());
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