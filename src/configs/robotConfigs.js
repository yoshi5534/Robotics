import staubliConfig from './staubli_rx90b.json';
import kukaConfig from './kuka_kr6.json';
import abbConfig from './abb_irb120.json';

const configs = {
    staubli_rx90b: staubliConfig,
    kuka_kr6: kukaConfig,
    abb_irb120: abbConfig
};

export async function loadRobotConfig(robotName) {
    if (!configs[robotName]) {
        throw new Error(`Robot configuration not found: ${robotName}`);
    }
    return configs[robotName];
} 