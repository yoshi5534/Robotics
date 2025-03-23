import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { execSync } from 'child_process';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Build the project using Vite
console.log('Building project...');
execSync('npm run build', { stdio: 'inherit' });

// Read the built files
const distDir = path.join(__dirname, '../dist');
const indexHtml = fs.readFileSync(path.join(distDir, 'index.html'), 'utf8');

// Extract the script and style sources from index.html
const scriptMatch = indexHtml.match(/<script type="module" crossorigin src="([^"]+)"><\/script>/);
const styleMatch = indexHtml.match(/<link rel="stylesheet" href="([^"]+)">/);

if (!scriptMatch) {
    throw new Error('Could not find script source in index.html');
}
if (!styleMatch) {
    throw new Error('Could not find style source in index.html');
}

const scriptPath = scriptMatch[1];
const stylePath = styleMatch[1];
const mainJs = fs.readFileSync(path.join(distDir, scriptPath), 'utf8');
const mainCss = fs.readFileSync(path.join(distDir, stylePath), 'utf8');

// Create the standalone HTML content
const standaloneHtml = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Kinematics Visualizer</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/mathjs/9.4.4/math.js"></script>
    <style>
        ${mainCss}
    </style>
</head>
<body>
    <div id="canvas-container"></div>
    <div id="controls">
        <div class="control-group">
            <h3>Robot Configuration</h3>
            <select id="robot-select">
                <option value="staubli_rx90b">Stäubli RX90B</option>
                <option value="kuka_kr6">KUKA KR6</option>
                <option value="abb_irb120">ABB IRB120</option>
            </select>
        </div>
        <div class="control-group">
            <h3>Joint Controls</h3>
            <div id="joint-controls"></div>
        </div>
        <div class="control-group">
            <h3>Results</h3>
            <div class="result">
                <p>Position: <span id="position">[0, 0, 0]</span></p>
                <p>Orientation: <span id="orientation">[0, 0, 0]</span></p>
            </div>
        </div>
    </div>
    <script>
        ${mainJs}
    </script>
</body>
</html>`;

// Write the standalone HTML file
fs.writeFileSync(path.join(distDir, 'standalone.html'), standaloneHtml);

console.log('Standalone HTML file created successfully!'); 