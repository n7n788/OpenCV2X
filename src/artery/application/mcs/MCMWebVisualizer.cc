#include "artery/application/mcs/MCMWebVisualizer.h"
#include <omnetpp.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

namespace artery {

MCMWebVisualizer& MCMWebVisualizer::getInstance() {
    static MCMWebVisualizer instance;
    return instance;
}

void MCMWebVisualizer::initialize(omnetpp::cModule* module, int port) {
    mModule = module;
    
    try {
        // データ出力ディレクトリを作成
        std::string dataDir = "mcm_visualization";
        // std::filesystemの代わりにmkdirを使用
        #ifdef _WIN32
            mkdir(dataDir.c_str());
        #else
            mkdir(dataDir.c_str(), 0755);
        #endif
        
        // HTMLファイルを作成
        std::string htmlPath = dataDir + "/index.html";
        std::ofstream htmlFile;
        htmlFile.open(htmlPath.c_str());
        if (!htmlFile.is_open()) {
            std::cerr << "MCMWebVisualizer: Could not open file for writing: " << htmlPath << std::endl;
            return;
        }
        htmlFile << R"(
<!DOCTYPE html>
<html>
<head>
    <title>Frenet Path Visualizer</title>
    <style>
        body { margin: 0; font-family: Arial, sans-serif; }
        #visualizer { width: 100%; height: 100vh; }
        .vehicle { position: absolute; width: 20px; height: 10px; border-radius: 3px; }
        .path { position: absolute; height: 2px; border-radius: 1px; }
        .planned-path { background-color: rgba(0, 100, 255, 0.7); }
        .desired-path { background-color: rgba(255, 50, 50, 0.7); border-style: dashed; }
        .controls { position: fixed; top: 10px; right: 10px; background: rgba(255,255,255,0.8); padding: 10px; }
    </style>
</head>
<body>
    <div id="visualizer"></div>
    <div class="controls">
        <label>スケール: <input type="range" id="scale" min="1" max="20" value="10"></label>
        <div>
            <span style="background-color: rgba(0, 100, 255, 0.7); display: inline-block; width: 20px; height: 10px;"></span> 予定経路
            <span style="background-color: rgba(255, 50, 50, 0.7); display: inline-block; width: 20px; height: 10px; margin-left: 10px;"></span> 希望経路
        </div>
    </div>
    
    <script>
        const visualizer = document.getElementById('visualizer');
        const scale = document.getElementById('scale');
        const vehicles = {};
        const SCALE_FACTOR = 10; // メートルをピクセルに変換
        
        // JSONデータを定期的に取得
        function fetchData() {
            fetch('data.json?' + new Date().getTime())
                .then(response => response.json())
                .then(data => {
                    updateVehicles(data);
                    setTimeout(fetchData, 500); // 500ミリ秒ごとに更新
                })
                .catch(err => {
                    console.error('Error fetching data:', err);
                    setTimeout(fetchData, 1000); // エラー時は1秒後に再試行
                });
        }
        
        function updateVehicles(data) {
            const scaleValue = parseInt(scale.value);
            
            for (const vehicleId in data) {
                const vehicleData = data[vehicleId];
                
                // 車両要素を取得または作成
                let vehicle = vehicles[vehicleId];
                if (!vehicle) {
                    vehicle = {
                        element: document.createElement('div'),
                        plannedPath: document.createElement('div'),
                        desiredPath: document.createElement('div'),
                        label: document.createElement('div')
                    };
                    
                    // 車両要素の設定
                    vehicle.element.className = 'vehicle';
                    vehicle.element.style.backgroundColor = '#2ca02c';
                    
                    // パス要素の設定
                    vehicle.plannedPath.className = 'path planned-path';
                    vehicle.desiredPath.className = 'path desired-path';
                    
                    // ラベル設定
                    vehicle.label.style.position = 'absolute';
                    vehicle.label.style.fontSize = '12px';
                    vehicle.label.textContent = vehicleId;
                    
                    visualizer.appendChild(vehicle.element);
                    visualizer.appendChild(vehicle.plannedPath);
                    visualizer.appendChild(vehicle.desiredPath);
                    visualizer.appendChild(vehicle.label);
                    
                    vehicles[vehicleId] = vehicle;
                }
                
                // 車両の位置を更新
                const x = vehicleData.position.x * scaleValue * SCALE_FACTOR;
                const y = vehicleData.position.y * scaleValue * SCALE_FACTOR;
                vehicle.element.style.transform = `translate(${x}px, ${y}px)`;
                
                // ラベルの位置を更新
                vehicle.label.style.transform = `translate(${x}px, ${y - 15}px)`;
                
                // 経路を更新
                if (vehicleData.plannedPath && vehicleData.plannedPath.length > 0) {
                    updatePath(vehicle.plannedPath, vehicleData.plannedPath, scaleValue);
                }
                
                if (vehicleData.desiredPath && vehicleData.desiredPath.length > 0) {
                    updatePath(vehicle.desiredPath, vehicleData.desiredPath, scaleValue);
                }
            }
        }
        
        function updatePath(pathElement, pathData, scaleValue) {
            if (pathData.length < 2) return;
            
            // パスの開始点
            const startX = pathData[0].x * scaleValue * SCALE_FACTOR;
            const startY = pathData[0].y * scaleValue * SCALE_FACTOR;
            
            // パスの終了点
            const endX = pathData[pathData.length - 1].x * scaleValue * SCALE_FACTOR;
            const endY = pathData[pathData.length - 1].y * scaleValue * SCALE_FACTOR;
            
            // 長さと角度を計算
            const length = Math.sqrt(Math.pow(endX - startX, 2) + Math.pow(endY - startY, 2));
            const angle = Math.atan2(endY - startY, endX - startX) * 180 / Math.PI;
            
            // パスの位置とスタイルを設定
            pathElement.style.width = `${length}px`;
            pathElement.style.transform = `translate(${startX}px, ${startY}px) rotate(${angle}deg)`;
            pathElement.style.transformOrigin = '0 0';
        }
        
        // データの取得を開始
        fetchData();
    </script>
</body>
</html>
        )";
        htmlFile.close();
        
        // 初期の空のデータファイルを作成
        std::string dataPath = dataDir + "/data.json";
        std::ofstream dataFile;
        dataFile.open(dataPath.c_str());
        if (!dataFile.is_open()) {
            std::cerr << "MCMWebVisualizer: Could not open file for writing: " << dataPath << std::endl;
            return;
        }
        dataFile << "{}";
        dataFile.close();
        
        std::cout << "MCMWebVisualizer: Created visualization files in mcm_visualization/" << std::endl;
        std::cout << "To view visualization: " << std::endl;
        std::cout << "  1. Run a HTTP server in your project directory:" << std::endl;
        std::cout << "     python3 -m http.server 8080" << std::endl;
        std::cout << "     (or for Python 2: python -m SimpleHTTPServer 8080)" << std::endl;
        std::cout << "  2. Open http://localhost:8080/mcm_visualization/ in your browser" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "MCMWebVisualizer: Error initializing files: " << e.what() << std::endl;
    }
}

void MCMWebVisualizer::visualizeMCM(const ManeuverCoordinationMessage* mcm) {
    const std::string& vehicleId = mcm->getTraciId();
    
    // 車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = mcm->getPlannedPath();
    data.desiredPath = mcm->getDesiredPath();
    data.latPos = mcm->getLatPos();
    data.lonPos = mcm->getLonPos();
    data.latSpeed = mcm->getLatSpeed();
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータをファイルに書き込む
    try {
        std::string jsonUpdate = createJsonUpdate();
        std::ofstream dataFile;
        dataFile.open("mcm_visualization/data.json");
        if (!dataFile.is_open()) {
            std::cerr << "MCMWebVisualizer: Could not open data.json for writing" << std::endl;
            return;
        }
        dataFile << jsonUpdate;
        dataFile.close();
    } catch (const std::exception& e) {
        std::cerr << "MCMWebVisualizer: Error writing data file: " << e.what() << std::endl;
    }
}

void MCMWebVisualizer::setEgoPaths(const std::string& vehicleId, const FrenetPath& plannedPath, const FrenetPath& desiredPath) {
    // 自車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = plannedPath;
    data.desiredPath = desiredPath;
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータをファイルに書き込む
    try {
        std::string jsonUpdate = createJsonUpdate();
        std::ofstream dataFile;
        dataFile.open("mcm_visualization/data.json");
        if (!dataFile.is_open()) {
            std::cerr << "MCMWebVisualizer: Could not open data.json for writing" << std::endl;
            return;
        }
        dataFile << jsonUpdate;
        dataFile.close();
    } catch (const std::exception& e) {
        std::cerr << "MCMWebVisualizer: Error writing data file: " << e.what() << std::endl;
    }
}

void MCMWebVisualizer::close() {
    // 保存されたデータをクリア
    mVehicleData.clear();
    
    // モジュール参照をクリア
    mModule = nullptr;
    
    std::cout << "MCMWebVisualizer closed" << std::endl;
}

std::string MCMWebVisualizer::pathToJson(const FrenetPath& path) {
    std::ostringstream json;
    json << "[";
    
    const auto& latTrajectory = path.getLatTrajectory().getPoses();
    const auto& lonTrajectory = path.getLonTrajectory().getPoses();
    
    size_t size = std::min(latTrajectory.size(), lonTrajectory.size());
    for (size_t i = 0; i < size; ++i) {
        json << "{\"x\":" << latTrajectory[i] << ",\"y\":" << lonTrajectory[i] << "}";
        if (i < size - 1) json << ",";
    }
    
    json << "]";
    return json.str();
}

std::string MCMWebVisualizer::createJsonUpdate() {
    std::ostringstream json;
    json << "{";
    
    bool first = true;
    for (const auto& pair : mVehicleData) {
        // 一定時間更新がない車両は除外
        if (omnetpp::simTime().dbl() - pair.second.lastUpdateTime > 5.0) continue;
        
        if (!first) json << ",";
        first = false;
        
        const std::string& vehicleId = pair.first;
        const VehiclePathData& data = pair.second;
        
        json << "\"" << vehicleId << "\":{";
        json << "\"position\":{\"x\":" << data.latPos << ",\"y\":" << data.lonPos << "},";
        json << "\"speed\":" << data.latSpeed << ",";
        json << "\"plannedPath\":" << pathToJson(data.plannedPath) << ",";
        json << "\"desiredPath\":" << pathToJson(data.desiredPath);
        json << "}";
    }
    
    json << "}";
    return json.str();
}

} // namespace artery