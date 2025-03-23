#include "MCMWebVisualizer.h"
#include "artery/utility/AsioScheduler.h"
#include <omnetpp.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/write.hpp>
#include <sstream>
#include <iostream>

namespace artery {

MCMWebVisualizer& MCMWebVisualizer::getInstance() {
    static MCMWebVisualizer instance;
    return instance;
}

void MCMWebVisualizer::initialize(omnetpp::cModule* module, int port) {
    mModule = module;
    
    // AsioSchedulerの取得
    auto scheduler = dynamic_cast<AsioScheduler*>(omnetpp::getSimulation()->getScheduler());
    if (!scheduler) {
        std::cout << "MCMWebVisualizer requires AsioScheduler" << std::endl;
        return;
    }
    
    // WebSocketサーバーを設立
    try {
        using namespace boost::asio::ip;
        
        // m_serviceに直接アクセス（getIoServiceメソッドは存在しない）
        // AsioSchedulerのm_serviceがpublicでない場合は、別の方法が必要
        boost::asio::io_service& io_service = scheduler->m_service;
        
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));
        
        // 接続を待つ
        tcp::socket socket(io_service);
        acceptor.accept(socket);
        
        // Asioタスクを作成
        mWebSocketTask.reset(new AsioTask(*scheduler, std::move(socket), *module));
        
        std::cout << "MCMWebVisualizer: Connection established on port " << port << std::endl;
        
        // HTMLページを送信
        std::string html = R"(
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
        
        // Webソケット接続
        const socket = new WebSocket('ws://' + window.location.hostname + ':8081');
        
        socket.onmessage = function(event) {
            const data = JSON.parse(event.data);
            updateVehicles(data);
        };
        
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
        
        // スケール変更時の更新
        scale.addEventListener('input', function() {
            socket.send(JSON.stringify({ action: 'requestUpdate' }));
        });
    </script>
</body>
</html>
        )";
        
        mWebSocketTask->write(boost::asio::buffer(html));
        
    } catch (const std::exception& e) {
        std::cerr << "MCMWebVisualizer: Error initializing WebSocket: " << e.what() << std::endl;
    }
}

void MCMWebVisualizer::visualizeMCM(const ManeuverCoordinationMessage* mcm) {
    if (!mWebSocketTask) return;
    
    const std::string& vehicleId = mcm->getTraciId();
    
    // 車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = mcm->getPlannedPath();
    data.desiredPath = mcm->getDesiredPath();
    data.latPos = mcm->getLatPos();
    data.lonPos = mcm->getLonPos();
    data.latSpeed = mcm->getLatSpeed();
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータを送信
    std::string jsonUpdate = createJsonUpdate();
    mWebSocketTask->write(boost::asio::buffer(jsonUpdate));
}

void MCMWebVisualizer::setEgoPaths(const std::string& vehicleId, const FrenetPath& plannedPath, const FrenetPath& desiredPath) {
    if (!mWebSocketTask) return;
    
    // 自車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = plannedPath;
    data.desiredPath = desiredPath;
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータを送信
    std::string jsonUpdate = createJsonUpdate();
    mWebSocketTask->write(boost::asio::buffer(jsonUpdate));
}

void MCMWebVisualizer::close() {
    // WebSocketタスクをクリーンアップ
    mWebSocketTask.reset();
    
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