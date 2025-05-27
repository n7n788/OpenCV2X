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
        // データ出力ディレクトリを確認（既に存在する前提）
        std::string dataDir = "mcm_visualization";
        
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
        
        // ログメッセージのみ表示
        std::cout << "MCMWebVisualizer: Initialized - using visualization files in mcm_visualization/" << std::endl;
        std::cout << "To view visualization: " << std::endl;
        std::cout << "  1. Run a HTTP server in your project directory:" << std::endl;
        std::cout << "  2. Open http://localhost:8080/mcm_visualization/ in your browser" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "MCMWebVisualizer: Error initializing: " << e.what() << std::endl;
    }
}

void MCMWebVisualizer::visualizeMCM(const ManeuverCoordinationMessage* mcm) {
    const std::string& vehicleId = mcm->getTraciId();
    
    // 車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = mcm->getPlannedPath();
    data.desiredPath = mcm->getDesiredPath();
    data.lonPos = mcm->getLonPos();
    data.latPos = mcm->getLatPos();
    data.lonSpeed = mcm->getLonSpeed();
    data.latSpeed = mcm->getLatSpeed();
    data.lonAccel = mcm->getLonAccel();
    data.latAccel = mcm->getLatAccel();
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

void MCMWebVisualizer::setEgoPaths(const std::string& vehicleId, const Path& plannedPath, const Path& desiredPath) {
    // 自車両データを更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.plannedPath = plannedPath;
    data.desiredPath = desiredPath;
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータをファイルに書き込む
    updateJsonFile();
}

void MCMWebVisualizer::setPathCandidates(const std::string& vehicleId, const std::vector<Path>& candidates) {
    // 候補経路を更新
    VehiclePathData& data = mVehicleData[vehicleId];
    data.pathCandidates = candidates;
    data.lastUpdateTime = omnetpp::simTime().dbl();
    
    // JSONデータをファイルに書き込む
    updateJsonFile();
}

void MCMWebVisualizer::close() {
    // 保存されたデータをクリア
    mVehicleData.clear();
    
    // モジュール参照をクリア
    mModule = nullptr;
    
    std::cout << "MCMWebVisualizer closed" << std::endl;
}

std::string MCMWebVisualizer::pathToJson(const Path& path) {
    std::ostringstream json;
    json << "[";
    
    const auto& lonTrajectory = path.getLonTrajectory().getPoses();
    const auto& latTrajectory = path.getLatTrajectory().getPoses();
    
    size_t size = std::min(lonTrajectory.size(), latTrajectory.size());
    for (size_t i = 0; i < size; ++i) {
        json << "{\"x\":" << lonTrajectory[i] << ",\"y\":" << latTrajectory[i] << "}";
        if (i < size - 1) json << ",";
    }
    
    json << "]";
    return json.str();
}

std::string MCMWebVisualizer::pathsArrayToJson(const std::vector<Path>& paths) {
    std::ostringstream json;
    json << "[";
    
    for (size_t p = 0; p < paths.size(); ++p) {
        json << pathToJson(paths[p]);
        if (p < paths.size() - 1) json << ",";
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
        json << "\"position\":{\"x\":" << data.lonPos << ",\"y\":" << data.latPos << "},";
        // 主な速度は縦方向の速度を使用
        json << "\"speed\":" << data.lonSpeed << ",";
        // 加速度も追加
        json << "\"acceleration\":" << data.lonAccel << ",";
        // 横方向の速度と加速度も追加
        json << "\"latSpeed\":" << data.latSpeed << ",";
        json << "\"latAccel\":" << data.latAccel << ",";
        json << "\"plannedPath\":" << pathToJson(data.plannedPath) << ",";
        json << "\"desiredPath\":" << pathToJson(data.desiredPath) << ",";
        json << "\"pathCandidates\":" << pathsArrayToJson(data.pathCandidates);
        json << "}";
    }
    
    json << "}";
    return json.str();
}

// 既存のsetEgoPathsやsetPathCandidatesメソッドからJSONデータ更新処理を分離
void MCMWebVisualizer::updateJsonFile() {
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
} // namespace artery
