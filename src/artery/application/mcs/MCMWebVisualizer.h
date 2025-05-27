#ifndef MCMWEBVISUALIZER_H
#define MCMWEBVISUALIZER_H

#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/utility/AsioTask.h"
#include <boost/asio/ip/tcp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace artery {

// WebSocketを使って経路データをブラウザに送信するクラス
class MCMWebVisualizer {
public:
    static MCMWebVisualizer& getInstance();
    
    // シミュレーションの開始時に呼び出す
    void initialize(omnetpp::cModule* module, int port = 8080);
    
    // MCMを受信したときに呼び出す
    void visualizeMCM(const ManeuverCoordinationMessage* mcm);
    
    // 自車の最新パスを設定
    void setEgoPaths(const std::string& vehicleId, const Path& plannedPath, const Path& desiredPath);
    
    // 候補経路を設定
    void setPathCandidates(const std::string& vehicleId, const std::vector<Path>& candidates);
    
    // クリーンアップ
    void close();

private:
    MCMWebVisualizer() = default;
    ~MCMWebVisualizer() = default;
    
    // JSONに変換するヘルパー関数
    std::string pathToJson(const Path& path);
    std::string pathsArrayToJson(const std::vector<Path>& paths);
    std::string createJsonUpdate();
    void updateJsonFile();
    
    // WebSocketコネクション
    std::unique_ptr<artery::AsioTask> mWebSocketTask;
    omnetpp::cModule* mModule = nullptr;
    
    // 車両データの保存
    struct VehiclePathData {
        Path plannedPath;
        Path desiredPath;
        std::vector<Path> pathCandidates;
        double lonPos;
        double latPos;
        double lonSpeed;
        double latSpeed;
        double lonAccel;
        double latAccel;
        double lastUpdateTime;
    };

    std::unordered_map<std::string, VehiclePathData> mVehicleData;
};

} // namespace artery

#endif // MCMWEBVISUALIZER_H
