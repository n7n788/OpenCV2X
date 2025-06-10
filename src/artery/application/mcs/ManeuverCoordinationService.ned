package artery.application.mcs;

import artery.application.ItsG5Service;

simple ManeuverCoordinationService like ItsG5Service
{
    parameters:
        // 車両の設定
        double vehicleLength @unit(m) = default(5m); // 車両長

        // 道路の設定
        int numLanes = default(3); // レーン数
        double laneWidth @unit(m) = default(3.2m); // レーン幅

        // MCSの設定
        double convergenceTime @unit(s) = default(5s); // 経路の時間幅
        double desiredCostThreshold = default(100.0); // 希望経路を選択するための予定経路とのコスト閾値
        double safetySecond @unit(s) = default(2.0s); // 安全秒数
        double laneChangeInterval @unit(s) = default(5.0s); // 連続したレーン変更を避けるための間隔

        // 可視化設定
        bool enableVisualization = default(true);
        int visualizationPort = default(8080);
}