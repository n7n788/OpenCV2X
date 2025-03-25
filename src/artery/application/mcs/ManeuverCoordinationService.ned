package artery.application.mcs;

import artery.application.ItsG5Service;

simple ManeuverCoordinationService like ItsG5Service
{
    parameters:
        // 更新間隔
        double updateInterval @unit(s) = default(0.1s);
        
        // 車両パラメータ
        double maxSpeed @unit(mps) = default(33.3mps); // 約120km/h
        double vehicleLength @unit(m) = default(5m);
        double safetyDistance @unit(m) = default(10m);
        
        // レーン設定
        int numLanes = default(3);
        double laneWidth @unit(m) = default(3.2m);
        
        // 経路計画パラメータ
        double convergenceTime @unit(s) = default(5s);

        // 可視化設定
        bool enableVisualization = default(true);
        int visualizationPort = default(8080);
}