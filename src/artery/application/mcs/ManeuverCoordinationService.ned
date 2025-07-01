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
        double exitLonPos @unit(m) = default(200m); // 道路を出る位置、旅行時間の計算に使用

        // MCSの設定
        double convergenceTime @unit(s) = default(5s); // 経路の時間幅
        double safetySecond @unit(s) = default(2.0s); // 安全秒数
        double laneChangeInterval @unit(s) = default(5.0s); // 連続したレーン変更を避けるための間隔
        double negotiationTimeout @unit(s) = default(1.0s); // 交渉のタイムアウト時間

        // コスト計算
        double desiredCostThreshold = default(200.0); // 希望経路を選択するための予定経路とのコスト閾値

        // 可視化設定
        bool enableVisualization = default(true);
        int visualizationPort = default(8080);

        // シグナル
        // 車速
        @signal[velocity](type="double");
        @statistic[velocity](title="velocity"; source="velocity"; record=vector);

        // 旅行時間
        @signal[travelTime](type="double");
        @statistic[travelTime](title="travelTime"; source="travelTime"; record=vector);

        // 交渉時間
        @signal[negotiationTime](type="double");
        @statistic[negotiationTime](title="negotiationTime"; source="negotiationTime"; record=vector);

        // 交渉結果 (0: 失敗, 1: 成功)
        @signal[negotiationResult](type="int");
        @statistic[negotiationResult](title="negotiationResult"; source="negotiationResult"; record=vector);
}
