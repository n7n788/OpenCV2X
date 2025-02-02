package artery.envmod.service;

import artery.application.ItsG5Service;

//
// CollectivePerceptionMockService generates mock CPMs, i.e. without ASN.1 encoding.
// These mock CPMs include the list of detected objects and field-of-view containers
// of the equipped sensors.
//
simple CollectivePerceptionMockService like ItsG5Service
{
    parameters:
        @signal[CpmSent](type=CollectivePerceptionMockMessage);
        @signal[CpmReceived](type=CollectivePerceptionMockMessage);

        @statistic[transmission](source=CpmSent; record=count,vector(cpmSource)?);
        @statistic[reception](source=CpmReceived; record=count,vector(cpmSource)?,vector(cpmGenerated)?);

        // cpmの送信時の生成時間を集めるシグナル
        @signal[cpmGenerationTime](type="omnetpp::SimTime");
        @statistic[cpmGenerationTime](title="cpmGenerationTime"; source="cpmGenerationTime"; record=vector);
        // cpmの送信元のSUMOのIDを集めるシグナル
        @signal[cpmSourceId](type="long");
        @statistic[cpmSourceId](title="cpmSourceId"; source="cpmSourceId"; record=vector);
        // cpmに含める車両のSUMOのIDを集めるシグナル
        @signal[cpmContainedId](type="long");
        @statistic[cpmContainedId](title="cpmContainedId"; source="cpmContainedId"; record=vector);

        // cpmに含める物体数のシグナル
        @signal[cpmContainedCnt](type="long");
        @statistic[cpmContainedCnt](title="cpmContainedCnt"; source="cpmContainedCnt"; record=vector);
        // cpmに含める候補数のシグナル
        @signal[cpmContainedAllCnt](type="long");
        @statistic[cpmContainedAllCnt](title="cpmContainedAllCnt"; source="cpmContainedAllCnt"; record=vector);
        // ETSIのルールからcpmに含める物体の候補数のシグナル
        @signal[cpmContainedEtsiCnt](type="long");
        @statistic[cpmContainedEtsiCnt](title="cpmContainedEtsiCnt"; source="cpmContainedEtsiCnt"; record=vector);

        //  物体認識確率を計算するためのシグナル
        // @signal[objectPerception](type="long");
        // @statistic[objectPerception](title="objectPerception"; source="objectPerception"; record=vector);

        // @signal[objectDistance](type="long");
        // @statistic[objectDistance](title="objectDistance"; source="objectDistance"; record=vector);

        // // AoIを計算するためのシグナル
        @signal[aoi](type="omnetpp::SimTime");
        @statistic[aoi](title="aoi"; source="aoi"; record=vector);

        @signal[perceptedObjectDistance](type="double");
        @statistic[perceptedObjectDistance](title="perceptedObjectDistance"; source="perceptedObjectDistance"; record=vector);

        @signal[cpmReceivedCount](type="long");
        @statistic[cpmReceivedCount](title="cpmReceivedCount100"; source="cpmReceivedCount"; record=vector);

        @signal[cpmReceivedCount200](type="long");
        @statistic[cpmReceivedCount200](title="cpmReceivedCount200"; source="cpmReceivedCount200"; record=vector);

        @signal[perceptedObjectReletiveVelocity](type="doule");
        @statistic[perceptedObjectReletiveVelocity](title="perceptedObjectReletiveVelocity"; source="perceptedObjectReletiveVelocity"; record=vector);

        // 対象をセンシングしているか
        @signal[sensing](type="long");
        @statistic[sensing](title="sensing"; source="sensing"; record=vector);

        //　対象が自動運転車かどうか
        @signal[connected](type="long");
        @statistic[connected](title="connected"; source="connected"; record=vector);
        
        // 加速または減速度合い
        @signal[riskRow](type="doule");
        @statistic[riskRow](title="riskRow"; source="riskRow"; record=vector);

        // 真の加速または減速度合い
        @signal[trueRiskRow](type="doule");
        @statistic[trueRiskRow](title="trueRiskRow"; source="trueRiskRow"; record=vector);

        // 対象物体の車両IDのハッシュ値
        @signal[hashId](type="long");
        @statistic[hashId](title="hashId"; source="hashId"; record=vector);
        
        @signal[riskClass](type="long");
        @statistic[riskClass](title="riskClass"; source="riskClass"; record=vector);

        @signal[collisionTime](type="double");
        @statistic[collisionTime](title="collisionTime"; source="collisionTime"; record=vector);

        @signal[perceptedVelocity](type="double");
        @statistic[perceptedVelocity](title="perceptedVelocity"; source="perceptedVelocity"; record=vector);
        
        @signal[perceptedAcceleration](type="double");
        @statistic[perceptedAcceleration](title="perceptedAcceleration"; source="perceptedAcceleration"; record=vector);

        @signal[perceptedYawrate](type="double");
        @statistic[perceptedYawrate](title="perceptedYawrate"; source="perceptedYawrate"; record=vector);
        
        // 位置誤差、速度誤差、進行方向の誤差、推定位置誤差を計算するシグナル
        @signal[posError](type="double");
        @statistic[posError](title="posError"; source="posError"; record=vector);

        @signal[estimatedPosError](type="double");
        @statistic[estimatedPosError](title="estimatedPosError"; source="estimatedPosError"; record=vector);

        @signal[speedError](type="double");
        @statistic[speedError](title="speedError"; source="speedError"; record=vector);

        @signal[headingError](type="double");
        @statistic[headingError](title="headingError"; source="headingError"; record=vector);

        // 危険度を送信するシグナル
        // @signal[risk](type="double");
        // @statistic[risk](title="risk"; source="risk"; record=vector);

        // // 危険度を計算したときの対象車両のx座標
        // @signal[riskPosX](type="double");
        // @statistic[riskPosX](title="riskPosX"; source="riskPosX"; record=vector);

        // // 危険度を計算したときの対象車両のy座標
        // @signal[riskPosY](type="double");
        // @statistic[riskPosY](title="riskPosY"; source="riskPosY"; record=vector);

        // // 危険度を計算したときの対象車両の速度
        // @signal[riskSpeed](type="double");
        // @statistic[riskSpeed](title="riskSpeed"; source="riskSpeed"; record=vector);

        // // 危険度を計算したときの対象車両の進行方向
        // @signal[riskHeading](type="double");
        // @statistic[riskHeading](title="riskHeading"; source="riskHeading"; record=vector);

        // // 危険度を計算したときの対象車両の加速度
        // @signal[riskAccelaration](type="double");
        // @statistic[riskAccelaration](title="riskAccelaration"; source="riskAccelaration"; record=vector);
        
        // // 危険度を計算したときの対象車両の加速度
        // @signal[riskYawrate](type="double");
        // @statistic[riskYawrate](title="riskYawrate"; source="riskYawrate"; record=vector);

        // // 受信した危険度を送信するシグナル
        // @signal[receivedRisk](type="double");
        // @statistic[receivedRisk](title="receivedRisk"; source="receivedRisk"; record=vector);

        string globalEnvironmentModule;
        bool generateAfterCam = default(false);
        bool withDccRestriction = default(true);
        
        double cpmInterval @unit(s) = default(0.1s);
        double cpmOffset @unit(s) = default(0.05s); // offset to trigger event (CAM or middleware)
        double fovInterval @unit(s) = default(1s);

        int dccProfile = default(3);
        int lengthHeader @unit(byte) = default(121B); // 121B
        int lengthFovContainer @unit(byte) = default(35B); // 35B
        int lengthObjectContainer @unit(byte) = default(35B); // 35B
        int lengthSensingIdContainer @unit(byte) = default(4B); // 4B
        // int lengthHeader @unit(byte) = default(20B);
        // int lengthFovContainer @unit(byte) = default(9B);
        // int lengthObjectContainer @unit(byte) = default(29B);

        // change in time including object in CPM (in s)
        double timeDelta @unit(s) = default(1s);
        
        // change in orientation including object in CPM (in degree)
        double headingDelta = default(4.0);

        // change in position including object in CPM
        double positionDelta @unit(m) = default(4.0m);

        // change in speed including object in CPM (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        string identityRegistryModule;

        // シミュレーション結果のログを取る範囲
        double lowerPosX = default(-10000);
        double upperPosX = default(10000);
        double lowerPosY = default(-10000);
        double upperPosY = default(10000); 

        
        // double intervalC0 @unit(s) = default(0.2s);
        // double intervalC1 @unit(s) = default(0.8s);
        // bool intervalFlag = default(false);
        // int allowedSentNum = default(5);
        // double changeInterval @unit(s) = default(1s);

        int cpmContainedCnt = default(3); // 1つのCPMに含めることが可能なオブジェクト数　提案手法では3つ、最大で15個
        bool etsiLimitedFlag = default(true); // ETSIの生成ルールで生成した後、cpmContainedCnt個までランダムに減らす
        bool proposedFlag2 = default(false); // 提案手法バージョン2を使用するかどうか

        int w = default(200); // 提案手法バージョン2で使用：受信数のウインドウサイズ
        double alpha = default(1); // 提案手法バージョン2で使用するパラメータ
        double beta = default(10); // 提案手法バージョン2で使用するパラメータ

        // 以下は使用してない
        bool proposedFlag = default(true); // 提案手法を使用するかどうか
        bool randomFlag = default(false); // CPMに含める物体を完全にランダムに決めるかどうか
        bool onlyCollisionTimeFlag = default(false); // 衝突予測時間のみから決めるかどうか
        bool allFlag = default(false); // センシングしている物体をすべてCPMに含めうるかどうか
       
        // 以上の5つがすべてfalseのときは、etsiのルールで可変長CPMを生成

        int k = 3; // ある車両のcpm送信台数の最大値
}
