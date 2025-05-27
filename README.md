# Maneuver Coordination Service (MCS) ドキュメント

## 1. 概要

Maneuver Coordination Service (MCS) は、車両間で経路情報を共有し、協調的な運転操作を実現するためのV2X（Vehicle-to-Everything）サービスです。各車両は自身の予定経路と希望経路をManeuver Coordination Message (MCM) として送信し、受信した情報を基に衝突回避と最適な経路選択を行います。

### 主な機能
- 車両の現在状態（位置、速度、加速度）の共有
- 予定経路（Planned Path）と希望経路（Desired Path）の生成と送信
- 他車両との衝突判定と回避
- 優先度に基づく経路調整
- リアルタイムでの経路可視化

## 2. システムアーキテクチャ

### 2.1 主要コンポーネント

#### ManeuverCoordinationService
- MCMの生成と送信を管理するメインサービス
- 他車両からのMCMを受信して処理
- 経路の衝突判定と最適化を実行

#### ManeuverCoordinationMessage (MCM)
- 車両間で交換されるメッセージ
- 車両ID、現在状態、予定経路、希望経路を含む

#### PathGenerator
- 様々な条件下での経路候補を生成
- 最高速度到達経路と前方車両追従経路の生成

#### Path & Trajectory
- Path: 縦方向と横方向の軌跡を組み合わせた経路
- Trajectory: 時系列の位置、速度、加速度、ジャーク情報

#### Polynomial (FourthDegreePolynomial, FifthDegreePolynomial)
- 滑らかな軌跡を生成するための多項式
- 境界条件（位置、速度、加速度）を満たす曲線を計算

#### MCMWebVisualizer
- Webブラウザでリアルタイムに経路を可視化
- JSON形式でデータを出力

## 3. クラス図

```mermaid
classDiagram
    class ManeuverCoordinationService {
        -string mTraciId
        -PathGenerator mPlanner
        -VehicleDataProvider* mVehicleDataProvider
        -VehicleController* mVehicleController
        -map~string,Path~ mReceivedPlannedPaths
        -map~string,Path~ mReceivedDesiredPaths
        -set~string~ mAcceptedIds
        +initialize(stage)
        +trigger()
        +indicate(indication, packet)
        -generate() ManeuverCoordinationMessage*
        -checkCollision(path1, path2) bool
        -hasPriority(senderId, path) bool
        -acceptPath(senderId, desiredPath) bool
    }

    class ManeuverCoordinationMessage {
        -string mTraciId
        -Path mPlannedPath
        -Path mDesiredPath
        -double mLonPos
        -double mLonSpeed
        -double mLonAccel
        -double mLatPos
        -double mLatSpeed
        -double mLatAccel
        +getTraciId() string
        +getPlannedPath() Path
        +getDesiredPath() Path
        +setTraciId(id)
        +setPlannedPath(path)
        +setDesiredPath(path)
    }

    class PathGenerator {
        +generateMaxSpeedPathCandidates() vector~Path~
        +generateMaxPosPathCandidates() vector~Path~
        +generateSpeedPath() Path
    }

    class Path {
        -Trajectory mLonTrajectory
        -Trajectory mLatTrajectory
        -double mCost
        +getLonTrajectory() Trajectory
        +getLatTrajectory() Trajectory
        +getCost() double
        +calculateCost(convergenceTime, targetLonPos, targetLonSpeed)
    }

    class Trajectory {
        -vector~double~ mPoses
        -vector~double~ mSpeeds
        -vector~double~ mAccels
        -vector~double~ mJerks
        +getPoses() vector~double~
        +getSpeeds() vector~double~
        +getAccels() vector~double~
        +getJerks() vector~double~
    }

    class Polynomial {
        <<abstract>>
        +calc_x(t) double
        +calc_v(t) double
        +calc_a(t) double
        +calc_j(t) double
    }

    class FourthDegreePolynomial {
        -double a0, a1, a2, a3, a4
        +calc_x(t) double
        +calc_v(t) double
        +calc_a(t) double
        +calc_j(t) double
    }

    class FifthDegreePolynomial {
        -double a0, a1, a2, a3, a4, a5
        +calc_x(t) double
        +calc_v(t) double
        +calc_a(t) double
        +calc_j(t) double
    }

    class MCMWebVisualizer {
        -map~string,VehiclePathData~ mVehicleData
        -cModule* mModule
        +initialize(module, port)
        +visualizeMCM(mcm)
        +setEgoPaths(vehicleId, plannedPath, desiredPath)
        +setPathCandidates(vehicleId, candidates)
        -createJsonUpdate() string
    }

    ManeuverCoordinationService --> ManeuverCoordinationMessage : generates
    ManeuverCoordinationService --> PathGenerator : uses
    ManeuverCoordinationService --> MCMWebVisualizer : uses
    ManeuverCoordinationService --> Path : manages
    PathGenerator --> Path : creates
    Path --> Trajectory : contains
    Trajectory --> Polynomial : uses
    Polynomial <|-- FourthDegreePolynomial
    Polynomial <|-- FifthDegreePolynomial
    MCMWebVisualizer --> ManeuverCoordinationMessage : visualizes
```

## 4. シーケンス図

### 4.1 MCM送信プロセス

```mermaid
sequenceDiagram
    participant Timer
    participant MCS as ManeuverCoordinationService
    participant PG as PathGenerator
    participant VDP as VehicleDataProvider
    participant VC as VehicleController
    participant Network
    participant Viz as MCMWebVisualizer

    Timer->>MCS: trigger()
    MCS->>MCS: generate()
    
    MCS->>VDP: getPosition(), getSpeed(), getAcceleration()
    VDP-->>MCS: 現在の車両状態
    
    MCS->>PG: generateMaxSpeedPathCandidates()
    PG-->>MCS: 最高速度到達経路候補
    
    MCS->>PG: generateMaxPosPathCandidates()
    PG-->>MCS: 前方車両追従経路候補
    
    Note over MCS: 予定経路の決定
    MCS->>MCS: 他車両の予定経路との衝突チェック
    MCS->>MCS: 優先度判定（hasPriority）
    MCS->>MCS: 受け入れ車両の希望経路との衝突チェック
    MCS->>MCS: 最小コストの予定経路を選択
    
    Note over MCS: 希望経路の決定
    MCS->>MCS: 受け入れ車両の希望経路との衝突チェック
    MCS->>MCS: 障害物との衝突チェック
    MCS->>MCS: 最小コストの希望経路を選択
    MCS->>MCS: コスト差分が閾値以上か確認
    
    MCS->>MCS: create MCM
    
    MCS->>VC: setSpeed() / changeLane()
    
    MCS->>Network: request(MCM)
    
    MCS->>Viz: setEgoPaths()
    MCS->>Viz: setPathCandidates()
```

### 4.2 MCM受信プロセス

```mermaid
sequenceDiagram
    participant Network
    participant MCS as ManeuverCoordinationService
    participant Viz as MCMWebVisualizer
    
    Network->>MCS: indicate(packet)
    MCS->>MCS: extract MCM from packet
    
    MCS->>MCS: 送信元車両IDを取得
    MCS->>MCS: mReceivedPlannedPaths[senderId] = plannedPath
    MCS->>MCS: mReceivedDesiredPaths[senderId] = desiredPath
    MCS->>MCS: mVehiclePoses[senderId] = position
    MCS->>MCS: mVehicleSpeeds[senderId] = speed
    
    MCS->>Viz: visualizeMCM(mcm)
    
    MCS->>MCS: delete packet
```

### 4.3 経路生成プロセス

```mermaid
sequenceDiagram
    participant MCS as ManeuverCoordinationService
    participant PG as PathGenerator
    participant P as Polynomial
    participant T as Trajectory
    participant Path

    MCS->>PG: generateMaxSpeedPathCandidates()
    
    loop 各レーン位置
        PG->>P: FifthDegreePolynomial(横方向)
        P-->>PG: 横方向多項式
        
        loop 各目標速度
            PG->>P: FourthDegreePolynomial(縦方向)
            P-->>PG: 縦方向多項式
            
            PG->>T: new Trajectory(polynomial)
            loop 各時間ステップ(0.1秒)
                T->>P: calc_x(t), calc_v(t), calc_a(t), calc_j(t)
                P-->>T: 位置、速度、加速度、ジャーク
            end
            
            PG->>Path: new Path(lonTrajectory, latTrajectory)
            Path->>Path: calculateCost()
            
            PG->>PG: pathCandidates.add(path)
        end
    end
    
    PG-->>MCS: pathCandidates
```

### 4.4 予定経路（Planned Path）決定プロセス

```mermaid
sequenceDiagram
    participant MCS as ManeuverCoordinationService
    participant PC as PathCandidates
    participant RP as ReceivedPaths
    participant AI as AcceptedIds

    MCS->>PC: 全経路候補を取得
    
    loop 各経路候補
        MCS->>MCS: isValid = true
        
        Note over MCS: 優先度の高い車両との衝突チェック
        loop 受信した予定経路
            MCS->>RP: 他車両IDと予定経路を取得
            MCS->>MCS: hasPriority(otherId, candidatePath)?
            alt 他車両の優先度が高い
                MCS->>MCS: checkCollision(candidatePath, otherPath)?
                alt 衝突あり
                    MCS->>MCS: isValid = false
                    MCS->>MCS: break
                end
            end
        end
        
        Note over MCS: 受け入れた車両の希望経路との衝突チェック
        loop 受け入れリストの車両ID
            MCS->>AI: acceptedIdを取得
            MCS->>RP: 該当車両の希望経路を取得
            MCS->>MCS: checkCollision(candidatePath, desiredPath)?
            alt 衝突あり
                MCS->>MCS: isValid = false
                MCS->>MCS: break
            end
        end
        
        alt isValid == true && cost < minCost
            MCS->>MCS: minCost = cost
            MCS->>MCS: plannedPath = candidatePath
        end
    end
    
    MCS-->>MCS: 選択された予定経路
```

### 4.5 希望経路（Desired Path）決定プロセス

```mermaid
sequenceDiagram
    participant MCS as ManeuverCoordinationService
    participant PC as PathCandidates
    participant RP as ReceivedPaths
    participant AI as AcceptedIds

    MCS->>PC: 全経路候補を取得
    
    loop 各経路候補
        MCS->>MCS: isValid = true
        
        Note over MCS: 受け入れた車両の希望経路との衝突チェック
        loop 受け入れリストの車両ID
            MCS->>AI: acceptedIdを取得
            MCS->>RP: 該当車両の希望経路を取得
            MCS->>MCS: checkCollision(candidatePath, desiredPath)?
            alt 衝突あり
                MCS->>MCS: isValid = false
                MCS->>MCS: break
            end
        end
        
        Note over MCS: 障害物との衝突チェック
        loop 受信した予定経路
            MCS->>RP: 他車両IDと予定経路を取得
            MCS->>MCS: isObstacle(otherId)?
            alt 障害物である
                MCS->>MCS: checkCollision(candidatePath, obstaclePath)?
                alt 衝突あり
                    MCS->>MCS: isValid = false
                    MCS->>MCS: break
                end
            end
        end
        
        alt isValid == true && cost < minCost
            MCS->>MCS: minCost = cost
            MCS->>MCS: desiredPath = candidatePath
        end
    end
    
    Note over MCS: コスト差分チェック
    MCS->>MCS: costDiff = plannedPath.cost - desiredPath.cost
    alt costDiff < threshold
        MCS->>MCS: desiredPath = empty
        Note over MCS: 希望経路を送信しない
    end
    
    MCS-->>MCS: 選択された希望経路
```

## 5. データフロー

### 5.1 MCM (Maneuver Coordination Message) 構造

```
MCM {
    - traciId: 車両識別子
    - plannedPath: 予定経路（他車両との調整済み）
    - desiredPath: 希望経路（理想的な経路）
    - lonPos: 現在の縦方向位置 [m]
    - lonSpeed: 現在の縦方向速度 [m/s]
    - lonAccel: 現在の縦方向加速度 [m/s²]
    - latPos: 現在の横方向位置 [m]
    - latSpeed: 現在の横方向速度 [m/s]
    - latAccel: 現在の横方向加速度 [m/s²]
}
```

### 5.2 Path構造

```
Path {
    - lonTrajectory: 縦方向の軌跡
    - latTrajectory: 横方向の軌跡
    - cost: 経路のコスト値
}

Trajectory {
    - poses[]: 位置の時系列データ [m]
    - speeds[]: 速度の時系列データ [m/s]
    - accels[]: 加速度の時系列データ [m/s²]
    - jerks[]: ジャークの時系列データ [m/s³]
}
```

## 6. アルゴリズム詳細

### 6.1 経路コスト計算

経路のコストは以下の要素から計算されます：

```
Cost = K_LON × (縦方向コスト) + K_LAT × (横方向コスト)

縦方向コスト = K_JERK × Σ(jerk²) + K_SPEED × (目標速度との差)²
横方向コスト = K_JERK × Σ(jerk²)
```

### 6.2 衝突判定

2つの経路が衝突するかどうかは、各時間ステップでの車両間距離で判定：

```
縦方向閾値 = 車両長 / 2 + 安全マージン
横方向閾値 = レーン幅 / 2

if (|Δ縦位置| < 縦方向閾値 AND |Δ横位置| < 横方向閾値) then
    衝突あり
```

### 6.3 優先度判定

以下の条件で優先度を決定：
1. 障害物は最高優先度
2. 車線変更時は、変更先レーンの車両が優先
3. 同一レーンでは前方車両が優先

### 6.4 多項式軌跡生成

#### 4次多項式（速度制御用）
- 境界条件：初期位置、初期速度、初期加速度、最終速度、最終加速度
- 用途：縦方向の速度変更

#### 5次多項式（位置制御用）
- 境界条件：初期位置、初期速度、初期加速度、最終位置、最終速度、最終加速度
- 用途：横方向の車線変更、縦方向の位置制御

## 7. 可視化システム

MCMWebVisualizerは、以下の情報をJSON形式で出力：
- 各車両の現在位置と速度
- 予定経路（plannedPath）
- 希望経路（desiredPath）
- 候補経路（pathCandidates）

ブラウザで `http://localhost:8080/mcm_visualization/` にアクセスすることで、リアルタイムで経路を確認できます。

## 8. パラメータ設定

主要なパラメータ：
- `laneWidth`: レーン幅 [m]
- `numLanes`: レーン数
- `vehicleLength`: 車両長 [m]
- `convergenceTime`: 経路収束時間 [s]
- `safetySecond`: 安全時間間隔 [s]
- `laneChangeInterval`: 車線変更後の待機時間 [s]
- `desiredCostThreshold`: 希望経路送信の閾値

## 9. 使用例

### 9.1 サービスの初期化

```cpp
// OMNeT++の設定ファイル（.ned）で定義
*.node[*].middleware.services = "ManeuverCoordinationService"
*.node[*].middleware.ManeuverCoordinationService.laneWidth = 3.5
*.node[*].middleware.ManeuverCoordinationService.numLanes = 3
*.node[*].middleware.ManeuverCoordinationService.convergenceTime = 5.0
```

### 9.2 経路生成の例

```cpp
// 最高速度到達経路の生成
std::vector<Path> speedPaths = mPlanner.generateMaxSpeedPathCandidates(
    lonPos, lonSpeed, lonAccel,
    latPos, latSpeed, latAccel,
    maxSpeed, laneCenterPositions, mConvTime
);

// 前方車両追従経路の生成
std::vector<Path> followPaths = mPlanner.generateMaxPosPathCandidates(
    lonPos, lonSpeed, lonAccel,
    latPos, latSpeed, latAccel,
    leadingPositions, leadingSpeeds, maxSpeed,
    leadingLatPositions, mConvTime
);
```

## 10. まとめ

MCSは、V2X通信を活用して車両間の協調的な運転を実現するシステムです。各車両が自身の意図（経路）を共有し、相互に調整することで、安全で効率的な交通流を実現します。滑らかな軌跡生成、リアルタイムの衝突判定、優先度に基づく調整により、自動運転車両の協調動作を支援します。
