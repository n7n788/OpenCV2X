# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re

MAX_DISTANCE = 1000 # プロットする最大距離（メートル）
DISTANCE_STEP = 10  # PDRの平均を取る距離のステップ（メートル）

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

# データフレームからPDRを距離ごとに計算して返す
# @return PDRのリスト
# example: [0.9, 0.8, 0.7, ...]
def calculate_pdr(df):
    # 通信距離を取得 
    pdr_dist_vector = 'txRxDistanceTB:vector'
    distances = df[(df["name"] == pdr_dist_vector) & (df["vectime"].notnull())]
    distances = distances[["module", "vectime", "vecvalue"]]
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 
    distances.rename(columns={"vectime": "time"}, inplace=True)
    
    # 受信パケットのデコード結果を取得 (1:成功, 0:失敗, -1:無視してよい)
    pdr_vector = 'tbDecoded:vector' 
    decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]
    decoded = decoded[["module", "vecvalue"]]
    decoded.rename(columns={"vecvalue": "decode"}, inplace=True)

    new_df = pd.merge(distances, decoded, on='module', how='inner')

    bins = [] # DISTANCE_STEPごとのデコードの成功数
    counts = [] # DISTANCE_STEPごとのデコードのサンプル数
    for i in range(int(MAX_DISTANCE / DISTANCE_STEP)):
        bins.append(0)
        counts.append(0)

    for row in new_df.itertuples():
        for i in range(len(row.distance)):
            if row.distance[i] < MAX_DISTANCE:
                remainder = int(row.distance[i] // DISTANCE_STEP)
                if row.decode[i] >= 0:
                    bins[remainder] += row.decode[i]
                    counts[remainder] += 1

    # PDRを計算
    for i in range(len(bins)):
        if counts[i] > 0:
            bins[i] /= counts[i]
        else:
            bins[i] = 0
    
    return bins

def plot_pdr(distances, pdrs):
    fix, ax = plt.subplots()
    ax.plot(distances, pdrs)
    ax.set(xlabel='Distance [m]', ylabel='Packet Delivery Ratio', title='Packet delivery ratio on each communication distance')
    ax.tick_params(direction='in')
    ax.set_xlim(0, MAX_DISTANCE)
    ax.set_ylim([0, 1.0])
    plt.xticks(np.arange(0, MAX_DISTANCE + 1, step=100))
    plt.yticks(np.arange(0, 1.1, step=0.1))

    output_file = re.sub(r'\.csv$', '_pdr', args[1])
    plt.savefig(output_file)
    plt.close()

args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

# x軸の距離のリストを作成
distances = np.arange(0, MAX_DISTANCE, DISTANCE_STEP) 
# y軸のPDRのリストを計算
pdrs = calculate_pdr(df)
# グラグを描画
plot_pdr(distances, pdrs)
