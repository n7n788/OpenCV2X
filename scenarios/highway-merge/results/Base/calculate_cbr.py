# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re

SIMULATION_TIME = 30  # シミュレーション時間（秒）
TIME_STEP = 0.1  # CBRの平均を取る時間ステップ（秒）

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

# データフレームからシミュレーション時間ごとのCBRの平均を返す
# example: [0.1, 0.2, 0.15, ...]
def calculate_cbr(df):
    cbr_vector = "cbr:vector"
    cbr_df = df[(df["name"] == cbr_vector) & (df["vectime"].notnull())]
    cbr_df = cbr_df[["module", "vecvalue", "vectime"]]

    bins = [] # TIME_STEPごとのCBRの合計値
    counts = [] # TIME_STEPごとのCBRのサンプル数
    for i in range(int(SIMULATION_TIME / TIME_STEP)):
        bins.append(0)
        counts.append(0)

    # 各車両が計測したCBRの合計とサンプル数を計算
    for row in cbr_df.itertuples():
        for i in range(len(row.vectime)):
           remainder = int(row.vectime[i] / TIME_STEP) 
           bins[remainder] += row.vecvalue[i]
           counts[remainder] += 1

    # CBRの平均値を計算
    for i in range(len(bins)):
        if counts[i] > 0:
            bins[i] /= counts[i]
        else:
            bins[i] = 0
    
    return bins

# CBRを時間軸に沿ってプロットして、グラフを保存
# @param time: 時間のリスト
# @param cbr: CBRのリスト
# 出力ファイル名は引数で指定されたファイル名に "_CBR.png" を付け加えたもの
def plot_cbr(times, cbrs):
    fig, ax = plt.subplots()
    ax.plot(times, cbrs)
    ax.set(xlabel='Simulation Time [s]', ylabel='Channel Busy Ratio',
           title='Channel busy ratio over simulation time')
    ax.tick_params(direction='in')
    ax.set_xlim(0, (max(times) + 1))
    ax.set_ylim([0, 1.0])
    plt.xticks(np.arange(0, (max(times) + 1), step=5))
    plt.yticks(np.arange(0, 1.1, step=0.1))

    output_file = re.sub(r'\.csv$', '_cbr', args[1])
    plt.savefig(output_file, dpi=300)
    plt.close(fig)

# コマンドライン引数で指定されたCSVファイルを読み込み、データフレームを作成
args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

# x軸の時間を生成
times = np.arange(0, SIMULATION_TIME, TIME_STEP)
# y軸のCBRを計算
cbrs = calculate_cbr(df)
# グラフを描画
plot_cbr(times, cbrs)

