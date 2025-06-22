# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

# データフレームから各車両の交渉時間を取得して返す
# @return 交渉時間のリスト
# example: [20.0, 25.0, 19.0, ...] 単位はs
def get_negotiation_time(df):
    negotiation_time_vector = 'negotiationTime:vector'
    negotiation_time = df[(df["name"] == negotiation_time_vector) & (df["vectime"].notnull())]
    
    negotiation_time = negotiation_time[["module", "vecvalue"]]

    results = [] # 速度のサンプルを格納するリスト
    for row in negotiation_time.itertuples():
        for i in range(len(row.vecvalue)):
            results.append(row.vecvalue[i])

    return results

# 交渉時間のリストを受け取り、累積密度分布をプロット
def plot_negotiation_time(negotiation_times):
    fig, ax = plt.subplots()
    val, base = np.histogram(negotiation_times, bins=100, density=True)
    y = np.add.accumulate(val) / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    ax.plot(x, y)

    ax.set(xlabel='negotiation time [m/s]', ylabel='Cumulative Density Distribution', title='Cumulative density distribution of negotiation time')
    ax.tick_params(direction='in')
    ax.set_xlim(0, max(negotiation_times))
    ax.set_ylim([0, 1.0])
    plt.xticks(np.arange(0, max(negotiation_times) + 1.0, step=2.0))
    plt.yticks(np.arange(0, 1.1, step=0.1))

    output_file = re.sub(r'\.csv$', '_negotiation_time', args[1])
    plt.savefig(output_file)
    plt.close()

args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

negotiation_times = get_negotiation_time(df)
plot_negotiation_time(negotiation_times)
