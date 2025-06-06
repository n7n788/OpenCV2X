# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
# import seaboan as sns
import re

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

def calculate_cpmContained():
    all_vector = 'cpmContainedAllCnt:vector'
    all_df = df[(df['name'] == all_vector) & (df["vectime"].notnull())]
    all_df = all_df[['module', 'vecvalue', 'vectime']]
    all_dt = []
    # all_bins = []
    
    etsi_vector = 'cpmContainedEtsiCnt:vector'
    etsi_df = df[(df['name'] == etsi_vector) & (df["vectime"].notnull())]
    etsi_df = etsi_df[['module', 'vecvalue', 'vectime']]
    etsi_dt = []
    # etsi_bins = []
    
    cpm_vector = 'cpmContainedCnt:vector'
    cpm_df = df[(df['name'] == cpm_vector) & (df["vectime"].notnull())]
    cpm_df = cpm_df[['module', 'vecvalue', 'vectime']]
    cpm_dt = []
    # cpm_bins = []
    
    for row in all_df.itertuples():
        for i in range(len(row.vecvalue)):
            all_dt.append(row.vecvalue[i])
            # print(row.vecvalue[i])
    for row in etsi_df.itertuples():
        for i in range(len(row.vecvalue)):
            etsi_dt.append(row.vecvalue[i])
            # print(row.vecvalue[i])
    for row in cpm_df.itertuples():
        for i in range(len(row.vecvalue)):
            cpm_dt.append(row.vecvalue[i])
    
    # all_bins = np.linspace(0, max(all_dt), max(all_dt))
    # etsi_bins = np.linspace(0, max(etsi_dt), max(etsi_dt))
    # cpm_bins = np.linspace(0, max(cpm_dt), max(all_dt))
    
    # センシングしている物体数、ETSIのルールに該当した物体数、実際にCPMに含めた物体数の確率分布グラフを描画
    all_val, all_base = np.histogram(all_dt)
    all_y = all_val / float(sum(all_val))
    all_x = np.convolve(all_base, np.ones(2) / 2, mode="same")[1:]
    
    etsi_val, etsi_base = np.histogram(etsi_dt) 
    etsi_y = etsi_val / float(sum(etsi_val))
    etsi_x = np.convolve(etsi_base, np.ones(2) / 2, mode="same")[1:]
    
    cpm_val, cpm_base = np.histogram(cpm_dt) 
    # print(cpm_val, cpm_base)
    cpm_y = cpm_val / float(sum(cpm_val))
    cpm_x = np.convolve(cpm_base, np.ones(2) / 2, mode="same")[1:]
    # print(all_x, all_y, sum(val1))
    
    fig, ax = plt.subplots()
    # ax.hist(all_dt, all_base, weights=np.ones(len(all_dt)) / len(all_dt),  label="Sensing Count")
    ax.plot(all_x, all_y, ls='-', color='r', marker='o', label='Sensing Count')
    ax.plot(etsi_x, etsi_y, ls='-', color='g', marker='o', label="'ETSI Dynamic Rule")
    ax.plot(cpm_x, cpm_y,  ls='-', color='b', marker='o', label='Cpm Contained')
    ax.legend()
    ax.set_xlim([0, max(all_base) + 1])
    ax.set_ylim([0, 1.1])
    ax.set_xlabel("Object Count")
    ax.set_ylabel("PDF")
    # ax.grid()
    plt.savefig(args[1] + "_cpmContained_pdf.png", dpi=300)
    plt.close()

calculate_cpmContained()