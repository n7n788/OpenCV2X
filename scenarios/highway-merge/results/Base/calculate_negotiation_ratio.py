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

# データフレームからを交渉結果を計算して返す
def calculate_negotiation_result(df):
    # 通信距離を取得 
    negotiation_result_vector = 'negotiationResult:vector'
    negotiation_result = df[(df["name"] == negotiation_result_vector) & (df["vectime"].notnull())]
    negotiation_result = negotiation_result[["module", "vectime", "vecvalue"]]

    success_count = 0  # 成功数
    fail_count = 0    # 失敗数
    for row in negotiation_result.itertuples():
        for i in range(len(row.vecvalue)):
            success_count += row.vecvalue[i]
            fail_count += 1 - row.vecvalue[i]
    
    success_ratio = success_count / (success_count + fail_count) if (success_count + fail_count) > 0 else 0

    print("Negotiation Success Count: ", success_count)
    print("Negotiation Fail Count: ", fail_count)
    print("Negotiation Success Ratio: {:.2f}".format(success_ratio))


args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

calculate_negotiation_result(df)
