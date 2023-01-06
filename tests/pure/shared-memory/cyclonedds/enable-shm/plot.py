import re
import numpy as np
from pathlib import Path
import pandas as pd
import plotly.express as px
import sys

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]

def load_data(path):
    [protocol, num] = path.stem.split('-')
    with open(path) as f:
        data = np.array([
            float(re.sub('\x1b.*?m', '', l))
            for l in [
                l.split('FPS:')[-1].strip()
                for l in f.readlines() if 'FPS' in l # ignore the first two
            ]
        ])[1:-1]
    
    data = reject_outliers(data)

    return {
        'protocol': protocol,
        'num': int(num),
        'mean': data.mean(),
        'std': data.std()
    }


def main():

    data = pd.DataFrame([
        load_data(p)
        for p in Path('./logs').glob('*.log')
    ]).sort_values(['protocol', 'num'])
    print(data)


    fig = px.line(
        data,
        x='num',
        y='mean',
        error_y='std',
        color='protocol',
        title='FPS comparison between P1 and P2 with Shared Memory - ROS2(CycloneDDS) / Zenoh',
        labels={
            'num': 'Number of cloned transfer nodes',
            'mean': 'FPS',
            'protocol': 'Protocol',
        }
    )

    fig.write_html("./compare.html")

if __name__ == "__main__":
   main()