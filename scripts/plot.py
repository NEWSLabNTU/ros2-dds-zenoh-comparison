import re
import numpy as np
from pathlib import Path
import pandas as pd
import plotly.express as px


def load_data(path):
    [protocol, num] = path.stem.split('-')
    with open(path) as f:
        data = np.array([
            float(re.sub('\x1b.*?m', '', l))
            for l in [
                l.split('FPS:')[-1].strip()
                for l in f.readlines() if 'FPS' in l
            ]
        ])[1:-1]
    return {
        'protocol': protocol,
        'num': num,
        'mean': data.mean(),
        'std': data.std()
    }

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
    title='FPS comparison between P1 and P2',
    labels={
        'num': 'Number of cloned transfer nodes',
        'mean': 'FPS',
        'protocol': 'Protocol',
    }
)
fig.show()
