#!/usr/bin/env python3

import re
import numpy as np
from pathlib import Path
import pandas as pd
import plotly.express as px
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument(
    '--logs',
    type=Path,
    required=True,
    help='Direcotry containing logs.'
)
parser.add_argument(
    '--title',
    default='FPS comparison between P1 and P2',
    help='Direcotry containing logs.'
)
parser.add_argument(
    '--out',
    default=None,
    help='Direcotry storing the plots'
)
args = parser.parse_args()


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
        'num': int(num),
        'mean': data.mean(),
        'std': data.std()
    }

data = pd.DataFrame([
    load_data(p)
    for p in args.logs.glob('*.log')
]).sort_values(['protocol', 'num'])
print(data)

fig = px.line(
    data,
    x='num',
    y='mean',
    error_y='std',
    color='protocol',
    title=args.title,
    labels={
        'num': 'Number of nodes',
        'mean': 'FPS',
        'protocol': 'Protocol',
    }
)

if args.out:
    os.makedirs(args.out, exist_ok=True)
    file = f'{args.out}/{args.title}.png'
    fig.write_image(file)
    print(f'Plot had been stored as {file}')
else:
    fig.show()
