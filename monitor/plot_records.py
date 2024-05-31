import argparse 
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd 

def to_dataFrame(filename):
    with open(filename, 'r') as f:
        file = f.readlines()
    line1 = file.pop(0)
    if '#' in file[0] or file[0].startswith(' '):
        line2 = file.pop(0)
        idx = int(line2.replace('#','').strip())
    else:
        idx = None 
    datas = np.fromiter(file, dtype=float)
    names = line1.replace('#', '').split(',')
    datas = datas.reshape(-1, len(names)-1)
    if idx:
        datas = np.roll(datas,  -(idx+1), axis=0)
        print("idx found")
    df = pd.DataFrame(datas, columns=names[:-1])
    # remove last row (BUG TO CORRECT)
    df = df[:-1]
    return df

def plot_df(df):
    fig, axs = plt.subplots(3, 1)
    tics = np.arange(len(df))
    try:
        pass
        # df['V_Low_estim'] = df['duty_cycle']*df['V_high']
        # df['w_estimate'] = df['w_estimate'] * 100e-6
    except:
        pass
    # if 'k_acquire' in df:
    #     del df['k_acquire']
    for s in df:
        if s.startswith('V'):
            axs[0].step(tics, df[s], label=s)
        elif s.startswith('I'):
            axs[1].step(tics, df[s], label=s)
        else:
            axs[2].step(tics, df[s], label=s)
    for ax in axs:
        ax.legend()
        # ax.set_ylim([-50, 50])
        ax.grid()

if __name__ == '__main__':
    plt.ion()
    parser = argparse.ArgumentParser("plot_records", "plot_records <filename>")
    parser.add_argument("filename")
    args = parser.parse_args()
    df = to_dataFrame(args.filename)
    plot_df(df)
    plt.show()
    input('enter char')

