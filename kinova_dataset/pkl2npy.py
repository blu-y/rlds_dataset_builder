# load pkl file and save as npy file
# and delete pkl file

import os
import pickle
import numpy as np
import glob

def pkl2npy(pkl_path, npy_path):
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    np.save(npy_path, data)
    os.remove(pkl_path)

if __name__ == '__main__':
    episodes = glob.glob('kinova_dataset/data/*/episode*.pkl')
    for episode in episodes:
        pkl2npy(episode, episode.replace('pkl', 'npy'))
