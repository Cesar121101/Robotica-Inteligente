import pandas as pd

cameraMatrix = pd.read_pickle('cameraMatrix.pkl')
print(cameraMatrix)

dist = pd.read_pickle('dist.pkl')
print(dist)

cali = pd.read_pickle('calibration.pkl')
print(cali)