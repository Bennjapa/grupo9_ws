import pandas as pd
import numpy as np
from sklearn.metrics import mean_squared_error

archivo_a_comparar = "sin"

# 1. Convertir archivos .txt a .csv (si están separados por comas)
df_odom = pd.read_csv("odometria.txt", sep=",")
df_odom.to_csv("odometria.csv", index=False)

df_path = pd.read_csv("path_" + archivo_a_comparar + ".txt", sep=",")
df_path.to_csv("path_" + archivo_a_comparar + ".csv", index=False)

# 2. Cargar archivos ya como .csv
df1 = pd.read_csv("path_" + archivo_a_comparar + ".csv")  # El archivo con menos puntos
df2 = pd.read_csv("odometria.csv")  # El archivo con más puntos

# 3. Interpolación: alinear odometría (df2) con el eje X del path (df1)
y2_interp = np.interp(df1['X'], df2['X'], df2['Y'])

# 4. Calcular error cuadrático medio
mse = mean_squared_error(df1['Y'], y2_interp)
print(f"Error cuadrático medio (MSE) entre path {archivo_a_comparar} y odometría: {mse:.4f}")
