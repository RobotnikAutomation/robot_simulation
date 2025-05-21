import subprocess
import time
import os
import sys

# Ruta al ejecutable de CoppeliaSim
COPPELIA_SIM_PATH = "/home/albert/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/coppeliaSim.sh"

# Carpeta donde están guardadas todas las escenas
SCENE_DIR = "/home/albert/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/scenes"

# Verificar que se ha pasado un argumento con el nombre de la escena
if len(sys.argv) != 2:
    print("Uso: python3 lanzar_escena.py <nombre_escena.ttt>")
    sys.exit(1)

scene_name = sys.argv[1]
scene_path = os.path.join(SCENE_DIR, scene_name)

# Verificar que la escena existe
if not os.path.isfile(scene_path):
    print(f"Error: escena no encontrada en {scene_path}")
    sys.exit(1)

# Lanzar CoppeliaSim con la escena especificada
print(f"Lanzando CoppeliaSim con la escena: {scene_name}")
subprocess.Popen([COPPELIA_SIM_PATH, scene_path])

# Esperar unos segundos para asegurarse de que la escena cargue
time.sleep(5)

print("CoppeliaSim lanzado. Escena cargada.")

