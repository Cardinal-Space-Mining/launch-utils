import sys
import os
import json
import tkinter as tk
from tkinter import filedialog

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from launch_utils.tf_converter import urdf_to_json

root = tk.Tk()
root.withdraw()

urdf_file = filedialog.askopenfilename(
    title = 'Select a file',
    filetypes = [("URDF files", "*.urdf"), ("All files", "*.*")] )

print(f"Attempting to load file '{urdf_file}'")

try:
    json_data = urdf_to_json(urdf_file)
except:
    raise RuntimeError(f"Failed to read file '{urdf_file}'")

print(json.dumps(json_data, indent=4))
