#!/usr/bin/env python3
import os
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

WORLD = "empty"

# NOTE: Update this path to your adjusted location
TRACK_DIR = os.path.expanduser("~/ros2_workspaces/bgr_ws/src/TracksV0/models/tracks")

# Map: track file -> model name inside that SDF
# IMPORTANT: model name must match <model name="..."> in each track sdf
TRACKS = {
    "track_acceleration.sdf": "track_acceleration",
    "track_skidpad.sdf": "track_skidpad",
    "track_trainingmap.sdf": "track_training",
    "track_competitionmap1.sdf": "track_comp1",
    "track_competitionmap2.sdf": "track_comp2",
    "track_competitionmap3.sdf": "track_comp3",
    "track_competitionmaptestday1.sdf": "track_test1",
    "track_competitionmaptestday2.sdf": "track_test2",
    "track_competitionmaptestday3.sdf": "track_test3",
}

current_track_model = None

def run_cmd(cmd):
    try:
        subprocess.run(cmd, check=True, text=True)
        return True
    except subprocess.CalledProcessError as e:
        messagebox.showerror("Command failed", f"{' '.join(cmd)}\n\n{e}")
        return False

def spawn_track(track_file):
    global current_track_model

    sdf_path = track_model_map.get(track_file)
    if not os.path.exists(sdf_path):
        messagebox.showerror("Missing file", f"Track SDF not found:\n{sdf_path}")
        return

    # Remove previous track if exists
    if current_track_model:
        remove_cmd = [
            "gz", "service", "-s", f"/world/{WORLD}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", f'name: "{current_track_model}" type: MODEL'
        ]
        run_cmd(remove_cmd)

    # Spawn selected track
    create_cmd = [
        "gz", "service", "-s", f"/world/{WORLD}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'sdf_filename: "{sdf_path}"'
    ]
    ok = run_cmd(create_cmd)
    if ok:
        current_track_model = track_file
        status_var.set(f"Spawned: {track_file}")

def refresh_tracks():
    global track_model_map

    if not os.path.isdir(TRACK_DIR):
        status_var.set(f"Track dir not found: {TRACK_DIR}")
        combo["values"] = []
        return

    track_model_map = {}
    tracks = []

    for d in sorted(os.listdir(TRACK_DIR)):
        model_dir = os.path.join(TRACK_DIR, d)
        sdf_path = os.path.join(model_dir, "model.sdf")

        if os.path.isdir(model_dir) and os.path.isfile(sdf_path):
            track_model_map[d] = sdf_path
            tracks.append(d)

    combo["values"] = tracks
    if tracks:
        combo.current(0)

    status_var.set(f"Found {len(tracks)} tracks")


root = tk.Tk()
root.title("FSA Track Selector")

frm = ttk.Frame(root, padding=12)
frm.grid()

ttk.Label(frm, text="Select track:").grid(column=0, row=0, sticky="w")

combo = ttk.Combobox(frm, state="readonly", width=40)
combo.grid(column=0, row=1, padx=0, pady=6)

def on_spawn():
    track_file = combo.get()
    if not track_file:
        return
    spawn_track(track_file)

ttk.Button(frm, text="Spawn track", command=on_spawn).grid(column=0, row=2, pady=6, sticky="ew")

status_var = tk.StringVar(value="Ready")
ttk.Label(frm, textvariable=status_var).grid(column=0, row=3, sticky="w")

refresh_tracks()
root.mainloop()
