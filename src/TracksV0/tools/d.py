import os
import csv
import numpy as np
import matplotlib.pyplot as plt


def main():

    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Color mapping (based on your remapped logic)
    plot_color_map = {
        1: "gold",
        2: "royalblue",
        3: "darkorange",
        4: "saddlebrown",
    }

    label_map = {
        1: "Yellow",
        2: "Blue",
        3: "Orange",
        4: "Orange big",
    }

    csv_files = [
        f for f in os.listdir(script_dir)
        if f.endswith(".csv")
    ]

    if not csv_files:
        print("No CSV files found in scripts folder.")
        return

    for csv_file in csv_files:
        csv_path = os.path.join(script_dir, csv_file)
        cones_by_color = {}

        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)

            required_fields = {"color_id", "color_name", "x_m", "y_m"}
            if not required_fields.issubset(reader.fieldnames):
                print(f"[!] Skipping {csv_file} (wrong format)")
                continue

            for row in reader:
                color = int(row["color_id"])
                x = float(row["x_m"])
                y = float(row["y_m"])
                cones_by_color.setdefault(color, []).append((x, y))

        # ---------- PLOT ONE CSV ----------
        fig, ax = plt.subplots()

        for color, points in cones_by_color.items():
            points = np.array(points)
            ax.scatter(
                points[:, 0],
                points[:, 1],
                s=20,
                c=plot_color_map.get(color, "black"),
                label=label_map.get(color, f"Color {color}")
            )

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(f"Cones from {csv_file}")
        ax.grid(True)
        ax.legend()

        plt.show()  # blocks until window is closed


if __name__ == "__main__":
    main()
