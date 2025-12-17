#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import messagebox
import threading

class LapGUI(Node):
    """
    צומת המציג ממשק גרפי (GUI) לנתוני ההקפה.
    הוא מקשיב לטופיק 'lap_summary_data' ומציג את המידע.
    """
    def __init__(self, root):
        super().__init__('lap_gui_node')
        self.root = root
        self.root.title("BGR Simulator - Lap Analysis")
        self.root.geometry("400x300")
        self.root.configure(bg="#f0f0f0")

        # 1. יצירת אלמנטים ב-GUI (תוויות טקסט)
        self.lbl_title = tk.Label(root, text="Waiting for Lap Data...", font=("Arial", 16, "bold"), bg="#f0f0f0")
        self.lbl_title.pack(pady=20)

        self.lbl_info = tk.Label(root, text="Drive across the start line to begin!", font=("Arial", 12), bg="#f0f0f0", justify="center")
        self.lbl_info.pack(pady=10)

        # כפתור יציאה
        self.btn_quit = tk.Button(root, text="Close", command=self.on_close, bg="#ffdddd")
        self.btn_quit.pack(side="bottom", pady=20)

        # 2. מנוי לטופיק שמפרסם את הסיכום (מהקובץ הקודם)
        # שימי לב: אנחנו מניחים ש-LapAnalyzer שולח הודעת String לטופיק הזה
        self.subscription = self.create_subscription(
            String,
            'lap_summary_data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """פונקציה שמופעלת כשמגיעה הודעה מ-ROS"""
        # מכיוון שזה רץ ב-Thread אחר, צריך לבקש מ-Tkinter לעדכן את המסך
        # אנחנו משתמשים ב-after כדי "להזריק" את הפעולה ללולאה הראשית של ה-GUI
        self.root.after(0, lambda: self.update_gui(msg.data))

    def update_gui(self, data_string):
        """מעדכן את המסך ומקפיץ הודעה"""
        # עדכון הטקסט בחלון הראשי
        self.lbl_title.config(text="🏁 New Lap Finished! 🏁", fg="green")
        
        # פירמוט הטקסט שיהיה יפה (החלפת פסיקים בשורות חדשות אם צריך)
        formatted_text = data_string.replace("|", "\n") 
        self.lbl_info.config(text=formatted_text)

        # === החלק שרצית: חלון קופץ (Popup) ===
        messagebox.showinfo("Lap Complete!", f"Great Job!\n\n{formatted_text}")

    def on_close(self):
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # 1. יצירת חלון Tkinter ראשי
    root = tk.Tk()
    
    # 2. יצירת צומת ה-ROS וחיבורו לחלון
    gui_node = LapGUI(root)
    
    # 3. הפעלת ROS ב-Thread נפרד (כדי שלא יתקע את ה-GUI)
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    ros_thread.start()
    
    # 4. הפעלת לולאת ה-GUI (זה חייב לרוץ ב-Main Thread)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()