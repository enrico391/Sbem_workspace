#!/usr/bin/env python3


from tkinter import *





class SBEMGraphicTool():
    def __init__(self,root):
        #super().__init__()

        # Frame principale
        self.tk = Frame(root)
        self.tk.pack(expand=True, fill="both")
        self.tk.configure(bg="white")


        # Frame sinistro
        self.left_frame = Frame(self.tk, bg="#c3c7d6", borderwidth = 2, highlightbackground="black")
        self.left_frame.pack(side="left", expand=True, fill="both",padx=10,pady=10)

        self.label_lidar = Label(self.left_frame, text="Lidar features", font=("Arial",15),  bg="#c3c7d6")
        self.label_lidar.pack(padx=5, pady=5)

        self.start_lidar_btn = Button(self.left_frame, text="Start Lidar", fg="white", bg="green" )
        self.start_lidar_btn.pack(side="top", padx=5, pady=5)


        self.stop_lidar_btn = Button(self.left_frame, text="Stop Lidar", fg="white", bg="red")
        self.stop_lidar_btn.pack(side="top", padx=5, pady=5)


        self.label_esp = Label(self.left_frame, text="ESP features", font=("Arial",15),  bg="#c3c7d6")
        self.label_esp.pack(side="top", padx=5, pady=5)
        
        self.restart_esp_btn = Button(self.left_frame, text="Restart ESP", fg="white", bg="red")
        self.restart_esp_btn.pack(side="top", padx=5, pady=5)

        # Frame destro
        self.right_frame =Frame(self.tk, bg="#c3c7d6", borderwidth = 2, highlightbackground="black")
        self.right_frame.pack(side="right", expand=True, fill="both", padx=10, pady=10)

        self.value_display_label_title = Label(self.right_frame, text="Encoder values : ", font=("Arial", 15), bg="#c3c7d6")

        self.value_encoder_left_label = Label(self.right_frame, text="left :", font=("Arial", 15),  bg="#c3c7d6")
        #self.value_encoder_left_label.pack(padx=5, pady=5)

        self.value_encoder_right_label = Label(self.right_frame, text="right :", font=("Arial", 15),  bg="#c3c7d6")
        #self.value_encoder_right_label.pack(padx=5, pady=5)

        # Spazio per visualizzare i valori numerici da ROS2
        self.value_display_label_left = Label(self.right_frame, text="", font=("Arial", 15),  bg="#c3c7d6")
        #self.value_display_label_left.pack(padx=5, pady=5)

        self.value_display_label_right = Label(self.right_frame, text="", font=("Arial", 15),  bg="#c3c7d6")
        #self.value_display_label_right.pack(padx=5, pady=5)
        
        self.value_display_label_title.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        self.value_encoder_left_label.grid(row=1, column=0, padx=5, pady=5)
        self.value_encoder_right_label.grid(row=2, column=0, padx=5, pady=5)
        self.value_display_label_left.grid(row=1, column=1, padx=5, pady=5)
        self.value_display_label_right.grid(row=2, column=1, padx=5, pady=5)
        



    def update_image(self):
        #self.tk.update()
        self.tk.update()


#def main(args=None):
#    app = SBEMGraphicTool()
#    app.mainloop()
#
#
#
#if __name__ == "__main__":
#    main()