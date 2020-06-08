import os
import glob
import time
import serial
import subprocess
import tkinter as tk
from tkinter import ttk

LARGE_FONT = ("Verdana", 15, "bold")
MEDIUM_FONT = ("Verdana", 11, "bold")
BGCOLOR = '#93ccfc'
LOCOLOR = '#fd944e'
LGCOLOR = '#c4fc9a'
BBCOLOR = '#67c8fe'
LPCOLOR = '#fee1ff'
COMPORT = 'COM15'
BAUDRATE = 500000


#MMI MODE SEND
def send(myList):

    ###
    #ser = serial.Serial('/dev/ttyACM0', baudrate = 115200)
    #ser = serial.Serial(COMPORT, BAUDRATE)
    byte_buffer = bytes(f":{myList}", 'utf8')
    print(byte_buffer)
    #ser.write(byte_buffer)
            
    #ser.close()

#OPEN NOTEPAD
def notepad(file):

    #subprocess.call(['notepad.exe','C:\Users\LERERXIS.000\Desktop\PY TEST\DroneGUI\{}'.format(file)]
    os.startfile(file)
    #subprocess.call(['cmd.exe', '/c', file])
    
#RUN MODE SEND
def run_send(file):

    ###
    #ser = serial.Serial('/dev/ttyACM0', baudrate = 115200)
    #ser = serial.Serial(COMPORT, BAUDRATE)
    #file_name = open(r'/home/pi/Documents/DroneGUI/{}'.format(file),'r')
    file_name = open(r'C:\Users\LERERXIS.000\Desktop\PY TEST\DroneGUI\{}'.format(file), 'r')
    lines_list = file_name.readlines()
    
    for i in range(len(lines_list)):
        #data_list = lines_list[i].split("|")
        #for j in range(len(data_list)):
        ##byte_buffer = bytes(data_list[j], 'utf8')
        byte_buffer = bytes(lines_list[i], 'utf8')
        print(byte_buffer)
            #ser.write(byte_buffer)
            #time.sleep(0.25)

    file_name.close()
    #ser.close() 

#EDIT MODE READ
#def read_file(file):
    
class DroneGUI(tk.Tk):

    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self, *args, **kwargs)
        #tk.Tk.iconbitmap(self,default="image.ico"
        tk.Tk.wm_title(self, "Drone GUI")
        
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand = True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, PageOne, PageTwo, PageThree):

            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartPage)

    def show_frame(self, cont):

        frame = self.frames[cont]
        frame.tkraise()

#MAIN MENU    
class StartPage(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self, parent, bg=BGCOLOR)
        label = tk.Label(self, text="MAIN MENU", font=LARGE_FONT, bg=BGCOLOR)
        blank = tk.Label(self, text="     ", font=LARGE_FONT, bg=BGCOLOR).grid(row=0, column=0)
        label.grid(row=0, column=1, pady=10)
        button01 = tk.Button(self, text="MDI", font=LARGE_FONT, bg=BBCOLOR, relief='groove', width=5, command=lambda:controller.show_frame(PageOne))
        button01.grid(row=1, column=1)
        button02 = tk.Button(self, text="EDIT", font=LARGE_FONT, bg=BBCOLOR, relief='groove', width=5, command=lambda:controller.show_frame(PageTwo))
        button02.grid(row=2, column=1, pady=5)
        button03 = tk.Button(self, text="RUN", font=LARGE_FONT, bg=BBCOLOR, relief='groove', width=5, command=lambda:controller.show_frame(PageThree))
        button03.grid(row=3, column=1, pady=5)
        button04 = tk.Button(self, text="EXIT", font=LARGE_FONT, fg='white', bg=LOCOLOR, relief='groove', width=5, command=lambda:os._exit(0))
        button04.grid(row=4, column=2, pady=5 )

#MANUAL DIGITAL INPUT
class PageOne(tk.Frame):
    
    def __init__(self, parent, controller):

        tk.Frame.__init__(self, parent, bg=BGCOLOR)
        menu_label = tk.Label(self, text="Manual Digital Input", font=LARGE_FONT, bg=BGCOLOR)
        menu_label.grid(row=0, column=1, pady=10)
        x_slider = tk.Scale(self, from_=-50, to=50, label="Coordenadas X", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        x_slider.grid(row=1, column=0)
        y_slider = tk.Scale(self, from_=-50, to=50, label="Coordenadas Y", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        y_slider.grid(row=2, column=0)
        z_slider = tk.Scale(self, from_=-50, to=50, label="Coordenadas Z", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        z_slider.grid(row=3, column=0)
        l1r1_slider = tk.Scale(self, from_=-50, to=50, label="Rotación L1_A", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        l1r1_slider.grid(row=1, column=1)
        l1r2_slider = tk.Scale(self, from_=-50, to=50, label="Rotación L1_B", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        l1r2_slider.grid(row=2, column=1)
        l2r1_slider = tk.Scale(self, from_=-50, to=50, label="Rotación L2_A", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        l2r1_slider.grid(row=1, column=2)
        l2r2_slider = tk.Scale(self, from_=-50, to=50, label="Rotación L2_B", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        l2r2_slider.grid(row=2, column=2)
        l1s_slider = tk.Scale(self, from_=0, to=1, label="Encendido L1", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 1)
        l1s_slider.grid(row=3, column=1)
        l2s_slider = tk.Scale(self, from_=0, to=1, label="Encendifo L2", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 1)
        l2s_slider.grid(row=3, column=2)
        t_slider = tk.Scale(self, from_=0, to=50, label="     Tiempo", bg=BGCOLOR, troughcolor=LPCOLOR, highlightbackground=BGCOLOR, orient = "horizontal", resolution = 0.1)
        t_slider.grid(row=4, column=0)
        button12 = tk.Button(self, text="Send", font=MEDIUM_FONT, bg=LGCOLOR, relief='groove', width=10, command=lambda:send([x_slider.get(),y_slider.get(),z_slider.get(),l1r1_slider.get(),l1r2_slider.get(),l2r1_slider.get(),l2r2_slider.get(),l1s_slider.get(),l2s_slider.get(),t_slider.get()]))
        button12.grid(row=4, column=1, sticky='S')
        button11 = tk.Button(self, text="Back ⏎", font=MEDIUM_FONT, bg=LOCOLOR, relief='groove', width=10, command=lambda:controller.show_frame(StartPage))
        button11.grid(row=4, column=2, sticky='S')
        
#EDIT MODE        
class PageTwo(tk.Frame):

    def __init__(self, parent, controller):

        ###
        #os.chdir(r'/home/pi/Documents/DroneGUI')
        os.chdir(r'C:\Users\LERERXIS.000\Desktop\PY TEST\DroneGUI')
        myFiles = glob.glob('*.txt') 
        var_edit = tk.StringVar()
        
        tk.Frame.__init__(self, parent, bg=BGCOLOR)
        label = tk.Label(self, text="EDIT MODE", font=LARGE_FONT, bg=BGCOLOR)
        label.pack(pady=10)
        edit_menu = ttk.OptionMenu(self, var_edit, myFiles[0], *myFiles)
        edit_menu.pack(pady=10)
        open_button = tk.Button(self, text="Open File", font=MEDIUM_FONT, bg=LGCOLOR, relief='groove', width=10, command=lambda: notepad(var_edit.get()))
        open_button.pack(pady=10)
        button21 = tk.Button(self, text="Back ⏎", font=MEDIUM_FONT, bg=LOCOLOR, relief='groove', width=10, command=lambda:controller.show_frame(StartPage))
        button21.pack(pady=10)

#RUN MODE
class PageThree(tk.Frame):
        
    def __init__(self, parent, controller):

        ###
        #os.chdir(r'/home/pi/Documents/DroneGUI')
        os.chdir(r'C:\Users\LERERXIS.000\Desktop\PY TEST\DroneGUI')
        myFiles = glob.glob('*.txt')
        var_run = tk.StringVar()
        
        tk.Frame.__init__(self, parent, bg=BGCOLOR)
        label = tk.Label(self, text="RUN MODE", font=LARGE_FONT, bg=BGCOLOR)
        label.pack(pady=10)
        run_menu = ttk.OptionMenu(self, var_run, myFiles[0], *myFiles)
        run_menu.pack(pady=10)
        run_button = tk.Button(self, text="Run File", font=MEDIUM_FONT, bg=LGCOLOR, relief='groove', width=10, command=lambda:run_send(var_run.get()))
        run_button.pack(pady=10)
        button31 = tk.Button(self, text="Back ⏎", font=MEDIUM_FONT, bg=LOCOLOR, relief='groove', width=10, command=lambda:controller.show_frame(StartPage))
        button31.pack(pady=10)
               
app = DroneGUI()
app.geometry("900x650")
app.mainloop()




#Funcion send,
#Files
#tiempo
