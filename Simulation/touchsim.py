from Tkinter import *
import random
import time
import serial


serialPath = '/dev/tty.usbmodem1411'


# Simulation GUI
class TouchScreenSim(object):
    def __init__(self,dim_x,dim_y,touch_lst):
        self.dim_x = dim_x
        self.dim_y = dim_y

        self.radius_min = 5
        self.radius_max = 30
        self.touch_pressure_max = 120
        self.touch_pressure_min = 1000
        self.slope = (self.radius_max - self.radius_min) / float(self.touch_pressure_max - self.touch_pressure_min)
        self.radius = self.radius_min

        self.root = Tk()
        self.canvas = Canvas(self.root, width=dim_x, height = dim_y)
        self.canvas.pack()
        self.touch_pnt = self.canvas.create_oval(touch_lst[0][0] - self.radius,touch_lst[0][1] + self.radius ,touch_lst[0][0] + self.radius ,touch_lst[0][1] - self.radius , outline='yellow',fill='blue')
        self.canvas.pack()

        self.root.after(0, self.live)
        self.touch_lst_raw = touch_lst
        self.root.mainloop()

    def set_radius_pressure(self,pressure):
        self.radius = (pressure - self.touch_pressure_min) * self.slope + self.radius_min
        #print "pressure = %d, radius = %d" %(pressure,self.radius)

    def set_circ_center(self,x,y):
        # reverse y axis since on the screen it increased going down
        y = self.dim_y - y
        self.canvas.coords(self.touch_pnt, x - self.radius,y + self.radius ,x + self.radius ,y - self.radius)

    def animation(self):
        for point in self.touch_lst_raw[1:]:
            self.set_circ_center(point[0],point[1])
            time.sleep(0.002)
            self.canvas.update()

    def live(self):
        while True:
            s = serial.Serial(port=serialPath, baudrate=9600)
            point = parse_cor_trio(s.readline())
            self.set_radius_pressure(point[2])
            self.set_circ_center(point[0],point[1])
            self.canvas.update()




def parse_cor_trio(strng):
    return tuple([int(i) for i in strng.split(",")])


def create_random_touch_list(n,dim_x,dim_y,dim_z):
    ret_lst = []
    touch_tup = (random.randrange(0,dim_x,1),random.randrange(0,dim_y,1),random.randrange(0,dim_z,1))
    ret_lst.append(touch_tup)
    for i in range(1,n):
        touch_tup = (ret_lst[i-1][0] + random.randrange(-2,2,1),
            ret_lst[i-1][1] + random.randrange(-2,2,1),
            ret_lst[i-1][2] + random.randrange(-2,2,1))
        ret_lst.append(touch_tup)

    return ret_lst


def load_touch_file(file_path):
    touch_lst = []
    fh = open(file_path,"r");
    for line in fh.readlines():
        touch_event = parse_cor_trio(line)
        touch_lst.append(touch_event)
    return touch_lst




if __name__ == '__main__':
    #rnd_lst =  create_random_touch_list(500,400,400,1000)
    #touch_list = load_touch_file("circle.txt")
    touch_list = load_touch_file("down_up.txt")
    #touch_list = load_touch_file("5sec_no_moving.txt")

    TouchScreenSim(900,900,touch_list)

