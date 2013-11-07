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
        self.avg_touch_pnt = self.canvas.create_oval(touch_lst[0][0] - self.radius,touch_lst[0][1] + self.radius ,touch_lst[0][0] + self.radius ,touch_lst[0][1] - self.radius , outline='red',fill='',width=2)
        self.canvas.pack()

        self.root.after(0, self.live)
        self.touch_lst_raw = touch_lst
        self.root.mainloop()

    def set_radius_pressure(self,pressure):
        self.radius = (pressure - self.touch_pressure_min) * self.slope + self.radius_min
        #print "pressure = %d, radius = %d" %(pressure,self.radius)

    def set_circ_center(self,obj,x,y):
        # reverse y axis since on the screen it increased going down
        y = self.dim_y - y
        self.canvas.coords(obj, x - self.radius,y + self.radius ,x + self.radius ,y - self.radius)

    def animation(self,is_live = FALSE):
        last_point = 0
        last_avg_point = 0
        for point in self.touch_lst_raw:
            # Display of real touch point
            self.set_radius_pressure(point[2])
            self.set_circ_center(self.touch_pnt,point[0],point[1])

            # Averaged touch point
            avg_point = CalcMovingAvg_Simple(Point(point[0],point[1],point[2]))
            self.set_circ_center(self.avg_touch_pnt,avg_point.x,avg_point.y)
            if (last_point != 0):
                self.canvas.create_line(last_point[0],self.dim_y - last_point[1],point[0],self.dim_y - point[1],fill="blue")
                self.canvas.create_line(last_avg_point.x,self.dim_y - last_avg_point.y,avg_point.x,self.dim_y - avg_point.y,fill="red")

            self.canvas.update()
            #time.sleep(0.010)
            last_point = point
            last_avg_point = avg_point

    def live(self):
        last_point = 0
        last_avg_point = 0

        while True:
            s = serial.Serial(port=serialPath, baudrate=9600)
            point = parse_cor_trio(s.readline())
            self.set_radius_pressure(point[2])
            self.set_circ_center(self.touch_pnt,point[0],point[1])
            # Averaged touch point
            avg_point = CalcMovingAvg_Median(Point(point[0],point[1],point[2]))
            self.set_circ_center(self.avg_touch_pnt,avg_point.x,avg_point.y)

            if (last_point != 0):
                self.canvas.create_line(last_point[0],self.dim_y - last_point[1],point[0],self.dim_y - point[1],fill="blue")
                self.canvas.create_line(last_avg_point.x,self.dim_y - last_avg_point.y,avg_point.x,self.dim_y - avg_point.y,fill="red")


            self.canvas.update()
            last_point = point
            last_avg_point = avg_point


class Point(object):
    def __init__(self,x = 0,y = 0,z = 0):
        self.x = x
        self.y = y
        self.z = z

AVG_NUM_OF_POINTS = 10
pnt_arr = [Point() for i in range(AVG_NUM_OF_POINTS)]
i = 0
sum_x = 0
sum_y = 0

def CalcMovingAvg_Simple(inPnt):
    global sum_x;
    global sum_y;
    global i;

    retPnt = Point()
    # Remove from sum the point in place i
    sum_x = sum_x - pnt_arr[i].x;
    sum_y = sum_y - pnt_arr[i].y;

    # insert the current point
    pnt_arr[i] = inPnt;

    sum_x = sum_x + inPnt.x;
    sum_y = sum_y + inPnt.y;

    retPnt.x = sum_x / AVG_NUM_OF_POINTS;
    retPnt.y = sum_y / AVG_NUM_OF_POINTS;

    i+= 1
    if (i == AVG_NUM_OF_POINTS): i=0
    return retPnt;


def median(mylist):
    sorts = sorted(mylist)
    length = len(sorts)
    if not length % 2:
        return (sorts[length / 2] + sorts[length / 2 - 1]) / 2.0
    return sorts[length / 2]



def CalcMovingAvg_Median(inPnt):
    global i;

    retPnt = Point()

    # insert the current point
    pnt_arr[i] = inPnt;

    retPnt.x = median([j.x for j in pnt_arr]);
    retPnt.y = median([j.y for j in pnt_arr]);

    i+= 1
    if (i == AVG_NUM_OF_POINTS): i=0
    return retPnt;




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

