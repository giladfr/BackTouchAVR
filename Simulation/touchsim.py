from Tkinter import *
import random
import time

class TouchScreenSim(object):
    def __init__(self,dim_x,dim_y,touch_lst):
        self.radius = 5
        self.root = Tk()
        self.canvas = Canvas(self.root, width=dim_x, height = dim_y)
        self.canvas.pack()
        self.touch_pnt = self.canvas.create_oval(touch_lst[0][0] - self.radius,touch_lst[0][1] + self.radius ,touch_lst[0][0] + self.radius ,touch_lst[0][1] - self.radius , outline='white',fill='blue')
        self.canvas.pack()
        self.root.after(0, self.animation)
        self.touch_lst_raw = touch_lst
        self.root.mainloop()

    def animation(self):
        for point in self.touch_lst_raw[1:]:
            time.sleep(0.005)
            self.canvas.coords(self.touch_pnt, point[0] - self.radius,point[1] + self.radius ,point[0] + self.radius ,point[1] - self.radius)
            self.canvas.update()




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

if __name__ == '__main__':
    rnd_lst =  create_random_touch_list(500,400,400,1000)
    TouchScreenSim(400,400,rnd_lst)

