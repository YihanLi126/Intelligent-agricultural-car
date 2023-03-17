import sensor, image, time, math,json,pyb
from time import sleep
from pyb import Pin, UART
uart = UART(3, 9600)
thresholds1=(30, 67, -69, -24, -2, 37)
thresholds2=(39, 76, -32, -16, -22, -2)
thresholds3=(0, 33, 14, 50, 13, 49)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
USB_VCP=pyb.USB_VCP()
a=('a'+"\n").encode("utf-8")
b=('b'+"\n").encode("utf-8")
c=('c'+"\n").encode("utf-8")
y=('y'+"\n").encode("utf-8")
r=('r'+"\n").encode("utf-8")
l=('l'+"\n").encode("utf-8")

while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    for blob1 in img.find_blobs([thresholds1], pixels_threshold=200, area_threshold=5000, merge=True):
        img.draw_rectangle(blob1.rect())
        print('a')
        USB_VCP.send(a)

    for blob2 in img.find_blobs([thresholds2], pixels_threshold=200, area_threshold=5000, merge=True):
        img.draw_rectangle(blob2.rect())
        print('b')
        USB_VCP.send(b)

    for blob3 in img.find_blobs([thresholds3], pixels_threshold=200, area_threshold=5000, merge=True):
        img.draw_rectangle(blob3.rect())
        print('c')
        USB_VCP.send(c)

    #for c in img.find_circles(threshold = 3500, pixels_threshold=20000,x_margin = 10, y_margin = 10, r_margin = 10,
                   #r_min = 2, r_max = 100, r_step = 2,merge=True):
        #area = (c.x()-c.r(), c.y()-c.r(), 2*c.r(), 2*c.r())
        #statistics = img.get_statistics(roi=area)
        #if 71<statistics.l_mode()<100 and -41<statistics.a_mode()<3 and -70<statistics.b_mode()<27:
            #img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
            #if (img.width()/3)<c.x()<(2*img.width()/3):
                #print('y')
                #USB_VCP.send(y)
            #elif c.x()>int(2*img.width()/3):
                #print('r')
                #USB_VCP.send(r)
            #elif int(img.width()/3)>c.x():
                #print('l')
                #USB_VCP.send(l)

