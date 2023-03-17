import sensor, image, time, math,json
from time import sleep
from pyb import Pin, UART
uart = UART(3, 9600)
thresholds1=(24, 52, -51, -21, 15, 54)
thresholds2=(30, 73, -38, -10, -25, 3)
thresholds3=(4, 26, 8, 42, -12, 37)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
h=0
num=0
#time.sleep(5000)

#while (uart.any()==0):


    #num=num+1

uart.write("ok")
while(True):
        clock.tick()
        red1=[]
        green1=[]
        blue1=[]
        shit1=[]
        order1=[]
        img = sensor.snapshot()
        area1=(0,0,int(img.width()/3),int(img.height()))
        img.draw_rectangle(0,0,int(img.width()/3),int(img.height()), color=(255,0,0))

        for gb1 in img.find_blobs([thresholds1],pixels_threshold1=200,area_threshold=200,merge=True,roi=area1):
            img.draw_rectangle(gb1.rect(),color = (255, 0, 0))
            if 1.5<=gb1.w()/gb1.h()<=2.8:
               green1.append(gb1.y())
            elif(0.8<=gb1.w()/gb1.h()<=1.4):
               green1.append(gb1.y())
               green1.append(gb1.y()+gb1.h()/2)
            elif(0.5<=gb1.w()/gb1.h()<0.8):
               green1.append(gb1.y())
               green1.append(gb1.y()+2*gb1.h()/3)
               green1.append(gb1.y()+gb1.h()/3)

        for bb1 in img.find_blobs([thresholds2],pixels_threshold=200,area_threshold=200,merge=True,roi=area1):
            img.draw_rectangle(bb1.rect(),color = (255, 0, 0))
            if 1.5<=bb1.w()/bb1.h()<=2.8:
                blue1.append(bb1.y())
            elif(0.8<=bb1.w()/bb1.h()<=1.4):
                blue1.append(bb1.y()+bb1.h()/2)
                blue1.append(bb1.y())
            elif(0.5<=bb1.w()/bb1.h()<0.8):
                blue1.append(bb1.y())
                blue1.append(bb1.y()+2*bb1.h()/3)
                blue1.append(bb1.y()+bb1.h()/3)

        for rb1 in img.find_blobs([thresholds3], pixels_threshold=200, area_threshold=200,roi=area1):
            img.draw_rectangle(rb1.rect(),color = (255, 0, 0))
            if 1.5<(rb1.w()/rb1.h())<2.8:
                 red1.append(rb1.y())
            elif (0.8<(rb1.w()/rb1.h())<1.4):
                red1.append(rb1.y())
                red1.append(rb1.y()+rb1.h()/2)
            elif (0.5<=(rb1.w()/rb1.h())<=0.79):
                red1.append(rb1.y()+2*rb1.h()/3)
                red1.append(rb1.y()+rb1.h()/3)
                red1.append(rb1.y())


        l11=len(red1)
        l21=len(green1)
        l31=len(blue1)
        i=0
        while i<l11:
           shit1.append(red1[i])
           order1.append(3)
           i=i+1
        while i<(l11+l21):

            shit1.append(green1[i-l11])
            order1.append(1)
            i=i+1

        while i<(l11+l21+l31):

            shit1.append(blue1[i-l11-l21])
            order1.append(2)
            i=i+1

        i=0
        n=0
        while len(shit1)<6:

            shit1.append(0)
            order1.append(0)
            n=n+1

        for i in range(5):
            for j in range(5-i):
                if shit1[j]>shit1[j+1]:

                    shit1[j],shit1[j+1]=shit1[j+1],shit1[j]
                    order1[j],order1[j+1]=order1[j+1],order1[j]

        red2=[]
        green2=[]
        blue2=[]
        shit2=[]
        order2=[]
        area2=(int(img.width()/3),0,int(img.width()/3),int(img.height()))
        for gb2 in img.find_blobs([thresholds1],pixels_threshold1=200,area_threshold1=200,merge=True,roi=area2):
            img.draw_rectangle(gb2.rect(),color = (255, 0, 0))
            if 1.5<=gb2.w()/gb2.h()<=2.8:
               green2.append(gb2.y())
            elif(0.8<=gb2.w()/gb2.h()<=1.4):
               green2.append(gb2.y())
               green2.append(gb2.y()+gb2.h()/2)
            elif(0.5<=gb2.w()/gb2.h()<0.8):
               green2.append(gb2.y())
               green2.append(gb2.y()+2*gb2.h()/3)
               green2.append(gb2.y()+gb2.h()/3)

        for bb2 in img.find_blobs([thresholds2],pixels_threshold=200,area_threshold=200,merge=True,roi=area2):
            img.draw_rectangle(bb2.rect(),color = (255, 0, 0))
            if 1.5<=bb2.w()/bb2.h()<=2.8:
                blue2.append(bb2.y())
            elif(0.8<=bb2.w()/bb2.h()<=1.4):
                blue2.append(bb2.y()+bb2.h()/2)
                blue2.append(bb2.y())
            elif(0.5<=bb2.w()/bb2.h()<0.8):
                blue2.append(bb2.y())
                blue2.append(bb2.y()+2*bb2.h()/3)
                blue2.append(bb2.y()+bb2.h()/3)

        for rb2 in img.find_blobs([thresholds3], pixels_threshold=200, area_threshold=200,roi=area2):
            img.draw_rectangle(rb2.rect(),color = (255, 0, 0))
            if 1.5<(rb2.w()/rb2.h())<2.8:
                 red2.append(rb2.y())
            elif (0.8<(rb2.w()/rb2.h())<1.4):
                red2.append(rb2.y())
                red2.append(rb2.y()+rb2.h()/2)
            elif (0.5<=(rb2.w()/rb2.h())<=0.79):
                red2.append(rb2.y()+2*rb2.h()/3)
                red2.append(rb2.y()+  rb2.h()/3)
                red2.append(rb2.y())


        l12=len(red2)
        l22=len(green2)
        l32=len(blue2)
        i=0
        while i<l12:
           shit2.append(red2[i])
           order2.append(3)
           i=i+1
        while i<(l12+l22):

            shit2.append(green2[i-l12])
            order2.append(1)
            i=i+1

        while i<(l12+l22+l32):

            shit2.append(blue2[i-l12-l22])
            order2.append(2)
            i=i+1

        i=0
        n=0
        while len(shit2)<6:

            shit2.append(0)
            order2.append(0)
            n=n+1

        for i in range(5):
            for j in range(5-i):
                if shit2[j]>shit2[j+1]:

                    shit2[j],shit2[j+1]=shit2[j+1],shit2[j]
                    order2[j],order2[j+1]=order2[j+1],order2[j]

        red3=[]
        green3=[]
        blue3=[]
        shit3=[]
        order3=[]
        area3=(int(2*img.width()/3),0,int(img.width()/3),int(img.height()))
        for gb3 in img.find_blobs([thresholds1],pixels_threshold1=200,area_threshold1=200,merge=True,roi=area3):
            img.draw_rectangle(gb3.rect(),color = (255, 0, 0))
            if 1.5<=gb3.w()/gb3.h()<=2.8:
               green3.append(gb3.y())
            elif(0.8<=gb3.w()/gb3.h()<=1.4):
               green3.append(gb3.y())
               green3.append(gb3.y()+gb3.h()/2)
            elif(0.5<=gb3.w()/gb3.h()<0.8):
               green3.append(gb3.y())
               green3.append(gb3.y()+2*gb3.h()/3)
               green3.append(gb3.y()+gb3.h()/3)

        for bb3 in img.find_blobs([thresholds2],pixels_threshold=200,area_threshold=200,merge=True,roi=area3):
            img.draw_rectangle(bb3.rect(),color = (255, 0, 0))
            if 1.5<=bb3.w()/bb3.h()<=2.8:
                blue3.append(bb3.y())
            elif(0.8<=bb3.w()/bb3.h()<=1.4):
                blue3.append(bb3.y()+bb3.h()/2)
                blue3.append(bb3.y())
            elif(0.5<=bb3.w()/bb3.h()<0.8):
                blue3.append(bb3.y())
                blue3.append(bb3.y()+2*bb3.h()/3)
                blue3.append(bb3.y()+bb3.h()/3)

        for rb3 in img.find_blobs([thresholds3], pixels_threshold=200, area_threshold=200,roi=area3):
            img.draw_rectangle(rb3.rect(),color = (255, 0, 0))
            if 1.5<(rb3.w()/rb3.h())<2.8:
                 red3.append(rb3.y())
            elif (0.8<(rb3.w()/rb3.h())<1.4):
                red3.append(rb3.y())
                red3.append(rb3.y()+rb3.h()/2)
            elif (0.5<=(rb3.w()/rb3.h())<=0.79):
                red3.append(rb3.y()+2*rb3.h()/3)
                red3.append(rb3.y()+rb3.h()/3)
                red3.append(rb3.y())


        l13=len(red3)
        l23=len(green3)
        l33=len(blue3)
        i=0
        while i<l13:
           shit3.append(red3[i])
           order3.append(3)
           i=i+1
        while i<(l13+l23):

            shit3.append(green3[i-l13])
            order3.append(1)
            i=i+1

        while i<(l13+l23+l33):

            shit3.append(blue3[i-l13-l23])
            order3.append(2)
            i=i+1

        i=0
        n=0
        while len(shit3)<6:

            shit3.append(0)
            order3.append(0)
            n=n+1

        for i in range(5):
            for j in range(5-i):
                if shit3[j]>shit3[j+1]:

                    shit3[j],shit3[j+1]=shit3[j+1],shit3[j]
                    order3[j],order3[j+1]=order3[j+1],order3[j]

        print(order1,order2,order3)

        h=h+1
        #if h==25:

        output_str1="[%d%d%d%d%d%d]" % (order1[0],order1[1],order1[2],order1[3],order1[4],order1[5])
        output_str2="[%d%d%d%d%d%d]" % (order2[0],order2[1],order2[2],order2[3],order2[4],order2[5])
        output_str3="[%d%d%d%d%d%d]" % (order3[0],order3[1],order3[2],order3[3],order3[4],order3[5])
        uart.write(output_str1+output_str2+output_str3+"\r\n")

                #out2=json.dumps(order2)
                #out3=json.dumps(order3)

                #uart.write(out2)
                #uart.write(out3)







