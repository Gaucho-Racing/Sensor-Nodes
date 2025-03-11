import matplotlib.pyplot as plt
import matplotlib.animation as ani
from matplotlib import style 
import serial as srl
import time

#Serial stuff
ser = srl.Serial('/dev/tty.usbmodem141791501', 115200, timeout=1)
fig= plt.figure()
ax = fig.add_subplot (1,1,1)
strt = time.time()
xs =[]
#counter
ys = []
#time
rs = []
#avg
def animate(i, xs, ys, rs):
    line = ser.readline()
    print(int(line.rstrip()))
    ys.append(int(line.rstrip()))
    print(time.time() - strt)
    xs.append(time.time() - strt)
    ys = ys[-1000:]
    xs = xs[-1000:]
    ax.clear()
    ax.plot(xs, ys)
    plt.xticks(rotation = 45, ha ="right")
    plt.subplots_adjust(bottom=0.30)
    plt.title('KMS')
    plt.ylabel ('Time to')
    plt.legend()
    plt.axis([1, None, -40000, 40000])

animation = ani.FuncAnimation(fig, animate, fargs=(xs,ys,rs), interval = 1)
plt .show()