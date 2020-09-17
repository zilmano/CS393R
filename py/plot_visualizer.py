import rospy
import matplotlib
from collections import defaultdict
from std_msgs.msg import String
from matplotlib import pyplot as plt

plt.style.use("ggplot")

named_x_lists = defaultdict(list)
named_y_lists = defaultdict(list)
named_ax = {}
initialized = False
init_x = -1


def callback(data):
    if not initialized:
        named_ax["root"] = plt.gca()
    name, x, y = str(data.data).split("|")
    x, y = float(x), float(y)
    if y > 5.0:
        y = 0
    global init_x
    if init_x < 0:
        init_x = x
    if name not in named_ax:
        named_ax[name], = named_ax["root"].plot([], "o-", label=name)
        plt.legend()
    named_x_lists[name].append(x - init_x)
    named_y_lists[name].append(y)

    for k in named_x_lists:
        named_ax[k].set_xdata(named_x_lists[k])
        named_ax[k].set_ydata(named_y_lists[k])

    named_ax["root"].relim()
    named_ax["root"].autoscale_view()
    plt.draw()
    plt.pause(0.01)


plt.ion()
plt.show()
rospy.init_node("py_plotter")
rospy.Subscriber("plotty", String, callback)
rospy.spin()
