from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import sys

motor_record = open(sys.argv[1], mode= "r")

time = np.array([])

wheel1_target = np.array([])
wheel1_rpm = np.array([])

wheel2_target = np.array([])
wheel2_rpm = np.array([])

wheel3_target = np.array([])
wheel3_rpm = np.array([])

wheel4_target = np.array([])
wheel4_rpm = np.array([])

while(True):

    data_temp = motor_record.readline()

    if(data_temp == ""):
        break

    data_temp = data_temp.split()

    data_number = []

    for num in data_temp:

        try:
            data_number.append(float(num))

        except ValueError:
            continue
        

    print(data_number, end="\n\n")

    time = np.append(time, data_number[0])

    wheel1_target = np.append(wheel1_target, data_number[2])
    wheel1_rpm = np.append(wheel1_rpm, data_number[3])

    wheel2_target = np.append(wheel2_target, data_number[5])
    wheel2_rpm = np.append(wheel2_rpm, data_number[6])

    wheel3_target = np.append(wheel3_target, data_number[8])
    wheel3_rpm = np.append(wheel3_rpm, data_number[9])

    wheel4_target = np.append(wheel4_target, data_number[11])
    wheel4_rpm = np.append(wheel4_rpm, data_number[12])

plt.subplot(3,2,1)
plt.plot(time, wheel1_target, "k-", label = "target")
plt.plot(time, wheel1_rpm, "or-", label = "rpm", ms = 5)
plt.xlabel("time(sec)")
plt.ylabel("velocity(RPM)")
plt.title("wheel1")
plt.legend(loc = 2)

plt.subplot(3,2,2)
plt.plot(time, wheel2_target, "k-", label = "target")
plt.plot(time, wheel2_rpm, "og-", label = "rpm", ms = 5)
plt.xlabel("time(sec)")
plt.ylabel("velocity(RPM)")
plt.title("wheel2")
plt.legend(loc = 2)

plt.subplot(3,2,3)
plt.plot(time, wheel3_target, "k-", label = "target")
plt.plot(time, wheel3_rpm, "om-", label = "rpm", ms = 5)
plt.xlabel("time(sec)")
plt.ylabel("velocity(RPM)")
plt.title("wheel3")
plt.legend(loc = 2)

plt.subplot(3,2,4)
plt.plot(time, wheel4_target, "k-", label = "target")
plt.plot(time, wheel4_rpm, "oc-", label = "rpm", ms = 5)
plt.xlabel("time(sec)")
plt.ylabel("velocity(RPM)")
plt.title("wheel4")
plt.legend(loc = 2)



for i in range(wheel1_rpm.size):
    wheel1_rpm[i] = abs(wheel1_rpm[i])

for i in range(wheel2_rpm.size):
    wheel2_rpm[i] = abs(wheel2_rpm[i])

for i in range(wheel3_rpm.size):
    wheel3_rpm[i] = abs(wheel3_rpm[i])

for i in range(wheel4_rpm.size):
    wheel4_rpm[i] = abs(wheel4_rpm[i])

for i in range(wheel1_target.size):
    wheel1_target[i] = abs (wheel1_target[i])

plt.subplot(3,2,5)
plt.plot(time, wheel1_target, "ok-", label = "target")
plt.plot(time, wheel1_rpm, "or-", label = "wheel_1_rpm", ms = 5)
plt.plot(time, wheel2_rpm, "og-", label = "wheel_2_rpm", ms = 5)
plt.plot(time, wheel3_rpm, "om-", label = "wheel_3_rpm", ms = 5)
plt.plot(time, wheel4_rpm, "oc-", label = "wheel_4_rpm", ms = 5)
plt.xlabel("time(sec)")
plt.ylabel("velocity(RPM)")
plt.title("wheel compare")
plt.legend(loc = 2)

plt.subplots_adjust(wspace = 0.5, hspace = 0.5)
plt.show()

motor_record.close()