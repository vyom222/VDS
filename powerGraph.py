import matplotlib.pyplot as plt
import json

f = open("goodwood2025_fp.txt", "r")
power = []
times = []
velocity = []
current = []
voltage = []
alt = []
time = 0

# Adjust all the data + remove any nulls
for line in filter(lambda x: len(x.strip()) > 0, f.readlines()):
    try:
        datum = json.loads(line.strip())
        if datum["p2_I"] != None and datum["24v_V"] != None and datum["gps_spd"] != None:
            datum["gps_spd"] *= 1.15078/2.237 # Convert from knots to metres per second
            if datum['gps_spd'] > 2: # Data less than 2 metres per second isn't very useful
                datum['power'] = datum['p2_I']/1000 * datum['24v_V']/1000

                power.append(datum['power'])
                velocity.append(datum['gps_spd'])
                times.append(time)
                alt.append(datum['alt'])
                time +=1
                current.append(datum['p2_I'])
                voltage.append(datum['24v_V'])
    except Exception as e:
        pass

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(times,current)
axs[0, 0].set_title('Current')
axs[0, 1].plot(times,velocity, 'tab:orange')
axs[0, 1].set_title('Velocity')
axs[1, 0].plot(times,voltage, 'tab:green')
axs[1, 0].set_title('Voltage')
axs[1, 1].plot(times,power, 'tab:red')
axs[1, 1].set_title('Power')
fig.tight_layout()

plt.show()
plt.plot(times, alt)
plt.show()
# for p in power:
#     print(p)