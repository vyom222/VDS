import matplotlib.pyplot as plt
import json

f = open("goodwood2024.txt", "r")
data = []
power = []
times = []
time = 0

# Adjust all the data + remove any nulls
for line in filter(lambda x: len(x.strip()) > 0, f.readlines()):
    try:
        datum = json.loads(line.strip())
        if datum["p2_I"] != None and datum["24v_V"] != None and datum["gps_spd"] != None and datum['lat'] != None and datum['long'] != None:
            datum["gps_spd"] *= 1.15078 # Convert from knots to mph
            if datum['gps_spd'] > 2: # Data less than 10mph isn't very useful
                data.append(datum)
                datum['power'] = datum['p2_I']/1000 * datum['24v_V']/1000
                power.append(datum['power'])
                times.append(time)
                time +=1
    except Exception as e:
        pass

plt.plot(times,power)
plt.show()

for p in power:
    print(p)