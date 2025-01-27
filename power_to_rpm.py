import matplotlib.pyplot as plt
import numpy as np
import json
import plotly.express as px
import math
import pandas as pd

f = open("goodwooddata.txt", "r")
data = []

# Adjust all the data + remove any nulls
for line in filter(lambda x: len(x.strip()) > 0, f.readlines()):
    # print(line)
    try:
        datum = json.loads(line.strip())
        if datum["gps_spd"] != None and datum["lat"] != None and datum["long"] != None:
            datum["gps_spd"] *= 1.15078 # Convert from knots to mph
            datum["p2_I"] /= 10
            datum["24v_V"] /=1000
            # datum["Wh"]
            #datum["pt_pit"] *= -1 # Pitot values are all negative
            if datum['gps_spd'] > 5: # Data less than 10mph isn't very useful
                data.append(datum)
    #except json.decoder.JSONDecodeError:
    except Exception as e:
        print(e)
        pass
print(data)
# Gaussian distribution to average data
pressure_data = np.array([datum["pt_pit"] for datum in data]) - 10
k = np.arange(-100,100,6)
k = np.exp(-k**2 / 5000)
k /= sum(k) # Keep the sum the same
smoothed_pressure_data = list(np.correlate(pressure_data,k, mode='same'))

E = 2.71828
DENSITY = 1.293 # Air density as taken from NASA
sortedspeed = []
for i, datum in enumerate(data):
    datum["pt_pit"] = (smoothed_pressure_data[i]/5)
    datum['pt_spd2'] = math.sqrt(abs(datum['pt_pit'] *2 /DENSITY)) # Calculate speed using pitot data
    #datum['GPS_pressure'] = datum['gps_spd']**2*DENSITY/2 # Inverse of pressure equation
    pair = [datum['gps_spd'],datum['pt_pit']]
    sortedspeed.append(pair)

    # Adjust the data using the difference between the curve of best fit and y = x + 10
    a = 4.053 * E ** -7
    b = - 0.0004213 
    c = 5.792
    # datum['pt_spd'] = a * datum['pt_pit'] ** 2 + b * datum['pt_pit'] + c 
    # datum['pt_spd'] = math.sqrt(datum['pt_pit']) // (4.053 * E ** -7) + 0.0004213 * datum['pt_pit']  - 5.792 + 10 +datum['pt_pit']
    # Complete the square to rearrange the formula 
    datum['pt_spd'] = (math.sqrt(abs((datum['pt_pit'] - c)//a)+0.057)) //2 + 10
    datum["wind_spd"] = datum["pt_spd"] - datum["gps_spd"]
    
    
sortedspeed.sort()

sortedGPS = []
sortedpitot = []
time = []
count = 0
for i in sortedspeed:
    sortedGPS.append(i[0])
    sortedpitot.append(i[1])
    time.append(count) # Create x axis (number of points)
    count+=1

df = pd.DataFrame({'x': time,
                   'y': sortedpitot })

# Using the quadratic as the line of best fit and scaling it
model2 = np.poly1d(np.polyfit(df.x, df.y, 2))
polyline = np.linspace(1, 10000, 9000)

# Rescaling the line for the graph with only certain points to see how it matches
k = 50 # Number of points you want - play with it until it looks right
xax=list(polyline)[:(k)]
yax= list(model2(polyline)[::len(list(model2(polyline)))//k]) # Takes points at an interval of k
print(len(xax), len(yax)) # yax might be slightly longer but this is adjusted in the next line
plt.plot(xax,yax[:k], color='red') 

# Plot new pitot speeds
print(len(data))
speeds = [i[1] for i in sortedspeed]
speeds2=list(speeds)[:(k)]
print(len(speeds2))
plt.scatter(time[:k],speeds2)

# Map windspeed
fig = px.scatter_mapbox(data, 
                        lat="lat", 
                        lon="long", 
                        hover_data=['gps_spd', 'pt_spd'],  
                        zoom=15, 
                        color="wind_spd", 
                        color_continuous_scale=['red', 'orange', 'yellow', 'green'], 
                        title="Wind speed")
fig.update_layout(mapbox_style="open-street-map")
fig.show()

# Take only a few data points since other data hard to see clearly
newGPS_array = []
newpitot_array = []
newtime = []
minVelocity = 10
gap = 0.5 # How often in mph you want to take points
for i in range(len(sortedGPS)):
    if sortedGPS[i] > minVelocity: # Finds the point closest to each MPH
        newGPS_array.append(sortedGPS[i])
        newpitot_array.append((math.sqrt(abs((sortedpitot[i] - c)//a)+0.057)) //2 + 10)
        minVelocity+=gap

# Create count of points
for i in range(len(newGPS_array)):
    newtime.append(i)

plt.scatter(newtime, newGPS_array, label='GPS')
plt.scatter(newtime, newpitot_array, label='pitot')
plt.xlabel('Point count')
plt.ylabel('Pitot pressure and GPS speed')
plt.show()

#Check to see how well the pitot speed matches the GPS velocity time graph
gps =  [datum['gps_spd'] for datum in data]
speeds3 = [datum['pt_spd'] for datum in data]

#Make a time axis
counter = []
for i in range(len(gps)):
    counter.append(i)

plt.plot(counter,gps)
plt.plot(counter,speeds3)
plt.xlabel('Time')
plt.ylabel('Pitot pressure and GPS speed')
plt.show()