import csv

CSV_HEADER = ['x', 'y', 'z', 'yaw']
fileName = '/home/jom/Documents/prgs/Term03/CarND-Capstone/data/wp_yaw.txt'

def load_waypoints(fileName):
    x = []
    y = []
    yaw = []
    with open(fileName) as wfile:
        reader = csv.DictReader(wfile, CSV_HEADER)
        for wp in reader:
            x.append(float(wp['x']))
            y.append(float(wp['y']))
            yaw.append(float(wp['yaw']))
    return x, y, yaw
    
