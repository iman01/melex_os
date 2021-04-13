import rosbag
from  tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math
import numpy as np

xs = []
ys = []
zs = []
thetas = []
stamps = []
columnAngleRads = []
columnTimes = []
positionTimes = []

for topic, msg, t in rosbag.Bag('/home/tgawron/melexbags/2021-04-13-00-16-50.bag').read_messages():
    if topic == "/bus_rear_axis_pose":
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
        zs.append(msg.pose.pose.position.z)
        orientation = msg.pose.pose.orientation
        quatList = [orientation.x, orientation.y, orientation.z, orientation.w]
        (r, p, y) = euler_from_quaternion(quatList)
        thetas.append(y)        
        stamps.append(msg.header.stamp)
        positionTimes.append(t.to_sec())
    if topic == '/turnAngle_degrees':
        columnAngleRads.append(msg.data*0.0174532925)     
        columnTimes.append(t.to_sec())


ts = []
for t in stamps:
    ts.append(t.to_sec())

dts = []
for t1, t2 in zip(stamps, stamps[1:]):
    dt = (t2-t1).to_sec()
    dts.append(dt)

dxs = []
for i, (x1, x2) in enumerate(zip(xs, xs[1:])):
    dxs.append((x2 - x1)/dts[i])

dthetas = []
for i, (th1, th2) in enumerate(zip(thetas, thetas[1:])):
    dthetas.append((th2 - th1)/dts[i])    

dys = []
for i, (y1, y2) in enumerate(zip(ys, ys[1:])):
    dys.append((y2 - y1)/dts[i])

vels = []
for dx, dy in zip(dxs, dys):
    vels.append(math.sqrt(dx**2 + dy**2))

ddxs = []
for i, (x1, x2) in enumerate(zip(dxs, dxs[1:])):
    ddxs.append((x2 - x1)/dts[i])

ddys = []
for i, (y1, y2) in enumerate(zip(dys, dys[1:])):
    ddys.append((y2 - y1)/dts[i])

betas = []
tsFiltered = []
for dx, dy, dtheta, t in zip(dxs, dys, dthetas, positionTimes):
    vsq = dx**2 + dy**2
    if vsq <= 0.2:
        continue
    kappa = dtheta/math.sqrt(vsq)
    beta = math.atan(1.63*kappa)
    if abs(beta) < 1.0:
        betas.append(beta)
        tsFiltered.append(t)

columnAngleRadsCorrelated = []
for beta, t in zip(betas, tsFiltered):
    timeDists = list(map(lambda ct: abs(t-ct), columnTimes))
    minIndex = np.argmin(timeDists)
    columnAngleRadsCorrelated.append(columnAngleRads[minIndex])


polyCoeffs = np.polyfit(columnAngleRadsCorrelated, betas, 1)
poly = np.poly1d(polyCoeffs)

plt.plot(xs, ys)
plt.figure()
plt.plot(zs)
# plt.figure()
# plt.plot(tsFiltered, columnAngleRadsCorrelated)
plt.figure()
plt.plot(tsFiltered, betas)
plt.plot(tsFiltered, list(map(lambda x: poly(x), columnAngleRadsCorrelated)))
plt.plot(tsFiltered, list(map(lambda x: polyCoeffs[0]*x + polyCoeffs[1], columnAngleRadsCorrelated)))
print(polyCoeffs)
plt.show()
#we will use only linear part of the fit, offset comes mostly from wrong turn sensor calibration at the beginning (never perfect)
#higher polynomial fits do not give us better results, and we know that this should be close to linear by construction

