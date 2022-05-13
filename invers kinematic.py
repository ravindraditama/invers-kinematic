from numpy import *


a2 = 5.2  # length of link a2 in cm (( L1 ))
a4 = 7  # length of link a4 in cm (( L2 ))

# Desired Position of End effector
x = -7
y = 9

# Equations for Inverse kinematics
xy = sqrt(x**2+y**2)  # eqn 1
e = arccos((L22-L12-r1**2)/(-2*L12*r1))  # eqn 2
ey = arctan2(y, x)  # eqn 3
theta_1 = rad2deg(ey-e)  # eqn 4 converted to degrees

phi_3 = arccos((r1**2-a2**2-a4**2)/(-2*a2*a4))
theta_2 = 180-rad2deg(phi_3)

print('theta one: ', theta_1)
print('theta two: ', theta_2