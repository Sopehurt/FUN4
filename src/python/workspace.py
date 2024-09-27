import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from math import pi,atan2
import matplotlib.pyplot as plt

L1 = 0.200
L2 = 0.120
L3 = 0.100
L4 = 0.250
L5 = 0.280


robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = L1     ,offset = 0.0),
        rtb.RevoluteMDH(alpha = -pi/2   ,a = 0.0      ,d = -L2    ,offset = -pi/2),
        rtb.RevoluteMDH(alpha = 0.0     ,a = L4       ,d = L3     ,offset = 0.0),
    ],tool = SE3([
    [0, 0, 1, L5],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]]),
    name = "FUN4_Robot"
)

print(robot)

q = [0,0,0]
T_0e = robot.fkine(q)
print(q)
print(T_0e)
robot.plot(q=q)


# กำหนดขอบเขตของมุมข้อต่อแต่ละข้อ
q_min = np.array([-np.pi, -np.pi, -np.pi])  # ค่า min ของมุมข้อต่อแต่ละข้อ
q_max = np.array([np.pi, np.pi, np.pi])  # ค่า max ของมุมข้อต่อแต่ละข้อ

# กำหนดจำนวนจุดที่ต้องการ sample ในแต่ละข้อต่อ
num_samples = 40  # จำนวนตัวอย่างต่อข้อต่อ

# สร้าง grid ของมุมข้อต่อแต่ละข้อ
q_samples = [np.linspace(q_min[i], q_max[i], num_samples) for i in range(robot.n)]

# เตรียม arrays เพื่อเก็บตำแหน่ง (position) ของแต่ละคำตอบ
x_positions = []
y_positions = []
z_positions = []

# ลูปหาค่า q จาก grid และคำนวณ Forward Kinematics
for q1 in q_samples[0]:
    for q2 in q_samples[1]:
        for q3 in q_samples[2]:
            q = [q1, q2, q3]  # มุมข้อต่อปัจจุบัน
            T_0e = robot.fkine(q)  # คำนวณ Forward Kinematics
            pos = T_0e.t  # ตำแหน่ง (x, y, z) ของปลายแขนกล
            x_positions.append(pos[0])
            y_positions.append(pos[1])
            z_positions.append(pos[2])
            
# แปลงข้อมูลเป็น numpy arrays เพื่อใช้ในการ plot
x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

# Plot workspace ของหุ่นยนต์
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_positions, y_positions, z_positions, c='b', marker='o', s=1)

# ตั้งค่าชื่อแกน
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')

# แสดงผล plot
plt.show()

print(len(x_positions))
print(len(y_positions))
print(len(z_positions))


input('enter to exit')