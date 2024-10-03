
# FUN4: Hello world

โครงการนี้เป็นการออกแบบระบบควบคุมแขนกล 3 DOF ซึ่งแบ่งเป็น 3 ส่วนดังนี้:

## 1. การตั้งค่า Environment และการใช้ Command Line

โปรเจคนี้เริ่มต้นด้วยการตั้งค่า ROS environment และการติดตั้งไลบรารีที่จำเป็นสำหรับการทำงานของหุ่นยนต์

### ขั้นตอน:

1. Clone โปรเจคจาก GitHub:
    ```bash
    cd ~
    git clone https://github.com/Sopehurt/FUN4.git
    cd FUN4
    ```

2. ตั้งค่า ROS2 Humble environment:
    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```

3. ติดตั้งไลบรารี Python ที่จำเป็นสำหรับโปรเจคนี้:
    ```bash
    sudo apt-get install python3-pip
    python3 -m pip install --upgrade pip
    pip3 install roboticstoolbox-python
    pip3 install numpy==1.24.4
    ```

4. รันโปรแกรม:

    ในหน้าต่าง terminal[1] รัน launch file
    ```bash
    ros2 launch dof3_controller robot_control.launch.py
    ```
    
    เปิดใหม่ให้ terminal[2] มาแล้วรันเพื่อเปิด node ควบคุมด้วย keyboard
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    เปิดใหม่ให้ terminal[3] มาแล้วรันเพื่อเปิด node ที่ใช้ในการเปลี่ยน mode
    ```bash
    ros2 run dof3_controller  teleop_mode_key.py
    ```

    จากนั้นให้ทำการปิดหน้าต่าง Joint Dtate Publisher เรื่องจากเราไม่ต้องการควบคุมแต่ละ Joint หรือใช้ฟังก์ชั่นนี้
    ![alt text](<src/image/Screenshot from 2024-10-04 04-34-13.png>)

## 2. การคำนวณ Workspace ของหุ่นยนต์

1. ในส่วนนี้จะอธิบายการคำนวณ workspace ของหุ่นยนต์ 3 DOF เพื่อให้มั่นใจว่าหุ่นยนต์สามารถทำงานได้ภายในขอบเขตที่กำหนด

### โค้ดสำหรับคำนวณ Workspace:

```python
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi

L1 = 0.200
L2 = 0.120
L3 = 0.100
L4 = 0.250
L5 = 0.280

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0.0, a=0.0, d=L1, offset=0.0),
        rtb.RevoluteMDH(alpha=-pi/2, a=0.0, d=-L2, offset=-pi/2),
        rtb.RevoluteMDH(alpha=0.0, a=L4, d=L3, offset=0.0),
    ],
    tool=SE3([
        [0, 0, 1, L5],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ]),
    name="FUN4_Robot"
)

q_min = np.array([-np.pi, -np.pi, -np.pi])  # ค่า min ของมุมข้อต่อแต่ละข้อ
q_max = np.array([np.pi, np.pi, np.pi])  # ค่า max ของมุมข้อต่อแต่ละข้อ

# กำหนดจำนวนจุดที่ต้องการ sample ในแต่ละข้อต่อ
num_samples = 40

# สร้าง grid ของมุมข้อต่อแต่ละข้อ
q_samples = [np.linspace(q_min[i], q_max[i], num_samples) for i in range(robot.n)]

x_positions, y_positions, z_positions = [], [], []

# ลูปหาค่า q จาก grid และคำนวณ Forward Kinematics
for q1 in q_samples[0]:
    for q2 in q_samples[1]:
        for q3 in q_samples[2]:
            q = [q1, q2, q3]
            T_0e = robot.fkine(q)
            pos = T_0e.t  # ตำแหน่ง (x, y, z) ของปลายแขนกล
            x_positions.append(pos[0])
            y_positions.append(pos[1])
            z_positions.append(pos[2])

# แปลงข้อมูลเป็น numpy arrays เพื่อใช้ในการ plot
x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

# Plot workspace ของหุ่นยนต์
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_positions, y_positions, z_positions, c='b', marker='o', s=1)

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
plt.show()
```

### คำอธิบาย

- โค้ดนี้ใช้คำนวณและแสดงตำแหน่ง (x, y, z) ของปลายแขนกลในรูปแบบ 3D workspace โดยการสร้าง grid ของมุมข้อต่อและคำนวณ Forward Kinematics
- ใช้ `matplotlib` ในการ plot ตำแหน่ง workspace ของหุ่นยนต์ที่ได้จากการคำนวณ

หลังจากคำนวณ workspace แล้วสามารถใช้ visual ที่ได้จาก plot เพื่อยืนยันว่าหุ่นยนต์สามารถเคลื่อนที่ภายในขอบเขต workspace ที่กำหนดได้อย่างถูกต้อง

2.  สามารถดูค่าตำแหน่งที่สุ่มมาจาก node สําหรับสุ่มเป้าหมายตําแหน่งปลายมือภายใน workspace ได้ด้วยคำสั่งนี้
    ```bash
    ros2 topic echo /target
    ```
3. สามารถดูค่าตําแหน่งปลายมือผ่านคำสั่งนี้
    ```bash
    ros2 topic echo /end_effector
    ```

4. มีการสร้าง custom serviece สำหรับการเปลี่ยนโหมดการทำงานและการรับส่งค่าที่ได้จาก node สําหรับสุ่มเป้าหมายตําแหน่งปลายมือ โดยไฟล์รายละเอียดจะอยู่ใน FUN4/src/fun4_interfaces/srv ชื่อไฟล์ ModeControl.srv

    ```
    string mode
    float64 ipk_x
    float64 ipk_y
    float64 ipk_z
    ---
    bool success
    string check
    float64 x
    float64 y
    float64 z
    ```

## 3. การทำงานในโหมดต่าง ๆ

หุ่นยนต์ 3 DOF นี้สามารถทำงานได้ 3 โหมด ได้แก่:

- **Inverse Pose Kinematics Mode (IPK)**: ใช้คำนวณค่า Configuration Space เพื่อเคลื่อนไปยังตำแหน่ง Taskspace ที่ต้องการ
    - หากพบคำตอบที่เป็นไปได้ หุ่นยนต์จะเคลื่อนที่ตาม solution ที่คำนวณได้ สังเกต `response success=True`
    - หากไม่พบคำตอบ หุ่นยนต์จะไม่เคลื่อนที่ สังเกต `response success=False`
    ```bash
    ros2 service call /cal_state fun4_interfaces/srv/ModeControl "mode: 'IPK'
    ipk_x: 0.123
    ipk_y: 0.234
    ipk_z: 0.4"
    ```
    - สามารถเปลี่ยน ipk_x, ipk_y, ipk_z ได้ตามใจชอบ ค่านี้จะอยู่ในหน่วยเมตร

    - ตัวอย่างการระบุที่สามารถไปได้
    ![alt text](<src/image/Screenshot from 2024-10-04 03-36-36.png>)

    - ตัวอย่างการระบุที่ไม่สามารถไปได้
    ![alt text](<src/image/Screenshot from 2024-10-04 03-37-33.png>)

    หรือสามารถใช้โหมดนี้ได้จาก terminal[3]
    แต่ต้องมั่นใจว่า mode ณ ปัจจุบันเป็น mode `IDLE` ถ้าเป็น mode อื่นให้ทำการกด `0` ใน terminal[3] ในหน้า terminal[1] จะแสดง `IDLE Mode`
    เมื่อมั่นใจแล้วกดเลข `1` ในหน้า terminal[3] ระบบจะแสดง
    ```bash
    Please enter the value for x: 
    ```
    - หน้าจอจะแสดงให้กรอกตำแหน่ง `x:` เมื่อกรอกตำแหน่งในหน่วยเมตรเรียบร้อยให้กด `enter` ทำแบบนี้จนครบ x, y, z เมื่อตำแหน่งที่ระบุอยู่ใน workspace หุ่นยนต์จะทำการเคลื่อนที่ไปยังตำแหน่งนั้นภายในเวลา 10 วินาที ดังนี้
    ![alt text](<src/image/Screenshot from 2024-10-04 04-25-19.png>)
    - เลือกโหมดสำเร็จได้ต้องมีการแสดง `IDLE Mode` ใน terminal[1] 
    - เมื่อสั่งตำแหน่งไปในจุดที่ไปได้คืออยู่ใน workspace หุ่นจะเคลื่อนไปยังจุดนั้นใน Rviz
    ![alt text](<src/image/Screenshot from 2024-10-04 03-52-37.png>)
    - เมื่อหุ่นเคลื่อนไปยังตำแหน่งนั้นสำเร็จแล้วหุ่นจะกลับมาอยู่ใน mode "IDLE" 
    เพื่อพร้อมรับคำสั่งต่อไป
    - ในระหว่างที่หุ่นเคลื่อนที่สามารถออกจากโหมดนี้ได้โดยการกดเลข `0` ใน terminal[3] หุ่นจะกลับมาสู่ mode `IDLE` เพื่อรอรับคำสั่งใหม่ต่อไป

- **Tele-operation Mode (Teleop)**: ควบคุมหุ่นยนต์ผ่านคีย์บอร์ด 
    - สามารถควบคุมที่อ้างอิงจากปลายมือหรือฐานของหุ่นยนต์
    - หากหุ่นยนต์เข้าใกล้ singularity หุ่นยนต์จะหยุดการเคลื่อนไหว
    - วิธีการใช้โหมดนี้ทำได้จาก terminal[3]
    แต่ต้องมั่นใจว่า mode ณ ปัจจุบันเป็น mode `IDLE` ถ้าเป็น mode อื่นให้ทำการกด `0` ใน terminal[3] 
    เมื่อมั่นใจแล้วกดเลข `2` ในหน้า terminal[3] ระบบจะแสดง
    ![alt text](<src/image/Screenshot from 2024-10-04 04-01-51.png>)
    - ในหน้านี้ให้ทำการเลือกโหมดโดยถ้ากดเลข `1` แล้ว `enter` จะเห็นในหน้า terminal[1] แสดง `TRef Mode`จะเป็นการเลือกการควบคุมที่อ้างอืงจากปลายมือของหุ่น แต่ถ้ากดเลข `2` จะเป็นการเลือกการควบคุมที่อ้างอืงจากฐานหุ่น ในหน้า terminal[1] จะแสดง `BRef Mode`
    - จากนั้นทำการกด `enter` เพื่อยืนยัน
    - จากนั้่นให้เลือกหน้า terminal[2] เพื่อควบคุมแกน x, y, z  ได้โดย
        - กดปุ่ม `u i o` : เป็นการ + x
        - กดปุ่ม `m , .` : เป็นการ - x
        - กด shift แล้วกดปุ่ม `J` : เป็นการ + y
        - กด shift แล้วกดปุ่ม `L` : เป็นการ - y
        - กดปุ่ม `t` : เป็นการ + z
        - กดปุ่ม `b` : เป็นการ - z
        - ถ้ากดปุ่มอื่นนอกจากนี้จะเป็นการหยุดการเคลื่อนไหวให้ทุกแกนมีความเร็วเป็น 0
        ![alt text](<src/image/Screenshot from 2024-10-04 04-10-07.png>)
    - ถ้าต้องการเปลี่ยนโหมดให้ทำการกด `0` ใน terminal[3] เพื่อให้กลับไปอยู่ใน mode `IDLE` แล้วจึงเปลี่ยนไปยังโหมดที่ต้องการ

- **Autonomous Mode (Auto)**: หุ่นยนต์จะสุ่มตำแหน่งใน Taskspace และเคลื่อนไปยังตำแหน่งนั้นภายใน 10 วินาที โดยที่ตำแหน่งนั้นจะอยู่ใน workspace เสมอ และเมื่อหุ่นเคลื่อนไปยังตำแหน่งนั้นแล้วหุ่นจะสุ่มตำแหน่งต่อไปทันทีและวนลูปอยู่อย่างนี้จนกว่าจะเปลี่ยนโหมดไปยัง `IDLE`
    - วิธีการใช้โหมดนี้ทำได้จาก terminal[3]
    แต่ต้องมั่นใจว่า mode ณ ปัจจุบันเป็น mode `IDLE` ถ้าเป็น mode อื่นให้ทำการกด `0` ใน terminal[3] 
    เมื่อมั่นใจแล้วกดเลข `3` ในหน้า terminal[1] จะแสดง `AUTO Mode`
    ![alt text](<src/image/Screenshot from 2024-10-04 04-26-47.png>)
    - ถ้าต้องการเปลี่ยนโหมดให้ทำการกด `0` ใน terminal[3] เพื่อให้กลับไปอยู่ใน mode `IDLE` แล้วจึงเปลี่ยนไปยังโหมดที่ต้องการ
