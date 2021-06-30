import roslibpy
import cv2
import numpy as np
import base64
import math

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

movepose_srv = roslibpy.Service(client, '/simulation_bridge/move_robot_pose', 'welding_sim_gz/MoveRobotPose')
movejoint_srv = roslibpy.Service(client, '/simulation_bridge/move_robot_joint', 'welding_sim_gz/MoveRobotJoint')

targetpose = {
    "targetpose": {
        "position": {
            "x": 0.4,
            "y": 0.0,
            "z": 0.8,
        },
        "orientation": {
            "x": 0.0,
            "y": 0.7071068,
            "z": 0.0,
            "w": 0.7071068
        }
    }
}

targetjoint = {
    "targetjoint": [
        0,
        -math.pi/4,
        0,
        -math.pi/2,
        0,
        math.pi/3,
        0
    ]
}

print('Calling Joint Movement service')
raw_image = movejoint_srv.call(targetjoint)
print('Calling Pose Movement service')
raw_image = movepose_srv.call(targetpose)

client.terminate()

#Decode Image Data
decoded_data = base64.b64decode(raw_image['result']['data'])
np_data = np.frombuffer(decoded_data,np.uint8)
img = cv2.imdecode(np_data, cv2.IMREAD_UNCHANGED)
cv2.imshow("test", img)
cv2.waitKey(0)
