import cv2
import mediapipe as mp
import socket
import struct
import time
import numpy as np
import math

class OneEuroFilter:
    def __init__(self, te, mincutoff=1.0, beta=0.007, dcutoff=1.0):
        self.x = None
        self.dx = 0
        self.te = te
        self.mincutoff = mincutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self.alpha = self._alpha(self.mincutoff)
        self.dalpha = self._alpha(self.dcutoff)

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / self.te)

    def predict(self, x, te):
        result = x
        if self.x is not None:
            edx = (x - self.x) / te
            self.dx = self.dx + (self.dalpha * (edx - self.dx))
            cutoff = self.mincutoff + self.beta * abs(self.dx)
            self.alpha = self._alpha(cutoff)
            result = self.x + self.alpha * (x - self.x)
        self.x = result
        return result

te=1.0/30

#定义在输出手指骨骼点之前，有多少个身体的骨骼点要输出
m=50

filters_pose = {i: OneEuroFilter(te, mincutoff=1.0, beta=0.007, dcutoff=1.0) for i in range(m)}
filters_hands = {i: OneEuroFilter(te, mincutoff=1.0, beta=0.007, dcutoff=1.0) for i in range(m, m + 2 * 21)}  # 假设有42个手的关节点

# 初始化socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ('127.0.0.1', 5054)

# 替换为要转换的视频文件路径
video_path = 'E:/JournalModel_V1/VideoTest/Video.mp4'
cap = cv2.VideoCapture(video_path)

# 获取视频的帧率
fps = cap.get(cv2.CAP_PROP_FPS)
send_rate = fps
interval = 1.0 / send_rate

mpPose = mp.solutions.pose
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=2,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
pose = mpPose.Pose(static_image_mode=False,
                    model_complexity=1,
                    smooth_landmarks=True,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

last_time_sent = 0
last_good_data = {}
loop_count = 0

n = 0  

# 加载MHFormer产生的骨骼点三维坐标npz文件数据
npz_data = np.load('E:/JournalModel_V1/VideoTest/output_keypoints_3d.npz')
reconstruction = npz_data['reconstruction']

while cap.isOpened():
    success, img = cap.read()
    if not success:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results1 = pose.process(imgRGB)
    results2 = hands.process(imgRGB)

    current_time = time.time()
    if current_time - last_time_sent > interval:
        if results1.pose_landmarks:
            mpDraw.draw_landmarks(img, results1.pose_landmarks, mpPose.POSE_CONNECTIONS)
            if results2.multi_hand_landmarks:  # 确保至少检测到一只手
                for hand_landmarks in results2.multi_hand_landmarks:
                    mpDraw.draw_landmarks(img, hand_landmarks, mpHands.HAND_CONNECTIONS)

            for pose_id, pose_lm in enumerate(results1.pose_landmarks.landmark):
                h, w, c = img.shape
                cx, cy = int(pose_lm.x * w), int(pose_lm.y * h)
                cz = pose_lm.z * 500
                visibility = pose_lm.visibility

                filtered_cx = filters_pose[pose_id].predict(cx, te)
                filtered_cy = filters_pose[pose_id].predict(cy, te)
                filtered_cz = filters_pose[pose_id].predict(cz, te)

                if visibility > 0.8:
                    # 更新最后一次visibility大于0.8的数据
                    last_good_data[pose_id] = (filtered_cx, filtered_cy, filtered_cz, visibility)
                else:
                    # 如果当前visibility小于等于0.8，使用保存的数据（如果存在的话）
                    if pose_id in last_good_data:
                        filtered_cx, filtered_cy, filtered_cz, visibility = last_good_data[pose_id]

                # 如果last_good_data中存在当前id的数据，则发送；否则不发送
                if pose_id in last_good_data:
                    data = struct.pack('4s4s4s4s', str(pose_id).encode(), str(-filtered_cz).encode(), str(filtered_cy).encode(), str(filtered_cx).encode())
                    print(pose_id, filtered_cx, filtered_cy, filtered_cz, visibility)
                    client_socket.sendto(data, serverAddressPort)
                    keypoint_n = reconstruction[n]

                    # 自定义关键点的起始 ID
                    custom_id = 33
                    for point in keypoint_n:
                        # 打包每个关键点为一个消息
                        data = struct.pack('4s4s4s4s', str(custom_id).encode(),
                                                       str(point[0]).encode(),
                                                       str(point[1]).encode(),
                                                       str(point[2]).encode())
                        print(custom_id, point[0], point[1], point[2], visibility)
                        client_socket.sendto(data, serverAddressPort)
                    # 为下一个关键点增加自定义 ID
                        custom_id +=1

            if results2.multi_hand_landmarks:
                # 初始化一个字典来存储每只手的标签和对应的关键点坐标
                hand_landmarks_dict =  {}  # 首先，检查是否有关于手的左右信息
                if results2.multi_handedness:
                    for hand_index, hand_info in enumerate(results2.multi_handedness):
                        # 提取是左手还是右手
                        hand_label = hand_info.classification[0].label
                        # 将手的标签和索引关联起来
                        hand_landmarks_dict[hand_label] = results2.multi_hand_landmarks[hand_index]

                # 然后，根据存储的关键点数据输出每只手的关键点坐标
                for hand_label, hand_landmarks in hand_landmarks_dict.items():
                    print(f"Processing {hand_label} hand landmarks...")
                    id_offset = (m + 21) if hand_label == 'Left' else m  # 左手的ID从m开始，右手从(m+21)开始

                    for hands_id, hands_lm in enumerate(hand_landmarks.landmark):
                        h, w, c = img.shape
                        cx, cy = int(hands_lm.x * w), int(hands_lm.y * h)
                        cz = hands_lm.z * 500  # 示例中的缩放因子，可能需要根据实际情况调整
                        visibility = hands_lm.visibility

                        adjusted_hands_id = hands_id + id_offset  # 调整hands_id以反映左/右手
                        # 构建要发送的数据包
                        data = struct.pack('4s4s4s4s', str(adjusted_hands_id).encode(), str(-cz).encode(), str(cy).encode(), str(cx).encode())
                        print(adjusted_hands_id, cx, cy, cz, visibility)
                        client_socket.sendto(data, serverAddressPort)

        last_time_sent = current_time

    n+=1

    cv2.imshow('Image', img)
    if cv2.waitKey(int(1000/fps)) & 0xFF == ord('q'):  # wait according to the video framerate
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
client_socket.close()
npz_data.close() 
