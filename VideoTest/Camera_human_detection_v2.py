import cv2
import mediapipe as mp
import socket
import struct
import time
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
m=33

filters_pose = {i: OneEuroFilter(te, mincutoff=1.0, beta=0.007, dcutoff=1.0) for i in range(m)}
filters_hands = {i: OneEuroFilter(te, mincutoff=1.0, beta=0.007, dcutoff=1.0) for i in range(m, m + 2 * 21)}  # 假设有42个手的关节点

# 初始化socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ('127.0.0.1', 5054)

cap = cv2.VideoCapture(1)


# 创建命名窗口
cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
# 调整窗口大小，宽度为800，高度为600
cv2.resizeWindow('Image', 800, 600)

mpPose = mp.solutions.pose
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=2,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
pose = mpPose.Pose(static_image_mode=False,
                    model_complexity=0,
                    smooth_landmarks=True,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

# 设置每秒发送数据的次数
send_rate = 10
# 计算发送数据包之间的时间间隔
interval = 1.0 / send_rate

# 记录上次发送数据的时间
last_time_sent = 0

# 用于存储最后一次visibility大于0.5的数据
last_good_data ={0: (103.48941253248526, 115.95394006218334, 31.80816983336915, 0.9981022477149963), 1: (107.46120654124661, 114.46404846314257, 33.96248393951093, 0.997654378414154), 2: (108.7633905708597, 115.16786805785905, 33.993855740476974, 0.9975407123565674), 3: (109.86790783018539, 115.77393201905885, 34.08103935813216, 0.9976898431777954),4: (103.09251962636098, 113.10492641770566, 34.547687997500034, 0.9973093271255493),5: (101.67111590020573, 112.34953724115825, 34.460267703041936, 0.9968141317367554),6: (99.83869667816711, 111.8671758879473, 34.607820671160226, 0.9971314668655396),7: (122.73026829958707, 124.05301031178918, 60.80389401817161, 0.9933122396469116),8: (109.69246047783298, 117.75337404046465, 61.12900656772611, 0.9945254921913147),9: (111.1804017899423, 122.11017903954253, 43.755625957470514, 0.995082676410675),10: (106.28513182350339, 120.56073291510214, 44.30903303194469, 0.9946399927139282),11: (144.3083782293274, 152.502819415191, 84.74140914539241, 0.9990116953849792),12: (108.52603531618416, 139.45337560850749, 84.90764544515858, 0.9981673955917358),13: (223.5905972164139, 201.05202047185037, 52.96139055452895, 0.8028899431228638),14: (162.7163155026077, 173.14991462186876, 76.32100135069257, 0.8119422793388367),15: (370.33019603876915, 672.7431437863745, -89.92338189683096, 0.8166356682777405),16: (32.55970166636166, 451.5592267277267, -583.0485887887686, 0.8951460123062134),17: (210.7751129189005, 343.32536524908767, -328.97053287828317, 0.8656802177429199),18: (53.78021373108288, 411.2764153928512, -633.4732819005089, 0.8576845526695251),19: (200.30931360521845, 312.971019360251, -308.08122774067164, 0.8725942373275757),20: (58.51126451159075, 368.6191660696261, -616.0331014052841, 0.8672649264335632),21: (197.87755172409766, 334.2092497138839, -275.7851318151397, 0.8438682556152344),22: (63.22310287395612, 390.2474488833391, -588.4323003239626, 0.809900164604187),23: (144.46032970770693, 212.25925119821824, 111.59963149893603, 0.9987196922302246),24: (122.74415387424655, 204.8389839965846, 108.88251715896445, 0.9981961846351624),25: (156.9846740640735, 275.086952454674, 138.91553083546373, 0.9455177783966064),26: (142.59588674602736, 265.0722892291608, 120.78654400697928, 0.9246092438697815),27: (201.09727383163658, 340.4468846516141, 174.0248600112675, 0.9503312706947327),28: (171.82328840028956, 320.1765915307104, 165.32059606237615, 0.9184041619300842),29: (202.3033554601878, 345.2309078080343, 175.10837357985048, 0.7776890993118286),30: (175.05132204691924, 327.14315734761533, 168.7744895271842, 0.695661723613739),31: (182.10776263350397, 362.7196934845872, 149.19309107476883, 0.9334942102432251),32: (158.02193378875398, 347.96526705151757, 139.90637148879983, 0.907093346118927)}
while True:
    success, img = cap.read()
    if not success:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results1 = pose.process(imgRGB)
    results2 = hands.process(imgRGB)

    current_time = time.time()
    # 检查是否到达了发送数据的时间间隔
    if current_time - last_time_sent > interval:
        if results1.pose_landmarks:
            mpDraw.draw_landmarks(img, results1.pose_landmarks, mpPose.POSE_CONNECTIONS)
            if results2.multi_hand_landmarks:  # 确保至少检测到一只手
                for hand_landmarks in results2.multi_hand_landmarks:
                    mpDraw.draw_landmarks(img, hand_landmarks, mpHands.HAND_CONNECTIONS)

            for pose_id, pose_lm in enumerate(results1.pose_landmarks.landmark):
                h, w, c = img.shape
                cx, cy = int(pose_lm.x * w), int(pose_lm.y * h)
                cz = pose_lm.z * 300
                visibility = pose_lm.visibility

                filtered_cx = filters_pose[pose_id].predict(cx, te)
                filtered_cy = filters_pose[pose_id].predict(cy, te)
                filtered_cz = filters_pose[pose_id].predict(cz, te)

                if pose_id>22 and pose_id<33:
                    last_good_data.update({23: (302.2308393134022, 483.8960809746402, 183.19335458444567, 0.11345458775758743), 24: (192.79909642240824, 440.62866343052735, 150.3614691560204, 0.12654505670070648), 25: (396.69480698145577, 742.9151971560309, 248.76073692922006, 0.06023108959197998), 26: (303.59163057537455, 728.8271528307644, 280.4678999090733, 0.07254263758659363), 27: (527.8907545434233, 940.9094280183409, 539.8814345470064, 0.016500936821103096), 28: (370.8396461888377, 959.2393260545532, 470.73372741287915, 0.03534586355090141), 29: (533.3901622172166, 969.8678760347705, 574.9253326799737, 0.011625710874795914), 30: (369.56451682443594, 995.8443029453464, 491.3597420983331, 0.023926319554448128), 31: (516.6424909055154, 1047.9872322701608, 480.78871697522106, 0.019263798370957375), 32: (380.11393588771705, 1079.1291452609216, 384.31994416682903, 0.03108781762421131)})
                    filtered_cx, filtered_cy, filtered_cz, visibility = last_good_data[pose_id]

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

        # 更新上次发送数据的时间
        last_time_sent = current_time

    cv2.imshow('Image', img)


    
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
client_socket.close()