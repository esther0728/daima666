#卡尔曼滤波识别激光
from kalmanFilter2d import KM

# 红色激光点的阈值（可根据实际情况微调）

# 卡尔曼滤波器类，用于一维平面

class Color_detector:
    def __init__(self, IMG_CX,IMG_CY,dt_x=1.0,process_noise_x=0.1,measurement_noise_x=0.3,dt_y=1.0,\
    process_noise_y=0.1, measurement_noise_y=0.3,threshold = (59, 100, 11, 127, 17, -128),):
        # 状态向量 [位置, 速度]
        self.kf_X= KM(dt=dt_x, process_noise=process_noise_x, \
        measurement_noise=measurement_noise_x)
        self.kf_Y= KM(dt=dt_y, process_noise=process_noise_y, \
        measurement_noise=measurement_noise_y)
        self.threshold = threshold

    def find(self,img,MAX_MISSED_FRAMES=10):
        blobs = img.find_blobs([self.threshold], pixels_threshold=10, area_threshold=10, merge=False)
        missed_frames = 0
        if blobs:
            missed_frames = 0
            laser_blob = min(blobs, key=lambda b: b.pixels())
            x = laser_blob.cx()
            y = laser_blob.cy()

            # 更新卡尔曼
            filtered_x = int(self.kf_X.update(x))
            filtered_y = int(self.kf_Y.update(y))

            return filtered_x,filtered_y
        else:
                missed_frames += 1
                if missed_frames <= MAX_MISSED_FRAMES:
                    # 使用预测值继续运行（但不更新）
                    filtered_x = int(self.kf_X.update(self.kf_X.x[0][0]))
                    filtered_y = int(self.kf_Y.update(self.kf_Y.x[0][0]))
                    img.draw_cross(filtered_x, filtered_y, color=(255, 0, 0))
                    return filtered_x,filtered_y
                else:
                    # 如果丢失太久，重置状态，避免漂移
                    filtered_x = self.IMG_CX
                    filtered_y = self.IMG_CY
                    self.kf_X.x = [[self.IMG_CX], [0.0]]
                    self.kf_Y.x = [[self.IMG_CY], [0.0]]
                    return filtered_x,filtered_y
