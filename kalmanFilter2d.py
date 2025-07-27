class KM:
    def __init__(self, dt=1.0, process_noise=1e-2, measurement_noise=1e-1):
        # 状态向量 [位置, 速度]
        self.x = [[0.0], [0.0]]

        # 状态转移矩阵 A
        self.F = [[1.0, dt],
                  [0.0, 1.0]]

        # 控制输入矩阵 B（这里不使用）
        self.B = [[0.0],
                  [0.0]]

        # 观测矩阵 H（只观测位置）
        self.H = [[1.0, 0.0]]

        # 状态协方差矩阵 P
        self.P = [[1.0, 0.0],
                  [0.0, 1.0]]

        # 过程噪声协方差 Q
        self.Q = [[process_noise, 0.0],
                  [0.0, process_noise]]

        # 观测噪声协方差 R
        self.R = [[measurement_noise]]

        # 单位矩阵 I
        self.I = [[1.0, 0.0],
                  [0.0, 1.0]]

    def update(self, measurement):
        # Prediction
        # x = F * x
        x0 = self.F[0][0] * self.x[0][0] + self.F[0][1] * self.x[1][0]
        x1 = self.F[1][0] * self.x[0][0] + self.F[1][1] * self.x[1][0]
        self.x = [[x0], [x1]]

        # P = F * P * F^T + Q
        FT = [[self.F[0][0], self.F[1][0]],
              [self.F[0][1], self.F[1][1]]]
        FP = [[self.F[0][0]*self.P[0][0] + self.F[0][1]*self.P[1][0],
               self.F[0][0]*self.P[0][1] + self.F[0][1]*self.P[1][1]],
              [self.F[1][0]*self.P[0][0] + self.F[1][1]*self.P[1][0],
               self.F[1][0]*self.P[0][1] + self.F[1][1]*self.P[1][1]]]
        self.P = [
            [FP[0][0]*FT[0][0] + FP[0][1]*FT[1][0] + self.Q[0][0],
             FP[0][0]*FT[0][1] + FP[0][1]*FT[1][1] + self.Q[0][1]],
            [FP[1][0]*FT[0][0] + FP[1][1]*FT[1][0] + self.Q[1][0],
             FP[1][0]*FT[0][1] + FP[1][1]*FT[1][1] + self.Q[1][1]]
        ]

        # Update
        # y = z - H * x
        z = measurement
        y = z - (self.H[0][0] * self.x[0][0] + self.H[0][1] * self.x[1][0])

        # S = H * P * H^T + R
        S = self.H[0][0]*self.P[0][0]*self.H[0][0] + \
            self.H[0][0]*self.P[0][1]*self.H[0][1] + \
            self.H[0][1]*self.P[1][0]*self.H[0][0] + \
            self.H[0][1]*self.P[1][1]*self.H[0][1] + self.R[0][0]

        # K = P * H^T / S
        K = [
            [(self.P[0][0]*self.H[0][0] + self.P[0][1]*self.H[0][1]) / S],
            [(self.P[1][0]*self.H[0][0] + self.P[1][1]*self.H[0][1]) / S]
        ]

        # x = x + K * y
        self.x[0][0] += K[0][0] * y
        self.x[1][0] += K[1][0] * y

        # P = (I - K*H) * P
        KH = [
            [K[0][0]*self.H[0][0], K[0][0]*self.H[0][1]],
            [K[1][0]*self.H[0][0], K[1][0]*self.H[0][1]]
        ]
        I_KH = [
            [self.I[0][0] - KH[0][0], self.I[0][1] - KH[0][1]],
            [self.I[1][0] - KH[1][0], self.I[1][1] - KH[1][1]]
        ]
        self.P = [
            [I_KH[0][0]*self.P[0][0] + I_KH[0][1]*self.P[1][0],
             I_KH[0][0]*self.P[0][1] + I_KH[0][1]*self.P[1][1]],
            [I_KH[1][0]*self.P[0][0] + I_KH[1][1]*self.P[1][0],
             I_KH[1][0]*self.P[0][1] + I_KH[1][1]*self.P[1][1]]
        ]

        return self.x[0][0]  # 返回估计的“位置”
