from pyb import millis
from math import pi, isnan

class PID:
    # 初始化 PID 参数和状态变量
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_derivative = _last_t = 0
    _RC = 1 / (2 * pi * 20)  # 低通滤波时间常数（用于微分滤波）

    def __init__(self, p=0, i=0, d=0, imax=0):
        self._kp = float(p)                 # 比例系数
        self._ki = float(i)                 # 积分系数
        self._kd = float(d)                 # 微分系数
        self._imax = abs(imax)              # 积分限幅
        self._last_derivative = float('nan')  # 上次微分值

    def get_pid(self, error, scaler):
        tnow = millis()                   # 获取当前时间（毫秒）
        dt = tnow - self._last_t          # 计算时间差
        output = 0                        # PID 输出初始值

        # 第一次运行或长时间未更新，跳过微分和积分部分
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()                # 重置积分器

        self._last_t = tnow               # 更新上次时间
        delta_time = float(dt) / 1000.0   # 时间差单位换算成秒

        # --- 比例项 ---
        output += error * self._kp

        # --- 微分项 ---
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time

            # 一阶低通滤波（抑制高频噪声）
            derivative = self._last_derivative + \
                         ((delta_time / (self._RC + delta_time)) * \
                         (derivative - self._last_derivative))

            self._last_error = error
            self._last_derivative = derivative

            output += self._kd * derivative

        output *= scaler  # 放大输出（比例缩放）

        # --- 积分项 ---
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time

            # 限制积分器范围，防止积分饱和（windup）
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax

            output += self._integrator

        return output

    def reset_I(self):
        # 重置积分器和微分器状态
        self._integrator = 0
        self._last_derivative = float('nan')




def pid_control(error,last_error,integral,kp,ki,kd):
    Proportional=kp*error
    integral+=error
    integral_term=ki*integral
    derivative=kd*(error-last_error)
    output=Proportional+integral_term+derivative
    last_error=error
    return output,last_error,integral
