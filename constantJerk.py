import math


class ConstantJerk:
    def __init__(self, jerk_max: float, acc_max: float, v_max: float, s: float):
        self.jerk_max = jerk_max
        self.acc_max = acc_max
        self.v_max = v_max

        self.trajectory_instance_case = self.__get_trajectory_instance_case(s)
        tj, ta, tv = self.__calculate_times(s)
        self.t1 = tj
        self.t2 = ta
        self.t3 = ta + tj
        self.t4 = tv
        self.t5 = tv + tj
        self.t6 = tv + ta
        self.t7 = tv + tj + ta

    def __get_trajectory_instance_case(self, s: float):
        # Case 1: a_max reached, v_max reached and constant for a time
        #          (s = 100,   j = 2000, a = 500,  vMax = 120)
        # Case 2: a_max not reached, v_max not reached
        #          (s = 15000, j = 2000, a = 5500, vMax = 20500)
        # Case 3: a_max not reached, v_max reached and constant for a time
        #          (s = 15000, j = 2000, a = 5500, vMax = 2500)
        # Case 4: a_max reached, v_max not reached
        #          (s = 57,    j = 2000, a = 500,  vMax = 120)
        # Case 5: a_max reached and constant for a time, v_max reached and constant for a time
        #          (s = 15000, j = 2000, a = 500,  vMax = 2500)
        # Case 6: a_max reached and constant for a time, v_max not reached
        #          (s = 15000, j = 2000, a = 500,  vMax = 20500)

        v_a = self.acc_max * self.acc_max / self.jerk_max
        s_a = 2 * self.acc_max * self.acc_max * self.acc_max / (self.jerk_max * self.jerk_max)
        s_v = 0.0
        if self.v_max * self.jerk_max < self.acc_max * self.acc_max:
            s_v = self.v_max * 2 * math.sqrt(self.v_max / self.jerk_max)
        else:
            s_v = self.v_max * (self.v_max / self.acc_max + self.acc_max / self.jerk_max)

        if self.v_max < v_a and s >= s_a:
            return 1
        elif self.v_max >= v_a and s < s_a:
            return 2
        elif self.v_max < v_a and s_a > s >= s_v:
            return 3
        elif self.v_max < v_a and s < s_a and s < s_v:
            return 4
        elif self.v_max >= v_a and s >= s_a and s >= s_v:
            return 5
        elif self.v_max >= v_a and s_a <= s < s_v:
            return 6
        else:
            return None

    def __calculate_times(self, s: float):
        if self.trajectory_instance_case == 1 or self.trajectory_instance_case == 3:
            tj = math.sqrt(self.v_max / self.jerk_max)
            ta = tj
            tv = s / self.v_max
            return tj, ta, tv
        elif self.trajectory_instance_case == 2 or self.trajectory_instance_case == 4:
            tj = Ramp.__calc_cube_root(s / (2 * self.jerk_max))
            ta = tj
            tv = 2 * tj
            return tj, ta, tv
        elif self.trajectory_instance_case == 5:
            tj = self.acc_max / self.jerk_max
            ta = self.v_max / self.acc_max
            tv = s / self.v_max
            return tj, ta, tv
        elif self.trajectory_instance_case == 6:
            tj = self.acc_max / self.jerk_max
            ta = 0.5 * (math.sqrt(
                (4 * s * self.jerk_max * self.jerk_max + self.acc_max * self.acc_max * self.acc_max) / (
                            self.acc_max * self.jerk_max * self.jerk_max)) - self.acc_max / self.jerk_max)
            tv = ta + tj
            return tj, ta, tv
        else:
            raise Exception("TrajectoryInstance must be between 1 and 6")

    @staticmethod
    def __calc_cube_root(x: float):
        return math.pow(x, (1.0 / 3))

    def get_status(self, t: float):
        if t <= self.t1:
            return self.__get_status1(t)
        else:
            j1, a1, v1, s1 = self.__get_status1(self.t1)
            if t <= self.t2:
                return self.__get_status2(t, a1, v1, s1)
            else:
                j2, a2, v2, s2 = self.__get_status2(self.t2, a1, v1, s1)
                if t <= self.t3:
                    return self.__get_status3(t, a2, v2, s2)
                else:
                    j3, a3, v3, s3 = self.__get_status3(self.t3, a2, v2, s2)
                    if t <= self.t4:
                        return self.__get_status4(t, v3, s3)
                    else:
                        j4, a4, v4, s4 = self.__get_status4(self.t4, v3, s3)
                        if t <= self.t5:
                            return self.__get_status5(t, v4, s4)
                        else:
                            j5, a5, v5, s5 = self.__get_status5(self.t5, v4, s4)
                            if t <= self.t6:
                                return self.__get_status6(t, a5, v5, s5)
                            else:
                                j6, a6, v6, s6 = self.__get_status6(self.t6, a5, v5, s5)
                                return self.__get_status7(min(t, self.t7), a6, v6, s6)

    def __get_status1(self, t: float):
        j = self.jerk_max
        a = self.jerk_max * t
        v = 0.5 * self.jerk_max * t * t
        s = self.jerk_max / 6 * t * t * t
        return j, a, v, s

    def __get_status2(self, t: float, a1: float, v1: float, s1: float):
        j = 0
        a = a1
        v = v1 + a1 * (t - self.t1)
        s = s1 + v1 * (t - self.t1) + 0.5 * a1 * (t - self.t1) * (t - self.t1)
        return j, a, v, s

    def __get_status3(self, t: float, a2: float, v2: float, s2: float):
        t_phase = t - self.t2
        t_phase2 = t_phase * t_phase
        t_phase3 = t_phase2 * t_phase
        j = -self.jerk_max
        a = a2 - self.jerk_max * t_phase
        v = v2 + a2 * t_phase + 0.5 * -self.jerk_max * t_phase2
        s = s2 + v2 * t_phase + 0.5 * a2 * t_phase2 + -self.jerk_max / 6 * t_phase3
        return j, a, v, s

    def __get_status4(self, t: float, v3: float, s3: float):
        t_phase = t - self.t3
        j = 0
        a = 0
        v = v3
        s = s3 + v3 * t_phase
        return j, a, v, s

    def __get_status5(self, t: float, v4: float, s4: float):
        t_phase = t - self.t4
        t_phase2 = t_phase * t_phase
        t_phase3 = t_phase2 * t_phase
        j = -self.jerk_max
        a = -self.jerk_max * t_phase
        v = v4 + 0.5 * -self.jerk_max * t_phase2
        s = s4 + v4 * t_phase + -self.jerk_max / 6 * t_phase3
        return j, a, v, s

    def __get_status6(self, t: float, a5: float, v5: float, s5: float):
        t_phase = t - self.t5
        t_phase2 = t_phase * t_phase
        j = 0
        a = a5
        v = v5 - self.acc_max * t_phase
        s = s5 + v5 * t_phase + 0.5 * a5 * t_phase2
        return j, a, v, s

    def __get_status7(self, t: float, a6: float, v6: float, s6: float):
        t_phase = t - self.t6
        t_phase2 = t_phase * t_phase
        t_phase3 = t_phase2 * t_phase
        j = self.jerk_max
        a = a6 + self.jerk_max * t_phase
        v = v6 + a6 * t_phase + 0.5 * self.jerk_max * t_phase2
        s = s6 + v6 * t_phase + 0.5 * a6 * t_phase2 + self.jerk_max / 6 * t_phase3
        return j, a, v, s
