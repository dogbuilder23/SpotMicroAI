class CatGait:
    def __init__(self, current_time):
        self.servos = 8
        self.frame_incr = 1
        self.frame = -1 * self.frame_incr
        self.fixed_seconds_per_frame = 0.04
        self.angles = []
        self.frame_start_time = current_time
        self.frame_start_angles = None
        self.first_step = True

    def getNumServos(self):
        return self.servos

    # TODO(dwind): Remove this. Make sleep time decisions outside of Gaits.
    def sleepSecs(self):
        return self.fixed_seconds_per_frame

    # Remove this, if no longer waiting to begin the simulation.
    def adjustStartTime(self, current_time):
        self.frame_start_time = current_time

    def getAngles(self, current_time, current_angles):
        values_per_frame = self.servos
        if self.fixed_seconds_per_frame is None:
            values_per_frame = values_per_frame + 1  # +1 for per-frame time
        frames = int(len(self.angles) / values_per_frame)
        next_frame = int((self.frame + self.frame_incr) % frames)
        next_frame_start_idx = next_frame * values_per_frame
        next_frame_end_idx = next_frame_start_idx + values_per_frame

        # Determine the deadline to reach the target angles.
        next_frame_end_time = self.fixed_seconds_per_frame
        if self.fixed_seconds_per_frame is None:
            next_frame_end_idx = next_frame_end_idx - 1
            next_frame_end_time = self.angles[next_frame_end_idx]

        print(f's={self.frame_start_time:.3f}. sa={self.frame_start_angles}. n={next_frame}. t={next_frame_end_time}.')

        # Cache the starting angles, if we're transitioning into new gait.
        if self.first_step:
            self.frame_state_angles = current_angles
            self.first_step = False

        # If we're at or past the deadline, move to the target angles now.
        if current_time >= next_frame_end_time + self.frame_start_time:
            self.frame = next_frame
            target_angles = self.angles[next_frame_start_idx:next_frame_end_idx]
            self.frame_state_angles = target_angles
            self.frame_start_time = current_time
            return target_angles

        step_ratio = (current_time - self.frame_start_time) / next_frame_end_time
        step_angles = [0] * self.servos
        for i in range(self.servos):
            target = self.angles[next_frame_start_idx + i];
            start = self.frame_state_angles[i]
            step_angles[i] = (target - start) * step_ratio + start

        print(f'n={next_frame}. t={next_frame_end_time}. r={step_ratio:.1f}.')

        return step_angles

class DadTwist(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.servos = 12
        self.fixed_seconds_per_frame = None  # use per-frame timing
        self.angles = [
            6,6,6,6,    44,44,-44,-44,   6,6,-6,-6,         0.500,
            -27,22,-27,22,  32,37,-32,-37,  67, 50, -67, -50,   0.4, #0.639, # Yaw Left
            6,6,6,6,    44,44,-44,-44,   6,6,-6,-6,         0.4, #0.639,
            6,6,6,6,    44,44,-44,-44,   6,6,-6,-6,         0.4, #0.639,
            22,-27,22,-27,  37,32,-37,-32,  50, 67, -50, -67,   0.4, #0.639, # Yaw Right
            6,6,6,6,    44,44,-44,-44,   6,6,-6,-6,         0.4, #0.639,
            5,5,9,9,    29,29,-33,-33,  77, 77, 28, 28,     0.740,
            6,6,6,6,44,44,-44,-44,6,6,-6,-6,0.740,
            9,9,5,5,33,33,-29,-29,-28,-28,-77,-77,0.740,
            6,6,6,6,44,44,-44,-44,6,6,-6,-6,0.740,
            -31,31,31,-31,  31,31,-31,-31,  20, 0,  0,-20, 0.437,
            6,6,6,6,    44,44,-44,-44,   6, 6, -6, -6, 0.437,
            31,-31,-31,31,  31,31,-31,-31,   0,20,-20,  0, 0.437,
            6,6,6,6,    44,44,-44,-44,   6, 6, -6, -6, 0.437,]

    def sleepSecs(self):
        return .1

class TestPose(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.servos = 12
        self.angles = [35,6,6,6,    44,44,-44,-44,   6, 6, -6, -6,]


class DadYawLeft(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.servos = 12
        self.angles = [-27,22,-27,22,  32,37,-32,-37,  -3, 67,   3, -67,   0.639,]

class JustRollRight(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.servos = 12
        self.angles = [-31,31,31,-31, 44,44,-44,-44, 6,6,-6,-6,]

class DadRollRight(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.servos = 12
        self.angles = [-31,31,31,-31,  31,31,-31,-31,  20, 0,  0,-20, 0.437,]


class CatBack(CatGait):  # aka BK

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             30, 39,-57,-64,  6, -9, -6,  9,
             28, 49,-58,-56,  7,-11, -7, 11,
             25, 58,-59,-47,  9,-10, -9, 11,
             23, 65,-60,-37, 10, -8,-10,  8,
             20, 66,-61,-34, 12, -6,-12,  6,
             17, 66,-62,-31, 14, -3,-14,  3,
             14, 64,-63,-28, 16,  1,-16, -1,
             12, 61,-63,-27, 18,  5,-18, -5,
              9, 58,-64,-27, 20,  8,-20, -8,
              6, 57,-64,-30, 22,  7,-22, -7,
              3, 55,-64,-32, 25,  6,-25, -6,
              0, 54,-64,-35, 28,  5,-28, -5,
             -3, 52,-64,-37, 31,  4,-32, -4,
             -7, 50,-63,-40, 35,  4,-35, -4,
            -10, 48,-64,-42, 38,  3,-38, -3,
            -10, 46,-66,-44, 36,  3,-36, -3,
             -9, 44,-69,-46, 32,  3,-32, -3,
             -6, 42,-73,-48, 25,  3,-25, -3,
             -2, 40,-75,-50, 19,  3,-19, -4,
              3, 38,-76,-52, 14,  4,-14, -4,
             15, 35,-74,-53,  4,  5, -4, -5,
             26, 33,-71,-55, -3,  5,  3, -5,
             37, 30,-65,-57, -8,  6,  8, -6,
             47, 28,-58,-58,-11,  7, 11, -7,
             56, 25,-49,-59,-11,  9, 11, -9,
             64, 23,-39,-60, -9, 10,  9,-10,
             66, 20,-35,-61, -7, 12,  7,-12,
             66, 17,-31,-62, -4, 14,  4,-14,
             65, 14,-29,-63,  0, 16,  0,-16,
             62, 12,-27,-63,  4, 18, -5,-18,
             59,  9,-27,-64,  7, 20, -7,-20,
             57,  6,-29,-64,  7, 22, -7,-22,
             56,  3,-32,-64,  6, 25, -6,-25,
             54,  0,-34,-64,  5, 28, -5,-28,
             52, -3,-37,-64,  4, 31, -4,-32,
             51, -7,-39,-63,  4, 35, -4,-35,
             49,-10,-41,-64,  3, 38, -3,-38,
             47,-10,-43,-66,  3, 36, -3,-36,
             45, -9,-46,-69,  3, 32, -3,-32,
             43, -6,-48,-73,  3, 25, -3,-25,
             40, -2,-49,-75,  3, 19, -4,-19,
             38,  3,-51,-76,  4, 14, -4,-14,
             36, 15,-53,-74,  5,  4, -5, -4,
             33, 26,-55,-71,  5, -3, -5,  3,
             31, 37,-56,-65,  6, -8, -6,  8,]


class CatBalance(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [30, 30, -30, -30, 30, 30, -30, -30,]
        # [50, 50,-50,-50, 50, 50,-50,-50,]


class CatBound(CatGait):  # aka BD

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             39, 39,-80,-80, 20, 20, 47, 47,
             29, 29,-73,-73, 24, 24, 48, 48,
             25, 25,-64,-64, 25, 25, 46, 46,
             23, 23,-55,-55, 24, 24, 43, 43,
             22, 22,-47,-47, 20, 20, 38, 38,
             21, 21,-41,-41, 15, 15, 32, 32,
             22, 22,-35,-35,  8,  8, 25, 25,
             24, 24,-31,-31,  0,  0, 19, 19,
             26, 26,-28,-28, -8, -8, 11, 11,
             29, 29,-25,-25,-15,-15,  3,  3,
             33, 33,-24,-24,-23,-23, -4, -4,
             38, 38,-23,-23,-30,-30,-10,-10,
             44, 44,-24,-24,-36,-36,-15,-15,
             52, 52,-26,-26,-42,-42,-18,-18,
             60, 60,-30,-30,-45,-45,-18,-18,
             68, 68,-36,-36,-47,-47,-21,-21,
             77, 77,-40,-40,-47,-47,-25,-25,
             84, 84,-43,-43,-46,-46,-25,-25,
             87, 87,-48,-48,-42,-42,-22,-22,
             89, 89,-54,-54,-37,-37,-17,-17,
             88, 88,-60,-60,-30,-30,-12,-12,
             85, 85,-66,-66,-24,-24, -5, -5,
             81, 81,-72,-72,-16,-16,  3,  3,
             76, 76,-78,-78, -8, -8, 11, 11,
             70, 70,-82,-82, -1, -1, 19, 19,
             64, 64,-86,-86,  5,  5, 26, 26,
             58, 58,-88,-88, 12, 12, 32, 32,
             52, 52,-89,-89, 16, 16, 39, 39,
             47, 47,-87,-87, 18, 18, 43, 43,
             41, 41,-82,-82, 19, 19, 46, 46,]


class CatCrawl(CatGait):  # aka CR

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             35, 37,-46,-53,-23,-32, -3, 12,
             39, 30,-43,-57,-24,-29, -3, 11,
             42, 24,-40,-61,-25,-26, -4, 12,
             47, 18,-37,-65,-26,-22, -5, 11,
             51, 12,-34,-69,-26,-19, -7, 10,
             54,  6,-31,-72,-27,-14, -8,  9,
             58,  1,-28,-74,-26, -9,-10,  6,
             61, -3,-25,-67,-26,  2,-11, -9,
             64,  1,-21,-64,-26, -1,-14, -9,
             67,  5,-18,-63,-25, -5,-16, -7,
             75,  9,-16,-61,-29, -8,-12, -6,
             73, 13,-24,-59,-35,-11, -1, -4,
             68, 16,-29,-57,-36,-13,  3, -4,
             62, 20,-34,-55,-36,-16,  5, -3,
             56, 25,-39,-52,-36,-18,  7, -3,
             50, 29,-43,-50,-35,-20,  9, -2,
             44, 32,-48,-47,-34,-22, 11, -3,
             37, 36,-53,-45,-32,-23, 12, -3,
             31, 40,-56,-42,-29,-24, 12, -4,
             25, 44,-61,-39,-26,-25, 12, -5,
             19, 48,-64,-36,-23,-26, 11, -6,
             13, 52,-68,-33,-19,-26, 10, -7,
              7, 56,-71,-30,-15,-27,  9, -9,
              1, 59,-74,-27, -9,-26,  7,-10,
             -4, 62,-69,-23,  2,-26, -7,-13,
              1, 65,-64,-20, -1,-26, -9,-15,
              4, 68,-63,-17, -4,-24, -7,-17,
              8, 76,-61,-20, -7,-33, -6, -5,
             12, 71,-59,-25,-10,-35, -5,  0,
             16, 66,-57,-31,-13,-36, -4,  3,
             20, 60,-55,-36,-15,-37, -3,  6,
             24, 54,-53,-40,-18,-36, -3,  7,
             28, 48,-50,-45,-19,-35, -3,  9,
             32, 42,-48,-49,-21,-33, -3, 11,]


class CatLY(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
        114,117,-45,-53, 52, 49,-38,-24,
        114,117,-39,-58, 52, 49,-42,-23,
        114,117,-34,-62, 52, 49,-46,-21,
        114,116,-26,-66, 52, 48,-54,-22,
        114,116,-22,-66, 54, 48,-59,-25,
        114,116,-25,-64, 54, 48,-50,-30,
        115,115,-30,-60, 52, 50,-42,-34,
        116,115,-35,-58, 50, 50,-36,-34,
        116,115,-42,-54, 50, 50,-31,-34,
        116,115,-47,-49, 50, 50,-28,-36,
        117,114,-53,-43, 49, 52,-25,-39,
        117,114,-58,-37, 49, 52,-22,-44,
        117,114,-63,-30, 49, 52,-21,-50,
        116,114,-66,-22, 48, 54,-22,-59,
        116,114,-66,-24, 48, 54,-25,-54,
        116,115,-63,-27, 48, 52,-31,-47,
        115,116,-60,-31, 50, 50,-34,-41,
        115,116,-58,-38, 50, 50,-33,-34,
        115,116,-53,-44, 50, 50,-34,-29,
        115,116,-48,-50, 50, 50,-36,-26,]


class CatMarch(CatGait):  # aka VT

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
         51, 39,-57,-43,-18,  7, 19, -7,
         42, 39,-47,-43,  1,  7,  0, -7,
         39, 39,-43,-43,  7,  7, -7, -7,
         39, 39,-43,-43,  7,  7, -7, -7,
         39, 42,-43,-47,  7,  0, -7,  0,
         39, 51,-43,-57,  7,-19, -7, 19,
         39, 59,-43,-67,  7,-36, -7, 36,
         39, 59,-43,-66,  7,-35, -7, 36,
         39, 51,-43,-57,  7,-18, -7, 19,
         39, 42,-43,-47,  7,  1, -7,  0,
         39, 39,-43,-43,  7,  7, -7, -7,
         39, 39,-43,-43,  7,  7, -7, -7,
         40, 39,-45,-43,  3,  7, -3, -7,
         50, 39,-56,-43,-16,  7, 16, -7,
         58, 39,-65,-43,-33,  7, 33, -7,
         60, 39,-68,-43,-38,  7, 38, -7,
         52, 39,-59,-43,-21,  7, 22, -7,]

class CatOurSit(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [45, 45, -90, -90, 5, 5, 60, 60,]

class CatPushUp(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            20, 20, 60, 60, 60, 60,-55,-55,
            28, 28, 56, 56, 39, 39,-55,-55,
            28, 28, 56, 56, 39, 39,-55,-55,
            36, 36, 51, 51, 18, 18,-55,-55,
            36, 36, 51, 51, 18, 18,-55,-55,
            44, 44, 47, 47, -3, -3,-55,-55,
            44, 44, 47, 47, -3, -3,-55,-55,
            52, 52, 43, 43,-24,-24,-55,-55,
            52, 52, 43, 43,-24,-24,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            60, 60, 40, 40,-45,-45,-55,-55,
            52, 52, 43, 43,-24,-24,-55,-55,
            52, 52, 43, 43,-24,-24,-55,-55,
            44, 44, 47, 47, -3, -3,-55,-55,
            44, 44, 47, 47, -3, -3,-55,-55,
            36, 36, 51, 51, 18, 18,-55,-55,
            36, 36, 51, 51, 18, 18,-55,-55,
            28, 28, 56, 56, 39, 39,-55,-55,
            28, 28, 56, 56, 39, 39,-55,-55,]

class CatSit(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [30, 30,-90,-90, 60, 60, 45, 45,]

class CatSleep(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [80, 80,-80,-80,-55,-55, 55, 55,]

class CatStretch(CatGait):  # aka str

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [-60,-60,-15,-15, 60, 60,-45,-45,]

class CatTrot(CatGait):  # aka TR

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
         43, 48,-51,-56,  7, -2, -8,  1,
         46, 37,-48,-63,  7,  0, -7, -4,
         49, 25,-45,-67,  7,  5, -7,-12,
         52, 12,-42,-68,  8, 14, -7,-25,
         54,  0,-38,-66, 10, 24, -7,-40,
         56, -4,-35,-62, 12, 30, -8,-48,
         58, -5,-31,-58, 14, 35, -9,-55,
         60, -2,-28,-58, 17, 33,-10,-51,
         60,  3,-23,-60, 20, 28,-13,-43,
         61,  7,-19,-61, 24, 24,-15,-36,
         61, 12,-15,-61, 28, 20,-18,-31,
         61, 16,-10,-61, 33, 17,-22,-27,
         62, 21, -8,-61, 35, 14,-22,-22,
         66, 25, -9,-60, 30, 12,-18,-19,
         68, 29,-12,-59, 24, 10,-13,-16,
         68, 33,-22,-57, 15,  8, -6,-13,
         64, 36,-35,-56,  5,  7,  0,-11,
         57, 40,-46,-53,  0,  7,  2, -9,
         48, 43,-56,-51, -2,  7,  1, -8,
         37, 46,-63,-48,  0,  7, -4, -7,
         25, 49,-67,-45,  5,  7,-12, -7,
         12, 52,-68,-42, 14,  8,-25, -7,
          0, 54,-66,-38, 24, 10,-40, -7,
         -4, 56,-62,-35, 30, 12,-48, -8,
         -5, 58,-58,-31, 35, 14,-55, -9,
         -2, 60,-58,-28, 33, 17,-51,-10,
          3, 60,-60,-23, 28, 20,-43,-13,
          7, 61,-61,-19, 24, 24,-36,-15,
         12, 61,-61,-15, 20, 28,-31,-18,
         16, 61,-61,-10, 17, 33,-27,-22,
         21, 62,-61, -8, 14, 35,-22,-22,
         25, 66,-60, -9, 12, 30,-19,-18,
         29, 68,-59,-12, 10, 24,-16,-13,
         33, 68,-57,-22,  8, 15,-13, -6,
         36, 64,-56,-35,  7,  5,-11,  0,
         40, 57,-53,-46,  7,  0, -9,  2,]


class CatWalk(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             12, 59,-55,-49, 23, 24, -2,-12,
             15, 59,-63,-47, 22, 27, -8,-11,
             18, 59,-67,-45, 20, 30,-20,-11,
             21, 59,-66,-43, 18, 34,-33,-10,
             24, 59,-64,-40, 16, 38,-37,-10,
             27, 58,-62,-37, 15, 43,-41,-11,
             30, 57,-60,-35, 13, 47,-45,-12,
             32, 58,-57,-32, 13, 47,-48,-13,
             35, 60,-57,-29, 12, 45,-47,-14,
             38, 62,-58,-26, 12, 41,-42,-15,
             40, 65,-59,-23, 11, 36,-37,-16,
             43, 66,-59,-20, 11, 32,-33,-18,
             45, 67,-59,-17, 11, 18,-30,-20,
             47, 62,-59,-14, 11,  7,-26,-22,
             49, 53,-59,-12, 12,  1,-24,-24,
             51, 40,-58,-12, 13,  2,-21,-22,
             52, 26,-57,-12, 14,  7,-19,-20,
             54, 17,-55,-14, 15, 13,-18,-16,
             55, 15,-54,-16, 17, 16,-16,-15,
             57, 13,-53,-23, 18, 19,-15, -9,
             58, 12,-51,-38, 21, 22,-13, -2,
             58, 12,-49,-51, 23, 24,-12, -1,
             59, 13,-47,-60, 26, 23,-11, -6,
             59, 17,-45,-66, 29, 20,-11,-15,
             59, 20,-43,-66, 32, 18,-10,-33,
             59, 23,-41,-65, 37, 17,-10,-35,
             58, 26,-38,-63, 41, 15,-11,-40,
             57, 29,-35,-61, 46, 14,-12,-44,
             58, 32,-33,-58, 47, 13,-13,-47,
             59, 34,-30,-57, 47, 12,-14,-48,
             61, 37,-27,-58, 43, 12,-15,-43,
             64, 40,-24,-59, 38, 11,-16,-38,
             65, 42,-21,-59, 34, 11,-17,-34,
             67, 44,-18,-59, 23, 11,-19,-31,
             64, 46,-15,-59, 10, 11,-21,-27,
             56, 48,-12,-59,  3, 12,-23,-24,
             45, 50,-12,-58,  1, 13,-23,-22,
             31, 52,-12,-57,  5, 14,-20,-19,
             18, 53,-14,-56, 13, 15,-17,-17,
             16, 55,-16,-55, 15, 17,-15,-16,
             14, 57,-18,-53, 17, 17,-13,-15,
             12, 57,-33,-52, 21, 20, -4,-14,
             12, 58,-47,-50, 23, 22,  0,-13,]


class CatWalkLeft(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             40, 59,-55,-51,  8, 24, -2, -9,
             41, 59,-63,-50,  8, 27, -8, -9,
             42, 59,-67,-49,  7, 30,-20, -8,
             42, 59,-66,-48,  7, 34,-33, -8,
             43, 59,-64,-48,  7, 38,-37, -8,
             44, 58,-62,-47,  7, 43,-41, -8,
             45, 57,-60,-46,  8, 47,-45, -8,
             46, 58,-57,-46,  8, 47,-48, -8,
             46, 60,-57,-45,  8, 45,-47, -8,
             47, 62,-58,-44,  8, 41,-42, -8,
             48, 65,-59,-43,  8, 36,-37, -8,
             48, 66,-59,-42,  8, 32,-33, -8,
             49, 67,-59,-41,  8, 18,-30, -8,
             50, 62,-59,-41,  9,  7,-26, -8,
             50, 53,-59,-40,  9,  1,-24, -9,
             51, 40,-58,-40,  9,  2,-21, -8,
             52, 26,-57,-40, 10,  7,-19, -7,
             52, 17,-55,-41, 10, 13,-18, -6,
             53, 15,-54,-41, 10, 16,-16, -6,
             53, 13,-53,-44, 11, 19,-15, -4,
             54, 12,-51,-48, 11, 22,-13, -4,
             54, 12,-49,-51, 12, 24,-12, -5,
             55, 13,-47,-54, 12, 23,-11, -6,
             55, 17,-45,-57, 13, 20,-11, -8,
             56, 20,-43,-60, 13, 18,-10,-11,
             56, 23,-41,-59, 14, 17,-10,-13,
             57, 26,-38,-59, 14, 15,-11,-13,
             57, 29,-35,-59, 15, 14,-12,-14,
             58, 32,-33,-58, 15, 13,-13,-15,
             58, 34,-30,-57, 14, 12,-14,-15,
             59, 37,-27,-57, 14, 12,-15,-15,
             59, 40,-24,-57, 12, 11,-16,-14,
             59, 42,-21,-56, 12, 11,-17,-14,
             59, 44,-18,-56, 10, 11,-19,-13,
             56, 46,-15,-56,  7, 11,-21,-12,
             53, 48,-12,-55,  6, 12,-23,-12,
             50, 50,-12,-54,  5, 13,-23,-12,
             46, 52,-12,-54,  4, 14,-20,-11,
             42, 53,-14,-53,  5, 15,-17,-11,
             41, 55,-16,-53,  6, 17,-15,-10,
             41, 57,-18,-52,  6, 17,-13,-10,
             40, 57,-33,-52,  7, 20, -4,-10,
             40, 58,-47,-51,  8, 22,  0, -9,]


class CatWalkRight(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [
             12, 55,-53,-49, 23, 12, -5,-12,
             15, 55,-55,-47, 22, 12, -7,-11,
             18, 56,-58,-45, 20, 13, -9,-11,
             21, 56,-60,-43, 18, 13,-11,-10,
             24, 57,-59,-40, 16, 14,-13,-10,
             27, 57,-59,-37, 15, 15,-14,-11,
             30, 58,-58,-35, 13, 15,-15,-12,
             32, 58,-58,-32, 13, 15,-15,-13,
             35, 58,-57,-29, 12, 14,-15,-14,
             38, 59,-57,-26, 12, 14,-15,-15,
             40, 59,-56,-23, 11, 12,-14,-16,
             43, 60,-56,-20, 11, 11,-13,-18,
             45, 58,-56,-17, 11,  9,-13,-20,
             47, 55,-55,-14, 11,  7,-13,-22,
             49, 52,-55,-12, 12,  5,-12,-24,
             51, 49,-54,-12, 13,  4,-11,-22,
             52, 45,-54,-12, 14,  4,-11,-20,
             54, 42,-53,-14, 15,  5,-10,-16,
             55, 41,-53,-16, 17,  6,-10,-15,
             57, 40,-52,-23, 18,  7,-10, -9,
             58, 40,-51,-38, 21,  7, -9, -2,
             58, 40,-51,-51, 23,  8, -9, -1,
             59, 40,-50,-60, 26,  8, -9, -6,
             59, 41,-49,-66, 29,  8, -9,-15,
             59, 42,-49,-66, 32,  7, -8,-33,
             59, 43,-48,-65, 37,  7, -8,-35,
             58, 44,-47,-63, 41,  7, -8,-40,
             57, 44,-47,-61, 46,  8, -8,-44,
             58, 45,-46,-58, 47,  8, -8,-47,
             59, 46,-45,-57, 47,  8, -8,-48,
             61, 47,-44,-58, 43,  8, -8,-43,
             64, 47,-43,-59, 38,  8, -8,-38,
             65, 48,-43,-59, 34,  8, -8,-34,
             67, 49,-42,-59, 23,  8, -8,-31,
             64, 49,-41,-59, 10,  8, -8,-27,
             56, 50,-40,-59,  3,  9, -9,-24,
             45, 51,-40,-58,  1,  9, -8,-22,
             31, 51,-40,-57,  5, 10, -7,-19,
             18, 52,-40,-56, 13, 10, -7,-17,
             16, 53,-41,-55, 15, 10, -6,-16,
             14, 53,-42,-53, 17, 11, -5,-15,
             12, 54,-46,-52, 21, 11, -4,-14,
             12, 54,-50,-50, 23, 11, -5,-13,]

class CatZero(CatGait):

    def __init__(self, current_time):
        super().__init__(current_time)
        self.angles = [0, 0, 0, 0, 0, 0, 0, 0,]
