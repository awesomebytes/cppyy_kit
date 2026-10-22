import math

def angle(x):
    return math.atan2(math.sin(x), math.cos(x))

class ForwardCameraPolicy:
    def configure(self, options):
        self.cone = math.pi / 6  # 30 degrees

    def prepare(self, yaw, path_heading, goal_distance):
        self.yaw, self.heading = yaw, path_heading
        self.near_goal = goal_distance <= 0.25

    def score(self, vx, vy, wz, final_heading):
        moving = math.hypot(vx, vy) > 0.0001
        if moving and abs(math.atan2(vy, vx)) > self.cone:
            return None
        if self.near_goal:
            return 0.0
        if moving and abs(angle(self.heading - self.yaw)) > self.cone:
            return None
        return 80.0 * angle(self.heading - final_heading)**2
