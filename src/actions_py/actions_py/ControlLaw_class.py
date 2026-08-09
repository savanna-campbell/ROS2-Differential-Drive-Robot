from dataclasses import dataclass
import math


@dataclass
class Parameters:
    r_R: float # radius of right wheel
    r_L: float # radius of left wheel
    l: float # distance from wheel to center of mass
    kg_R: float # gain of right wheel
    kg_L: float # gain of left wheel
    ka: float # effective damping coefficient
    kf: float # effective friction coefficient
    kq: float  # effective drag coefficient
    alpha: float # alpha term in tanh
    beta: float # beta term in tanh

@dataclass
class Control:
    u_R: float # normalized right PWM command
    u_L: float # normalized left PWM command
    #rho1: float
    #rho2: float

@dataclass
class State:
    x: float
    y: float
    theta: float
    omega_R: float # angular velocity of right wheel
    omega_L: float # angular velocity of left wheel
   # v: float # velocity
   # omega: float # angular velocity (theta dot)

class ControlLaw:
    def __init__(self, fwd_speed, turn_rad, params):
        self.prevTheta = None
        self.fwd_speed = fwd_speed
        self.turn_radius = turn_rad
        self.parameters = params
    
    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # takes target speeds and decides PWM speeds
    def find_speeds(self, right_speed, left_speed):
        damping_R = self.parameters.ka * right_speed
        damping_L = self.parameters.ka * left_speed

        friction_R = self.parameters.kf * math.tanh(self.parameters.alpha * right_speed)
        friction_L = self.parameters.kf * math.tanh(self.parameters.alpha * left_speed)

        drag_R = self.parameters.kq * math.tanh(self.parameters.beta * right_speed) * (right_speed) * right_speed
        drag_L = self.parameters.kq * math.tanh(self.parameters.beta * left_speed) * left_speed * left_speed
        
        u_R = (damping_R + friction_R + drag_R) / self.parameters.kg_R
        u_L = (damping_L + friction_L + drag_L) / self.parameters.kg_L
        
        return Control(u_R, u_L)


    def straight_line(self, state):
        angular_speed_R = self.fwd_speed / self.parameters.r_R
        angular_speed_L = self.fwd_speed / self.parameters.r_L
        return self.find_speeds(angular_speed_R, angular_speed_L)
        
    def circle(self, state):
        right_speed = self.fwd_speed / self.parameters.r_R * (1 + self.parameters.l / self.turn_radius)
        left_speed = self.fwd_speed / self.parameters.r_L * (1 - self.parameters.l / self.turn_radius)
        
        return self.find_speeds(right_speed, left_speed)

    def figure_8(self, state):

        if self.prevTheta == None:
            self.prevTheta = state.theta
            self.accumulated_rotation = 0.0
        
        delta = self.wrap_angle(state.theta - self.prevTheta)
        self.accumulated_rotation += delta
        self.prevTheta = state.theta
        
        print(f"theta={state.theta:.3f}  delta={delta:.4f}  accumulated={self.accumulated_rotation:.4f}")


        if abs(self.accumulated_rotation) < 2*math.pi:
            return self.circle(state)
        
        else:
            self.accumulated_rotation = 0.0
            self.turn_radius = -self.turn_radius
            return self.circle(state)
            
  