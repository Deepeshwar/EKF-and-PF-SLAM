# The full Kalman filter, consisting of prediction and correction step.
# slam_07_f_kalman_filter
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_d_library import get_observations, write_cylinders


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    @staticmethod
    def g(state, control, w):
        
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta
        

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        R, alpha = 0.0, 0.0
        
        if r != l:
            alpha = (r - l)/w
            R = l/alpha
            theta_ = theta + alpha
            Rw2 = (R+(w/2))
            m = array([[1, 0, (Rw2 * (cos(theta_) - cos(theta)))], [0, 1, (Rw2 * (sin(theta_) - sin(theta)))], [0, 0, 1]])  # Replace this.

        else:

            m = array([[1, 0, (-l * sin(theta))], [0, 1, l * cos(theta)], [0, 0, 1]])  # Replace this.

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        alpha, theta_ = 0.0, 0.0
        g1_l, g1_r, g2_l, g2_r, g3_l, g3_r = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        if r != l:
            alpha = (r - l)/w
            theta_ = theta + alpha
            g1_l = (((w*r)/((r-l)**2)) * (sin(theta_) - sin(theta))) - (((r+l)/(2*(r-l))) * cos(theta_))
            g1_r = ((((-w)*l)/((r-l)**2)) * (sin(theta_) - sin(theta))) + (((r+l)/(2*(r-l))) * cos(theta_))
            g2_l = (((w*r)/((r-l)**2)) * (-cos(theta_) + cos(theta))) - (((r+l)/(2*(r-l))) * sin(theta_))
            g2_r = ((((-w)*l)/((r-l)**2)) * (-cos(theta_) + cos(theta))) + (((r+l)/(2*(r-l))) * sin(theta_))
            g3_l = (-1)/w
            g3_r = 1/w
            
            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])            
            
        else:
            g1_l = (cos(theta) + ((l/w)*sin(theta)))/2
            g1_r = (((-l/w)*sin(theta)) + cos(theta))/2
            g2_l = (sin(theta) - ((l/w)*cos(theta)))/2
            g2_r = ((l/w)*cos(theta) + sin(theta))/2
            g3_l = (-1)/w
            g3_r = 1/w
            
            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):

        left, right = control
        
        control_covariance = diag([(((self.control_motion_factor * left)**2) + ((self.control_turn_factor * (left - right))**2)), (((self.control_motion_factor * right)**2) + ((self.control_turn_factor * (left - right))**2))])
        
        G = []
        V = []
        
        G = ExtendedKalmanFilter.dg_dstate(self.state, control, self.robot_width)
        V = ExtendedKalmanFilter.dg_dcontrol(self.state, control, self.robot_width)
        
        self.covariance = (dot(dot(G, self.covariance), G.T)) + (dot(dot(V, control_covariance), V.T))
        
        self.state = ExtendedKalmanFilter.g(self.state, control, self.robot_width)


    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        
        r_x = (-dx)/r
        r_y = (-dy)/r
        r_theta = (scanner_displacement/r) * ((dx * sin(state[2])) - (dy * cos(state[2])))
        
        alpha_x = dy/(r**2)
        alpha_y = (-dx)/(r**2)
        alpha_theta = ((-scanner_displacement/(r**2)) * (dx * cos(state[2]) + dy * sin(state[2]))) - 1
        
        return array([[r_x, r_y, r_theta], [alpha_x, alpha_y, alpha_theta]]) 


    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""

        # --->>> Put your new code here.
        #
        # You will have to compute:
        # H, using dh_dstate(...).
        H = ExtendedKalmanFilter.dh_dstate(self.state, landmark, self.scanner_displacement)
        
        Q = diag([(self.measurement_distance_stddev ** 2), (self.measurement_angle_stddev ** 2)])
        
        K = dot(dot(self.covariance, H.T), linalg.inv(((dot(dot(H, self.covariance), H.T)) + Q)))
        
        
        
        innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi  #Make sure final range within -pi to pi
        
        # Then, you'll have to compute the new self.state.
        self.state += dot(K, innovation)
        self.covariance = dot((eye(3) - dot(K, H)), self.covariance)

        
if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in xrange(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction_and_correction.txt", "w")
    for i in xrange(len(states)):
        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0])
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))
        # Also, write matched cylinders.
        write_cylinders(f, "W C", matched_ref_cylinders[i])        

    f.close()
