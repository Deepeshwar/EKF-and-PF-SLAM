# Comparison of the Kalman filter and the histogram filter.
# 06_f_kalman_vs_histogram_filter
# Claus Brenner, 29 NOV 2012
from distribution import *
from math import sqrt
from matplotlib.mlab import normpdf
from pylab import plot, show, ylim

# Import the helper functions from a previous file.
# If you implemented these functions in another file, put the filename here.
#
# Helpers.
#
class Density:
    def __init__(self, mu, sigma2):
        self.mu = float(mu)
        self.sigma2 = float(sigma2)


def kalman_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot([normpdf(x, prediction.mu, sqrt(prediction.sigma2))
          for x in range(*arena)], color = 'b', linewidth=2)
    plot([normpdf(x, measurement.mu, sqrt(measurement.sigma2))
          for x in range(*arena)], color = 'g', linewidth=2)
    plot([normpdf(x, correction.mu, sqrt(correction.sigma2))
          for x in range(*arena)], color = 'r', linewidth=2)

#
# Kalman filter step.
#
def kalman_filter_step(belief, control, measurement):
    """Bayes filter step implementation: Kalman filter."""

    # --->>> Put your code here.
    K = 0.0
    # Prediction.
    prediction = Density((belief.mu + control.mu), (belief.sigma2 + control.sigma2))
    #prediction = Density(belief.mu + 10.0, belief.sigma2 + 100.0)  # Replace
    
    # Correction.
    K = (prediction.sigma2)/(prediction.sigma2 + measurement.sigma2)
    
    correction = Density((prediction.mu + K * (measurement.mu - prediction.mu)), ((1- K) * prediction.sigma2))
       
    
    return (prediction, correction)

#
# Main
#
if __name__ == '__main__':
    arena = (0,200)
    
    # Start position. Well known, so the distribution is narrow.
    position_ = Density(10, 1)  # Kalman

    # Controls and measurements.
    controls_ = [ Density(40, 10**2), Density(70, 10**2) ]  # Kalman
    measurements_ = [ Density(60, 10**2), Density(140, 20**2) ]  # Kalman

    # This is the filter loop.
    for i in xrange(len(controls_)):
        # Kalman
        (prediction_, position_) = kalman_filter_step(position_, controls_[i], measurements_[i])
        kalman_plot(prediction_, measurements_[i], position_)

    ylim(0.0, 0.06)
    show()
