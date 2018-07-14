# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
from pylab import plot, show
from math import *
from distribution import *

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    sum = 0.0
    # --->>> Put your code here.
    c = (a * b)
    c = ((a.sum() + b.sum())/2.0) * (c.astype(float) / c.astype(float).sum())
    a = c
    #for i in range(len(a.values)):
        #a.values[i] *= b.values[i+90]
        #sum += a.values[i]
        
    #for j in range(len(a.values)):
        #a.values[i] /= sum
    
        #a.append((a.values[i]))
    return a  # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', linestyle='steps')
    print(position)
    #print(len(position.values))

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 410
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', linestyle='steps')
    
    print(measurement)
    #print(len(measurement.values))
    
    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', linestyle='steps')

    show()
