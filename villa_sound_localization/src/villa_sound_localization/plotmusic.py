import numpy
import pylab

class PlotMusic(object):

    GLOBAL_MIN_POWER = 20.


    def __init__(self):
        pylab.ion()
        pylab.hold(False)

        # plot width in samples
        self.width = 51
        self.zrange = [-1, 1]

        self.buffer = None

        self.count = 0


    def calculate(self, spec):
        # Record max powers
        if self.count == 0:
            self.max_powers = []
        elif self.count % 100 == 0:
            self.max_powers.sort(reverse=True)
            print(self.max_powers[:10])
            self.max_powers = []
        self.count += 1

        # Check if spectrum is valid or not
        data = numpy.array(spec.data)
        if data.sum() == 0:
            return

        # Update buffer
        if self.buffer is None:
            self.buffer = data[:, numpy.newaxis]
        else:
            self.buffer = numpy.c_[self.buffer, data][:, -self.width:]

        #  update vmax/vmin 
        self.zrange[0] = max(self.zrange[0], self.buffer.max())
        self.zrange[1] = min(PlotMusic.GLOBAL_MIN_POWER, self.buffer.min())
        #self.zrange[1] = min(self.zrange[1], self.buffer.min())

        self.max_powers.append(data.max())

        # plot spectrum
        pylab.clf()
        pylab.imshow(numpy.flipud(self.buffer), aspect="auto",
                     interpolation="nearest",
                     vmax=self.zrange[0],
                     vmin=self.zrange[1])
        pylab.colorbar()
        pylab.draw()

