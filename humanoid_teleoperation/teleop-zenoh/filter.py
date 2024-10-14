"""
Author: WANG Wenhao
Date: 2024-06-29
Version: 1.0.0
copyright (c) 2024 All Rights Reserved
"""

import math
import numpy as np


def smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)


def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev


class OneEuroFilter:
    def __init__(self, t0, x0, dx0=None, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        """Initialize the one euro filter for a 14-dimensional numpy array."""
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)

        # Ensure x0 is a numpy array
        self.x_prev = np.array(x0, dtype=float)

        # If dx0 is not provided, initialize it as zeros with the same shape as x0
        if dx0 is None:
            self.dx_prev = np.zeros_like(self.x_prev)
        else:
            self.dx_prev = np.array(dx0, dtype=float)

        self.t_prev = float(t0)

    def __call__(self, t, x):
        """Compute the filtered signal for a 14-dimensional numpy array."""
        t_e = t - self.t_prev

        # The filtered derivative of the signal
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = exponential_smoothing(a, x, self.x_prev)

        # Memorize the previous values
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat


class AverageFilter:
    def __init__(self, window_size, dimensions):
        self.window_size = window_size
        self.dimensions = dimensions
        self.values = np.zeros((0, dimensions))
        self.current_sum = np.zeros(dimensions)

    def add_value(self, value):
        if len(value) != self.dimensions:
            raise ValueError(
                "Input value must have the same number of dimensions as specified"
            )

        # Append the new value to the values array
        self.values = np.vstack((self.values, value))
        self.current_sum += value

        # If the length exceeds the window size, subtract the oldest value and remove it
        if self.values.shape[0] > self.window_size:
            self.current_sum -= self.values[0]
            self.values = self.values[1:]

    def get_average(self):
        if self.values.shape[0] == 0:
            return np.zeros(self.dimensions)
        return self.current_sum / self.values.shape[0]
