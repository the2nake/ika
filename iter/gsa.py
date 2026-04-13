"""Implements GSA (Gravitational Search Algorithm)"""

import numpy as np
import math


class Mass:
    def __init__(self, x: np.ndarray, m: float):
        self.m = m
        self.x = x
        self.v = np.zeros_like(x)
        self.f = np.zeros_like(x)

    def move(self, dt) -> None:
        self.v = self.v + dt * (self.f / self.m)
        self.x = self.x + dt * self.v
        self.f = np.zeros_like(self.x)

    def force_to(self, towards: Mass) -> np.ndarray:
        magnitude = self.m * towards.m / \
            math.hypot(*(self.x.T - towards.x.T)[0])
        return magnitude * (towards.x - self.x) / np.linalg.norm(towards.x - self.x)


def gsa(score_fn):
    # generate sample points
    points = []
    for modulus in range(20, 30):
        for argument in np.arange(0, math.pi/2, 0.1):
            points.append(Mass(np.array([[modulus],
                                         [argument]]),
                               0.0))

    max = 0
    best: Mass = points[0]
    for i, _ in enumerate(points):
        points[i].m = score_fn(points[i])
        if points[i].m > max:
            max = points[i].m
            best = points[i]
    print(f"max: {max}")
    print(best.x)

    for iter in range(1000):
        
        print(f"iter: {iter}")
        # sum = np.array([[0.0],
        #                 [0.0]])
        # for i, _ in enumerate(points):
        #     sum += points[i].x
        # print(sum / len(points))
        for i, _ in enumerate(points):
            for j in range(i + 1, len(points)):
                force_vector = points[i].force_to(points[j])
                points[i].f += force_vector
                points[j].f -= force_vector

        for i, _ in enumerate(points):
            points[i].move(0.001)
            
        max = 0
        best: Mass = points[0]
        for i, _ in enumerate(points):
            s= score_fn(points[i])
            if s > max:
                max = s
                best = points[i]
        print(f"max: {max}")
        print(best.x)

    max = 0
    best: Mass = points[0]
    for i, _ in enumerate(points):
        points[i].m = score_fn(points[i])
        if points[i].m > max:
            max = points[i].m
            best = points[i]
    print(f"max: {max}")
    print(best.x)


def score_mass(mass: Mass):  # 1st is modulus, second is argument
    dx = 10 - mass.x[0, 0] * math.cos(mass.x[1, 0])
    dy = 20 - mass.x[0, 0] * math.sin(mass.x[1, 0])
    return 1 / (dx * dx + dy * dy)


if __name__ == "__main__":
    gsa(score_mass)
    print(f"theoretical best (mod): {math.hypot(10, 20)}")
    print(f"theoretical best (arg): {math.atan2(20, 10)}")
