def integrand(x):
    return (1 / sqrt(2 * pi * self.sigmaHit ** 2)) * exp(-0.5 * ((x - zt_star) ** 2) / (self.sigmaHit ** 2))


