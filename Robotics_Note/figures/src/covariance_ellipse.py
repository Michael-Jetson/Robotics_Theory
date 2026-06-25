# covariance_ellipse.py -- Gaussian uncertainty as covariance ellipses.
# Draws 1-sigma / 2-sigma / 3-sigma confidence ellipses of a 2D Gaussian from
# its covariance matrix (eigen-decomposition method). Deterministic (seeded)
# so the figure is reproducible.
# Run:  python3 covariance_ellipse.py   ->  covariance_ellipse.pdf (vector)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# ---- a 2x2 covariance matrix and its mean ----
mu = np.array([0.0, 0.0])
Sigma = np.array([[2.0, 0.9],
                  [0.9, 0.6]])

# eigen-decomposition: eigenvectors = axis directions, eigenvalues = variances
vals, vecs = np.linalg.eigh(Sigma)            # ascending order
order = vals.argsort()[::-1]                   # largest first
vals, vecs = vals[order], vecs[:, order]
angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))   # major-axis angle (deg)

fig, ax = plt.subplots(figsize=(4.2, 4.0))
colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
for k, c in zip([1, 2, 3], colors):
    # full axis length = 2 * k * sqrt(eigenvalue)
    width, height = 2 * k * np.sqrt(vals)
    ax.add_patch(Ellipse(mu, width, height, angle=angle,
                         fill=False, edgecolor=c, lw=1.6,
                         label=r"$%d\sigma$" % k))

# sample points from the Gaussian, for intuition
rng = np.random.default_rng(0)
pts = rng.multivariate_normal(mu, Sigma, size=400)
ax.scatter(pts[:, 0], pts[:, 1], s=6, c="0.6", alpha=0.5, zorder=0)
ax.plot(mu[0], mu[1], "k+", ms=10, mew=1.5)

ax.set_aspect("equal")
ax.set_xlabel(r"$x$")
ax.set_ylabel(r"$y$")
ax.set_title(r"Covariance ellipses ($k\sigma$) of a 2D Gaussian")
ax.legend(loc="upper left", frameon=False)
ax.grid(True, ls=":", lw=0.5, alpha=0.6)
fig.tight_layout()
fig.savefig("covariance_ellipse.pdf", bbox_inches="tight")
print("wrote covariance_ellipse.pdf")
