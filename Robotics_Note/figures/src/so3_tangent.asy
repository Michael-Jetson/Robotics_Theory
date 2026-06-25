// so3_tangent.asy -- wireframe sphere (manifold) + tangent plane (Lie algebra).
// Visualizes a curved manifold M and its tangent space T_pM at a point p.
// Pedagogical cartoon for SO(3) / so(3): SO(3) is a 3-manifold, NOT a 2-sphere;
// the sphere is the standard stand-in (cf. Sola, "A micro Lie theory ...").
// Build:  asy -f pdf -o so3_tangent so3_tangent.asy
import three;
import graph3;

usepackage("amsmath");             // \mathbf, etc.
usepackage("amssymb");             // \mathfrak (so(3) label)

size(9cm);
currentprojection = perspective(camera=(4,2,3), up=Z);
currentlight = nolight;            // wireframe look: no shading

real R = 1;

// ---- manifold M: wireframe sphere, latitude + longitude circles only ----
int nlat = 9, nlong = 12;
pen spen = gray(0.45) + 0.4bp;
for (int i = 1; i < nlat; ++i) {                 // parallels (latitude)
  real th = pi*i/nlat;
  draw(circle(c=(0,0,R*cos(th)), r=R*sin(th), normal=Z), spen);
}
for (int j = 0; j < nlong; ++j) {                // meridians (longitude)
  real phi = 2*pi*j/nlong;
  draw(circle(c=O, r=R, normal=(sin(phi), -cos(phi), 0)), spen);
}

// ---- tangent point p (kept off the poles) ----
real th = 0.9, ph = 0.6;
triple p = R*(sin(th)*cos(ph), sin(th)*sin(ph), cos(th));
triple n = unit(p);                              // plane normal = radius
triple u = unit(cross(n, Z));                    // orthonormal basis of T_pM
triple v = cross(n, u);

// ---- tangent plane T_pM = so(3): faint fill + grid ----
real s = 0.95;
path3 quad = p+s*u+s*v -- p-s*u+s*v -- p-s*u-s*v -- p+s*u-s*v -- cycle;
draw(surface(quad), gray(0.85)+opacity(0.30));
draw(quad, black+0.6bp);
int ng = 6;
pen gpen = blue+0.35bp;
for (int i = 0; i <= ng; ++i) {
  real t = -s + 2*s*i/ng;
  draw((p+t*u-s*v) -- (p+t*u+s*v), gpen);
  draw((p-s*u+t*v) -- (p+s*u+t*v), gpen);
}

// ---- a tangent vector xi^ in so(3), and the base point ----
triple xi = 0.6*u + 0.3*v;
draw(p -- p+xi, red+1bp, Arrow3());
dot(p, red);

// ---- labels (LaTeX math) ----
label("$\mathcal{M}=SO(3)$", (0,0,-1.45*R));
label("$T_p\mathcal{M}=\mathfrak{so}(3)$", p+s*u+s*v, NE);
label("$p$", p, SW);
label("$\xi^{\wedge}$", p+xi, N);
