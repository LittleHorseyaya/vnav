# LAB5 individual exercises

The image plane is $z=1$, with normalized coordinates $(u,v)$, camera
center at the origin, and focal length one.

## Deliverable 1: projection of a sphere

Let the sphere have center $C=(0,0,d)$, radius $r$, and $d>r+1$. A ray
through pixel $(u,v)$ has direction $w=(u,v,1)$. It is tangent to the
sphere exactly when the distance from $C$ to the ray is $r$:

$$
\frac{\|C\times w\|^2}{\|w\|^2}
=\frac{d^2(u^2+v^2)}{u^2+v^2+1}=r^2.
$$

Therefore the silhouette boundary is

$$
u^2+v^2=\frac{r^2}{d^2-r^2}.
$$

It is a circle centered at the principal point with image radius
$r/\sqrt{d^2-r^2}$. The projected solid sphere fills its interior.

For a center $C=(a,b,d)$ that remains entirely in front of the camera
($d>r$), the tangent-ray equation is

$$
(au+bv+d)^2=(a^2+b^2+d^2-r^2)(u^2+v^2+1).
$$

Let $\rho=\sqrt{a^2+b^2}$, $x=(au+bv)/\rho$, and
$y=(-bu+av)/\rho$. For $\rho>0$, completing the square gives

$$
\frac{\left(x-\frac{\rho d}{d^2-r^2}\right)^2}
     {\frac{r^2(\rho^2+d^2-r^2)}{(d^2-r^2)^2}}
+\frac{y^2}{\frac{r^2}{d^2-r^2}}=1.
$$

Thus an off-axis sphere projects to an ellipse. When $a=b=0$, the
semiaxes become equal, giving the circle above.

## Deliverable 2: vanishing points

Two parallel 3D lines have a common nonzero direction
$q=(q_x,q_y,q_z)$ and points $P_i(\lambda)=P_{i0}+\lambda q$.
Their image coordinates are

$$
(u_i(\lambda),v_i(\lambda))=
\left(\frac{X_{i0}+\lambda q_x}{Z_{i0}+\lambda q_z},
      \frac{Y_{i0}+\lambda q_y}{Z_{i0}+\lambda q_z}\right).
$$

If $q_z\ne0$, taking $\lambda\to\infty$ gives the same finite vanishing
point for both lines:

$$
(u_\infty,v_\infty)=
\left(\frac{q_x}{q_z},\frac{q_y}{q_z}\right).
$$

Its homogeneous image coordinates are $[q_x:q_y:q_z]$.

If $q_z=0$, each projected line instead has the form

$$
(u_i(\lambda),v_i(\lambda))=
\left(\frac{X_{i0}}{Z_{i0}},\frac{Y_{i0}}{Z_{i0}}\right)
+\lambda\left(\frac{q_x}{Z_{i0}},\frac{q_y}{Z_{i0}}\right).
$$

Both image-line directions are proportional to $(q_x,q_y)$, so the
projected lines remain parallel (or coincide in a degenerate view).
Their shared vanishing point is at infinity, $[q_x:q_y:0]$. Hence the
condition is that the 3D direction be parallel to the image plane:
$q_z=0$.
