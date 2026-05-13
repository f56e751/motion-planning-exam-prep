# Motion Planning Cheat Sheet

양면 A4 빽빽 layout의 마크다운 버전. PDF (`cheatsheet.pdf`)가 시험장 지참용. 본 md는 텍스트 검색/리뷰용.

각 섹션 우측 `[L##]`은 강의안 번호.

**강의안 인덱스**: L00 Path Planning 1 | L01 Shortest Path Search | L02 Probabilistic (PRM/RRT) | L03 LPA*/D* Lite | L04 Constraints (nonholonomic, Dubins) | L05 Trajgen & Controllability | L06 Polynomial & Bézier | L07 Collision Check | L08 CHOMP | L09 Drone Corridor | L10 iLQR/LQR | L11 Linear MPC | L12 NMPC/PMP

---

## 1. C-space & Completeness [L00, L02]

$\mathcal{C}_{\text{obs}} = \{q : A(q) \cap O \neq \emptyset\}$, $\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}$. Plan: continuous $\tau: [0, 1] \to \mathcal{C}_{\text{free}}$.

- **Complete**: returns path iff exists.
- **Resolution complete**: at chosen discretization.
- **Probabilistic complete**: $P(\text{fail}) \to 0$ as $n \to \infty$.

Minkowski sum: $\mathcal{C}_{\text{obs}} = O \oplus A$ for point robot.

## 2. Roadmaps [L00]

- **Visibility graph**: nodes = obstacle vertices + start/goal. Edge if segment is obstacle edge or doesn't cross. $O(n^3)$ naive, $O(n^2)$ optimal.
- **Voronoi diagram**: $\{q : |\text{near}(q)| > 1\}$ — maximally clears obstacles. Straight arc (edge-edge / vertex-vertex), parabolic (edge-vertex). Retraction $\rho: \mathcal{C}_{\text{free}} \to \text{Vor}$.
- **Cell decomposition**: trapezoidal (exact, complete), quadtree/octree (approximate). Build adjacency graph, search.

## 3. Potential Field [L00]

$U_{\text{att}} = \tfrac12 k_a \|x - x_g\|^2$, $F_{\text{att}} = -k_a (x - x_g)$.
$F_{\text{rep}} = k_r (\tfrac1\rho - \tfrac1{\rho_0}) \tfrac1{\rho^2} \nabla\rho$ if $\rho \le \rho_0$.
Issue: local minima. Fix: random walks (RPP), navigation function (no local min, $\infty$ at obstacles, smooth), wave-front (resolution complete, search from goal).

## 4. Graph Search [L00, L01]

BFS: $O(b^d)$ time/space, complete, optimal (unweighted). DFS: $O(h)$ space, not optimal.

- **Dijkstra**: $f(n) = g(n)$ (no heuristic).
- **A***: $f(n) = g(n) + h(n)$.
  - Optimal iff $h$ **admissible** ($h(n) \le h^*(n)$).
  - **Consistent**: $h(s) \le c(s, s') + h(s')$, $h(\text{goal}) = 0$.
  - Open list (priority queue), Closed list.
  - Optimally efficient among same-$h$ algorithms.
- **Best-first / Greedy**: $f = h$. **Beam search**: keep $k$ best per level. **RBFS**: DFS-like with $f$-value backup, $O(bd)$ memory.

## 5. D*, LPA*, D* Lite [L01, L03]

**LPA*** (incremental A*, fixed start):
$rhs(s) = \min_{s' \in \text{Pred}(s)} (g(s') + c(s', s))$, $rhs(s_{\text{start}}) = 0$.
**Locally consistent**: $g = rhs$. Inconsistent in priority queue.
Key $[k_1, k_2] = [\min(g, rhs) + h;\ \min(g, rhs)]$.
- $g > rhs$ (overconsistent): set $g = rhs$.
- $g < rhs$ (underconsistent): set $g = \infty$.

**D* Lite** (reverse, moving agent): Search goal → start. $rhs(s) = \min_{s' \in \text{Succ}(s)} (c(s, s') + g(s'))$, $rhs(s_{\text{goal}}) = 0$.
Key modifier $k_m \mathrel{+}= h(s_{\text{last}}, s_{\text{curr}})$ avoids re-sorting queue when robot moves.

## 6. PRM (multi-query) [L02]

**Construction**: sample $n$ collision-free $q$'s, connect each to $k$ nearest via local planner (e.g. straight line + incremental/subdivision collision check). **Query**: connect $q_i, q_g$ to roadmap, A*/Dijkstra.
Variants: OBPRM (sample near obstacle surfaces), Lazy PRM.
Distance: $d(q, q') = w_1 \|X - X'\| + w_2 f(R, R')$.

## 7. RRT & RRT* [L02]

**Build**: $q_{\text{rand}}$ → nearest $q_{\text{near}}$ → step toward → $q_{\text{new}}$. Add if collision-free. Goal bias 5–10%.
**Merge** (bidirectional): grow two trees, alternate.
Nodes with largest Voronoi region most likely expanded.

**RRT***: Within radius of $q_{\text{new}}$:
1. Choose parent minimizing total cost.
2. **Rewire**: if going through $q_{\text{new}}$ cheaper for $q_{\text{near}}$, change parent.

⇒ probabilistic completeness + **asymptotic optimality**.

## 8. Constraints & Nonholonomy [L04]

**Pfaffian**: $A(q)\dot q = 0$, $k$ constraints, $n - k$ DoF velocity.
**Holonomic** (integrable): can write as $h(q) = 0$ ⇒ lose dimension. **Nonholonomic**: not integrable; velocity constrained but reachability preserved.

**Simple car**: $\dot x = v\cos\theta$, $\dot y = v\sin\theta$, $\dot\theta = v\tan\phi/L$, Pfaffian $\dot x\sin\theta - \dot y\cos\theta = 0$.

Control-affine driftless: $\dot q = \sum g_i(q) u_i = G(q) u$.

**Lie bracket**: $[f, g] = \frac{\partial g}{\partial x} f - \frac{\partial f}{\partial x} g$. Generates new feasible directions over time.

**Frobenius**: $\mathcal{D} = \text{span}\{g_i\}$ involutive ($[g_i, g_j] \in \mathcal{D}$) ⟺ holonomic.

**Rank test**: if rank $[g_1 \dots g_m\ [g_i, g_j]] > m$ for some $(i, j)$ → nonholonomic.

## 9. Controllability [L04, L05]

**Pfaffian → control system**: $H(q)\dot q = 0$ ($k$ rows). Null space basis $\{g_1, \dots, g_{n-k}\}$ ⇒ $\dot q = G(q) u$, $m = n - k$ inputs. Basis choice 비유일 (다른 $u$ 의미).

**Controllable**: $\forall q_i, q_f \in \mathcal{C}$, $\exists u(t)$ steering $q_i \to q_f$. 제약이 도달성에 영향 X.

$\Delta_{\mathcal{A}} = \mathcal{L}(\Delta)$ = involutive closure / **Lie algebra** of $\Delta = \text{span}\{g_1, \dots, g_m\}$ = span of all nested Lie brackets:
$$\mathcal{L}(\Delta) = \text{span}\{g_i,\ [g_i, g_j],\ [g_i, [g_j, g_k]], \dots\}$$
Accessible dim $\nu = \dim \mathcal{L}(\Delta)$.

**왜 nested?**: 1단 bracket으론 부족할 수 있음.
- Unicycle (n=3, m=2): 1단 $[g_1, g_2]$로 충분.
- Chained-form (n=5, m=2): 1단 rank 3, 2단 4, 3단 5 — 3단까지 필요.
- Snakeboard: drift와 $[f, g_i]$, $[f, [f, g_i]]$ 등 더 깊이.

**Termination**: 새 bracket이 rank 못 늘리면 stop. $\dim\mathcal{L} \le n$에서 포화.

**Chow's theorem**: $\dim\mathcal{L}(\Delta)(x) = n\ \forall x$ ⟺ controllable (driftless).
Frobenius (1단)로 holonomic 판정 ↔ Chow (전체 nested)로 controllability 판정.

**3-way 분류** ($p$ = integrable combos, $\nu = n - p$):

| case | $\nu$ vs $n, m$ | $p$ | 성질 |
|---|---|---|---|
| 완전 nonhol | $\nu = n$ | $p = 0$ | controllable, STLC |
| 부분 적분 | $m < \nu < n$ | $0 < p < k$ | still nonhol |
| 완전 holonomic | $\nu = m$ | $p = k$ | integrable |

With drift: accessibility ⇏ controllability.
Degree of nonholonomy $\kappa \le n - m + 1$.

## 10. Dubins / Reeds-Shepp [L04]

**Dubins** (forward only, min radius $\rho$): optimal ∈ $\{LRL, RLR, LSL, LSR, RSL, RSR\}$, $\alpha, \gamma \in [0, 2\pi)$, $\beta \in (\pi, 2\pi)$, $d \ge 0$. CCC if $|q_g - q_i| < 4\rho$.

**Reeds-Shepp** (forward + reverse): 48 base words including $C|C|C$, $CC|C$, $CSC$ 등.

## 11. Chained Form [L05]

2-input, $n$-state: $\dot z_1 = u_1$, $\dot z_2 = u_2$, $\dot z_3 = z_2 u_1$, ..., $\dot z_n = z_{n-1} u_1$.

**Sufficient conditions**: $\Delta_0 = \text{span}\{g_1, g_2, \text{ad}_{g_1}g_2, \dots, \text{ad}_{g_1}^{n-2}g_2\}$ has dim $n$; $\Delta_1, \Delta_2$ involutive; $\exists h_1$ with $dh_1 \Delta_1 = 0$, $dh_1 g_1 = 1$; $\exists h_2$ indep of $h_1$ with $dh_2 \Delta_2 = 0$. Then $z_1 = h_1$, $z_k = L_{g_1}^{n-k} h_2$.

**Unicycle**: $z_1 = \theta$, $z_2 = x\cos\theta + y\sin\theta$, $z_3 = x\sin\theta - y\cos\theta$. $\omega = v_1$, $v = v_2 + z_3 v_1$.

**Bicycle (RWD)**: $z_1 = x$, $z_2 = \sec^3\theta \tan\phi / L$, $z_3 = \tan\theta$, $z_4 = y$.
$v = v_1/\cos\theta$, $\omega = -\tfrac{3}{L} v_1 \sec\theta \sin^2\phi + \tfrac{1}{L} v_2 \cos^3\theta \cos^2\phi$. Singular at $\cos\theta = 0$.

## 12. Differential Flatness [L05, L06, L09]

$\exists$ flat output $y$ s.t. $x = x(y, \dot y, \dots, y^{(r)})$, $u = u(y, \dot y, \dots, y^{(r)})$.
Driftless: **flatness ⟺ chained form transform**; flat outputs are $z_1, z_n$.

- **Unicycle / bicycle**: $(x, y)$ flat. $\theta = \arctan(\dot y/\dot x)$, $v = \pm\sqrt{\dot x^2 + \dot y^2}$, $\omega = (\ddot y \dot x - \ddot x \dot y)/(\dot x^2 + \dot y^2)$.
- **Quadrotor**: $(x, y, z, \psi)$ flat. $f = m\|\mathbf{a}\|$, $\mathbf{b}_3 = \mathbf{a}/\|\mathbf{a}\|$ with $\mathbf{a} = g\mathbf{e}_3 - \ddot{\mathbf{p}}$. Snap → moments.

## 13. Polynomial Trajectory [L06]

Cost $J = \int_0^T (P^{(r)}(t))^2 dt$.
Euler–Lagrange: $\sum_{k=0}^r (-1)^k \frac{d^k}{dt^k} \frac{\partial L}{\partial x^{(k)}} = 0$.
Optimal $P$: degree $2r - 1$, requires $2r$ BCs.

- $r = 1$ min vel: cubic
- $r = 2$ min accel: quintic
- $r = 3$ min jerk: septic
- $r = 4$ min snap: 9th order

QP form: $\min p^T Q p$ s.t. $A p = b$. With $T^{i+k-2r+1}/(i+k-2r+1)$ entries.

**Multi-segment**: enforce $C^0, \dots, C^{N-1}$ at waypoints, $[A_T^i\ -A_0^{i+1}][p^i; p^{i+1}] = 0$. Time alloc: $J_{\text{aug}} = J + \sum c_k T_k$.

**Quintic Hermite (5th order)**: $p(u) = 10u^3 - 15u^4 + 6u^5$ for unit BC.
**Min jerk (7th)**: $35u^4 - 84u^5 + 70u^6 - 20u^7$.

## 14. Bézier / Bernstein [L06, L09]

$B_{k,N}(t) = \binom{N}{k} t^k (1 - t)^{N-k}$, $r(t) = \sum b_k B_{k,N}$.

- **Endpoint interpolation**: $r(0) = b_0$, $r(1) = b_N$.
- **Convex hull**: $r(t) \in \text{conv}\{b_k\}$ — safety: constrain control points!
- **Hodograph**: $r'$ is Bézier degree $N - 1$, control points $b_k^{(1)} = N(b_{k+1} - b_k)$. Velocity/accel bounds = bounds on finite diffs of $b_k$.
- **Variation diminishing**: curve crosses line ≤ control polygon crosses.
- **Degree elevation**: $\tilde b_k = \tfrac{k}{N+1} b_{k-1} + (1 - \tfrac{k}{N+1}) b_k$.
- **de Casteljau**: $b_i^k = (1 - t) b_{i-1}^{k-1} + t b_i^{k-1}$. Evaluate + subdivide.

$C^0$: $P_3 = Q_0$. $C^1$: $Q_1 = 2 P_3 - P_2$. $C^2$: $Q_2 = 4 P_3 - 4 P_2 + P_1$.

## 15. Collision Checking [L07]

**Segment intersection** (2D): $\text{orient}(A, B, C) = (B_x - A_x)(C_y - A_y) - (B_y - A_y)(C_x - A_x)$. Two segments cross iff orient signs on opposite sides for both pairs.

**SAT** (convex polygons): for each edge normal $u$, compute projection intervals; if any separating axis → no collision.

**GJK**: $A \cap B \ne \emptyset \iff 0 \in A - B$. Build simplex via support: $\text{support}_{A-B}(d) = \text{support}_A(d) - \text{support}_B(-d)$. Iteratively add support points toward origin; if simplex contains 0 → collision.

**EPA** (after GJK collision): expand polytope to find closest boundary point. Penetration depth $d$ + normal $n$. Recover contact: $v_i = a_i - b_i$, $p_A = \sum \lambda_i a_i$, $p_B = \sum \lambda_i b_i$, $p_A = p_B + dn$.

**SDF** $\phi(x)$: signed distance (negative inside). Precompute on grid; query forward kinematics $x(q, u)$. Robot ≈ sphere: clearance $= \phi(x) - r$.

## 16. CHOMP [L08]

$\min U(\xi) = f_{\text{prior}}(\xi) + f_{\text{obs}}(\xi)$, $\xi = (q_1, \dots, q_n)$.

**Covariant update**: $\xi \leftarrow \xi - \tfrac1\lambda M^{-1} \nabla U$. $M$ = smoothness metric (PSD).

**Smoothness**: $f_{\text{prior}} = \tfrac12 \sum w_d \|K_d \xi + e_d\|^2$ where $K_d$ = $d$-th order finite difference. $= \tfrac12 \xi^T A \xi + \xi^T b + c$.

**Obstacle gradient** (workspace): $\tfrac{\delta J}{\delta x} = \|x'\| [(I - \hat x' \hat x'^T) \nabla c - c(x) \kappa]$.
Project to C-space: $\nabla_q f = J_{\text{robot}}^T \nabla_x f$.

Curvature: $\kappa = \tfrac{1}{\|x'\|^2} (I - \hat x' \hat x'^T) x''$.

Obstacle cost: $c(x) = \tfrac{1}{2\epsilon} (d(x) - \epsilon)^2$ for $0 \le d \le \epsilon$, $-d + \tfrac\epsilon2$ if $d < 0$, else 0.

**HMC inside CHOMP**: $p(\xi) \propto \exp(-\alpha U)$, momentum $\gamma$, Hamiltonian $H = U + \tfrac12 \gamma^T \gamma$. Escape local minima.

## 17. Safe Flight Corridor [L09]

Given path $\{p_i \to p_{i+1}\}$:
1. **Ellipsoid** around segment $L_i$: start sphere, find nearest obstacle $p^*$, shrink to touch, repeat.
2. **Polyhedron**: tangent hyperplanes at obstacle points. $a_j = 2 E^{-1} E^{-T} (p_j^c - d)$, $b_j = a_j^T p_j^c$. $C_i = \bigcap H_j$.
3. Optimize Bézier with control points in $C_i$ (convex hull → safety).

**FMM** (Fast Marching): $|\nabla T| = 1/f(x)$ (Eikonal). $f(d)$ from SDF: low near obstacles. $O(N \log N)$. Path: gradient descent on $T$.

**Cube corridor**: axis-aligned inflated cubes — linear constraints $A_i p \le b_i$.

## 18. LQR & Riccati [L10]

$x_{k+1} = A_k x_k + B_k u_k$, $J = \tfrac12 \sum x^T Q x + u^T R u + \tfrac12 x_N^T Q_N x_N$.
$V_k(x) = \tfrac12 x^T P_k x$.

$$K_k = (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$$
$$P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$$

$P_N = Q_N$. Time-varying linear feedback $u_k^* = -K_k x_k$.

## 19. iLQR / DDP [L10, L12]

Nominal $\{\bar x_k, \bar u_k\}$. Linearize $\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k$.

**Q-function**:
$Q_x = \ell_x + A^T V_x$, $Q_u = \ell_u + B^T V_x$
$Q_{xx} = \ell_{xx} + A^T V_{xx} A$, $Q_{uu} = \ell_{uu} + B^T V_{xx} B$, $Q_{ux} = \ell_{ux} + B^T V_{xx} A$.

**Local policy**: $\delta u^* = k + K \delta x$, $k = -Q_{uu}^{-1} Q_u$, $K = -Q_{uu}^{-1} Q_{ux}$.

**Value update**: $V_x = Q_x - Q_{ux}^T Q_{uu}^{-1} Q_u$, $V_{xx} = Q_{xx} - Q_{ux}^T Q_{uu}^{-1} Q_{ux}$.

**Forward rollout**: $u_k^{\text{new}} = \bar u_k + \alpha k_k + K_k (x_k^{\text{new}} - \bar x_k)$, line search $\alpha \in (0, 1]$.

**Regularization**: $Q_{uu}^{\text{reg}} = Q_{uu} + \lambda I$.

**DDP**: includes $f_{xx}, f_{ux}, f_{uu}$ — extra terms $V_{x, k+1} \cdot F_{**}$ in $Q_{**}$. iLQR drops them (Gauss-Newton).

## 20. Linear MPC [L11]

$\min \sum_{i=0}^{N-1} (x^T Q x + u^T R u) + x_N^T P x_N$ s.t. dyn + $x \in \mathcal{X}$, $u \in \mathcal{U}$, $x_N \in \mathcal{X}_f$. Apply only $u_0^*$, shift, repeat.

**Condensed QP**: $X = \Phi x_t + \Gamma U$, $J = U^T H U + 2 x^T F^T U + \text{const}$, $H = \Gamma^T \bar Q \Gamma + \bar R$. Small variable, dense.

**Sparse QP**: $Z = [x_0; u_0; \dots; x_N]$, block-banded $A_{eq}$. Larger but exploits sparsity.

**KKT**: $\begin{pmatrix} H & A^T \\ A & 0 \end{pmatrix} \begin{pmatrix} Z \\ \lambda \end{pmatrix} = \begin{pmatrix} -q \\ b \end{pmatrix}$.

## 21. MPC Stability [L11]

**Terminal cost** $V_f$ approximates $V_\infty$. Choose $K = K_{\text{LQR}}$, $A_K = A + BK$, solve DARE: $P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q$.

**Terminal set** $\mathcal{X}_f$: invariant under $K$, $K \mathcal{X}_f \subseteq \mathcal{U}$, $\mathcal{X}_f \subseteq \mathcal{X}$, $V_f(A_K x) - V_f(x) \le -\ell(x, Kx)$.

**Recursive feasibility**: shift optimal sequence + append $K x_N^*$ remains feasible.

**Lyapunov decrease**: $V_N(x_{t+1}) - V_N(x_t) \le -\ell(x_t, \kappa_N(x_t))$. Sum to $\infty$ → $x_t \to 0$.

**Pitfall**: terminal cost alone insufficient; need both cost+set.

Soft constraint: $C x \le d + s$, $s \ge 0$, penalty $\rho_1 \mathbf{1}^T s + \rho_2 s^T s$.

## 22. NMPC & PMP [L12]

**Lagrangian**: $\mathcal{L} = \sum \ell + \ell_f + \sum \lambda_{k+1}^T (F(x_k, u_k) - x_{k+1})$.

**Costate** (backward): $\lambda_N = \ell_{f, x}$, $\lambda_k = \ell_x + F_x^T \lambda_{k+1}$.

**Optimality**: $\ell_u + F_u^T \lambda_{k+1} = 0$ ⇒ $u_k^* = \arg\min_u [\ell + \lambda_{k+1}^T F]$.

**SQP**: linearize + quadratic cost → QP for $(\delta x, \delta u)$, $\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k$, update $\bar u_k \leftarrow \bar u_k + \alpha \delta u_k$.

**Continuous PMP**: $H(x, u, \lambda) = \ell + \lambda^T f$, $\dot x = \partial_\lambda H$, $\dot \lambda = -\partial_x H$, $u^* = \arg\min_u H$, $\lambda(T) = \partial_x \ell_f$. Two-point BVP (shooting).

**Discrete ↔ PMP**: $\lambda_k$ discrete = $\lambda(t) \Delta t$ continuous, costate recursion matches $\dot \lambda = -\partial_x H$ as $\Delta t \to 0$.

## 23. Stability (discrete) [L11]

$V(x) > 0$, $V(0) = 0$, $\Delta V = V(f(x)) - V(x)$.

- Lyapunov: $\Delta V \le 0$
- Asymptotic: $\Delta V \le -\ell(x)$, $\ell > 0$
- Exponential: $\Delta V \le -\alpha V$

LTI $x_{k+1} = A x_k$, $\rho(A) < 1$: $V = x^T P x$ where $A^T P A - P = -Q$, $Q > 0$.

## 24. Method Comparison [전체]

| Method | Use case |
|---|---|
| A* | static, known map |
| D* Lite | moving agent, changing map |
| PRM | multi-query, high-D |
| RRT | single-query, fast |
| RRT* | asymptotic optimal |
| CHOMP | local refinement, smooth |
| SFC+QP | drone, real-time, convex |
| iLQR | nonlinear, fast, smooth cost |
| SQP | hard constraints |
| DDP | 2nd-order, more accurate |
| MPC | online + constraints |

**Key formulas to remember**:
- Pfaffian for car: $\sin\theta \dot x - \cos\theta \dot y = 0$
- Curvature: $\kappa = (\dot x \ddot y - \dot y \ddot x)/(\dot x^2 + \dot y^2)^{3/2}$
- Min-jerk BC: pos+vel+accel+jerk = 0 each end
- DARE: $P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q$
- iLQR gain: $K = -Q_{uu}^{-1} Q_{ux}$, $k = -Q_{uu}^{-1} Q_u$
- Bézier hodograph: $b_k^{(1)} = N(b_{k+1} - b_k)$
- Hamiltonian: $H = \ell + \lambda^T f$, $\dot \lambda = -H_x$
