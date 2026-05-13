# Motion Planning 기말 예상 문제집

각 강의안(L00–L12)별 출제 가능성 높은 문제 2개 (메인 + 보조), 총 26문제. 풀이는 문제 바로 아래.

조건: cheat sheet 지참 가능, 계산기 사용 불가.

---

# Part I — Basic Planning (L00–L03)

## L00. Path Planning Basics

### [메인] Visibility graph vs Voronoi diagram ★★★

2D 다각형 환경에 5개의 직사각형 장애물이 있고 시작/목표 점이 주어졌다.

**(a)** Visibility graph의 노드 수와 (최악의 경우) 엣지 수를 식으로 나타내라.

**(b)** 같은 환경에서 Voronoi diagram이 visibility graph에 비해 어떤 경로 품질 측면에서 유리한가? 그리고 그 단점은?

**풀이.**
**(a)** 직사각형 1개에 꼭짓점 4개 → 5개 장애물에 $5\times 4 = 20$개. 시작/목표 추가 → **노드 $n = 22$개**. 모든 노드쌍 $\binom{n}{2}$ 중 일부만 연결 → 최악 **$O(n^2)$ 엣지** (대략 $\binom{22}{2}=231$의 일부).

**(b)** Voronoi diagram은 정의상 "가장 가까운 두 장애물 경계로부터 등거리"인 점들의 집합이므로 **clearance(장애물과의 여유)를 최대화**한다. 안전한 경로 생성에 유리. 단점: **최단 경로가 아님** (visibility graph는 obstacle vertex로 꺾이는 최단경로를 준다).

### [보조] Local minimum과 navigation function ★★

**(a)** Attractive + repulsive potential 합으로 만든 potential field에서 robot이 local minimum에 갇히는 환경 예시를 하나 그림으로 설명하라.

**(b)** Navigation function이 갖춰야 할 4가지 조건을 나열하라.

**풀이.**
**(a)** **U자형 (오목) 장애물 안쪽**에 robot이 들어가고, 그 정면에 goal이 있는 경우. 두 벽의 반발력이 좌우 대칭으로 robot을 안쪽으로 밀고, 정면 인력은 벽 너머 goal을 가리키지만, robot은 벽 사이의 점에서 합력 = 0이 되어 정지.

**(b)** Navigation function:
1. **Global minimum at goal**: $U(q_g) = \min_q U(q)$.
2. **No local minima**: 모든 critical point가 goal 또는 saddle.
3. **$U \to \infty$ near obstacles**: 충돌 방지.
4. **Smooth** ($C^2$이상): gradient descent 가능.

---

## L01. A* / Dijkstra

### [메인] 작은 그래프에서 A* trace ★★★

다음 5-노드 그래프를 보자. 비용은 엣지 옆 숫자, heuristic $h$는 노드 옆 괄호. 목표는 $G$.

```
        S(h=4)
       /     \
    1 /       \ 2
     /         \
  B(h=2)     A(h=3)
   |   \      /
 4 |    \3  /5
   |     \ /
   G(0)   *
```

단순화: $S\to B$ cost 1, $S\to A$ cost 2, $B\to G$ cost 4, $A\to G$ cost 5, $B\to A$ cost 3.

A*로 $S\to G$ 최적 경로를 찾고, 각 iteration의 open list와 expand 순서를 표로 정리하라.

**풀이.**

| Iter | Expand | Open list 갱신 후 ($f = g + h$) |
|---|---|---|
| 1 | $S$ ($f=0+4=4$) | $\{B(1+2=3),\ A(2+3=5)\}$ |
| 2 | $B$ ($f=3$) | $\{A(2+3=5\text{ vs }1+3+3=7), G(1+4+0=5)\}$ → $A(5), G(5)$ |
| 3 | $A$ ($f=5$, tie-break) | $G$를 통한 갱신: $A\to G$로 $g=2+5=7 > 5$이므로 변화 없음 |
| 4 | $G$ ($f=5$) | **goal 도달, 종료** |

**최적 경로**: $S\to B\to G$, **cost 5**.

### [보조] Admissible but not consistent ★★

3-노드 그래프 $A\to B\to G$, 비용 $c(A,B)=1$, $c(B,G)=10$이라 하자. 다음 heuristic이 admissible이지만 consistent가 아님을 보여라.
$$h(A)=5,\ h(B)=1,\ h(G)=0.$$
또한 Dijkstra가 A*의 특수 케이스인 이유를 한 줄로 설명하라.

**풀이.**
**Admissible**: 실제 최단거리 $h^*(A) = 11$, $h^*(B) = 10$, $h^*(G)=0$. $h\le h^*$ 모두 만족 ✓.

**Consistent**: $h(A) \le c(A,B) + h(B)$ 검사 → $5 \le 1 + 1 = 2$? **거짓** → consistent 아님.

**Dijkstra = A*** with $h(n)=0$ for all $n$. $h=0$은 항상 admissible+consistent이므로 Dijkstra는 A*의 자명한 특수 케이스.

---

## L02. PRM, RRT, RRT*

### [메인] RRT 의사코드 채우기 ★

다음 RRT 의사코드의 빈칸 (1)–(3)을 채우고, goal bias의 역할을 설명하라.
```
T = {q_init}
for i = 1..N:
    q_rand = random_config()     # (with goal bias)
    q_near = (1) ____________
    q_new  = (2) ____________
    if (3) ____________ :
        T.add_vertex(q_new); T.add_edge(q_near, q_new)
```

**풀이.**
- (1) **`nearest(T, q_rand)`** — 트리 안에서 $q_{\text{rand}}$에 가장 가까운 노드 (Euclidean 또는 적절한 metric).
- (2) **`q_near + step \cdot (q_{rand} - q_{near}) / \|q_{rand} - q_{near}\|`** — $q_{\text{near}}$에서 $q_{\text{rand}}$ 방향으로 step만큼 전진.
- (3) **`collision\_free(q_{near}, q_{new})`** — 두 점 사이 segment가 자유 공간에 있는지.

**Goal bias**: 5–10% 확률로 $q_{\text{rand}} = q_{\text{goal}}$로 두기. 순수 무작위 샘플은 균등하게 트리를 키우지만 목표 도달이 느림. Goal bias로 트리가 자연스럽게 목표 방향으로 편향 → **수렴 가속**.

### [보조] RRT*의 rewire 조건과 asymptotic optimality ★★★

**(a)** RRT*에서 $X_{\text{near}}$ 안의 노드 $x_{\text{near}}$에 대해 어떤 조건이 성립할 때 부모를 $x_{\text{new}}$로 바꾸는지(rewire) 식으로 쓰라.

**(b)** RRT가 asymptotic optimal이 아닌 이유와, RRT*는 이를 어떻게 해결하는지 한 단락.

**풀이.**
**(a)** Rewire 조건:
$$\mathrm{cost}(x_{\text{new}}) + c(x_{\text{new}}\to x_{\text{near}}) < \mathrm{cost}(x_{\text{near}})$$
이면 $x_{\text{near}}$의 부모를 (기존)에서 $x_{\text{new}}$로 변경.

**(b)** RRT는 첫 발견한 경로를 그대로 유지하며 한 번 정해진 부모는 바뀌지 않는다. 새로 발견한 더 짧은 경로를 반영하지 못함 → **최적이 아닌 비용에 수렴**. RRT*는 (i) 새 노드에 대해 $X_{\text{near}}$ 내에서 **최소 비용 부모 선택**, (ii) 새 노드를 통해 더 싸지는 이웃의 **부모 재연결(rewire)** — 두 단계로 트리를 지속적으로 개선하여 **almost-sure convergence to optimum**.

---

## L03. LPA* / D* Lite

### [메인] LPA*에서 차단 후 inconsistent 노드 찾기 ★

3×3 grid, 4-connectivity, 모든 셀 비용 1, 처음 모두 free. 시작 $S = (1, 1)$, 목표 $G = (3, 3)$.

LPA*가 처음 수렴해서 $g$ 값이 시작점에서의 최단 거리 (Manhattan)와 같다. 즉:
$$g = \begin{pmatrix}0 & 1 & 2\\ 1 & 2 & 3\\ 2 & 3 & 4\end{pmatrix}.$$

이제 셀 $(1, 2)$가 차단된다.

**(a)** 차단 후 inconsistent 해질 가능성이 있는 노드 (= $(1,2)$의 4-이웃) 를 나열하라.

**(b)** 각 이웃의 새 $rhs$를 계산하고 inconsistent 여부를 판별하라. ($g \ne rhs$이면 priority queue에 다시 들어간다.)

**풀이.**

**(a)** $(1,2)$의 4-이웃: **$\{(1,1),\ (1,3),\ (2,2)\}$**.

**(b)** 차단 후 $g(1,2) = \infty$. 각 이웃의 $rhs$를 다시 계산:

- **$(1,1)$**: 시작점이므로 LPA*에서 $rhs(s_{\text{start}}) = 0$ 고정. $g = 0 = rhs$ → **consistent (불변)**.

- **$(1,3)$**: 4-이웃은 $(1,2), (2,3)$. $rhs = \min\{g(1,2)+1,\ g(2,3)+1\} = \min\{\infty,\ 4\} = 4$.
  기존 $g(1,3) = 2 \ne 4$ → **inconsistent** ($g < rhs$, **underconsistent**).
  LPA*는 $g(1,3) \leftarrow \infty$로 설정하고 $(1,3)$의 이웃들로 propagate.

- **$(2,2)$**: 4-이웃은 $(1,2), (2,1), (2,3), (3,2)$. $rhs = \min\{\infty,\ 2,\ 4,\ 4\} = 2$.
  $g(2,2) = 2 = rhs$ → **consistent (불변)**. $(2,1)$을 통한 대체 경로가 동일 거리.

**결론**: $(1,3)$ 한 개만 underconsistent. $(2,1)$ 같은 대체 경로 덕분에 $(2,2)$는 영향 없음. 이게 LPA* "변경 부분만 재계산"의 핵심 — 차단된 셀이 어떤 이웃의 **유일한** 최단 경로에 있을 때만 inconsistent가 발생한다.

### [보조] $k_m$이 없으면 ★

D* Lite에서 key modifier $k_m$이 도입된 이유와, 만약 $k_m$이 없으면 어떤 비효율이 생기는지 설명하라.

**풀이.**
D* Lite의 key는 $k_1(s) = \min(g, rhs) + h(s_{\text{start}}, s) + k_m$. 로봇이 매 step 움직이면 $s_{\text{start}}$가 바뀌므로 **모든 priority queue 노드의 $h$ 값이 바뀐다**. 이를 매번 반영하려면 queue 전체를 재정렬 ($O(N\log N)$) — 비효율.

$k_m$ 트릭: 기존 queue의 모든 key를 일괄 갱신하는 대신, **누적 이동거리** $k_m \mathrel{+}= h(s_{\text{last}}, s_{\text{curr}})$를 유지하고, **새로 삽입되는 노드에만 $k_m$을 더한 key**를 부여. 기존 노드는 그대로 두어도 상대적 순서가 보존됨. → $O(\log N)$ 삽입만 발생, 전체 재정렬 불필요.

---

# Part II — Constraints & Controllability (L04–L05)

## L04. Nonholonomic Constraints

### [메인] Simple car의 Pfaffian과 Lie bracket ★★★★★

Simple car 운동학:
$$\dot x = v\cos\theta,\quad \dot y = v\sin\theta,\quad \dot\theta = \omega.$$

**(a)** 후륜 미끄럼 없음 조건에서 Pfaffian 제약 $A(q)\dot q = 0$을 유도하라.

**(b)** Control-affine 형태 $\dot q = g_1 v + g_2 \omega$의 $g_1, g_2$를 쓰고 $[g_1, g_2]$를 계산하라.

**(c)** $\det[g_1\ g_2\ [g_1, g_2]]$을 계산하여 nonholonomic임을 증명하라.

**풀이.**
**(a)** 후륜은 heading $\theta$ 방향으로만 움직임 → 속도 벡터 $(\dot x, \dot y)$가 $\theta$에 수직 성분 0:
$$(\dot x, \dot y) \cdot (-\sin\theta, \cos\theta) = -\dot x \sin\theta + \dot y \cos\theta = 0.$$
$$\boxed{A(q) = [\sin\theta\ -\cos\theta\ 0],\quad A(q)\dot q = \dot x\sin\theta - \dot y\cos\theta = 0.}$$

**(b)**
$$g_1 = \begin{bmatrix}\cos\theta\\\sin\theta\\0\end{bmatrix},\quad g_2 = \begin{bmatrix}0\\0\\1\end{bmatrix}.$$

$[g_1, g_2] = \frac{\partial g_2}{\partial q}g_1 - \frac{\partial g_1}{\partial q}g_2$. $g_2$ 상수 → $\partial g_2/\partial q = 0$.
$$\frac{\partial g_1}{\partial q} = \begin{bmatrix}0 & 0 & -\sin\theta\\0 & 0 & \cos\theta\\0 & 0 & 0\end{bmatrix},\quad \frac{\partial g_1}{\partial q}g_2 = \begin{bmatrix}-\sin\theta\\\cos\theta\\0\end{bmatrix}.$$
$$\boxed{[g_1, g_2] = -\begin{bmatrix}-\sin\theta\\\cos\theta\\0\end{bmatrix} = \begin{bmatrix}\sin\theta\\-\cos\theta\\0\end{bmatrix}.}$$

**(c)**
$$\det\begin{bmatrix}\cos\theta & 0 & \sin\theta\\\sin\theta & 0 & -\cos\theta\\0 & 1 & 0\end{bmatrix}.$$
3행 2열 cofactor 전개 ($a_{32}=1$, 나머지 0):
$$= -1 \cdot \det\begin{bmatrix}\cos\theta & \sin\theta\\\sin\theta & -\cos\theta\end{bmatrix} = -1\cdot(-\cos^2\theta-\sin^2\theta) = 1 \neq 0.$$
따라서 $\dim\mathcal{L}(\Delta) = 3 = n$, $\Delta$는 **non-involutive** → **nonholonomic**. $\square$

### [보조] 자동차 변종 비교 ★

Tricycle, Simple car, Reeds-Shepp, Dubins의 입력 집합을 적고, 네 변종 중 **최소 회전반경 $\rho_{\min}$ 개념이 의미 있는** 것을 모두 골라라 (이유 설명).

**풀이.**

| 변종 | $U_v$ | $U_\phi$ |
|---|---|---|
| Tricycle | $[-1, 1]$ | $[-\pi/2, \pi/2]$ |
| Simple car | $[-1, 1]$ | $(-\phi_{\max}, \phi_{\max})$, $\phi_{\max} < \pi/2$ |
| Reeds-Shepp | $\{-1, 0, 1\}$ | $(-\phi_{\max}, \phi_{\max})$ |
| Dubins | $\{0, 1\}$ | $(-\phi_{\max}, \phi_{\max})$ |

$\rho_{\min} = L/\tan\phi_{\max}$.
- **Tricycle**: $u_\phi = \pm\pi/2$이면 제자리 회전 가능 → $\tan(\pi/2) = \infty$ → $\rho_{\min} = 0$, 개념 **무의미**.
- **Simple car / Reeds-Shepp / Dubins**: $\phi_{\max} < \pi/2$로 제한되어 $\rho_{\min} > 0$이 유한 양수 → **의미 있음**.

따라서 의미 있는 것: **Simple car, Reeds-Shepp, Dubins**.

---

## L05. Controllability & Chained Form

### [메인] Unicycle chained form 검증 ★★★★

Unicycle $\dot x = v\cos\theta$, $\dot y = v\sin\theta$, $\dot\theta = \omega$. 좌표/입력 변환
$$z_1 = \theta,\quad z_2 = x\cos\theta + y\sin\theta,\quad z_3 = x\sin\theta - y\cos\theta,$$
$$u_1 = \omega,\quad u_2 = v - z_3 u_1$$
가 chained form $\dot z_1 = u_1,\ \dot z_2 = u_2,\ \dot z_3 = z_2 u_1$을 만족함을 보여라.

**풀이.**

**$\dot z_1$**: $\dot z_1 = \dot\theta = \omega = u_1$ ✓.

**$\dot z_2$**:
$$\dot z_2 = \dot x\cos\theta - x\sin\theta\,\dot\theta + \dot y\sin\theta + y\cos\theta\,\dot\theta.$$
$\dot x = v\cos\theta$, $\dot y = v\sin\theta$ 대입:
$$= v\cos^2\theta + v\sin^2\theta + \omega(y\cos\theta - x\sin\theta) = v - z_3\omega = v - z_3 u_1.$$
$u_2 = v - z_3 u_1$로 정의했으므로 $\dot z_2 = u_2$ ✓.

**$\dot z_3$**:
$$\dot z_3 = \dot x\sin\theta + x\cos\theta\,\dot\theta - \dot y\cos\theta + y\sin\theta\,\dot\theta.$$
대입:
$$= v\cos\theta\sin\theta - v\sin\theta\cos\theta + \omega(x\cos\theta + y\sin\theta) = \omega \cdot z_2 = z_2 u_1.$$
✓.

따라서 $(z_1, z_2, z_3, u_1, u_2)$가 3-state 2-input chained form을 이룬다. $\square$

### [보조] Chow–Rashevskii로 Unicycle controllability ★★

Unicycle $(n=3, m=2)$가 small-time locally controllable임을 Chow–Rashevskii 정리로 보여라.

**풀이.**
Driftless 시스템 $\dot q = g_1 v + g_2 \omega$.
- $g_1, g_2$는 L04 메인에서 정의.
- $[g_1, g_2] = [\sin\theta, -\cos\theta, 0]^T$ (L04 메인에서 계산).

$\dim\mathcal{L}(\Delta) = \mathrm{rank}[g_1\ g_2\ [g_1, g_2]] = 3$ (L04에서 행렬식 1로 입증).

$\dim\mathcal{L}(\Delta) = n = 3$이므로 **Chow–Rashevskii**에 의해 small-time locally controllable. $\square$

해석: 입력 $(v, \omega)$만으로는 순간 sideways 운동 불가하지만, Lie bracket $[g_1, g_2]$가 sideways 방향 $[\sin\theta, -\cos\theta, 0]^T$를 생성 → 4-단계 motion primitive (translate-rotate-untranslate-unrotate)로 임의 위치 도달 가능.

### [보조2] Bicycle-like (자동차) chained form 검증 ★★★★

Rear-wheel drive 자동차, wheelbase $L$:
$$\dot x = v\cos\theta,\ \dot y = v\sin\theta,\ \dot\theta = (v/L)\tan\phi,\ \dot\phi = \omega.$$
$q = (x, y, \theta, \phi)$.

**(a)** $g_1, g_2$를 쓰고 $g_3 = [g_1, g_2]$, $g_4 = [g_1, g_3]$를 계산하라.

**(b)** $\det[g_1\ g_2\ g_3\ g_4]$를 구하고 nonholonomic임을 보여라.

**풀이.**

**(a)**
$$g_1 = \begin{bmatrix}\cos\theta\\\sin\theta\\\tan\phi/L\\0\end{bmatrix},\quad g_2 = \begin{bmatrix}0\\0\\0\\1\end{bmatrix}.$$

$g_2$ 상수 → $\partial g_2/\partial q = 0$. $g_1$의 Jacobian에서 $\phi$ 미분만 nonzero (in 4번째 열):
$$\frac{\partial g_1}{\partial\phi} = \begin{bmatrix}0\\0\\\sec^2\phi/L\\0\end{bmatrix},\quad \frac{\partial g_1}{\partial q}g_2 = \begin{bmatrix}0\\0\\\sec^2\phi/L\\0\end{bmatrix}.$$

$$\boxed{g_3 = [g_1, g_2] = -\frac{\partial g_1}{\partial q}g_2 = \begin{bmatrix}0\\0\\-1/(L\cos^2\phi)\\0\end{bmatrix}.}$$

$g_3$는 $\phi$만 의존, $g_1$의 4번째 성분은 0이므로 $\partial g_3/\partial q \cdot g_1 = 0$. $\partial g_1/\partial q \cdot g_3$의 1, 2행만 nonzero:
$$\frac{\partial g_1}{\partial\theta}\cdot g_3^{(3)} = \begin{bmatrix}-\sin\theta\\\cos\theta\\0\\0\end{bmatrix}\cdot\left(-\frac{1}{L\cos^2\phi}\right) = \begin{bmatrix}\sin\theta/(L\cos^2\phi)\\-\cos\theta/(L\cos^2\phi)\\0\\0\end{bmatrix}.$$

$$\boxed{g_4 = [g_1, g_3] = -\frac{\partial g_1}{\partial q}g_3 = \begin{bmatrix}-\sin\theta/(L\cos^2\phi)\\\cos\theta/(L\cos^2\phi)\\0\\0\end{bmatrix}.}$$

**(b)** 4번째 행 cofactor 전개 ($a_{42} = 1$, 나머지 0). Cofactor 부호 $(-1)^{4+2} = +1$:
$$\det[g_1\ g_2\ g_3\ g_4] = +\det\begin{bmatrix}\cos\theta & 0 & -\sin\theta/(L\cos^2\phi)\\\sin\theta & 0 & \cos\theta/(L\cos^2\phi)\\\tan\phi/L & -1/(L\cos^2\phi) & 0\end{bmatrix}.$$

3×3에서 2번째 열 cofactor ($a_{32} = -1/(L\cos^2\phi)$, 부호 $(-1)^{3+2} = -1$):
$$= (-1)\cdot\left(-\tfrac{1}{L\cos^2\phi}\right)\det\begin{bmatrix}\cos\theta & -\sin\theta/(L\cos^2\phi)\\\sin\theta & \cos\theta/(L\cos^2\phi)\end{bmatrix} = \frac{1}{L\cos^2\phi}\cdot\frac{\cos^2\theta + \sin^2\theta}{L\cos^2\phi}.$$

$$\boxed{\det[g_1\ g_2\ g_3\ g_4] = +\frac{1}{L^2\cos^4\phi} \neq 0\quad (\cos\phi \neq 0).}$$

따라서 $\dim\mathcal{L}(\Delta) = 4 = n$ → **nonholonomic, controllable** (Chow–Rashevskii). 이 결과가 chained form 변환 $z_1 = x, z_2 = \sec^3\theta\tan\phi/L, z_3 = \tan\theta, z_4 = y$의 정당성을 뒷받침. (HW2 1(a)-(c)와 직결.)

---

# Part III — Trajectory & Collision (L06–L09)

## L06. Polynomial Trajectory & Bézier

### [메인] $J = \int (P^{(3)})^2 dt$ 최소화 ★★★★

비용 $J = \int_0^T (P^{(3)}(t))^2 dt$를 최소화하는 다항식 $P(t)$의 차수와 필요한 BC 개수를 Euler–Lagrange equation으로 유도하라.

**풀이.**
$\mathcal{L} = (P^{(3)})^2$. 일반 Euler–Lagrange:
$$\sum_{k=0}^r (-1)^k \frac{d^k}{dt^k}\frac{\partial \mathcal{L}}{\partial x^{(k)}} = 0.$$

$\partial\mathcal{L}/\partial P^{(3)} = 2P^{(3)}$, 나머지는 0. 따라서:
$$(-1)^3 \frac{d^3}{dt^3}\bigl(2 P^{(3)}\bigr) = 0 \quad\Rightarrow\quad \frac{d^6}{dt^6} P(t) = 0.$$

→ $P$는 **5차 다항식** (degree 5). 계수 6개 → **BC 6개** 필요.
일반적으로 양 끝점에서 **위치, 속도, 가속도** (양 끝 합쳐 $3 \times 2 = 6$).

(이것이 **minimum-jerk trajectory**, quintic Hermite.)

### [보조2] Minimum-snap trajectory 차수 ★★★

비용 $J = \int_0^T (P^{(4)}(t))^2 dt$ (snap의 제곱) 최소화에서 최적 $P(t)$의 차수와 BC 개수를 Euler–Lagrange로 유도하고, minimum-jerk와 비교하라.

**풀이.**

$\mathcal{L} = (P^{(4)})^2$. Higher-order Euler–Lagrange:
$$\sum_{k=0}^4 (-1)^k\frac{d^k}{dt^k}\frac{\partial\mathcal{L}}{\partial P^{(k)}} = 0.$$

$\partial\mathcal{L}/\partial P^{(4)} = 2 P^{(4)}$만 nonzero:
$$(-1)^4 \frac{d^4}{dt^4}(2P^{(4)}) = 0 \quad\Rightarrow\quad \frac{d^8}{dt^8}P(t) = 0.$$

→ $P$는 **7차 다항식** (degree 7). 계수 8개 → **BC 8개**.
양 끝에서 **위치, 속도, 가속도, jerk** 각각 ($4 \times 2 = 8$).

**Minimum-jerk vs minimum-snap 비교**:

| 비용 | 차수 | BC 개수 | 양 끝 BC |
|---|:-:|:-:|---|
| $\int(P^{(3)})^2 dt$ (min-jerk) | 5 (quintic) | 6 | 위치, 속도, 가속도 |
| $\int(P^{(4)})^2 dt$ (min-snap) | 7 (septic) | 8 | + jerk |

**Quadrotor flatness 연계**: $\ddddot{\mathbf p}$가 모멘트 $M$을 결정 → trajectory의 snap을 최소화 = 입력 노력을 최소화하는 자연스러운 척도 (Mellinger & Kumar 2011).

### [보조] Bézier $C^2$ 연속 ★★★

두 큐빅 Bézier ($N=3$) segment를 $P_0, P_1, P_2, P_3$과 $Q_0, Q_1, Q_2, Q_3$의 control points로 표현하자. 두 segment를 잇는 점 $P_3 = Q_0$에서 $C^0, C^1, C^2$ 연속을 만족하려면 $Q_0, Q_1, Q_2$를 각각 $P_1, P_2, P_3$로 표현하라.

**풀이.**
Bézier hodograph: $r'(t) = N\sum (b_{k+1}-b_k)B_{k,N-1}(t)$, 즉 1차 미분의 control point는 $N(b_{k+1}-b_k)$.

- **$C^0$**: $r_1(1) = r_2(0) \Rightarrow$ $\boxed{Q_0 = P_3}$.
- **$C^1$**: $r_1'(1) = r_2'(0)$. $r_1'(1) = 3(P_3 - P_2)$, $r_2'(0) = 3(Q_1 - Q_0)$. 등식 → $Q_1 - Q_0 = P_3 - P_2 \Rightarrow$ $\boxed{Q_1 = 2P_3 - P_2}$.
- **$C^2$**: $r_1''(1) = r_2''(0)$. $r_1''(1) = 6(P_3 - 2P_2 + P_1)$, $r_2''(0) = 6(Q_2 - 2Q_1 + Q_0)$. 등식 → $Q_2 - 2Q_1 + Q_0 = P_3 - 2P_2 + P_1$. $Q_0, Q_1$ 대입:
$$Q_2 = (P_3 - 2P_2 + P_1) + 2(2P_3 - P_2) - P_3 = \boxed{4P_3 - 4P_2 + P_1}.$$

---

## L07. Collision Checking

### [메인] SAT으로 두 사각형 충돌 판정 ★★

축 정렬 사각형 $A = [0, 2] \times [0, 2]$와 $B = [1, 3] \times [2, 4]$의 충돌 여부를 SAT으로 판정하라.

**풀이.**
두 사각형 모두 축 정렬이므로 분리축 후보는 $x$축 $u_x = (1,0)$과 $y$축 $u_y = (0,1)$만.

- **$u_x$ 사영**: $A \to [0, 2]$, $B \to [1, 3]$. 교집합 $[1, 2]$, 겹침 (길이 1).
- **$u_y$ 사영**: $A \to [0, 2]$, $B \to [2, 4]$. 교집합 $\{2\}$, 단 한 점 (length 0).

두 축 모두 disjoint하지 않음 → **충돌 (정확히는 경계 접촉)**. 분리축이 없으므로 SAT은 collision으로 판정.

(만약 $B = [3, 5] \times [2, 4]$였다면 $u_x$ 사영이 $[3,5]$로 $A=[0,2]$와 disjoint → no collision.)

### [보조] GJK 알고리즘 단계 ★

GJK가 $A\cap B$ 판정에서 사용하는 핵심 아이디어 (Minkowski 차) 와 **support function**을 설명하고, 아래 순서로 시뮬레이션:

초기 방향 $d_0 = (1, 0)$. Support point 차례로 $a_1 = (4, -2)$, 새 방향 $d_1$ 결정 후 $a_2 = (-7, 2)$, 다시 $d_2$로 $a_3 = (0, 4)$. 이때 simplex가 원점을 포함하는지 어떻게 판단하는가?

**풀이.**
**핵심**: $A \cap B \neq \emptyset \iff 0 \in A - B$ (Minkowski 차).
$$\mathrm{support}_{A-B}(d) = \mathrm{support}_A(d) - \mathrm{support}_B(-d) = \arg\max_{x\in A-B} d^T x.$$

**Simulation**:
1. $a_1 = (4, -2)$. Simplex $\{a_1\}$. 원점은 simplex 외부, 다음 방향 $d_1 = -a_1 = (-4, 2)$.
2. $a_2 = (-7, 2)$. Simplex $\{a_1, a_2\}$. 원점이 segment 외부 → 원점 방향의 수직 분할로 새 방향 $d_2 \propto (24, 66)$ (segment에서 원점 쪽).
3. $a_3 = (0, 4)$. Simplex $\{a_1, a_2, a_3\}$ = triangle.

**원점 포함 판정**: 삼각형 $\{(4,-2), (-7,2), (0,4)\}$의 **barycentric 좌표**로 $0 = \lambda_1 a_1 + \lambda_2 a_2 + \lambda_3 a_3$, $\sum\lambda_i = 1$ 풀이.
$\lambda_1 \approx 0.56,\ \lambda_2 \approx 0.32,\ \lambda_3 \approx 0.12$ — **모두 $\ge 0$, 합 1**이므로 $0$이 삼각형 내부 → **collision**.

(만약 어떤 $\lambda_i < 0$이면 원점은 그 vertex 반대편이므로 그쪽 edge로 simplex를 축소하고 새 support point 계산.)

---

## L08. CHOMP

### [메인] Projection $(I - \hat x'\hat x'^T)$의 의미 ★★

CHOMP obstacle gradient:
$$\frac{\delta J}{\delta x} = \|x'\|\left[(I - \hat x'\hat x'^T)\nabla c - c(x)\kappa\right].$$

**(a)** 행렬 $(I - \hat x'\hat x'^T)$가 임의의 벡터에 작용할 때 기하학적으로 무엇을 하는지 설명하라.

**(b)** 왜 이 projection이 obstacle gradient에 등장하는가? 이를 arc-length parameterization과 연관지어 설명하라.

**풀이.**
**(a)** $\hat x' = x'/\|x'\|$는 trajectory의 단위 접선벡터. $(I - \hat x'\hat x'^T) v$는 $v$를 **접선에 수직인 평면으로 정사영**한 벡터:
$$v = \underbrace{(\hat x'^T v)\hat x'}_{\text{tangential}} + \underbrace{(I - \hat x'\hat x'^T)v}_{\text{normal}}.$$

**(b)** Obstacle cost를 **arc-length로 적분** (시간 무관)하기 위해 $\int c(x)\|x'\|\, dt$ 형태. 이때 trajectory를 **접선 방향으로 reparameterize** (속도 조정)해도 cost 불변이어야 한다.

기존 $\nabla c$를 그대로 쓰면 접선 방향 변형도 cost를 바꿀 수 있다. Projection을 취하면 **접선 성분이 제거** → 오직 **trajectory를 옆으로 밀어내는 변형(normal)** 만 살아남음. 이는 reparameterization invariance를 보장하고, 더 부드러운 obstacle avoidance를 유도.

추가 항 $-c(x)\kappa$는 curvature가 큰 곳에서 trajectory를 곧게 펴는 역할 (path length 단축).

### [보조] $K_2$ Finite difference matrix (4 waypoint) ★

$\xi = (q_1, q_2, q_3, q_4)^T$ (1차원, 4 waypoint). $K_2$ finite difference operator를 명시적으로 행렬로 쓰고, $K_2\xi$의 의미를 설명하라.

**풀이.**
$K_2$의 $i$-th 행은 가속도 ($q_{i+1}-2q_i+q_{i-1}$). 강의안 정의에 따라:
$$K_2 = \begin{bmatrix}1 & 0 & 0 & 0\\-2 & 1 & 0 & 0\\1 & -2 & 1 & 0\\0 & 1 & -2 & 1\end{bmatrix},\quad K_2\xi = \begin{bmatrix}q_1\\-2q_1 + q_2\\q_1 - 2q_2 + q_3\\q_2 - 2q_3 + q_4\end{bmatrix}.$$

(끝점 boundary는 $K_2$의 첫 두 행이 "절반 stencil"이고, 추가 $e_2$ 벡터로 boundary condition 보정.)

$\|K_2\xi\|^2$는 trajectory의 **discrete acceleration 제곱합** → minimum-acceleration smoothness penalty. $A = K_2^T K_2 \succ 0$ — CHOMP의 metric matrix.

---

## L09. Quadrotor Flatness & SFC

### [메인] Quadrotor thrust와 body z-axis ★★★

질량 $m = 1$, 중력 $g = 10$ (단순화). $\ddot{\mathbf{p}}$가 두 경우로 주어진다:

**(a)** 호버 $\ddot{\mathbf{p}} = [0, 0, 0]^T$ — thrust $f$와 body z-축 $\mathbf{b}_3$를 구하라.

**(b)** $\ddot{\mathbf{p}} = [3, 0, 6]^T$ (위 + 앞 방향 가속) — $f, \mathbf{b}_3$를 구하라.

**풀이.**
공식: $\mathbf{a} = g\mathbf{e}_3 - \ddot{\mathbf{p}}$, $f = m\|\mathbf{a}\|$, $\mathbf{b}_3 = \mathbf{a}/\|\mathbf{a}\|$. $\mathbf{e}_3 = [0,0,1]^T$.

**(a)** $\mathbf{a} = [0, 0, 10] - [0, 0, 0] = [0, 0, 10]$. $\|\mathbf{a}\| = 10$.
$$\boxed{f = 10,\quad \mathbf{b}_3 = [0,0,1]^T}$$
수직 상방, 중력과 평형.

**(b)** $\mathbf{a} = [0, 0, 10] - [3, 0, 6] = [-3, 0, 4]$. $\|\mathbf{a}\| = \sqrt{9 + 16} = 5$.
$$\boxed{f = 5,\quad \mathbf{b}_3 = [-3, 0, 4]^T/5}$$

**의미**: thrust 방향 $\mathbf{b}_3$는 가속 요구와 중력 보상의 합력 방향. 평탄출력 $(x, y, z, \psi)$의 2차 미분만으로 attitude 2자유도($\mathbf{b}_3$)와 $f$가 결정됨 → flatness.

### [보조] SFC ellipsoid tangent plane ★

Ellipsoid $\xi(E, d) = \{p : (p-d)^T E(p-d) \le 1\}$의 경계점 $p^c$에서의 접평면이 다음과 같음을 보여라:
$$a_j^T p \le b_j,\quad a_j = 2 E(p^c - d),\quad b_j = a_j^T p^c.$$

(강의안에는 $E^{-1}E^{-T}$로 나오는데, 여기서는 ellipsoid의 정의를 $(p-d)^T E (p-d) \le 1$, $E \succ 0$로 통일한 경우의 결과.)

**풀이.**
Ellipsoid 경계: $g(p) = (p-d)^T E(p-d) - 1 = 0$. 접평면의 법선은 $\nabla g(p^c) = 2 E(p^c - d) =: a$.

접평면 방정식: $a^T(p - p^c) = 0$, 즉 $a^T p = a^T p^c =: b$.

내부/외부 결정: 경계의 안쪽($g < 0$)이 $a^T p < b$인지 확인. $p = d$ (중심) 대입: $a^T d = 2(p^c-d)^T E\,d$. $a^T p^c - a^T d = 2(p^c - d)^T E(p^c - d) = 2 \cdot 1 = 2 > 0$ → 중심 $d$는 $a^T p < b$ 쪽. 따라서 ellipsoid는 $\{p : a^T p \le b\}$ halfspace에 포함되고, 이 hyperplane은 ellipsoid의 **지지 hyperplane**.

(강의안의 $E^{-1}E^{-T}$ 형식은 $\xi$ 정의가 $(p-d)^T (E E^T)^{-1} (p-d) \le 1$일 때 — affine map $E$로 unit ball을 변환한 ellipsoid. 두 표현은 동치.)

---

# Part IV — Optimal Control & MPC (L10–L12)

## L10. LQR / iLQR

### [메인] Riccati 한 step 손계산 ★★★★

2D 시스템 $A = \begin{pmatrix}1 & 1\\0 & 1\end{pmatrix}$, $B = \begin{pmatrix}0\\1\end{pmatrix}$ (double integrator with $\Delta t = 1$). $Q = I_2$, $R = 1$, $P_2 = Q = I_2$ ($N = 2$, terminal at $k = 2$).

$K_1, P_1$을 한 단계 Riccati로 손계산하라.

**풀이.**
공식:
$$K_k = (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A,$$
$$P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A.$$

$P_2 = I$.

**1. $B^T P_2 B$**: $B^T I B = B^T B = 0^2 + 1^2 = 1$. $R + B^T P_2 B = 1 + 1 = 2$.

**2. $B^T P_2 A$**:
$$B^T P_2 A = (0, 1) \begin{pmatrix}1 & 1\\0 & 1\end{pmatrix} = (0, 1).$$

**3. $K_1$**:
$$K_1 = (1/2)(0, 1) = (0, 1/2).$$

**4. $A^T P_2 A$**:
$$A^T = \begin{pmatrix}1 & 0\\1 & 1\end{pmatrix},\ A^T P_2 = A^T,\ A^T A = \begin{pmatrix}1 & 0\\1 & 1\end{pmatrix}\begin{pmatrix}1 & 1\\0 & 1\end{pmatrix} = \begin{pmatrix}1 & 1\\1 & 2\end{pmatrix}.$$

**5. $A^T P_2 B (B^T P_2 A)/2$**:
$$A^T P_2 B = \begin{pmatrix}1 & 0\\1 & 1\end{pmatrix}\begin{pmatrix}0\\1\end{pmatrix} = \begin{pmatrix}0\\1\end{pmatrix}.$$
$$A^T P_2 B \cdot (B^T P_2 A)/2 = \begin{pmatrix}0\\1\end{pmatrix}(0, 1)/2 = \frac{1}{2}\begin{pmatrix}0 & 0\\0 & 1\end{pmatrix}.$$

**6. $P_1$**:
$$P_1 = I + \begin{pmatrix}1 & 1\\1 & 2\end{pmatrix} - \frac{1}{2}\begin{pmatrix}0 & 0\\0 & 1\end{pmatrix} = \begin{pmatrix}2 & 1\\1 & 5/2\end{pmatrix}.$$

$$\boxed{K_1 = (0,\ 1/2),\quad P_1 = \begin{pmatrix}2 & 1\\1 & 5/2\end{pmatrix}.}$$

해석: 최적 제어 $u^* = -K_1 x = -(1/2)v$. 속도에 비례한 감속 ($B = e_2$ 구조이므로 위치는 직접 제어 안 됨).

### [보조] iLQR의 $k, K$ 역할과 line search ★★

iLQR backward pass에서 $\delta u^* = k + K \delta x$, $k = -Q_{uu}^{-1}Q_u$, $K = -Q_{uu}^{-1}Q_{ux}$.

**(a)** $k$와 $K$ 각각이 forward rollout $u^{\text{new}} = \bar u + \alpha k + K(x^{\text{new}} - \bar x)$에서 어떤 역할을 하는지 설명하라.

**(b)** $\alpha \in (0, 1]$ line search가 왜 필요한가?

**풀이.**
**(a)**
- **$k$ (feedforward)**: 명목 trajectory에서 **open-loop 개선 방향**. $\bar u$에 더해져 더 낮은 cost로 이동. $\bar u$가 이미 최적이면 $Q_u = 0$ → $k = 0$.
- **$K$ (feedback)**: state 편차 $x^{\text{new}} - \bar x$를 보정하는 **closed-loop gain**. forward rollout 중 비선형성으로 nominal에서 벗어나도 안정화. LQR의 시간가변 feedback gain과 동일한 구조.

**(b)** Backward pass는 nominal 주위에서의 **2차 근사**. Full step $\alpha = 1$은 근사가 정확할 때만 보장. 비선형성이 크거나 nominal에서 멀어지면 cost가 오히려 증가할 수 있다.

→ $\alpha$를 1부터 줄여가며 (0.5, 0.25, …) **실제 cost가 감소하는 첫 $\alpha$** 채택. 또한 backward pass에 LM regularization $Q_{uu} + \lambda I$를 추가해 안정화.

---

## L11. Linear MPC Stability

### [메인] Recursive feasibility 핵심 증명 ★★★★★

Linear MPC의 recursive feasibility를 다음 단계로 증명하라:

(i) 시각 $t$에서 optimal control sequence $\mathbf{u}_t^* = (u_0^*, u_1^*, \ldots, u_{N-1}^*)$와 predicted states $(x_0^*, x_1^*, \ldots, x_N^*)$가 존재하고, $x_N^* \in \mathcal{X}_f$이라 하자. 첫 입력 $u_0^*$ 적용 후 다음 상태가 $x_{t+1} = x_1^*$.

(ii) 시각 $t+1$에서 후보 control sequence $\tilde{\mathbf{u}} = (u_1^*, \ldots, u_{N-1}^*, K x_N^*)$ ($K$: terminal feedback)가 feasible함을 보여라.

**풀이.**

후보 sequence $\tilde{\mathbf{u}}$를 적용했을 때의 predicted states:
$$\tilde x_i = x_{i+1}^*,\quad i = 0, 1, \ldots, N-1,\quad \tilde x_N = A x_N^* + B(K x_N^*) = (A + BK)x_N^* = A_K x_N^*.$$

**Feasibility 점검**:
1. **입력 제약 $\tilde u_i \in \mathcal{U}$**:
   - $i = 0, \ldots, N-2$: $\tilde u_i = u_{i+1}^*$이고 시각 $t$에서 $u_{i+1}^* \in \mathcal{U}$였으므로 ✓.
   - $i = N-1$: $\tilde u_{N-1} = K x_N^*$. Terminal set 정의 $K\mathcal{X}_f \subseteq \mathcal{U}$이고 $x_N^* \in \mathcal{X}_f$이므로 $K x_N^* \in \mathcal{U}$ ✓.

2. **상태 제약 $\tilde x_i \in \mathcal{X}$**:
   - $i = 0, \ldots, N-1$: $\tilde x_i = x_{i+1}^* \in \mathcal{X}$ ✓ (시각 $t$ optimal solution의 제약).
   - $i = N$: $\tilde x_N = A_K x_N^*$. Terminal set 정의 $\mathcal{X}_f \subseteq \mathcal{X}$이고 곧 보이듯 $\tilde x_N \in \mathcal{X}_f \subseteq \mathcal{X}$ ✓.

3. **Terminal constraint $\tilde x_N \in \mathcal{X}_f$**: Terminal set의 **invariance** 조건 $A_K \mathcal{X}_f \subseteq \mathcal{X}_f$로부터 $\tilde x_N = A_K x_N^* \in \mathcal{X}_f$ ✓.

따라서 $\tilde{\mathbf{u}}$는 시각 $t+1$에서 feasible solution. MPC는 이보다 같거나 더 좋은 optimal solution을 찾을 수 있으므로 **feasibility는 보존**된다. $\square$

핵심: terminal set의 **invariance under $K$**가 결정적.

### [보조] Condensed vs Sparse QP ★

Linear MPC를 풀 때 두 가지 QP formulation이 있다.
- **Condensed**: 다이내믹스를 소거하고 $U = (u_0, \ldots, u_{N-1})$만 변수.
- **Sparse**: $Z = (x_0, u_0, x_1, u_1, \ldots, x_N)$ 전부 변수.

**(a)** 두 형식의 변수 수를 $N, n, m$로 표현하라.

**(b)** 어느 쪽이 horizon이 길어질수록 유리한지, 그 이유를 한 단락으로.

**풀이.**

**(a)**
- Condensed: $|U| = Nm$.
- Sparse: $|Z| = (N+1)n + Nm$.

(Condensed가 변수 적음.)

**(b)** Condensed는 $X = \Phi x_t + \Gamma U$ 형태로 $X$를 $U$로 표현하는데, $\Gamma$가 **lower triangular block matrix로 dense** (실제로 $\Gamma_{ij} = A^{i-j-1}B$). Hessian $H = \Gamma^T \bar Q \Gamma + \bar R$ 역시 dense. $N$이 커질수록 $O((Nm)^2)$ 행렬 연산이 비싸진다.

Sparse는 변수는 많지만 equality constraint matrix $A_{eq}$가 **block-banded** (인접 시점만 연결). KKT system도 banded → **$O(N)$ factorization** 가능 (Riccati-like). Long horizon에서는 sparse가 효율적.

---

## L12. NMPC & PMP

### [메인] Costate recursion 유도 ★★★★

강의안 표준 식 $\lambda_k = \ell_x + F_x^T\lambda_{k+1}$이 직접 나오도록 multiplier 인덱스를 $\lambda_{k+1}$로, 부호는 $F - x$ 방향으로 정한 Lagrangian:
$$\mathcal{L} = \sum_{k=0}^{N-1}\ell(x_k, u_k) + \ell_f(x_N) + \sum_{k=0}^{N-1}\lambda_{k+1}^T\bigl(F(x_k, u_k) - x_{k+1}\bigr).$$

$\partial\mathcal{L}/\partial x_N = 0$과 $\partial\mathcal{L}/\partial x_k = 0$ ($0 < k < N$)로부터 다음을 유도하라:
$$\lambda_N = \ell_{f,x}(x_N),\qquad \lambda_k = \ell_x(x_k, u_k) + F_x(x_k, u_k)^T \lambda_{k+1}.$$

**풀이.**

**Terminal** ($k = N$). $x_N$이 등장하는 항: $\ell_f(x_N)$, 그리고 $\sum$의 $k = N-1$ 항에서 $-\lambda_N^T x_N$.
$$\frac{\partial \mathcal{L}}{\partial x_N} = \ell_{f,x}(x_N) - \lambda_N = 0 \quad\Rightarrow\quad \boxed{\lambda_N = \ell_{f,x}(x_N).}$$

**Internal** ($0 < k < N$). $x_k$가 등장하는 항:
- $\ell(x_k, u_k)$ → $\ell_x$
- $\lambda_{k+1}^T F(x_k, u_k)$ ($\sum$의 $k$항) → $F_x^T \lambda_{k+1}$
- $-\lambda_k^T x_k$ ($\sum$의 $k-1$항: $-x_{(k-1)+1} = -x_k$) → $-\lambda_k$

합 = 0:
$$\ell_x + F_x^T \lambda_{k+1} - \lambda_k = 0 \quad\Rightarrow\quad \boxed{\lambda_k = \ell_x(x_k, u_k) + F_x(x_k, u_k)^T \lambda_{k+1}.}$$

**해석**: $\lambda_k$는 시각 $k$의 상태가 **미래 cost에 미치는 sensitivity** (costate). 역방향으로 단계마다 한 step씩 $F_x^T$로 propagate하면서 즉각 cost gradient $\ell_x$를 누적한다. PMP의 이산판.

### [보조2] Discrete costate → Continuous PMP 극한 ★★★★

연속시간 dynamics $\dot x = f(x, u)$를 Euler 이산화하면 $F(x, u) = x + \Delta t\, f(x, u)$, stage cost는 $\Delta t \cdot \ell(x, u)$. 이산 NMPC의 1차 최적성 조건은
$$\lambda_k = \Delta t\,\ell_x(x_k, u_k) + F_x(x_k, u_k)^T \lambda_{k+1}, \qquad \Delta t\,\ell_u(x_k, u_k) + F_u(x_k, u_k)^T \lambda_{k+1} = 0.$$

$\Delta t \to 0$ 극한을 취해 연속시간 **Pontryagin 최소원리** 식
$$\dot\lambda = -\frac{\partial H}{\partial x}, \qquad \frac{\partial H}{\partial u} = 0,\quad H(x, u, \lambda) = \ell + \lambda^T f$$
가 나옴을 보여라.

**풀이.**

**Step 1 — Jacobian 대입**. $F = I \cdot x + \Delta t\, f$이므로
$$F_x = I + \Delta t\, f_x, \qquad F_u = \Delta t\, f_u.$$

**Step 2 — Costate 식 정리**. 대입하면
$$\lambda_k = \Delta t\,\ell_x + (I + \Delta t\, f_x)^T \lambda_{k+1} = \lambda_{k+1} + \Delta t\bigl(\ell_x + f_x^T \lambda_{k+1}\bigr).$$
양변에서 $\lambda_{k+1}$를 빼고 $\Delta t$로 나누면
$$\frac{\lambda_k - \lambda_{k+1}}{\Delta t} = \ell_x + f_x^T \lambda_{k+1}.$$

**Step 3 — Limit $\Delta t \to 0$**. $k\,\Delta t \to t$로 두고 $\lambda_k \to \lambda(t)$, $\lambda_{k+1} \to \lambda(t)$. 좌변의 backward difference는 $-\dot\lambda(t)$:
$$-\dot\lambda = \ell_x + f_x^T \lambda.$$
$H = \ell + \lambda^T f$로 두면 $\partial_x H = \ell_x + f_x^T \lambda$이므로
$$\boxed{\dot\lambda(t) = -\frac{\partial H}{\partial x}.}$$

**Step 4 — Control optimality**. $\Delta t\,\ell_u + (\Delta t\, f_u)^T \lambda_{k+1} = 0$ 양변을 $\Delta t$로 나누면
$$\ell_u + f_u^T \lambda = 0 \quad\Leftrightarrow\quad \boxed{\frac{\partial H}{\partial u} = 0.}$$

**해석**: 이산 NMPC의 FONC가 곧 PMP의 이산화임. costate $\lambda$는 시각 $t$의 상태가 미래 cost에 미치는 sensitivity이고, Hamiltonian의 $x$-편미분에 음의 부호로 역방향 전파. DDP의 backward pass는 이 PMP 식의 직접적인 이산 구현이다.

### [보조] PMP를 LQR에 적용 ★★

연속시간 LQR: $\ell(x, u) = x^T Q x + u^T R u$, $f(x, u) = Ax + Bu$, $R \succ 0$.

Hamiltonian을 쓰고, $\partial H / \partial u = 0$으로부터 최적 입력을 $\lambda$로 표현하라.

**풀이.**

Hamiltonian:
$$H(x, u, \lambda) = \ell + \lambda^T f = x^T Q x + u^T R u + \lambda^T(Ax + Bu).$$

$u$에 대한 stationary:
$$\frac{\partial H}{\partial u} = 2 R u + B^T \lambda = 0.$$

$R \succ 0$이므로:
$$\boxed{u^*(t) = -\frac{1}{2} R^{-1} B^T \lambda(t).}$$

(이산판에서는 $\lambda \to \lambda_{k+1}$로 대체.)

이 식과 costate dynamics $\dot\lambda = -\partial_x H = -2Qx - A^T\lambda$를 합치면 두 ODE가 결합되어 Hamiltonian system을 이룬다. Riccati ansatz $\lambda = 2 P x$를 대입하면 $P$가 만족하는 식이 정확히 **continuous-time ARE**:
$$A^T P + P A - P B R^{-1} B^T P + Q = 0.$$

→ LQR optimal gain $K_{\text{LQR}} = R^{-1} B^T P$. PMP가 Riccati와 일치.

---

# 정리 + 출제 가능성 티어 (5단계)

| 티어 | 의미 | 문제 수 |
|:-:|---|:-:|
| ★★★★★ | 거의 확정적 출제 | 2 |
| ★★★★ | 매우 높음 | 6 |
| ★★★ | 높음 | 6 |
| ★★ | 중간 | 7 |
| ★ | 낮음 | 8 |

총 **29문제** (L05, L06, L12에 보조2 추가).

| L | 메인 | 티어 | 보조 | 티어 |
|---|---|:-:|---|:-:|
| 00 | Visibility/Voronoi 비교 | ★★★ | Local minima + nav func 조건 | ★★ |
| 01 | A* trace | ★★★ | Admissible≠consistent | ★★ |
| 02 | RRT 의사코드 | ★ | RRT* rewire + asymp.\ optimal | ★★★ |
| 03 | LPA* update | ★ | $k_m$ 역할 | ★ |
| 04 | Pfaffian + Lie bracket | ★★★★★ | 차량 변종 비교 | ★ |
| 05 | Unicycle chained form | ★★★★ | Chow–Rashevskii / **자동차 chained form** | ★★ / **★★★★** |
| 06 | Min-jerk 차수 유도 | ★★★★ | Bézier $C^2$ / **Min-snap 차수** | ★★★ / **★★★** |
| 07 | SAT 두 사각형 | ★★ | GJK trace | ★ |
| 08 | $(I-\hat x'\hat x'^T)$ 의미 | ★★ | $K_2$ matrix | ★ |
| 09 | Quadrotor $f, \mathbf{b}_3$ | ★★★ | Ellipsoid tangent | ★ |
| 10 | Riccati 한 step | ★★★★ | iLQR $k, K, \alpha$ | ★★ |
| 11 | Recursive feasibility | ★★★★★ | Condensed vs Sparse | ★ |
| 12 | Costate recursion 유도 | ★★★★ | PMP on LQR / **Discrete→Cont PMP 극한** | ★★ / **★★★★** |

## 등급별 정리

### ★★★★★ 거의 확정적 출제 (2)
1. **L11 메인** Recursive feasibility — MPC 안정성 핵심, 강의안에 자세히 다룸.
2. **L04 메인** Pfaffian + Lie bracket — nonholonomic 핵심, HW에서도 다룸.

### ★★★★ 매우 높음 (6)
3. **L05 메인** Unicycle chained form — HW2 직결 패턴.
4. **L05 보조2** 자동차 chained form (Lie bracket 4×4) — HW2 1(a)-(c) 직결.
5. **L10 메인** Riccati 한 step — 손계산 표준 문제.
6. **L06 메인** Min-jerk 차수 유도 — Euler–Lagrange 클래식.
7. **L12 메인** Costate recursion 유도 — NMPC 핵심.
8. **L12 보조2** Discrete → Continuous PMP 극한 — NMPC ↔ PMP 본질 연결.

### ★★★ 높음 (6)
9. **L01 메인** A* trace — 손계산 클래식.
10. **L09 메인** Quadrotor $f, \mathbf{b}_3$ — flatness 손계산.
11. **L00 메인** Visibility / Voronoi 비교 — 개념 비교 단골.
12. **L06 보조** Bézier $C^2$ — 트라젝토리 핵심.
13. **L06 보조2** Min-snap 차수 유도 — min-jerk와의 일반화.
14. **L02 보조** RRT* rewire + asymp.\ optimal — 식 + 직관.

### ★★ 중간 (7)
15. **L05 보조** Chow–Rashevskii — Lie algebra 짧은 적용.
16. **L00 보조** Local minima + nav func 4조건.
17. **L01 보조** Admissible ≠ consistent — 미묘한 차이.
18. **L08 메인** $(I-\hat x'\hat x'^T)$ projection 의미 — CHOMP 핵심.
19. **L10 보조** iLQR $k, K, \alpha$ — 개념 설명.
20. **L12 보조** PMP on LQR — 짧은 유도.
21. **L07 메인** SAT 두 사각형 — 손계산.

### ★ 낮음 (8)
22. **L02 메인** RRT 의사코드 — 단순 알고리즘 채우기.
23. **L03 메인** LPA* update — 약간 복잡.
24. **L03 보조** $k_m$ 역할 — 개념 설명.
25. **L07 보조** GJK trace — 약간 복잡.
26. **L04 보조** 자동차 변종 — 표 정리.
27. **L08 보조** $K_2$ 행렬 — 단순 작성.
28. **L09 보조** ellipsoid tangent — 유도.
29. **L11 보조** condensed vs sparse — 비교.

## 출제 패턴 추정

- **계산 문제 2–3개**: L01 A* trace / L10 Riccati / L04 Lie bracket / L09 Quadrotor 중.
- **유도 문제 1–2개**: L06 min-jerk 차수 / L05 chained form / L12 costate or PMP 극한 중.
- **증명/논리 1개**: L11 recursive feasibility는 거의 확정.
- **개념 비교 1개**: L00 visibility vs Voronoi / L02 RRT vs RRT* 등.
- **개념 서술 1개**: L08 CHOMP projection / L09 flatness 의미 등.

총 **29문제**. 풀이는 cheat sheet 없이도 따라갈 수 있도록 단계별로 작성. 손계산 가능한 수치만 사용.
