# Heterogeneous Population Parameters and Family Cohesion Force Model

To improve the realism of crowd evacuation simulation, pedestrians are divided into three categories: children, elderly people, and adults. Different physical parameters, desired walking speeds, and disaster response capabilities are assigned to different pedestrian types.

Let the type of pedestrian \(i\) be denoted as \(c_i\). The basic parameters of pedestrian \(i\) include height \(h_i\), body radius \(r_i\), and desired speed \(v_i^0\). The body radius is used to represent the occupied horizontal space of each pedestrian and is involved in hard collision separation, interpersonal repulsive force calculation, and minimum-distance constraints during initial position generation. The desired speed represents the basic walking speed of a pedestrian under free-flow conditions without congestion or obstacles.

The basic parameters of the three pedestrian types are shown in Table 1.

**Table 1. Basic parameters of different pedestrian types**

| Pedestrian type | Generation ratio | Height / m | Radius / m | Desired speed / (m/s) |
|---|---:|---:|---:|---:|
| Child | 20% | 1.20 | 0.20 | 0.90 |
| Elderly | 30% | 1.60 | 0.28 | 0.95 |
| Adult | 50% | 1.75 | 0.32 | 1.35 |

In a fire scenario, pedestrians adjust their movement speed according to the NavMesh adjacency distance between their current position and the fire-affected region. Let the fire intensity be \(I \in [0,1]\). The fire warning radius is defined as:

```math
R_{\mathrm{fire}} = R_0 + R_I I
```

where \(R_0 = 5\,\mathrm{m}\) and \(R_I = 7\,\mathrm{m}\).

When a pedestrian is located within the fire warning radius, the desired speed is multiplied by a type-dependent acceleration factor. The maximum fire-induced speed multipliers for adults, children, and elderly people are set to \(1.50\), \(1.40\), and \(1.25\), respectively. To avoid frequent switching near the warning boundary, a hysteresis distance of \(3\,\mathrm{m}\) is introduced. That is, the fire acceleration state is removed only after the pedestrian moves outside \(R_{\mathrm{fire}} + 3\,\mathrm{m}\).

In addition, when the smoke concentration \(S\) exceeds the threshold \(S_{\mathrm{slow}} = 0.55\), the speed multiplier of all pedestrian types is reduced to \(0.80\). This setting represents the negative influence of dense smoke on visibility, breathing, and movement ability.

Therefore, the effective desired speed of pedestrian \(i\) is expressed as:

```math
v_i^{\mathrm{eff}} = v_i^0 \lambda_i^{\mathrm{fire}} \lambda_i^{\mathrm{rain}}
```

where \(\lambda_i^{\mathrm{fire}}\) is the speed correction coefficient under fire conditions, and \(\lambda_i^{\mathrm{rain}}\) is the speed correction coefficient under heavy rain or water-accumulation conditions. If the pedestrian is not affected by the corresponding disaster factor, the correction coefficient is set to \(1\).

In addition to individual differences, this study further considers family-based cooperative behavior during evacuation. In real evacuation scenarios, children tend to stay close to adults, and elderly people may also be assisted by family members. Therefore, pedestrians should not be treated as completely independent individuals. To describe this phenomenon, a visibility-constrained Gaussian family cohesion force is introduced.

Let \(F(i)\) denote the family group to which pedestrian \(i\) belongs. The distance between pedestrian \(i\) and family member \(j\) is defined as:

```math
d_{ij} = \left\| \mathbf{x}_j - \mathbf{x}_i \right\|
```

The unit direction vector from pedestrian \(i\) to pedestrian \(j\) is defined as:

```math
\mathbf{e}_{ij} =
\frac{\mathbf{x}_j-\mathbf{x}_i}
{\left\|\mathbf{x}_j-\mathbf{x}_i\right\|}
```

The family cohesion force acting on pedestrian \(i\) is defined as:

```math
\mathbf{F}_{\mathrm{fam}}(i)
=
\sum_{j \in F(i),\, j \ne i}
V_{ij} w_{ij} g(d_{ij}) \mathbf{e}_{ij}
```

where \(V_{ij}\) is the visibility constraint term, \(w_{ij}\) is the relationship weight between family members, and \(g(d_{ij})\) is the distance-dependent Gaussian cohesion function.

The visibility constraint \(V_{ij}\) is defined as:

```math
V_{ij}
=
\begin{cases}
1, & d_{ij}<d_{\mathrm{view}} \ \mathrm{and}\ \mathrm{visible}(i,j)=1,\\
0, & \mathrm{otherwise}.
\end{cases}
```

where \(d_{\mathrm{view}} = 30\,\mathrm{m}\). The term \(\mathrm{visible}(i,j)=1\) means that there is no obstacle between pedestrian \(i\) and pedestrian \(j\) on the NavMesh; otherwise, \(\mathrm{visible}(i,j)=0\). This constraint prevents unreasonable long-range attraction between family members who are too far away or blocked by walls.

The distance-dependent function \(g(d)\) is defined as the product of a Gaussian bell-shaped function and a short-distance suppression term:

```math
g(d)
=
\exp\left(
-\frac{(d-d_{\mathrm{peak}})^2}{2\sigma^2}
\right)
\left[
1-\exp\left(
-\frac{d^2}{2d_{\min}^2}
\right)
\right],
\quad 0 \le d < d_{\mathrm{view}}
```

When the distance exceeds the visual range, \(g(d)\) is set to zero:

```math
g(d)=0,\quad d \ge d_{\mathrm{view}}
```

where \(d_{\min}=1\,\mathrm{m}\) denotes the short-distance suppression range, which prevents excessive aggregation among family members. The parameter \(d_{\mathrm{peak}}=8\,\mathrm{m}\) denotes the distance at which the family cohesion force reaches its maximum value. The parameter \(\sigma=6\,\mathrm{m}\) controls the decay width of the Gaussian function, and \(d_{\mathrm{view}}=30\,\mathrm{m}\) denotes the maximum visual range.

This function allows the family cohesion force to exhibit the following behavior: weak attraction at very short distances, strong attraction at medium distances, gradual decay at long distances, and disappearance outside the visual range.

The relationship weight \(w_{ij}\) is determined according to the types of the two family members, as shown in Table 2.

**Table 2. Relationship weights between different family-member types**

| Family-member relationship | Weight \(w_{ij}\) |
|---|---:|
| Child--Adult | 1.25 |
| Elderly--Adult | 0.90 |
| Elderly--Child | 0.60 |
| Adult--Adult | 0.70 |
| Child--Child | 0.50 |
| Elderly--Elderly | 0.50 |

The weight between children and adults is the highest, indicating that children are more dependent on adults during evacuation. The weight between elderly people and adults is also relatively high, representing the assistance provided by adults to elderly family members. The weights between same-type members are lower to avoid excessive attraction and local congestion.

During motion updating, the family cohesion force does not replace global path planning. Instead, it acts as a local motion correction term together with the path-following force and interpersonal repulsive force. The desired movement direction of pedestrian \(i\) is expressed as:

```math
\mathbf{d}_i
=
\operatorname{normalize}
\left(
\alpha \mathbf{d}_{\mathrm{path}}
+
\beta \mathbf{F}_{\mathrm{person}}
+
\gamma \mathbf{F}_{\mathrm{fam}}(i)
\right)
```

where \(\mathbf{d}_{\mathrm{path}}\) is the target direction generated by the Detour global path and the path-corridor following mechanism, \(\mathbf{F}_{\mathrm{person}}\) denotes the local interpersonal repulsive force, and \(\mathbf{F}_{\mathrm{fam}}(i)\) denotes the family cohesion force acting on pedestrian \(i\). The parameters \(\alpha\), \(\beta\), and \(\gamma\) are the weights of the path-following term, the interpersonal repulsive term, and the family cohesion term, respectively.

The final desired velocity is given by:

```math
\mathbf{v}_i^{\mathrm{des}}
=
v_i^{\mathrm{eff}} \mathbf{d}_i
```

To prevent visible overlap between pedestrians, hard collision separation is performed after motion updating. For any two pedestrians \(i\) and \(j\), if their distance satisfies:

```math
d_{ij} < r_i + r_j + \epsilon
```

their positions are corrected along the line connecting them so that the final distance is not smaller than the sum of their body radii. Here, \(\epsilon\) is a small safety margin. This process ensures that visible overlap between pedestrians does not occur even under high-density or disaster-accelerated conditions.

In summary, this model integrates heterogeneous pedestrian parameters, fire-induced speed response, visibility-constrained family cohesion, and hard collision separation into the evacuation simulation process. By jointly considering individual differences, social relationships, and local physical constraints, the proposed model improves both the realism and interpretability of simulated crowd behavior.
