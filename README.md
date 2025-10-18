# Hydra
Heterogeneous, Yet Dynamic Drone.

## B-Aligned To‑Do Checklist

### Measure worst-case compute times per APE
- [ ] Log **max** wall-clock per event for APE1/APE2/APE3 (no averages needed).

### Set selector thresholds from those maxima
- [ ] Set 't_hard = APE1_max + 1 tick, t_med = APE2_max + 1 tick.'
- [ ] Keep APE3 as the “full-time” option beyond 't_med'

### Confirm event timing bounds
- [ ] Log inter-event gaps **Δt** and verify: **APE1_max < min(Δt)** and **APE3_max < max(Δt)**.
- [ ] If not true, adjust APE budgets or event rates until it is.

### Keep selector structurally independent
- [ ] No signaling/awaiting of APEs; selector only reads which proposals are ready and picks by the time bands:
- 'tl > t_med → prefer APE3 → APE2 → APE1'
- 't_hard < tl ≤ t_med → APE2 → APE1'
- 'tl ≤ t_hard → APE1'

### Guard against over-waiting
- [ ] With tl < one tick (~commit_guard), accept any ready proposal to avoid deadline misses.

### Record only the essential outcomes
- [ ] On first apply within window: log 'RESOLVED' with **resolver_ape** and **time_left.**
- [ ] If none applied by deadline: log 'DEADLINE_MISS'.

---