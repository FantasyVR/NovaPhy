---
name: benchmark
description: Run performance benchmarks and stress tests on NovaPhy's simulation
argument-hint: "[scenario]"
allowed-tools: Bash, Read, Write
---

Run performance benchmarks on NovaPhy to measure simulation throughput and detect regressions.

## Built-in Stress Test

Create and run a benchmark script that measures simulation performance:

```python
import time
import novaphy as np

def benchmark_stack(n_bodies, n_steps, dt=1.0/60.0):
    """Benchmark: N cubes stacking under gravity."""
    builder = np.ModelBuilder()
    builder.set_gravity([0, -9.81, 0])

    # Ground plane
    builder.add_shape(np.Plane())

    # Stack of boxes
    for i in range(n_bodies):
        b = builder.add_body(mass=1.0)
        builder.add_shape(np.Box(0.5, 0.5, 0.5), body_index=b, position=[0, 0.5 + i * 1.01, 0])

    model = builder.build()
    world = np.World(model)

    start = time.perf_counter()
    for _ in range(n_steps):
        world.step(dt)
    elapsed = time.perf_counter() - start

    return elapsed, n_steps / elapsed

# Run benchmarks
scenarios = [
    ("Small stack (10 bodies)",   10, 1000),
    ("Medium stack (100 bodies)", 100, 500),
    ("Large stack (500 bodies)",  500, 100),
    ("Stress test (1000 bodies)", 1000, 50),
]

print(f"{'Scenario':<30} {'Bodies':>6} {'Steps':>6} {'Time (s)':>10} {'Steps/s':>10}")
print("-" * 70)
for name, n, steps in scenarios:
    t, rate = benchmark_stack(n, steps)
    print(f"{name:<30} {n:>6} {steps:>6} {t:>10.3f} {rate:>10.1f}")
```

## Custom Scenarios

If `$ARGUMENTS` is provided, adapt the benchmark to that scenario:
- `"collision"` — Focus on broadphase/narrowphase with many overlapping bodies
- `"articulated"` — Benchmark articulated body chains of various lengths
- `"ipc"` — Benchmark IPC solver if available

## After Benchmarking

1. Report results in a clear table format
2. Compare against previous results if available
3. Identify bottlenecks (broadphase, narrowphase, solver, integration)
4. Suggest optimization targets if performance is below expected
