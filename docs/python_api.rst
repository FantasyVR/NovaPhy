Python API
==========

This page renders Python API docs via ``sphinx.ext.autodoc``.

Module Example
--------------

Profiling
---------

``World`` and ``FluidWorld`` expose a ``performance_monitor`` object for
opt-in runtime profiling.

Typical usage:

.. code-block:: python

   monitor = world.performance_monitor
   monitor.enabled = True
   monitor.trace_enabled = True

   for _ in range(60):
       world.step(1.0 / 120.0)

   for stat in monitor.phase_stats():
       print(stat.name, stat.avg_ms)

   for metric in monitor.last_frame_metrics():
       print(metric.name, metric.value)

   monitor.write_trace_json("build/trace.json")

.. automodule:: novaphy
   :members:
   :undoc-members:
   :show-inheritance:
