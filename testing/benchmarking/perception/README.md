# Perception benchmarking

[`perception_benchmark_tools`](perception_benchmark_tools/README.md) contains the
relocated offline terrain helper timing runner. It uses the synthetic generator
in `testing/mocks/perception/perception_test_tools`, or reads recorded XYZ points.
It is not a mission accuracy observer or full production-node validation.

Existing live synthetic feeders and integration checks remain in
`testing/mocks/perception/perception_test_tools`. Mission measurements and truth
interfaces still need agreement before adding an accuracy observer.
