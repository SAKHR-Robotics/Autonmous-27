# PathPlanning

**Start with [info.md](info.md).** It is the single module handoff and operating
guide: specifications, packages, publisher/subscriber contracts, actions, clock/TF
ownership, launch arguments, required mocks, testing steps and pending work.

Production contains only real PathPlanner components. Independent testing uses
the existing `testing/mocks/pathplanning/pathplanner_mock_tools` package and runs
the real production planner with mocked dependencies.

Current checkpoint: builds and isolated mocked operation pass; boundary and
stale-input stopping gates fail, and Gazebo motion validation is blocked by EKF
drift. See [info.md](info.md#1-current-status-and-next-work) before testing.

The [validation record](VALIDATION.md) retains detailed evidence. Older `docs/`
and `Prop&Solu.md` are historical references, not the current runbook.
