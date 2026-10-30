# Preserved upstream workflows

These 24 workflow files are unchanged copies from the Copter-4.6.3 source.
They were moved out of `.github/workflows/` on this custom firmware branch so
each S2 change does not launch the full upstream build and simulation suite.
GitHub paused this fork's workflows because of Actions usage.

The active workflows on this branch are:

- `build-cubeorangeplus-s2.yml`: test and compile the modified S2 firmware.
- `build-cubeorangeplus-baseline.yml`: preserve the separate official-release
  build, triggered only when that workflow file changes.

To restore an individual upstream check, move its YAML file back into
`.github/workflows/` and review its events before pushing. These copies retain
their original contents; moving them does not change the firmware source.
Other branches retain their own workflow files and historical runs remain
visible on the repository's Actions page.
