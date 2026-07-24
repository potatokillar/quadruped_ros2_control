# Repository Instructions

## Experiment Records

Only a run explicitly designated as an experiment by the user, or performed
after the user confirms an experiment plan, is an experiment. Builds, smoke
tests, functional tests, debugging captures, and routine validation are not
experiments by default.

Every experiment that produces data must have one dedicated experiment report,
including failed, aborted, and invalid experiments.

- Store experiment reports in the Obsidian vault at
  `/home/wl/docs/wl-researchs/运控/实验报告/`. Do not store experiment reports or
  report templates in this code repository.
- Follow `/home/wl/docs/wl-researchs/运控/实验报告规范.md`.
- Create the report before starting another experiment.
- Use a unique experiment ID and record the exact raw-data path, Git revision,
  dirty-worktree files, controlled variable, procedure, quantitative evidence,
  anomalies, and conclusion.
- Do not create a formal experiment report for an ordinary test unless the user
  later designates that test as an experiment.
