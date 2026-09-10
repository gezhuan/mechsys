# Research baseline 2026-09 and recovered-change audit

Documentation draft, 2026-09-10. Canonical tree: /home/gezhuan/mechsys.
No source/Git changes or solver execution performed by this architecture-audit task.

## Verified repository identity

- main / origin/main: 1eb8dccb6b6f8da628052c8e562157041841c345.
- Commit subject: Recover pre-Git research modifications.
- runnable-baseline-2026-09 is an annotated tag; tag object 75f0cef1eadb5501decb3769076348265c7b6f65.
- Local upstream/main and upstream/HEAD: 727a821c1db42fe870ed2e5dd64a7011648bceb7.
- origin: git@github.com:gezhuan/mechsys.git.
- upstream: https://github.com/Axtal/mechsys.git.
- Existing legacy remote branch/tag were observed and left unchanged.
- Working tree was clean at audit entry. Documentation drafts are intentionally untracked/uncommitted at delivery.

“upstream/main” here means the existing local ref. This task performed no fetch and makes no claim about today's live upstream tip.

```text
* 1eb8dccb (main, origin/main, runnable-baseline-2026-09)
| Recover pre-Git research modifications
* 727a821c (upstream/main, upstream/HEAD)
| user defined Re for tclbm01.cu
* 198245b1 bug corrected in DEM domain.h
```

## Complete change scope versus 727a821c

| File | Insertions / deletions | Change |
|---|---:|---|
| .gitignore | +3 / -0 | Comment, spacing, anchored /libstable/ ignore entry |
| lib/dem/domain.h | +73 / -13 | High report window, state initialization, validation, output scheduling/download logic |
| lib/lbmdem/Domain.h | +74 / -8 | Same coupled output control and ParticlesOnly fluid-download gate |

Total +150/-21. No other tracked source changes are in this baseline commit. lib/flbm, dem.cuh, lbmdem.cuh, collision/force/integration functions, examples and CMake remain at comparison-base content.

Read-only reproduction:
```bash
GIT_OPTIONAL_LOCKS=0 git -C /home/gezhuan/mechsys diff --no-ext-diff --no-textconv 727a821c1db42fe870ed2e5dd64a7011648bceb7 1eb8dccb6b6f8da628052c8e562157041841c345 -- .gitignore lib/dem/domain.h lib/lbmdem/Domain.h
```

## Modification-by-modification review

### .gitignore

The root-only /libstable/ rule removes the preserved upstream library copy from routine status; it neither deletes nor changes those files. The earlier recovery comparison found 123 files identical to the upstream lib snapshot. This directory is not the active mechsys include symlink target.

### DEM

- Declarations at [lib/dem/domain.h:128](../lib/dem/domain.h#L128); state fields at :193.
- Constructor initializes Enabled=false and all interval state to zero (:307).
- SetHighFrequencyOutput (:464) and ClearHighFrequencyOutput (:476) control the optional window.
- Solve validates dtOut, creates separate regular field/report and high schedules (:561).
- At each loop entrance, after Setup, due output downloads particles/vertices; contacts/history only when a regular event is due (:618).
- Energy-based stopping is checked for regular field events, not each high event.
- Report is called once for regular/high union; WriteXDMF/WriteBF remain regular-only.
- idx_out increments only with field output; schedule advances skip elapsed times.

### LBM–DEM

- Declarations/state at [lib/lbmdem/Domain.h:73](../lib/lbmdem/Domain.h#L73)/:98.
- Both constructors initialize new fields, including ParticlesOnly=true (:157, :208).
- SetHighFrequencyOutput (:551) adds the optional ParticlesOnly argument.
- Solve scheduling is at :702; every due event downloads DEM with force=true (:717).
- High-only ParticlesOnly events skip FLBM rho/u and Gamma download; ParticlesOnly=false requests these too.
- No torque/population download was added; no persistent hydro torque/moment fields were added.
- Built-in component output remains regular-only; final callback behavior is unchanged.

## Does it change physics?

**No numerical force, collision or integration equations were edited.** That is a diff-level conclusion, not a proof of identical trajectories for arbitrary callbacks/settings.

It changes execution behavior: additional Report invocations; more host synchronization; tolerance and skip-ahead timing; output-interval validation; host mirror/contact state updates. Since Report is mutable and may perform control, additional calls can change the simulation. CPU callbacks can modify live state; CUDA callbacks can launch device kernels. Even a read-only reporter requires fresh-data discipline.

The intended use is observational higher-frequency research output. It extends the existing callback architecture rather than creating a parallel physics/output engine. Coincident schedules coalesce into one callback; no deliberate double sampling at the same loop time. Callback-side heavy field reductions or std::endl flushing can still produce large overhead.

See [output architecture](mechsys_output_architecture.md) for exact scheduling, lifecycle, stale-data matrix and source-derived edge-case probes.

## Identities, periodicity and file contract

The baseline does not add IDs to examples, immutable particle IDs, image counters, unwrapped coordinates, metadata sidecars, checkpoint fields or new history filenames. Tag can repeat; Index is current array position. A future trajectory report must explicitly define these.

Time in a coupled report should be outer Domain::Time, not DEMDOM.Time or LBMDOM.Time. Sampled force and position are not guaranteed to describe the same force-evaluation configuration. For moments, retain the arm and force at imprint time.

## Baseline evidence / validation boundaries

- All tracked files were hashed before documentation work, including the symlink target, and rechecked afterwards.
- Aggregate entry fingerprint (JSON list of tracked path + content SHA-256 or symlink target):
  cac145430b99a2d4ad77ac6889f3579a27bdf674437c2f656ea92f4122dd611c.
- Modified header SHA-256 values:
  - lib/dem/domain.h: deb1ad94d51721ea83a5df2f2807c55b19db0905cacdebb51cc7c65a3f333e66.
  - lib/lbmdem/Domain.h: d6652811c48af351bac8c908a163c0eab47b27b004ab7638935b8841b066d797.
- The architecture audit traced the actual CPU/CUDA calls and buffer transfers. It did not rebuild, benchmark, run a simulation, validate a restart or execute a GPU race detector.
- Existing pre-management backup from the preceding recovery task:
  /home/gezhuan/mechsys-safety-backups/pre-git-management-20260910T060319Z/mechsys-complete.tar.gz.
  That archive describes the pre-commit repository state, not a newly generated archive of today's Git metadata.

The tag's “runnable baseline” name identifies the recovered research baseline. It is not a claim that every optional build mode or CPU/CUDA configuration is verified correct.

## Future-agent handoff

Start with [architecture](mechsys_architecture.md), then [coupling](lbmdem_execution_flow.md), [parity](cpu_cuda_parity.md) and [development map](mechsys_development_map.md). Preserve the baseline. Define diagnostics before altering force laws. Do not promote an unvalidated discrete moment to a physical stresslet or treat visualization files as exact restart state.
