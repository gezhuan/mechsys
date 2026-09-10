# CPU/CUDA parity, synchronization and risk register

Draft at 1eb8dccb6b6f8da628052c8e562157041841c345, 2026-09-10.
Canonical root: /home/gezhuan/mechsys. Static audit; no GPU workload or sanitizer run.

## CUDA launch hierarchy

DEM::Domain::Solve ([lib/dem/domain.h:664](../lib/dem/domain.h#L664)):
pReset -> pForceVV -> pForceEE -> pForceVF -> pForceFV -> pTranslate -> pRotate -> MaxD -> thrust::max_element -> optional UpdateContactsDevice.

LBMDEM::Domain::Solve ([lib/lbmdem/Domain.h:759](../lib/lbmdem/Domain.h#L759)):
cudaReset -> DEM pReset -> four DEM contact kernels -> active cudaImprintLatticeVC -> cudaImprintLatticeFC -> DEM pTranslate/pRotate -> MaxD/reduction -> optional hybrid rebuild -> FLBM cudaCollideSCDEM -> pointer swap -> FLBM cudaStream1 -> pointer swap -> coupled cudaStream2.

The default single-stream launch order provides ordering between these kernels. Many explicit cudaDeviceSynchronize calls are commented out; this does not mean device-to-host copies or host-read reductions are asynchronous/free. Thrust host-vector assignment and reading the maximum introduce synchronization/data transfer.

Function pointers are initialized in DEM constructor ([lib/dem/domain.h:321](../lib/dem/domain.h#L321)); ContactLaw chooses the sphere kernel. Examples can replace kernels and use pExtraParams. Audit an executable's actual callbacks/kernel substitutions before applying default-path conclusions.

## Host/device field availability

| Field | Device location | Normal host download | Consequence |
|---|---|---|---|
| x, xb, v, w, wa, wb, Q | DynParticleCU | Yes, [lib/dem/particle.h:1293](../lib/dem/particle.h#L1293) | Current after download, with integration-phase caveat |
| F / Flbm | DynParticleCU | Yes | Total / hydro forces accessible in host Report |
| T | ParticleCU | **No** | Host total torque stale; cannot serve as current CUDA hydro torque |
| Tag / Index | ParticleCU plus host original | No dynamic refresh | Host identifiers valid only while ordering/identity unchanged |
| fixed flags, Ff, Tf, Flbmf, mass/inertia | ParticleCU plus host original | No dynamic refresh | Device-only constraint/load changes not automatically reflected on host |
| vertices | bVertsCU | Yes, even force=false | Particle-only output still incurs geometry copy |
| edge/face derived geometry | From vertices | Recomputed on host | Download has CPU work and writes host geometry |
| contact normal/tangential summaries | ComInteractonCU | Only force=true | force=false reports must not use them as current |
| contact tangential/rolling history | DynInteractonCU | Only force=true; converted back to host histories | This is mutable history synchronization, not raw force-only telemetry |
| fluid rho/u | bRho/bVel | FLBM download | Current only when that routine is called |
| distributions / BForce / masks | pF/pFtemp/pBForce/pIsSolid | No normal refresh | Host population-based diagnostics invalid without additional transfer |
| Gamma | pGamma | Coupled download | Refreshed on regular or explicitly full-fluid high output |
| Omeis / Gammaf / inside fields | Coupling buffers | No normal refresh | Not a retained node-force history |
| node arm, node force, hydro-only torque | Kernel locals | No | Capture inside imprint if needed |

ParticleCU/DynParticleCU definitions: [lib/dem/particle.h:1190](../lib/dem/particle.h#L1190) and :1215.
Contact device structures: [lib/dem/interacton.h:1039](../lib/dem/interacton.h#L1039).
Transfers: [lib/dem/domain.h:2849](../lib/dem/domain.h#L2849)/:3173, [lib/flbm/Domain.h:2019](../lib/flbm/Domain.h#L2019)/:2151, [lib/lbmdem/Domain.h:917](../lib/lbmdem/Domain.h#L917)/:1032.

## Hybrid neighbor rebuild

MaxD computes vertex displacements and advances the device DEM auxiliary clock ([lib/dem/dem.cuh:642](../lib/dem/dem.cuh#L642)). Host Solve separately advances its demaux copy. UpdateContactsDevice (:3302 in domain.h) wraps/reset-reference vertices on device, downloads dynamic particles and contact histories, rebuilds CPU contacts, then uploads contact records/auxiliary state. It does not upload a fresh full ParticleCU/DynParticleCU array on every rebuild: that occurs only on first=true.

In coupled Solve the outer Time is authoritative for callbacks. DEMDOM.Time and LBMDOM.Time are not visibly advanced alongside outer Time in the coupled loop; device auxiliary clocks are separate. Always label a coupled diagnostic with the outer LBMDEM::Domain::Time and an explicit force phase.

## Proven code differences

| Difference | Evidence | Implication |
|---|---|---|
| CPU coupled contact-law argument omitted | lbmdem/Domain.h:845 calls CalcForce(dt,Per,iter); Interacton default at interacton.h:50 is 0 | ContactLaw=1 setup does not reach CPU sphere CalcForce as intended; CUDA chooses Hertz kernel in constructor |
| Sphere coverage cutoff differs | CPU lbmdem/Domain.h:313 versus CUDA lbmdem.cuh:415 | Boundary-shell nodes can contribute on CUDA but be skipped on CPU |
| Fconv only in CUDA sphere force | lbmdem.cuh:441; absent CPU :395 and face :537 | Non-unit Fconv breaks expected shape/backend scaling parity |
| Overlap function configurable only for CUDA sphere and collision | pfBn call at Domain.h:777/:817; face hard-coded at lbmdem.cuh:527 | Alternate Bn needs coordinated CPU/face work |
| Prescribed occupancy reset differs | CPU Domain.h:525 uses IsSolid; CUDA lbmdem.cuh:133 uses Gammaf for fluid nodes | Nonzero Gammaf has different behavior |
| Hydro baseline reset differs | CPU Domain.h:536 zeros Flbm; CUDA dem.cuh:630 copies Flbmf | Hydro-only totals differ if Flbmf is nonzero |
| CPU body damping omitted by default CUDA | particle.h:681/:743 versus dem.cuh:524/:549 | Gv/Gm nonzero is not a parity configuration |
| RotPar behavior differs | DEM CPU Solve :758 versus unconditional CUDA :686 | RotPar=false is not equivalent |
| Contact shape/bond coverage | CPU interaction feature calls at interacton.h:295; CUDA upload domain.h:2970 | CUDA supports selected collision geometry; no equivalent full bond/torus/cylinder kernel set |
| Positivity policy differs | FLBM CPU Domain.h:1438; CUDA lbm.cuh:265 | CPU unbounded retry with -1e-12 threshold; CUDA max two passes and <0 threshold |
| Final host state differs | DEM domain.h:795 precedes final download; coupled Domain.h:905 lacks final download | Final callback can see stale data |

These differences predate the recovered high-frequency patch except its new scheduling/download choices. They should not be silently “fixed” as part of a diagnostic-only change.

## Concurrency and safety findings

**Critical, conditional: shared-node occupancy race.** CPU ImprintLattice compares/writes Gamma and Omeis outside the particle lock ([lib/lbmdem/Domain.h:374](../lib/lbmdem/Domain.h#L374)–394). CUDA sphere/face paths perform non-atomic shared Gamma/Omeis writes (lbmdem.cuh:421/:440 and :521/:536). Two candidate particles covering the same node can race within a kernel/CPU parallel loop. Particle force atomics/locks do not protect node ownership. Even sequential max-selection can accumulate earlier particle force contributions later superseded in node occupancy. A dense-suspension conservation/parity test is required.

**Critical, conditional: bond records treated as collision records.** CUDA upload derives a CInteracton through PairtoCInt for every Interactons entry (domain.h:2986–2991), while active Interactons can include BInteracton (:2414). The download also static_casts active entries to CInteracton (:3279). Bonded CUDA configurations are not established safe/equivalent by this implementation.

**Important: host contact-history updates can race.** DnLoadDevice(force=true) parallelizes feature records while assigning feature-keyed maps on shared CInteracton objects (domain.h:3228–3273). Multiple features for the same particle pair require an ownership/thread-safety review; unlike particle accumulation, these writes are not protected by a pair lock. This is a source-level risk, not a measured sanitizer result.

**Important: rolling/history unit conversions deserve validation.** Sphere upload uses Fdr*Kt*beta (:3065), whereas CPU sphere CalcForce advances Fdr with Kr*Vr*dt (interacton.h:717). Download divides device Fr by Kt and beta (:3219). A rolling history round-trip test is needed; do not assume host Fdr is simply a displacement.

**Important: diagnostics can synchronize every timestep.** max_element is already on the normal path; high-frequency report adds host-vector allocations, dynamic particles, vertices and optionally multiple contact arrays. ParticlesOnly in coupled mode still requests force=true contact downloads.

**Minor/expected: atomic reduction order.** Particle force/torque atomics prevent lost per-component additions but do not make floating-point summation bitwise deterministic.

## Minimum parity validation matrix (proposed, not executed)

1. One free sphere with prescribed Ff; zero contacts/fluid; verify x/v against CPU and constraints.
2. Two spheres, linear and Hertz separately; tangential history, rolling and rebuild transitions.
3. One nonspherical contact and one nonspherical fluid-coupled particle; verify supported feature paths.
4. Fixed sphere drag: same mesh/nu/dx/dt/Sc/forcing and Fconv=1 on both backends.
5. Single sphere across periodic face/edge/corner; compare Gamma, force and minimum-image torque.
6. Two nearby particles sharing nodes, multiple OpenMP thread counts and CUDA runs; measure force/fluid-momentum consistency.
7. Output off versus high-frequency read-only callback; compare device trajectories/checksums without using stale host observations.
8. Explicit device total torque copy versus normal host T, and separate hydro torque sum.
9. Contact rebuild before/after round-trip histories; especially beta!=1 and Hertz.
10. All checks must use harmonized examples; shipped CPU/CUDA examples differ in physical parameters.
