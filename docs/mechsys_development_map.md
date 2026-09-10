# MechSys research development map

Documentation draft, 2026-09-10, for canonical /home/gezhuan/mechsys at 1eb8dccb6b6f8da628052c8e562157041841c345.
These are proposed insertion points and validation cases, not implemented changes. Read the [master map](mechsys_architecture.md), [coupling equations](lbmdem_execution_flow.md), [output contract](mechsys_output_architecture.md) and [CPU/CUDA evidence](cpu_cuda_parity.md) first.

## Minimal ownership and sampling contract

Keep solver state in its existing owner. Put coupling diagnostics in LBMDEM::Domain, not in a second particle implementation or a replacement fluid solver. Optional arrays indexed by current particle Index are sufficient for an initial research implementation if particle order is fixed and the output explicitly records that assumption. Introduce permanent IDs only when insertion/deletion/reordering requires them.

For every quantity document units, force-on-particle sign, world/body frame, periodic arm, force-evaluation time and output time. Compute node-derived aggregates at imprint time. Reset once before imprint, accumulate in both CPU and CUDA paths, transfer only requested compact aggregates, and write in Report with an independent sample counter. Avoid calling CalcForce or ImprintLattice a second time to inspect them: they modify histories, forces and lattice state.

The current output event occurs before the current force stage. An aggregate retained from the previous stage must carry that stage's time/iteration; it must not silently be paired with the current particle geometry. Use the outer coupled Time, not embedded subdomain clocks. Keep diagnostics optional so baseline runs can demonstrate unchanged trajectories.

## Feature insertion map

| Feature | Best module and owner | CPU insertion | CUDA insertion | Output integration | Minimum proposed validation |
|---|---|---|---|---|---|
| High-frequency trajectory | Report/UserData owns stream and sample counter; DEM owns kinematics and any future image counters | Read refreshed x/v/Q/w at Solve report boundary; image counters would need updates at wrapping in DEM::Domain::ResetDisplacements | Existing dynamic download; image counters need device ResetMaxD wrapping support and compact download | Existing high window; write time, stable identity policy, wrapped/unwrapped flag and frame; do not use idx_out as high-sample ID | One particle crossing a periodic face and corner; compare output-enabled/disabled integration |
| Contact diagnostics | DEM::Domain diagnostic arrays or records keyed by pair/features; contact objects retain histories | Capture final per-feature force/arm inside CInteracton force routines, without reevaluation | Capture inside VV/EE/VF/FV kernels and reduce to pair records | Report after appropriate contact refresh; WriteBF is an elastic-summary visualization reference | Two-sphere oblique contact with damping/friction and a neighbor rebuild |
| Contact stress | DEM::Domain owns per-particle/global accumulator and normalization metadata | Use contact force on a specified endpoint and consistent minimum-image branch in interacton.h; include required damping/bond terms explicitly | Same endpoint convention in contact kernels; compact tensor reduction | Dedicated Report columns/datasets with volume/sign convention | Two particles with known force/branch; periodic translation invariance; symmetric pair counting |
| Hydrodynamic force | Existing Particle::Flbm / DynParticleCU::Flbm | ImprintLattice accumulation, Domain.h:403 | Active VC and FC force atomics | Existing download + Report; PHForce visualization | Harmonized fixed-sphere CPU/CUDA drag and fluid-momentum balance |
| Hydrodynamic torque | New coupling-owned world-frame T_h per particle; keep it separate from total body torque | Save B cross F immediately before inverse rotation at Domain.h:397 | Save local world torque before inverse rotation at lbmdem.cuh:443/:539 | Copy new compact T_h array; current host T is not a valid CUDA substitute | Fixed-sphere symmetry then rotating-sphere torque convergence |
| First hydrodynamic moment | Coupling-owned 9-component M per particle | B and Flbm local contribution after Domain.h:395, before torque rotation | Local B and Flbm after lbmdem.cuh:441/:537 in both active kernels | Optional compact tensor download; record force-phase time | Sum explicit one-step node pairs; torque identity and origin-shift identity |
| Stresslet | Derived diagnostic from M plus an explicitly stated physical convention | Symmetric/deviatoric algebra after reduction | Reuse downloaded M initially; no second device field needed | Report Sym(M)-trace(M)I/3 as a discrete coupling moment until physically validated | Algebraic trace/symmetry checks; isolated sphere in imposed strain with resolution/domain convergence |
| Lubrication correction | Coupling-domain pair model; explicit parameters/cutoff, separate F_lub/T_lub | Pair stage after hydrodynamic imprint and before particle integration; ensure candidate range covers cutoff | Dedicated pair kernel in same stage, equal/opposite atomics, supported geometry contract | Separate diagnostic totals; never hide correction in an unlabeled drag scalar | Two-sphere normal approach/separation, gap sweep, resolved-grid overlap/double-counting study |
| Drag decomposition | Coupling-owned labeled contributions; node debug records optional | Instrument existing Omeis-derived transfer first; pressure/viscous split needs a defined reconstruction beyond net exchange | Equivalent VC/FC instrumentation; selective debug transfer | Report net and explicitly defined components | Fixed sphere across grid refinement; sum components back to net force |
| Thermal/frictional heating | Contact dissipation owner in DEM; temperature/heat balance requires a new explicit model | Capture dissipative force/work and rolling terms during actual contact evaluation; existing energy fields are not a complete heat equation | Match every supported contact kernel and integrate power consistently | Time-integrated energy/heat balance in Report | Controlled sliding pair with known frictional work, no negative dissipation artifacts |
| Dense-suspension diagnostics | Coupling/DEM aggregate owner for occupancy, coordination, force moments and volume averages | Reuse force-stage aggregates and contact geometry; settle shared-node ownership before interpreting results | Reductions after imprint/contact stages; avoid full lattice transfers for particle statistics | Buffered histories at data frequency, sparse field frames | Two particles sharing nodes, periodic small suspension, thread/backend and conservation checks |

Specific extension anchors: DEM::Domain::ResetDisplacements at [lib/dem/domain.h:2186](../lib/dem/domain.h#L2186); device ResetMaxD at [lib/dem/dem.cuh:659](../lib/dem/dem.cuh#L659); contact force implementation at [lib/dem/interacton.h:259](../lib/dem/interacton.h#L259)/:593/:937; CUDA contact kernels at [lib/dem/dem.cuh:75](../lib/dem/dem.cuh#L75)/:168/:264/:350/:437. Coupled reset is [lib/lbmdem/Domain.h:518](../lib/lbmdem/Domain.h#L518); Solve stages are :759–903; device coupling reset is [lib/lbmdem/lbmdem.cuh:127](../lib/lbmdem/lbmdem.cuh#L127). See execution documents for callers and modified fields.

Lubrication and heating are physics changes, not telemetry. No named lubrication correction or persistent hydro moment/stresslet was found in the active primary coupling path. Separate FLBM thermal/transport modes do not establish a contact-to-temperature energy coupling. These features require formulation and validation before implementation; this audit does not prescribe a new force law.

## Moment contract to preserve

For force on the particle, at the same evaluation instant:

```text
M_ab = sum_nodes B_a * Fnode_b
Sym = (M + transpose(M))/2
Anti = (M - transpose(M))/2
S_discrete = Sym - trace(M)*I/3
T_world = (M_yz-M_zy, M_zx-M_xz, M_xy-M_yx)
M_about_shifted_origin = M - a outer F_total
```

B is the implementation's minimum-image particle-to-node arm, not an arm reconstructed from output coordinates after integration. The identity with torque is a strong implementation check. A physical surface-traction stresslet may require additional terms/conventions; node momentum exchange alone does not provide surface area, normals and resolved traction quadrature. Do not promise Level 6 by merely dumping node forces.

## Representative examples and execution paths

These are inspected architecture references, not newly executed tests. “Minimal” means the simplest useful existing setup; some shipped grids are expensive.

| Role | Example and actual path | Validation limit |
|---|---|---|
| DEM reference | [tdem/test_01.cpp:32](../tdem/test_01.cpp#L32) creates a sliding cube and fixed plane, assigns gravity/materials, calls Solve at :60; DEM Initialize -> contact detection -> force -> translation/rotation -> output | Prints stopping distance, returns success without a strong automated assertion |
| DEM dynamics check | [tdem/test_dynamics.cpp:22](../tdem/test_dynamics.cpp#L22) constructs periodic rigid bodies; checks momentum/energy at :102 | Useful integrator reference; not an LBM or contact-history parity suite |
| Legacy LBM reference | [tlbm/tlbm01.cpp:76](../tlbm/tlbm01.cpp#L76) creates D2Q9 channel/obstacle; Setup at :36 reconstructs inlet/outlet populations; LBM::Domain::Solve collides/streams and reports | Uses old Cell/Lattice family; do not implement current coupled diagnostics here |
| Current LBM reference | [tflbm/tflbm01.cpp:84](../tflbm/tflbm01.cpp#L84) -> FLBM::Domain, boundary Setup :38 -> Solve :143; CUDA [tflbm/tclbm01.cu:95](../tflbm/tclbm01.cu#L95) uses boundary kernels and Solve :156 | Host/device boundary callbacks differ; harmonize parameters before parity |
| CPU coupled reference | [tlbmdem/tlbmdem01.cpp:94](../tlbmdem/tlbmdem01.cpp#L94) fixed sphere; Setup :35 applies fluid forcing; Report :47 computes drag-related values; coupled Solve maps nodes, imprints, integrates and streams | Report traverses fluid and total torque; force validation is primary |
| CUDA coupled reference | [tlbmdem/tlbmdem_cu_01.cu:95](../tlbmdem/tlbmdem_cu_01.cu#L95) fixed sphere; forcing kernel/Setup :35/:42; Report :48; same orchestrator's CUDA branch | Grid and acceleration differ from CPU case; host torque is stale |
| Moving-force reference | [tlbmdem/tlbmdem02.cpp:67](../tlbmdem/tlbmdem02.cpp#L67) and [tlbmdem/tlbmdem_cu_02.cu:67](../tlbmdem/tlbmdem_cu_02.cu#L67); Report :47 compares norm(Flbm) to 6*pi*nu*R*speed | Density convention is implicit in this expression; shipped viscosity/radius/forcing/walls differ |
| Buoyancy reference | [tlbmdem/tlbmdem_cu_03.cu:94](../tlbmdem/tlbmdem_cu_03.cu#L94) initializes a hydrostatic field and fixed sphere; Report :73 compares vertical hydro force with displaced-fluid weight | Release-kernel call is commented; do not describe this as a verified falling-sphere test |
| Torque reference | No ready analytical torque-validation case found among the inspected primary coupled examples | Start with zero torque by fixed-sphere symmetry; add prescribed rotation with validated torque telemetry later |
| Snapshot reference | [tdem/test_write.cpp:27](../tdem/test_write.cpp#L27) and [tdem/test_read.cpp:28](../tdem/test_read.cpp#L28) | Demonstrate geometry/particle persistence, not full restart equivalence |

The older [tlbm/magnus.cpp:35](../tlbm/magnus.cpp#L35) is not a substitute for a torque test: it sets up legacy disks, starts angular velocity at zero in the inspected setup and does not pass its Report function to Solve. CUDA example Report reading T is also not evidence of a correct current torque download.

## Prioritized correctness and performance register

Severity below reflects potential consequence when the stated configuration is used, not measured prevalence. No runtime fault or performance measurement is claimed.

| Severity | Finding and evidence | Practical implication / next check |
|---|---|---|
| Critical, conditional | Shared Gamma/Omeis writes in CPU/CUDA imprint; Domain.h:374–394 and lbmdem.cuh:421/:521 | Overlapping particle-node candidates can race or leave inconsistent occupancy/force attribution; resolve ownership and test momentum conservation before dense-suspension conclusions |
| Critical, conditional | CUDA active interaction upload/download assumes collision records although bonds may be active; dem/domain.h:2991/:3279 | Bonded CUDA configurations need type-safe coverage before being considered supported |
| Critical, conditional | FLBM constructor reads Nl/Solver before assignments; flbm/Domain.h:461/:470/:480 | Undefined behavior in construction; isolate with compiler/sanitizer checks in a future fix task |
| Critical, conditional | WriteXDMF_DEM allocates floor(Ndim/Step) blocks but iterates stepped original dimensions; flbm/Domain.h:790 onward | Non-divisor Step can overrun arrays/read grid bounds; keep Step=1 or validated divisors until fixed |
| Important | CUDA host T not copied; high-only fluid/contact freshness differs; particle.h:1293 and download routines | A plausible report can contain stale quantities; establish field-specific freshness contract |
| Important | ContactLaw, shell cutoff, Fconv, Gammaf, damping and rotation CPU/CUDA asymmetries | Do not infer parity from matching class names; see dedicated matrix |
| Important | force=true download parallel feature-map writes; dem/domain.h:3228–3273 | Pair-level map ownership needs review under multiple feature contacts/OpenMP |
| Important | MostlySpheres fast path skips object creation while later indexing collision objects; dem/domain.h:2373/:2399 | Do not enable as a harmless speed flag; test first; alternate force path also handles periodicity differently |
| Important | Save/Load omits integration/contact/fluid state; dem/domain.h:1656/:1842 | Snapshot is not an exact checkpoint; avoid claiming reproducible resumed trajectories |
| Important | Wrapped trajectories have no persistent image counters; force and output geometry have different phases | Long-time displacement and moment reconstruction can be wrong without explicit metadata |
| Important | High schedule bounds scheduled time rather than current Time; tiny/nonfinite increments not robust | Window overshoot or nonadvancing scheduler possible; tests detailed in output document |
| Important | Coupled repeated Solve appends free/fixed lists and allocates thread state; Domain.h:599 onward | Repeated Solve lifecycle is not equivalent to a freshly constructed simulation |
| Important | Particle/vertex/contact transfers, thrust host reductions and temporary allocation in output/rebuild | High-frequency reporting can synchronize large data repeatedly; profile with matched output workloads |
| Important | CPU positivity retry unbounded; CUDA retry policy differs | Extreme states may hang/diverge differently; validation needs stressed populations and clear failure policy |
| Minor | std::endl per RES row; HDF5 create/flush/close per frame and temporary field arrays | Buffer histories and keep sparse field output when implementing a measured optimization |
| Minor | HDF5 group/destructor cleanup incomplete | Repeated snapshots/domain lifetimes can leak resources; relevant to long-running embedding |
| Minor | Floating-point atomics are order dependent | Use tolerance-based conservation/parity metrics, not bitwise CPU/GPU equality |
| Acceptable research shortcut | Public mutable callback state, header implementations, per-run fixed particle order | Document contracts; avoid a framework rewrite solely for diagnostics |
| Acceptable research shortcut | File-per-frame visualization, default Step=1 and manually chosen examples | Adequate for controlled runs if scale and validation limits are recorded |
| Acceptable research shortcut | Hard-coded grids/forcing and example analytical normalizations | Preserve example context; do not present unmatched examples as a controlled parity test |

No recurring ASCII write was found inside the active node/contact numerical loops traced here; ordinary ASCII histories are in callbacks. Error/debug branches and arbitrary application callbacks can write. High-frequency callbacks execute on loop events and can make previously occasional output effectively per-step. File count and transfer cost, not only text formatting, dominate the architecture concerns.

## Suggested development sequence

1. Preserve this tag and use these documents as the navigation baseline. Record the exact executable, flags, compiler/device and physical parameters with each future run.
2. Establish small harmonized CPU/CUDA force, contact and periodic tests. Treat the conditional critical findings as separate correction tasks rather than changing them unnoticed in a diagnostic patch.
3. Implement optional hydro-only torque and first-moment aggregates together at the existing node contribution sites. Begin with compact per-particle arrays, explicit reset and transfer; avoid modifying unrelated particle ABI.
4. Verify zero diagnostic side effects, node-sum reconstruction, torque/antisymmetric identity, origin shift and periodic invariance. Only then add the symmetric-deviatoric output and physical validation.
5. Add buffered trajectories/contact histories with independent IDs/sample counters and clear force-phase timestamps. Extend checkpoint support only under an explicit exact-restart contract.
6. Consider lubrication, drag reconstruction and thermal coupling in separate physics work after force accounting and CPU/CUDA coverage are established.

This task has implemented none of these proposals. The smallest useful next development is a validated coupling diagnostic accumulator and its output contract, not a redesign of DEM, FLBM or Git management.
