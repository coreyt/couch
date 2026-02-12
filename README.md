# Unity-SOFA Ankle ROM Simulation

Surgical simulation of Total Ankle Replacement (TAR) using **Unity 6** for real-time visualization and **SOFA Framework** for biomechanically accurate FEM physics.

## Purpose

Simulate the before-and-after of ankle surgical intervention:

1. **Pre-Op ROM** - Measure arthritic ankle range of motion (22-31deg typical)
2. **Bone Resection** - Virtual tibial and talar bone cuts
3. **Implant Placement** - Position tibial, talar, and bearing components
4. **Post-Op ROM** - Measure post-operative range of motion (33-53deg typical)

## Documentation

| Document | Description |
|----------|-------------|
| [01 - Clinical TAR Research](docs/research/01-clinical-tar-research.md) | Surgical procedure, biomechanics, anatomy, material properties |
| [02 - Unity 6 Research](docs/research/02-unity6-research.md) | Physics, mesh API, rendering, plugin architecture |
| [03 - SOFA Framework Research](docs/research/03-sofa-framework-research.md) | FEM solvers, collision, cutting, Unity integration |
| [04 - Specialized Agents](docs/04-specialized-agents.md) | TAR Clinical, Unity, SOFA, and Integration expert definitions |
| [05 - User Needs](docs/05-user-needs.md) | 8 user needs with acceptance criteria |
| [06 - Software Architecture](docs/06-software-architecture.md) | Layered architecture, data flow, technology stack |
| [07 - Software Requirements](docs/07-software-requirements.md) | 55+ traceable requirements (ANAT, ROM, RES, IMP, SIM, INT, PERF, UI) |
| [08 - Component Designs](docs/08-component-designs.md) | 7 components with C# interfaces and data contracts |
| [09 - TDD Implementation Plan](docs/09-tdd-implementation-plan.md) | 10-sprint plan with 131 tests across EditMode, PlayMode, and SOFA |

## Technology Stack

- **Unity 6** (6000.x LTS) with URP
- **SOFA Framework** v25.12 (FEM physics backend)
- **SofaUnity** (InfinyTech3D) for in-process integration
- **EzySlice** (MIT) for runtime mesh cutting
- **3D Slicer** for DICOM-to-mesh pipeline
