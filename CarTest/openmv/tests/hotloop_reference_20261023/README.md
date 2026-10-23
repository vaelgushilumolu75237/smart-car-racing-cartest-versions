# Pre-hot-loop reference source

These two modules preserve the `fast_path.py` and `fast_topology.py` runtime
source immediately before the 2026-10-23 hot-loop optimization. They are test
and future board-differential inputs only; the production uploader includes
the modules under `openmv/`, not this directory.

The snapshot reverses only the candidate-record center cache in `fast_path.py`
and the six-bit visited-map epoch in `fast_topology.py`. It keeps the active V2
branch's other source and settings, including `SURFACE_ROI_FLOOR_Y = 8` and the
four-row/eight-span support floor. The independent test imports the reference
path detector beside the optimized detector and compares complete outputs and
connect/select state for the same input masks.

SHA-256:

| File | SHA-256 |
| --- | --- |
| `fast_path.py` | `20d2f66b00a426b1596c77eed344f67e85f0f6153de813989541b617ba1b3018` |
| `fast_topology.py` | `218ac677002290c055a813414fe306b0c4fa3838b3b93ae788a925ea189751e8` |

The host differential is not board timing evidence. A board comparison should
use the same camera, configuration, input scene, protocol package and capture
procedure, changing only these two source modules.
