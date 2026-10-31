"""Optional CUDA implementation; imported only when Triton is available."""

import torch
import triton
import triton.language as tl


@triton.jit
def _scan(
    types,
    ids,
    forces,
    counts,
    output,
    CAP: tl.constexpr,
    NV: tl.constexpr,
    FRICTION: tl.constexpr,
    BLOCK: tl.constexpr,
):
    world = tl.program_id(0)
    row = tl.program_id(1) * BLOCK + tl.arange(0, BLOCK)
    valid = (row < CAP) & (row < tl.load(counts + world))
    kind = tl.load(types + world * CAP + row, valid, other=-1)
    keep = valid & (kind == FRICTION)
    dof = tl.load(ids + world * CAP + row, keep, other=0)
    force = tl.load(forces + world * CAP + row, keep, other=0.0)
    tl.atomic_add(output + world * NV + dof, force, mask=keep)


def friction_force(types, ids, forces, counts, nv, friction_type):
    output = torch.zeros(types.shape[0], nv, device=forces.device, dtype=forces.dtype)
    if types.shape[0] and types.shape[1]:
        _scan[(types.shape[0], triton.cdiv(types.shape[1], 256))](
            types,
            ids,
            forces,
            counts,
            output,
            types.shape[1],
            nv,
            friction_type,
            256,
        )
    return output
