"""Exercise masking, stale rows and duplicate DOFs without a simulator."""

import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("triton")

from bam._mjlab_friction import friction_force


@pytest.mark.skipif(not torch.cuda.is_available(), reason="Requires CUDA")
@pytest.mark.parametrize("capacity", [0, 17, 257, 1504])
def test_friction_scan(capacity):
    device = "cuda"
    types = torch.full((3, capacity), 1, dtype=torch.int32, device=device)
    ids = torch.full_like(types, 99999)
    forces = torch.ones((3, capacity), device=device)
    counts = torch.tensor(
        [0, capacity, capacity // 2], dtype=torch.int32, device=device
    )
    expected = torch.zeros(3, 4, device=device)
    for world in (1, 2):
        count = int(counts[world])
        ids[world, :count] = 2
        expected[world, 2] = count
    actual = friction_force(types, ids, forces, counts, 4, 1)
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    types.fill_(6)
    torch.testing.assert_close(
        friction_force(types, ids, forces, counts, 4, 1), torch.zeros_like(expected)
    )
