"""Shared production magnet pocket layout for the 30 x 30 mm forearm pad."""

POCKET_CENTERS: tuple[tuple[float, float], ...] = (
    (9.0, 9.0),
    (21.0, 9.0),
    (9.0, 21.0),
    (21.0, 21.0),
)

POCKET_ORDER = "row-major: low-Y row left-to-right, then high-Y row left-to-right"
