"""Appearance-only vegetation veto on raw BGR, before any overlay is drawn."""
import cv2
import numpy as np


def green_vegetation_mask(frame, *, hue_min=25, hue_max=95, saturation_min=45,
                          value_min=20, excess_green_min=.08, dilation_px=2,
                          support_radius_px=0, support_fraction=.5):
    """Reject green appearance; optional support suppresses isolated colour noise.

    With support enabled, retain a whole connected green component only when
    at least one of its pixels has a sufficiently green neighbourhood. Keep
    all original edges/blades of a supported component, then dilate as before.
    This is not grass recognition: small isolated green objects may disappear
    from this colour heuristic. Never use it to filter semantic/obstacle vetoes.
    Radius is measured in the original capture, before detector downsampling.
    Zero radius preserves the original per-pixel behaviour.
    """
    if (type(support_radius_px) is not int or not 0 <= support_radius_px <= 12 or
            type(support_fraction) not in (int, float) or
            not np.isfinite(support_fraction) or not 0 < support_fraction <= 1):
        raise ValueError("invalid vegetation spatial-support settings")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    b, g, r = np.moveaxis(frame.astype(np.float32), -1, 0)
    excess = (2*g-r-b)/np.maximum(r+g+b, 1.)
    mask = ((hsv[:,:,0] >= hue_min) & (hsv[:,:,0] <= hue_max) &
            (hsv[:,:,1] >= saturation_min) & (hsv[:,:,2] >= value_min) &
            (excess >= excess_green_min)).astype(np.uint8)*255
    if support_radius_px:
        size = 2*support_radius_px+1
        density = cv2.boxFilter((mask > 0).astype(np.float32), -1, (size, size),
                               borderType=cv2.BORDER_CONSTANT)
        count, components = cv2.connectedComponents(mask, connectivity=8)
        supported = np.zeros(count, dtype=bool)
        supported[np.unique(components[(mask > 0) & (density >= support_fraction)])] = True
        supported[0] = False
        mask[~supported[components]] = 0
    if dilation_px:
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
            (2*dilation_px+1, 2*dilation_px+1)))
    return mask
