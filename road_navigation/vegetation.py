"""Appearance-only vegetation veto on raw BGR, before any overlay is drawn."""
import cv2
import numpy as np


def green_vegetation_mask(frame, *, hue_min=25, hue_max=95, saturation_min=45,
                          value_min=20, excess_green_min=.08, dilation_px=2):
    """Reject green-dominant pixels; does not recognize all grass or certify ground."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    b, g, r = np.moveaxis(frame.astype(np.float32), -1, 0)
    excess = (2*g-r-b)/np.maximum(r+g+b, 1.)
    mask = ((hsv[:,:,0] >= hue_min) & (hsv[:,:,0] <= hue_max) &
            (hsv[:,:,1] >= saturation_min) & (hsv[:,:,2] >= value_min) &
            (excess >= excess_green_min)).astype(np.uint8)*255
    if dilation_px:
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
            (2*dilation_px+1, 2*dilation_px+1)))
    return mask
