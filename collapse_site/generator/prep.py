"""Clean the top-down screenshot: remove hotspot overlays, build a valid mask."""
import cv2, numpy as np
im = cv2.imread('reference/top.png', cv2.IMREAD_COLOR).astype(np.float32)
H, W = im.shape[:2]
b = im.mean(2); sat = im.max(2) - im.min(2)
yy, xx = np.mgrid[0:H, 0:W]
guess = [(370,72),(618,199),(670,86),(661,197),(642,270),(305,231)]
fits = []
for (gx, gy) in guess:
    best = None
    for dx in range(-4, 5):
        for dy in range(-4, 5):
            cx, cy = gx+dx, gy+dy
            r = np.hypot(xx[cy-35:cy+36, cx-35:cx+36]-cx, yy[cy-35:cy+36, cx-35:cx+36]-cy)
            p = b[cy-35:cy+36, cx-35:cx+36]
            for r0 in np.arange(20, 27, 0.5):
                on = p[np.abs(r-r0) < 1.2].mean(); off = p[(np.abs(r-r0) > 3) & (np.abs(r-r0) < 6)].mean()
                sc = on - off
                if best is None or sc > best[0]: best = (sc, cx, cy, r0)
    fits.append(best[1:]); print('hotspot', best)
dark = np.zeros((H,W), bool); hole = np.zeros((H,W), bool)
med = cv2.medianBlur(b.astype(np.uint8), 9).astype(np.float32)
for (cx, cy, r0) in fits:
    r = np.hypot(xx-cx, yy-cy)
    dark |= r < r0 - 1.5
    hole |= np.abs(r - r0) < 4.0                                   # ring
    hole |= (r < 15) & (((b > med + 30) & (sat < 80)) | (b > 160))  # numeral
    hole |= (np.abs(r - r0) < 8) & (b > med + 22) & (sat < 60)      # ring halo residue
hole = cv2.dilate(hole.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))).astype(bool)
out = im.copy()
k = dark & ~hole
out[k] = np.clip(im[k] * 2.0, 0, 255)
o8 = np.clip(out, 0, 255).astype(np.uint8)
inp = cv2.inpaint(o8, hole.astype(np.uint8), 7, cv2.INPAINT_TELEA).astype(np.float32)
# texture noise for inpainted pixels from a far, hole-free neighbourhood
hf = out - cv2.GaussianBlur(out, (0,0), 2.0)
hf[hole] = 0
src = np.roll(hf, (-61, 73), axis=(0,1)); srcm = np.roll(hole, (-61, 73), axis=(0,1))
src[srcm] = 0
m = cv2.GaussianBlur(hole.astype(np.float32), (0,0), 1.0)[..., None]
clean = np.clip(inp + m*src*0.9, 0, 255).astype(np.uint8)
valid = (b > 12).astype(np.uint8)
n, lab, stats, _ = cv2.connectedComponentsWithStats(valid, 8)
valid = (lab == 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])).astype(np.uint8)
valid = cv2.morphologyEx(valid, cv2.MORPH_CLOSE, np.ones((9,9),np.uint8))
ff = valid.copy(); h2 = np.zeros((H+2,W+2),np.uint8); cv2.floodFill(ff, h2, (0,0), 1)
valid = valid | (1-ff)
valid = cv2.erode(valid, np.ones((5,5),np.uint8))
cv2.imwrite('build/ortho_clean.png', clean)
cv2.imwrite('build/valid.png', valid*255)
crops = np.hstack([clean[cy-40:cy+40, cx-40:cx+40] for cx,cy,_ in fits])
cv2.imwrite('build/hotspot_check.png', cv2.resize(crops, None, fx=3, fy=3, interpolation=cv2.INTER_LANCZOS4))
