import sys
from PIL import Image, ImageDraw, ImageFilter, ImageFont
SRC, OUT = sys.argv[1], sys.argv[2]
im = Image.open(SRC).convert("RGB").crop((0, 0, 1280, 720)).resize((1920, 1080), Image.LANCZOS)
S = 1.5
P = lambda x, y: (x * S, y * S)
mast, d1, d2, d3 = P(810, 52), P(160, 205), P(372, 448), P(545, 462)
pile_phone = P(330, 120)
TEAL, BLUE = (43, 210, 190), (90, 150, 255)
glow = Image.new("RGBA", im.size, (0, 0, 0, 0)); g = ImageDraw.Draw(glow)
sharp = Image.new("RGBA", im.size, (0, 0, 0, 0)); s = ImageDraw.Draw(sharp)

def link(a, b, col, dash=False):
    g.line([a, b], fill=col + (150,), width=18)
    if dash:
        import math
        L = math.dist(a, b); n = int(L // 22)
        for k in range(n):
            if k % 2: continue
            t0, t1 = k / n, (k + 1) / n
            s.line([(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0), (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)], fill=col + (255,), width=5)
    else:
        s.line([a, b], fill=col + (255,), width=5)

# Wi-Fi bubbles around drones (2.4 GHz)
for c, r in ((d1, 230), (d2, 150), (d3, 150)):
    g.ellipse([c[0] - r, c[1] - r * 0.55, c[0] + r, c[1] + r * 0.55], fill=BLUE + (40,), outline=BLUE + (160,), width=6)
    s.ellipse([c[0] - r, c[1] - r * 0.55, c[0] + r, c[1] + r * 0.55], outline=BLUE + (200,), width=3)
# mesh links (5.8 GHz): d1 -> d2 -> van mast, d3 -> mast
link(d1, d2, TEAL); link(d2, mast, TEAL); link(d3, mast, TEAL); link(d1, mast, TEAL, dash=True)
# phone under the pile talking to the airborne drone
link(pile_phone, d1, BLUE, dash=True)
for c, rr in ((mast, 16), (d1, 13), (d2, 13), (d3, 13)):
    s.ellipse([c[0] - rr, c[1] - rr, c[0] + rr, c[1] + rr], fill=(255, 255, 255, 255), outline=TEAL + (255,), width=5)
s.ellipse([pile_phone[0] - 11, pile_phone[1] - 11, pile_phone[0] + 11, pile_phone[1] + 11], fill=BLUE + (255,), outline=(255, 255, 255, 255), width=3)
glow = glow.filter(ImageFilter.GaussianBlur(10))
out = Image.alpha_composite(Image.alpha_composite(im.convert("RGBA"), glow), sharp)
d = ImageDraw.Draw(out)
def font(sz, b=False):
    try: return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if b else ""), sz)
    except OSError: return ImageFont.load_default()
def pill(xy, text, col, sub=None):
    f, fs = font(30, True), font(22)
    tw = d.textbbox((0, 0), text, font=f)[2]; sw = d.textbbox((0, 0), sub, font=fs)[2] if sub else 0
    w = max(tw, sw) + 40; h = 58 + (34 if sub else 0)
    x, y = xy
    d.rounded_rectangle([x, y, x + w, y + h], radius=14, fill=(12, 16, 22, 215), outline=col + (255,), width=3)
    d.text((x + 20, y + 12), text, fill=(240, 244, 248), font=f)
    if sub: d.text((x + 20, y + 54), sub, fill=(170, 180, 192), font=fs)
pill((1300, 40), "Command van", TEAL, "mesh base station on the mast · dashboard")
pill((640, 230), "5.8 GHz drone mesh", TEAL, "telemetry · LiDAR map · alerts · phone traffic")
pill((40, 880), "Rescue Wi-Fi 2.4 GHz", BLUE, "trapped phones connect to the nearest drone")
out.convert("RGB").save(OUT, quality=95)
print("ok")
