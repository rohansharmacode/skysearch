"""Author the Isaac Sim USD: root .usda (materials, lights, cameras, render settings) + binary geometry sublayers."""
import json, os, pickle, glob, numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdLux, UsdPhysics, Sdf, Vt, Gf
import sitecfg as S

ROOT = 'out/collapse_site'
LIB = json.load(open('build/texlib.json'))
os.makedirs(f'{ROOT}/geo', exist_ok=True)
SITE = '/CollapseSite'
LOOKS = f'{SITE}/Looks'


def tex(name, kind):
    for ext in ('jpg', 'png'):
        p = f'textures/{name}_{kind}.{ext}'
        if os.path.exists(f'{ROOT}/{p}'):
            return './' + p
    return None


# ------------------------------------------------------------------------------------ materials
def make_material(stage, name, albedo=None, normal=None, rough_tex=None, rough=0.85, metal=0.0, color=None,
                  wrap='repeat', bump=1.0):
    mpath = f'{LOOKS}/{name}'
    mat = UsdShade.Material.Define(stage, mpath)
    # --- MDL OmniPBR (what Isaac/RTX renders) ---
    sh = UsdShade.Shader.Define(stage, f'{mpath}/OmniPBR')
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath('OmniPBR.mdl'), 'mdl')
    sh.SetSourceAssetSubIdentifier('OmniPBR', 'mdl')
    out = sh.CreateOutput('out', Sdf.ValueTypeNames.Token)
    for o in ('surface', 'displacement', 'volume'):
        mat.CreateOutput(f'mdl:{o}', Sdf.ValueTypeNames.Token).ConnectToSource(out)
    if albedo:
        a = sh.CreateInput('diffuse_texture', Sdf.ValueTypeNames.Asset); a.Set(Sdf.AssetPath(albedo))
        a.GetAttr().SetMetadata('colorSpace', 'sRGB')
    else:
        sh.CreateInput('diffuse_color_constant', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if normal:
        n = sh.CreateInput('normalmap_texture', Sdf.ValueTypeNames.Asset); n.Set(Sdf.AssetPath(normal))
        n.GetAttr().SetMetadata('colorSpace', 'raw')
        sh.CreateInput('bump_factor', Sdf.ValueTypeNames.Float).Set(float(bump))
    sh.CreateInput('reflection_roughness_constant', Sdf.ValueTypeNames.Float).Set(float(rough))
    if rough_tex:
        r = sh.CreateInput('reflectionroughness_texture', Sdf.ValueTypeNames.Asset); r.Set(Sdf.AssetPath(rough_tex))
        r.GetAttr().SetMetadata('colorSpace', 'raw')
        sh.CreateInput('reflectionroughness_texture_influence', Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput('metallic_constant', Sdf.ValueTypeNames.Float).Set(float(metal))
    sh.CreateInput('specular_level', Sdf.ValueTypeNames.Float).Set(0.35)
    # --- UsdPreviewSurface fallback (usdview, Blender, Omniverse non-RTX) ---
    ps = UsdShade.Shader.Define(stage, f'{mpath}/PreviewSurface')
    ps.CreateIdAttr('UsdPreviewSurface')
    ps.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(rough))
    ps.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metal))
    mat.CreateSurfaceOutput().ConnectToSource(ps.CreateOutput('surface', Sdf.ValueTypeNames.Token))
    if albedo or normal or rough_tex:
        st = UsdShade.Shader.Define(stage, f'{mpath}/stReader')
        st.CreateIdAttr('UsdPrimvarReader_float2')
        st.CreateInput('varname', Sdf.ValueTypeNames.String).Set('st')
        sto = st.CreateOutput('result', Sdf.ValueTypeNames.Float2)

        def uvtex(nm, path, cs):
            t = UsdShade.Shader.Define(stage, f'{mpath}/{nm}')
            t.CreateIdAttr('UsdUVTexture')
            t.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(path))
            t.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(sto)
            t.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set(wrap)
            t.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set(wrap)
            t.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set(cs)
            return t
        if albedo:
            t = uvtex('albedoTex', albedo, 'sRGB')
            ps.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(t.CreateOutput('rgb', Sdf.ValueTypeNames.Float3))
        if normal:
            t = uvtex('normalTex', normal, 'raw')
            t.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
            t.CreateInput('bias', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
            ps.CreateInput('normal', Sdf.ValueTypeNames.Normal3f).ConnectToSource(t.CreateOutput('rgb', Sdf.ValueTypeNames.Float3))
        if rough_tex:
            t = uvtex('roughTex', rough_tex, 'raw')
            ps.CreateInput('roughness', Sdf.ValueTypeNames.Float).ConnectToSource(t.CreateOutput('r', Sdf.ValueTypeNames.Float))
    else:
        ps.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    return mat


# ------------------------------------------------------------------------------------ meshes
def make_mesh(stage, path, V, F, UV=None, N=None, mat=None, collide=True, double=False):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(V, np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(F.ravel(), np.int32)))
    m.CreateSubdivisionSchemeAttr('none')
    m.CreateDoubleSidedAttr(double)
    lo, hi = V.min(0), V.max(0)
    m.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    if N is not None:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(N, np.float32)))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if UV is not None:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(UV, np.float32)))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        mc = UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim())
        mc.CreateApproximationAttr(UsdPhysics.Tokens.none)
    return m


def vertex_normals(V, F):
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    vn = np.zeros_like(V)
    for k in range(3):
        np.add.at(vn, F[:, k], fn)
    vn /= np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12
    return vn.astype(np.float32)


def matrix_from(xf):
    """4x4 (row-vector convention as Gf.Matrix4d) from our xform dicts"""
    M = np.eye(4)
    def rot(axis, deg):
        axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
        a = np.deg2rad(deg); c, s = np.cos(a), np.sin(a); x, y, z = axis
        R = np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                      [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                      [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])
        T = np.eye(4); T[:3, :3] = R; return T
    def tr(v):
        T = np.eye(4); T[:3, 3] = v; return T
    if xf.get('wall_tilt'):
        wt = xf['wall_tilt']; p = np.array([*wt['pivot'], 0.0])
        M = tr(p) @ rot((1, 0, 0), wt['tilt'][0]) @ rot((0, 1, 0), wt['tilt'][1]) @ tr(-p) @ M
    if xf.get('tilt'):
        t = xf['tilt']; p = np.array([*t['pivot'], 0.0])
        M = tr(p) @ rot((*t['axis'], 0.0), t['deg']) @ tr(-p) @ M
    M = rot((0, 0, 1), xf.get('yaw', 0.0)) @ M
    M = tr(xf.get('translate', (0, 0, 0))) @ M
    return Gf.Matrix4d(*M.T.ravel().tolist())   # Gf uses row vectors -> transpose


def set_xform(prim, M):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(M)


# ------------------------------------------------------------------------------------ layers
def stage_for(path):
    if os.path.exists(path):
        os.remove(path)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    return st


def build_terrain():
    st = stage_for(f'{ROOT}/geo/terrain.usdc')
    site = UsdGeom.Xform.Define(st, SITE); st.SetDefaultPrim(site.GetPrim())
    UsdGeom.Xform.Define(st, f'{SITE}/Terrain')
    for f in sorted(glob.glob('build/meshes/terrain_*.npz')):
        name = os.path.basename(f)[:-4]
        d = np.load(f)
        V, F = d['v'], d['f']
        x0, ytop = float(d['x0']), float(d['ytop'])
        UV = np.stack([(V[:, 0] - x0) / 80.0, (V[:, 1] - (ytop - 80.0)) / 80.0], 1)
        N = vertex_normals(V.astype(np.float64), F).astype(np.float32)
        mat = UsdShade.Material(st.OverridePrim(f'{LOOKS}/{name}'))
        make_mesh(st, f'{SITE}/Terrain/{name}', V, F, UV, N, mat=None, collide=True)
        UsdShade.MaterialBindingAPI.Apply(st.GetPrimAtPath(f'{SITE}/Terrain/{name}')).Bind(
            UsdShade.Material(st.GetPrimAtPath(f'{LOOKS}/{name}')))
    # horizon apron: 2 km ground plane just below the terrain so the site doesn't float in the void
    R = 1000.0
    V = np.array([[-R, -R, -0.35], [R, -R, -0.35], [R, R, -0.35], [-R, R, -0.35]], np.float32)
    F = np.array([[0, 1, 2], [0, 2, 3]], np.int32)
    UV = (V[:, :2] / 40.0).astype(np.float32)
    N = np.tile(np.array([[0, 0, 1.0]], np.float32), (4, 1))
    m = make_mesh(st, f'{SITE}/Terrain/horizon_apron', V, F, UV, N, collide=True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material(st.OverridePrim(f'{LOOKS}/ground_far')))
    # remove the placeholder overs so this layer only carries geometry + binding targets
    st.RemovePrim(LOOKS)
    st.GetRootLayer().Save()


def build_buildings():
    st = stage_for(f'{ROOT}/geo/buildings.usdc')
    site = UsdGeom.Xform.Define(st, SITE); st.SetDefaultPrim(site.GetPrim())
    UsdGeom.Xform.Define(st, f'{SITE}/Buildings')
    UsdGeom.Xform.Define(st, f'{SITE}/Buildings/Scanned')
    UsdGeom.Xform.Define(st, f'{SITE}/Buildings/Surroundings')
    items = pickle.load(open('build/buildings.pkl', 'rb'))
    groups = {}
    for it in items:
        nm = it['name']
        if nm.startswith('core_'):
            parent = f'{SITE}/Buildings/Scanned'
            gname = nm.rsplit('_', 1)[0]
        else:
            parent = f'{SITE}/Buildings/Surroundings'
            parts = nm.split('_')
            gname = '_'.join(parts[:2])
        gpath = f'{parent}/{gname}'
        if gpath not in groups:
            g = UsdGeom.Xform.Define(st, gpath)
            if it['xform']:
                set_xform(g.GetPrim(), matrix_from(it['xform']))
            groups[gpath] = g
        mpath = f'{gpath}/{nm[len(gname) + 1:] or "mesh"}'
        # wall remnants carry their own tilt; nest them so each piece pivots independently
        if it['xform'] and it['xform'].get('wall_tilt'):
            set_xform(st.GetPrimAtPath(gpath), Gf.Matrix4d(1))
            sub = UsdGeom.Xform.Define(st, f'{gpath}/{nm.split("_")[2]}')
            set_xform(sub.GetPrim(), matrix_from(it['xform']))
            mpath = f'{gpath}/{nm.split("_")[2]}/{nm.split("_")[3]}'
        m = make_mesh(st, mpath, it['V'], it['F'], it['UV'], it['N'], collide=it['collide'])
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material(st.OverridePrim(f'{LOOKS}/{it["material"]}')))
    st.RemovePrim(LOOKS)
    st.GetRootLayer().Save()


def build_debris():
    st = stage_for(f'{ROOT}/geo/debris.usdc')
    site = UsdGeom.Xform.Define(st, SITE); st.SetDefaultPrim(site.GetPrim())
    UsdGeom.Xform.Define(st, f'{SITE}/Debris')
    d = pickle.load(open('build/debris.pkl', 'rb'))
    ip = f'{SITE}/Debris/Scatter'
    pi = UsdGeom.PointInstancer.Define(st, ip)
    protos = UsdGeom.Scope.Define(st, f'{ip}/Prototypes')
    targets = []
    for (name, mat, (V, F, UV, N)) in d['protos']:
        pp = f'{ip}/Prototypes/{name}'
        m = make_mesh(st, pp, V, F, UV, N, collide=False)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material(st.OverridePrim(f'{LOOKS}/{mat}')))
        targets.append(Sdf.Path(pp))
    pi.CreatePrototypesRel().SetTargets(targets)
    pi.CreateProtoIndicesAttr(Vt.IntArray.FromNumpy(d['pid']))
    pi.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(d['pos']))
    pi.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(d['scale']))
    q = d['ori']
    pi.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(float(w), float(x), float(y), float(z)) for w, x, y, z in q]))
    lo = d['pos'].min(0) - 3; hi = d['pos'].max(0) + 3
    pi.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    st.RemovePrim(LOOKS)
    st.GetRootLayer().Save()


RENDER = {   # RTX post-processing preset (Sketchfab-like look: AO, contrast, mild sharpening, vignette, haze)
    'rtx:post:tonemap:op': 6,                         # ACES
    'rtx:post:tonemap:filmIso': 100.0,
    'rtx:post:tonemap:cameraShutter': 50.0,
    'rtx:post:tonemap:fNumber': 5.0,
    'rtx:post:histogram:enabled': False,
    'rtx:ambientOcclusion:enabled': True,
    'rtx:ambientOcclusion:rayLength': 3.0,
    'rtx:indirectDiffuse:enabled': True,
    'rtx:shadows:enabled': True,
    'rtx:reflections:enabled': True,
    'rtx:post:colorcorr:enabled': True,
    'rtx:post:colorcorr:saturation': Gf.Vec3f(1.08, 1.08, 1.08),
    'rtx:post:colorcorr:contrast': Gf.Vec3f(1.12, 1.12, 1.12),
    'rtx:post:colorcorr:gamma': Gf.Vec3f(1.0, 1.0, 1.0),
    'rtx:post:colorcorr:gain': Gf.Vec3f(1.0, 1.0, 1.0),
    'rtx:post:colorcorr:offset': Gf.Vec3f(0.0, 0.0, 0.0),
    'rtx:post:tvNoise:enabled': True,
    'rtx:post:tvNoise:enableVignetting': True,
    'rtx:post:tvNoise:vignettingSize': 107.0,
    'rtx:post:tvNoise:vignettingStrength': 0.55,
    'rtx:post:tvNoise:enableFilmGrain': True,
    'rtx:post:tvNoise:grainAmount': 0.025,
    'rtx:post:tvNoise:grainSize': 1.4,
    'rtx:post:tvNoise:enableScanlines': False,
    'rtx:post:tvNoise:enableScrollBug': False,
    'rtx:post:tvNoise:enableVignettingFlickering': False,
    'rtx:post:tvNoise:enableGhostFlickering': False,
    'rtx:post:tvNoise:enableWaveDistortion': False,
    'rtx:post:tvNoise:enableVerticalLines': False,
    'rtx:post:tvNoise:enableRandomSplotches': False,
    'rtx:post:lensFlares:enabled': True,
    'rtx:post:lensFlares:flareScale': 0.04,
    'rtx:fog:enabled': True,
    'rtx:fog:fogZup:enabled': True,
    'rtx:fog:fogColor': Gf.Vec3f(0.78, 0.74, 0.68),
    'rtx:fog:fogColorIntensity': 1.0,
    'rtx:fog:fogStartDist': 60.0,
    'rtx:fog:fogEndDist': 900.0,
    'rtx:fog:fogDistanceDensity': 0.35,
    'rtx:fog:fogStartHeight': 0.0,
    'rtx:fog:fogEndHeight': 60.0,
    'rtx:fog:fogHeightDensity': 0.25,
}


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    z = eye - target; z /= np.linalg.norm(z)                 # camera looks down -Z
    x = np.cross(up, z); x /= np.linalg.norm(x); y = np.cross(z, x)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, eye
    return Gf.Matrix4d(*M.T.ravel().tolist())


def build_root():
    path = f'{ROOT}/collapse_site.usda'
    st = stage_for(path)
    lyr = st.GetRootLayer()
    lyr.subLayerPaths.append('./geo/terrain.usdc')
    lyr.subLayerPaths.append('./geo/buildings.usdc')
    lyr.subLayerPaths.append('./geo/debris.usdc')
    site = UsdGeom.Xform.Define(st, SITE)
    st.SetDefaultPrim(site.GetPrim())
    Usd.ModelAPI(site.GetPrim()).SetKind('assembly')
    lyr.documentation = ('Collapsed-building disaster site for Isaac Sim (Larpverse / AgentVerse 2026). '
                         'Z-up, metres. Core 105 m reconstructed from top-down + oblique screenshots of a photogrammetry '
                         'scan; surroundings (320 m x 320 m) procedurally extrapolated.')
    cld = dict(lyr.customLayerData)
    cld['renderSettings'] = RENDER
    cld['larpverse'] = {'frame': 'x=east y=north z=up, metres', 'extent_m': 320.0, 'core_extent_m': 105.0,
                        'origin': 'centre of scanned site', 'generator': 'build_collapse_site.py'}
    lyr.customLayerData = cld
    UsdGeom.Scope.Define(st, LOOKS)
    # terrain tile materials
    for f in sorted(glob.glob('build/meshes/terrain_*.npz')):
        name = os.path.basename(f)[:-4]
        make_material(st, name, tex(name, 'albedo'), tex(name, 'normal'), tex(name, 'rough'), rough=0.9, wrap='clamp', bump=1.0)
    for name, meta in LIB.items():
        if 'src' in meta:
            continue
        make_material(st, name, tex(name, 'albedo'), tex(name, 'normal'), None, rough=meta['rough'], metal=meta['metal'],
                      wrap='clamp' if name == 'scan_roofs' else 'repeat', bump=0.8)
    make_material(st, 'rebar', color=(0.22, 0.13, 0.08), rough=0.55, metal=0.7)
    for nm, col in (('car_white', (0.62, 0.62, 0.6)), ('car_silver', (0.42, 0.43, 0.45)), ('car_red', (0.38, 0.06, 0.05)),
                    ('car_blue', (0.08, 0.14, 0.3)), ('car_dark', (0.06, 0.06, 0.07))):
        make_material(st, nm, color=col, rough=0.45, metal=0.4)
    # lights: sun from ESE (matches baked photo shadows), cool sky dome fill
    UsdGeom.Xform.Define(st, f'{SITE}/Lights')
    sun = UsdLux.DistantLight.Define(st, f'{SITE}/Lights/Sun')
    sun.CreateIntensityAttr(2600.0); sun.CreateAngleAttr(0.53)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    az, el = np.deg2rad(-22.0), np.deg2rad(38.0)
    sv = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    set_xform(sun.GetPrim(), look_at(sv * 100, (0, 0, 0)))
    dome = UsdLux.DomeLight.Define(st, f'{SITE}/Lights/Sky')
    dome.CreateIntensityAttr(750.0); dome.CreateColorAttr(Gf.Vec3f(0.72, 0.82, 1.0))
    # cameras matching the reference screenshots + a drone-height orbit start
    UsdGeom.Xform.Define(st, f'{SITE}/Cameras')
    cams = {
        'TopDown': ((0, 0, 230), (0, 0.01, 0), 28.0),
        'Oblique_South': ((10, -125, 95), (0, 0, 2), 24.0),
        'Oblique_NNW': ((-55, 120, 70), (5, -5, 2), 24.0),
        'Low_East': ((135, 15, 26), (0, 0, 4), 24.0),
        'Wide_Establishing': ((-210, -230, 160), (0, 0, 0), 26.0),
        'Drone_Orbit_A2': ((-40, -40, 35), (0, 0, 5), 20.0),
    }
    for nm, (eye, tgt, fl) in cams.items():
        c = UsdGeom.Camera.Define(st, f'{SITE}/Cameras/{nm}')
        c.CreateFocalLengthAttr(fl); c.CreateHorizontalApertureAttr(36.0); c.CreateVerticalApertureAttr(20.25)
        c.CreateClippingRangeAttr(Gf.Vec2f(0.1, 5000.0))
        set_xform(c.GetPrim(), look_at(eye, tgt))
    lyr.Save()
    return path


if __name__ == '__main__':
    build_terrain(); print('terrain ok')
    build_buildings(); print('buildings ok')
    build_debris(); print('debris ok')
    p = build_root(); print('root', p)
