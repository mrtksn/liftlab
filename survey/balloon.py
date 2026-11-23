"""Inflate a closed room envelope from the registered trajectory."""
import fast_simplification
import numpy as np
from scipy.ndimage import binary_fill_holes, gaussian_filter
from scipy.spatial import ConvexHull, cKDTree
from skimage.measure import marching_cubes
from skimage.morphology import isotropic_closing

from cloth_surfaces import ray_lattice


def closed_sphere(vertices, faces):
    """Check connectivity, opposite edge winding and genus on the exported mesh."""
    if len(faces)<4:
        return False
    edges = np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    unique, inverse, counts = np.unique(np.sort(edges,axis=1),axis=0,
                                        return_inverse=True,return_counts=True)
    if not np.all(counts==2) or len(vertices)-len(unique)+len(faces)!=2:
        return False
    if np.any(np.bincount(inverse,weights=np.where(edges[:,0]<edges[:,1],1,-1))):
        return False
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    graph = coo_matrix((np.ones(len(unique)),unique.T),shape=(len(vertices),len(vertices)))
    return connected_components(graph,directed=False,return_labels=False)==1


def path_samples(path, spacing):
    if len(path)==1:
        return path
    arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    arc, keep = np.unique(arc,return_index=True)
    t = np.linspace(0,arc[-1],min(4096,int(np.ceil(arc[-1]/spacing))+1))
    return np.column_stack([np.interp(t,arc,path[keep,k]) for k in range(3)])


def build_balloon(xyz, poses, detail, reach):
    path = np.asarray([p['xyz'] for p in poses if 'rotation' in p],float)
    if not len(path) or not np.isfinite(path).all():
        raise ValueError('Balloon inflation needs registered camera positions.')
    # Bound the volume even for a very fine user setting or a long walk.
    lo, hi = path.min(axis=0)-reach,path.max(axis=0)+reach
    cell = max(detail,np.max(hi-lo)/112)
    origin = lo-3*cell
    shape = np.ceil((hi-origin)/cell).astype(int)+4
    samples = path_samples(path,cell*.5)
    arc_length = np.linalg.norm(np.diff(path,axis=0),axis=1).sum()
    stations = path_samples(path,max(3*cell,reach/4,arc_length/64))
    rays,_ = ray_lattice()
    direction_tree = cKDTree(rays)
    neighbours = direction_tree.query(rays,k=7)[1]
    point_tree = cKDTree(xyz)
    field = np.full(shape,-2.,np.float32)
    contacts = 0
    for eye in stations:
        idx = point_tree.query_ball_point(eye,reach+cell)
        delta = xyz[idx]-eye
        lengths = np.linalg.norm(delta,axis=1)
        valid = lengths>cell
        radii = np.full(len(rays),reach)
        if valid.any():
            bins = direction_tree.query(delta[valid]/lengths[valid,None])[1]
            np.minimum.at(radii,bins,lengths[valid])
            contacts += int(np.sum(radii<reach))
            # Tension suppresses isolated contacts. Nearby supported points hold the skin.
            hits = radii < reach
            if hits.any():
                gap, nearest = cKDTree(rays[hits]).query(rays[~hits],k=min(3,int(hits.sum())))
                gap = gap.reshape((~hits).sum(),-1) if (~hits).any() else np.empty((0,1))
                if len(gap):
                    nearest = nearest.reshape(gap.shape)
                    inferred = np.median(radii[hits][nearest],axis=1)
                    blend = np.clip(gap[:,0]/.75,0,1)
                    radii[~hits] = inferred*(1-blend)+reach*blend
            radii = np.median(radii[neighbours],axis=1)
        start = np.maximum(0,np.floor((eye-reach-origin)/cell).astype(int)-1)
        end = np.minimum(shape,np.ceil((eye+reach-origin)/cell).astype(int)+2)
        region = tuple(slice(i,j) for i,j in zip(start,end))
        offsets = (np.indices(end-start).reshape(3,-1).T+start)*cell+origin-eye
        distance = np.linalg.norm(offsets,axis=1)
        inside = distance<=reach+cell
        # Interpolate nearby angular stops so the expanding front has no radial seams.
        d, sectors = direction_tree.query(offsets[inside]/np.maximum(distance[inside,None],1e-12),k=3)
        weights = 1/np.maximum(d,.001)**2
        stop = (weights*radii[sectors]).sum(axis=1)/weights.sum(axis=1)
        local = np.full(len(distance),-2.,np.float32)
        local[inside] = np.clip((stop-distance[inside])/cell,-2,2)
        np.maximum(field[region],local.reshape(end-start),out=field[region])
    if not contacts:
        raise ValueError('No cloud points within the inflation limit. Increase the limit.')
    # Keep the interior connected along the path even through sharp bends.
    route = np.rint((samples-origin)/cell).astype(int)
    for shift in ([0,0,0],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]):
        field[tuple((route+shift).T)] = 1.
    smooth = gaussian_filter(field,.65)
    skin = smooth>0
    for closing in (0,2,4,8):
        solid = isotropic_closing(skin,closing) if closing else skin
        solid = binary_fill_holes(solid)
        level = np.where(solid,np.maximum(.001,smooth),np.minimum(-.001,smooth))
        vertices,faces,_,_ = marching_cubes(np.pad(level,1,constant_values=-2),0.,allow_degenerate=False)
        if closed_sphere(vertices,faces):
            break
    else:
        # Explicitly mark the final enclosing approximation on difficult looped paths.
        hull = ConvexHull(vertices)
        faces = hull.simplices.copy()
        tri = vertices[faces]
        reverse = (np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])*hull.equations[:,:3]).sum(axis=1)<0
        faces[reverse] = faces[reverse,::-1]
        used,faces = np.unique(faces,return_inverse=True)
        vertices,faces = vertices[used],faces.reshape(-1,3)
        closing = 'convex fallback'
    vertices = (vertices-1)*cell+origin
    if len(faces)>5000:
        reduced,triangles = fast_simplification.simplify(vertices,faces,target_count=5000)
        if closed_sphere(reduced,triangles):
            vertices,faces = reduced,triangles
    if not closed_sphere(vertices,faces):
        raise ValueError('Balloon closure verification failed.')
    near = point_tree.query(vertices[faces].mean(axis=1))[0]<=2*cell
    patches = [dict(vertices=t,landmarks=[]) for t in vertices[faces].tolist()]
    return patches,dict(stations=len(stations),cell=float(cell),closing=closing,
                        supported=int(near.sum()),faces=len(faces))
