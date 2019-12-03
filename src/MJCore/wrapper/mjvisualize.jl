const mjNGROUP = 6
const mMAXOVERLAY = 500

const mjMAXLINE = 100
const mjMAXLINEPNT = 1000
const mjMAXPLANEGRID = 200

@cenum mjtCatBit::Cint begin  # bitflags for vGeom category
    mjCAT_STATIC = 1    # model elements in body 0
    mjCAT_DYNAMIC = 2    # model elements in all other bodies
    mjCAT_DECOR = 4    # decorative geoms
    mjCAT_ALL = 7    # select all categories
end

@cenum mjtMouse::Cint begin   # mouse interaction mode
    mjMOUSE_NONE = 0   # no action
    mjMOUSE_ROTATE_V           # rotate, vertical plane
    mjMOUSE_ROTATE_H           # rotate, horizontal plane
    mjMOUSE_MOVE_V             # move, vertical plane
    mjMOUSE_MOVE_H             # move, horizontal plane
    mjMOUSE_ZOOM               # zoom
    mjMOUSE_SELECT             # selection
end

@cenum mjtPertBit::Cint begin # mouse perturbations
    mjPERT_TRANSLATE = 1    # translation
    mjPERT_ROTATE = 2    # rotation
end

@cenum mjtCamera::Cint begin  # abstract camera type
    mjCAMERA_FREE = 0   # free camera
    mjCAMERA_TRACKING          # tracking camera; uses trackbodyid
    mjCAMERA_FIXED             # fixed camera; uses fixedcamid
    mjCAMERA_USER              # user is responsible for setting OpenGL camera
end

@cenum mjtLabel::Cint begin   # object labeling
    mjLABEL_NONE = 0    # nothing
    mjLABEL_BODY               # body labels
    mjLABEL_JOINT              # joint labels
    mjLABEL_GEOM               # geom labels
    mjLABEL_SITE               # site labels
    mjLABEL_CAMERA             # camera labels
    mjLABEL_LIGHT              # light labels
    mjLABEL_TENDON             # tendon labels
    mjLABEL_ACTUATOR           # actuator labels
    mjLABEL_CONSTRAINT         # constraint labels
    mjLABEL_SKIN               # skin labels
    mjLABEL_SELECTION          # selected object
    mjLABEL_SELPNT             # coordinates of selection point
    mjLABEL_CONTACTFORCE       # magnitude of contact force

    mjNLABEL                   # number of label types
end

@cenum mjtFrame::Cint begin   # frame visualization
    mjFRAME_NONE = 0    # no frames
    mjFRAME_BODY               # body frames
    mjFRAME_GEOM               # geom frames
    mjFRAME_SITE               # site frames
    mjFRAME_CAMERA             # camera frames
    mjFRAME_LIGHT              # light frames
    mjFRAME_WORLD              # world frame

    mjNFRAME                   # number of visualization frames
end

@cenum mjtVisFlag::Cint begin # flags enabling model element visualization
    mjVIS_CONVEXHULL = 0    # mesh convex hull
    mjVIS_TEXTURE              # textures
    mjVIS_JOINT                # joints
    mjVIS_ACTUATOR             # actuators
    mjVIS_CAMERA               # cameras
    mjVIS_LIGHT                # lights
    mjVIS_TENDON               # tendons
    mjVIS_RANGEFINDER          # rangefinder sensors
    mjVIS_CONSTRAINT           # point constraints
    mjVIS_INERTIA              # equivalent inertia boxes
    mjVIS_SCLINERTIA           # scale equivalent inertia boxes with mass
    mjVIS_PERTFORCE            # perturbation force
    mjVIS_PERTOBJ              # perturbation object
    mjVIS_CONTACTPOINT         # contact points
    mjVIS_CONTACTFORCE         # contact force
    mjVIS_CONTACTSPLIT         # split contact force into normal and tanget
    mjVIS_TRANSPARENT          # make dynamic geoms more transparent
    mjVIS_AUTOCONNECT          # auto connect joints and body coms
    mjVIS_COM                  # center of mass
    mjVIS_SELECT               # selection point
    mjVIS_STATIC               # static bodies
    mjVIS_SKIN                 # skin

    mjNVISFLAG                 # number of visualization flags
end

@cenum mjtRndFlag::Cint begin # flags enabling rendering effects
    mjRND_SHADOW = 0    # shadows
    mjRND_WIREFRAME            # wireframe
    mjRND_REFLECTION           # reflections
    mjRND_ADDITIVE             # additive transparency
    mjRND_SKYBOX               # skybox
    mjRND_FOG                  # fog
    mjRND_HAZE                 # haze
    mjRND_SEGMENT              # segmentation with random color
    mjRND_IDCOLOR              # segmentation with segid color

    mjNRNDFLAG                 # number of rendering flags
end

@cenum mjtStereo::Cint begin  # type of stereo rendering
    mjSTEREO_NONE = 0    # no stereo; use left eye only
    mjSTEREO_QUADBUFFERED      # quad buffered; revert to side-by-side if no hardware support
    mjSTEREO_SIDEBYSIDE        # side-by-side
end

@uninitable mutable struct mjvPerturb
    select::Cint
    skinselect::Cint
    active::Cint
    refpos::SVector{3,mjtNum}
    refquat::SVector{4,mjtNum}
    localpos::SVector{3,mjtNum}
    scale::mjtNum
end

@uninitable mutable struct mjvCamera
    type::Cint
    fixedcamid::Cint
    trackbodyid::Cint
    lookat::SVector{3,mjtNum}
    distance::mjtNum
    azimuth::mjtNum
    elevation::mjtNum
end

@uninitable struct mjvGLCamera
    pos::SVector{3,Cfloat}
    forward::SVector{3,Cfloat}
    up::SVector{3,Cfloat}
    frustum_center::Cfloat
    frustum_bottom::Cfloat
    frustum_top::Cfloat
    frustum_near::Cfloat
    frustum_far::Cfloat
end

@uninitable struct mjvGeom
    type::Cint
    dataid::Cint
    objtype::Cint
    objid::Cint
    category::Cint
    texid::Cint
    texuniform::Cint
    texcoord::Cint
    segid::Cint
    texrepeat::SVector{2,Cfloat}
    size::SVector{3,Cfloat}
    pos::SVector{3,Cfloat}
    mat::SVector{9,Cfloat}
    rgba::SVector{4,Cfloat}
    emission::Cfloat
    specular::Cfloat
    shininess::Cfloat
    reflectance::Cfloat
    label::SVector{100,UInt8}
    camdist::Cfloat
    rbound::Cfloat
    transparent::mjtByte
end

@uninitable struct mjvLight
    pos::SVector{3,Cfloat}
    dir::SVector{3,Cfloat}
    attenuation::SVector{3,Cfloat}
    cutoff::Cfloat
    exponent::Cfloat
    ambient::SVector{3,Cfloat}
    diffuse::SVector{3,Cfloat}
    specular::SVector{3,Cfloat}
    headlight::mjtByte
    directional::mjtByte
    castshadow::mjtByte
end

@uninitable mutable struct mjvOption
    label::Cint
    frame::Cint
    geomgroup::SVector{mjNGROUP,mjtByte}
    sitegroup::SVector{mjNGROUP,mjtByte}
    jointgroup::SVector{mjNGROUP,mjtByte}
    tendongroup::SVector{mjNGROUP,mjtByte}
    actuatorgroup::SVector{mjNGROUP,mjtByte}
    flags::SVector{Int(mjNVISFLAG),mjtByte}
end

@uninitable mutable struct mjvScene
    maxgeom::Cint
    ngeom::Cint
    geoms::Ptr{mjvGeom}
    geomorder::Ptr{Cint}

    nskin::Cint
    skinfacenum::Ptr{Cint}
    skinvertadr::Ptr{Cint}
    skinvertnum::Ptr{Cint}
    skinvert::Ptr{Cfloat}
    skinnormal::Ptr{Cfloat}

    nlight::Cint
    lights::SVector{8,mjvLight}
    camera::SVector{2,mjvGLCamera}
    enabletransform::mjtByte
    translate::SVector{3,Cfloat}
    rotate::SVector{4,Cfloat}
    scale::Cfloat
    stereo::Cint
    flags::SVector{Int(mjNRNDFLAG),mjtByte}
end

@uninitable mutable struct mjvFigure
    flg_legend::Cint
    flg_ticklabel::SVector{2,Cint}
    flg_extend::Cint
    flg_barplot::Cint
    flg_selection::Cint
    flg_symmetric::Cint

    legendoff::Cint
    gridsize::SVector{2,Cint}
    selection::Cint
    highlight::SVector{2,Cint}
    gridrgb::SVector{3,Cfloat}
    gridwidth::Cfloat
    figurergba::SVector{4,Cfloat}
    panergba::SVector{4,Cfloat}
    legendrgba::SVector{4,Cfloat}
    textrgb::SVector{3,Cfloat}
    range::SVector{2,SVector{2,Cfloat}}
    xlabel::SVector{100,UInt8}
    title::SVector{100,UInt8}
    xformat::SVector{20,UInt8}
    yformat::SVector{20,UInt8}
    minwidth::SVector{20,UInt8}

    linepnt::SVector{mjMAXLINE,Cint}
    linergb::SVector{mjMAXLINE,SVector{3,Cfloat}}
    linewidth::SVector{mjMAXLINE,Cfloat}
    linedata::SVector{mjMAXLINE,SVector{2 * mjMAXLINEPNT,Cfloat}}
    linename::SVector{mjMAXLINE,SVector{100,UInt8}}

    xaxispixel::SVector{2,Cint}
    yaxispixel::SVector{2,Cint}
    xaxisdata::SVector{2,Cint}
    yaxisdata::SVector{2,Cint}
end
