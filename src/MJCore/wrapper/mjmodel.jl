const mjtNum = Cdouble
const mjMINVAL = 1.0e-15

const mjPI = 3.14159265358979323846
const mjMAXVAL = 1.0e10
const mjMINMU = 1.0e-5
const mjMINIMP = 0.0001
const mjMAXIMP = 0.9999
const mjMAXCONPAIR = 50
const mjMAXVFS = 200
const mjMAXVFSNAME = 100

const mjNEQDATA = 7
const mjNDYN = 10
const mjNGAIN = 10
const mjNBIAS = 10
const mjNREF = 2
const mjNIMP = 5
const mjNSOLVER = 1000

const mjtByte = Cuchar

@cenum mjtDisableBit::Cint begin      # disable default feature bitflags
    mjDSBL_CONSTRAINT = 1 << 0         # entire constraint solver
    mjDSBL_EQUALITY = 1 << 1         # equality constraints
    mjDSBL_FRICTIONLOSS = 1 << 2         # joint and tendon frictionloss constraints
    mjDSBL_LIMIT = 1 << 3         # joint and tendon limit constraints
    mjDSBL_CONTACT = 1 << 4         # contact constraints
    mjDSBL_PASSIVE = 1 << 5         # passive forces
    mjDSBL_GRAVITY = 1 << 6         # gravitational forces
    mjDSBL_CLAMPCTRL = 1 << 7         # clamp control to specified range
    mjDSBL_WARMSTART = 1 << 8         # warmstart constraint solver
    mjDSBL_FILTERPARENT = 1 << 9         # remove collisions with parent body
    mjDSBL_ACTUATION = 1 << 10        # apply actuation forces
    mjDSBL_REFSAFE = 1 << 11        # integrator safety: make ref[0]>=2*timestep

    mjNDISABLE = 12           # number of disable flags
end

@cenum mjtEnableBit::Cint begin       # enable optional feature bitflags
    mjENBL_OVERRIDE = 1 << 0         # override contact parameters
    mjENBL_ENERGY = 1 << 1         # energy computation
    mjENBL_FWDINV = 1 << 2         # record solver statistics
    mjENBL_SENSORNOISE = 1 << 3         # add noise to sensor data

    mjNENABLE = 4          # number of enable flags
end

@cenum mjtJoint::Cint begin           # type of degree of freedom
    mjJNT_FREE = 0            # global position and orientation (quat)       (7)
    mjJNT_BALL                         # orientation (quat) relative to parent        (4)
    mjJNT_SLIDE                        # sliding distance along body-fixed axis       (1)
    mjJNT_HINGE                        # rotation angle (rad) around body-fixed axis  (1)
end

@cenum mjtGeom::Cint begin            # type of geometric shape
    # regular geom types
    mjGEOM_PLANE = 0            # plane
    mjGEOM_HFIELD                      # height field
    mjGEOM_SPHERE                      # sphere
    mjGEOM_CAPSULE                     # capsule
    mjGEOM_ELLIPSOID                   # ellipsoid
    mjGEOM_CYLINDER                    # cylinder
    mjGEOM_BOX                         # box
    mjGEOM_MESH                        # mesh

    mjNGEOMTYPES                       # number of regular geom types

                                     # rendering-only geom types: not used in mjModel not counted in mjNGEOMTYPES
    mjGEOM_ARROW = 100          # arrow
    mjGEOM_ARROW1                      # arrow without wedges
    mjGEOM_ARROW2                      # arrow in both directions
    mjGEOM_LINE                        # line
    mjGEOM_SKIN                        # skin
    mjGEOM_LABEL                       # text label

    mGEOM_NONE = 1001         # missing geom type
end

@cenum mjtCamLight::Cint begin        # tracking mode for camera and light
    mjCAMLIGHT_FIXED = 0            # pos and rot fixed in body
    mjCAMLIGHT_TRACK                   # pos tracks body, rot fixed in global
    mjCAMLIGHT_TRACKCOM                # pos tracks subtree com, rot fixed in body
    mjCAMLIGHT_TARGETBODY              # pos fixed in body, rot tracks target body
    mjCAMLIGHT_TARGETBODYCOM           # pos fixed in body rot tracks target subtree com
end

@cenum mjtTexture::Cint begin         # type of texture
    mjTEXTURE_2D = 0            # 2d texture, suitable for planes and hfields
    mjTEXTURE_CUBE                     # cube texture, suitable for all other geom types
    mjTEXTURE_SKYBOX                   # cube texture used as skybox
end

@cenum mjtIntegrator::Cint begin      # integrator mode
    mjINT_EULER = 0            # semi-implicit Euler
    mjINT_RK4                          # 4th-order Runge Kutta
end

@cenum mjtCollision::Cint begin       # collision mode for selecting geom pairs
    mjCOL_ALL = 0            # test precomputed and dynamic pairs
    mjCOL_PAIR                         # test predefined pairs only
    mjCOL_DYNAMIC                      # test dynamic pairs only
end

@cenum mjtCone::Cint begin            # type of friction cone
    mjCONE_PYRAMIDAL = 0           # pyramidal
    mjCONE_ELLIPTIC                    # elliptic
end

@cenum mjtJacobian::Cint begin        # type of constraint Jacobian
    mjJAC_DENSE = 0           # dense
    mjJAC_SPARSE                       # sparse
    mjJAC_AUTO                         # dense if nv<60 sparse otherwise
end

@cenum mjtSolver::Cint begin          # constraint solver algorithm
    mjSOL_PGS = 0           # PGS    (dual)
    mjSOL_CG                           # CG     (primal)
    mjSOL_NEWTON                       # Newton (primal)
end

@cenum mjtEq::Cint begin              # type of equality constraint
    mjEQ_CONNECT = 0            # connect two bodies at a point (ball joint)
    mjEQ_WELD                          # fix relative position and orientation of two bodies
    mjEQ_JOINT                         # couple the values of two scalar joints with cubic
    mjEQ_TENDON                        # couple the lengths of two tendons with cubic
    mjEQ_DISTANCE                      # fix the contact distance betweent two geoms
end

@cenum mjtWrap::Cint begin            # type of tendon wrap object
    mjWRAP_NONE = 0            # null object
    mjWRAP_JOINT                       # constant moment arm
    mjWRAP_PULLEY                      # pulley used to split tendon
    mjWRAP_SITE                        # pass through site
    mjWRAP_SPHERE                      # wrap around sphere
    mjWRAP_CYLINDER                    # wrap around (infinite) cylinder
end

@cenum mjtTrn::Cint begin             # type of actuator transmission
    mjTRN_JOINT = 0            # force on joint
    mjTRN_JOINTINPARENT                # force on joint, expressed in parent frame
    mjTRN_SLIDERCRANK                  # force via slider-crank linkage
    mjTRN_TENDON                       # force on tendon
    mjTRN_SITE                         # force on site

    mjTRN_UNDEFINED = 1000         # undefined transmission type
end

@cenum mjtDyn::Cint begin             # type of actuator dynamics
    mjDYN_NONE = 0            # no internal dynamics; ctrl specifies force
    mjDYN_INTEGRATOR                   # integrator: da/dt = u
    mjDYN_FILTER                       # linear filter: da/dt = (u-a) / tau
    mjDYN_MUSCLE                       # piece-wise linear filter with two time constants
    mjDYN_USER                         # user-defined dynamics type
end

@cenum mjtGain::Cint begin            # type of actuator gain
    mjGAIN_FIXED = 0            # fixed gain
    mjGAIN_MUSCLE                      # muscle FLV curve computed by mju_muscleGain()
    mjGAIN_USER                        # user-defined gain type
end

@cenum mjtBias::Cint begin            # type of actuator bias
    mjBIAS_NONE = 0            # no bias
    mjBIAS_AFFINE                      # const + kp*length + kv*velocity
    mjBIAS_MUSCLE                      # muscle passive force computed by mju_muscleBias()
    mjBIAS_USER                        # user-defined bias type
end

@cenum mjtObj::Cint begin             # type of MujoCo object
    mjOBJ_UNKNOWN = 0            # unknown object type
    mjOBJ_BODY                         # body
    mjOBJ_XBODY                        # body, used to access regular frame instead of i-frame
    mjOBJ_JOINT                        # joint
    mjOBJ_DOF                          # dof
    mjOBJ_GEOM                         # geom
    mjOBJ_SITE                         # site
    mjOBJ_CAMERA                       # camera
    mjOBJ_LIGHT                        # light
    mjOBJ_MESH                         # mesh
    mjOBJ_SKIN                         # skin
    mjOBJ_HFIELD                       # heightfield
    mjOBJ_TEXTURE                      # texture
    mjOBJ_MATERIAL                     # material for rendering
    mjOBJ_PAIR                         # geom pair to include
    mjOBJ_EXCLUDE                      # body pair to exclude
    mjOBJ_EQUALITY                     # equality constraint
    mjOBJ_TENDON                       # tendon
    mjOBJ_ACTUATOR                     # actuator
    mjOBJ_SENSOR                       # sensor
    mjOBJ_NUMERIC                      # numeric
    mjOBJ_TEXT                         # text
    mjOBJ_TUPLE                        # tuple
    mjOBJ_KEY                          # keyframe
end

@cenum mjtConstraint::Cint begin      # type of constraint
    mjCNSTR_EQUALITY = 0            # equality constraint
    mjCNSTR_FRICTION_DOF               # dof friction
    mjCNSTR_FRICTION_TENDON            # tendon friction
    mjCNSTR_LIMIT_JOINT                # joint limit
    mjCNSTR_LIMIT_TENDON               # tendon limit
    mjCNSTR_CONTACT_FRICTIONLESS       # frictionless contact
    mjCNSTR_CONTACT_PYRAMIDAL          # frictional contact, pyramidal friction cone
    mjCNSTR_CONTACT_ELLIPTIC           # frictional contact elliptic friction cone
end

@cenum mjtConstraintState::Cint begin # constraint state
    mjCNSTRSTATE_SATISFIED = 0         # constraint satisfied, zero cost (limit, contact)
    mjCNSTRSTATE_QUADRATIC             # quadratic cost (equality, friction, limit, contact)
    mjCNSTRSTATE_LINEARNEG             # linear cost, negative side (friction)
    mjCNSTRSTATE_LINEARPOS             # linear cost, positive side (friction)
    mjCNSTRSTATE_CONE                  # squared distance to cone cost (elliptic contact)
end

@cenum mjtSensor::Cint begin          # type of sensor
                                     # common robotic sensors attached to a site
    mjSENS_TOUCH = 0            # scalar contact normal forces summed over sensor zone
    mjSENS_ACCELEROMETER               # 3D linear acceleration, in local frame
    mjSENS_VELOCIMETER                 # 3D linear velocity, in local frame
    mjSENS_GYRO                        # 3D angular velocity, in local frame
    mjSENS_FORCE                       # 3D force between site's body and its parent body
    mjSENS_TORQUE                      # 3D torque between site's body and its parent body
    mjSENS_MAGNETOMETER                # 3D magnetometer
    mjSENS_RANGEFINDER                 # scalar distance to nearest geom or site along z-axis

                                     # sensors related to scalar joints tendons, actuators
    mjSENS_JOINTPOS                    # scalar joint position (hinge and slide only)
    mjSENS_JOINTVEL                    # scalar joint velocity (hinge and slide only)
    mjSENS_TENDONPOS                   # scalar tendon position
    mjSENS_TENDONVEL                   # scalar tendon velocity
    mjSENS_ACTUATORPOS                 # scalar actuator position
    mjSENS_ACTUATORVEL                 # scalar actuator velocity
    mjSENS_ACTUATORFRC                 # scalar actuator force

                                     # sensors related to ball joints
    mjSENS_BALLQUAT                    # 4D ball joint quaterion
    mjSENS_BALLANGVEL                  # 3D ball joint angular velocity

                                     # joint and tendon limit sensors in constraint space
    mjSENS_JOINTLIMITPOS               # joint limit distance-margin
    mjSENS_JOINTLIMITVEL               # joint limit velocity
    mjSENS_JOINTLIMITFRC               # joint limit force
    mjSENS_TENDONLIMITPOS              # tendon limit distance-margin
    mjSENS_TENDONLIMITVEL              # tendon limit velocity
    mjSENS_TENDONLIMITFRC              # tendon limit force

                                     # sensors attached to an object with spatial frame: (x)body geom, site, camera
    mjSENS_FRAMEPOS                    # 3D position
    mjSENS_FRAMEQUAT                   # 4D unit quaternion orientation
    mjSENS_FRAMEXAXIS                  # 3D unit vector: x-axis of object's frame
    mjSENS_FRAMEYAXIS                  # 3D unit vector: y-axis of object's frame
    mjSENS_FRAMEZAXIS                  # 3D unit vector: z-axis of object's frame
    mjSENS_FRAMELINVEL                 # 3D linear velocity
    mjSENS_FRAMEANGVEL                 # 3D angular velocity
    mjSENS_FRAMELINACC                 # 3D linear acceleration
    mjSENS_FRAMEANGACC                 # 3D angular acceleration

                                     # sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    mjSENS_SUBTREECOM                  # 3D center of mass of subtree
    mjSENS_SUBTREELINVEL               # 3D linear velocity of subtree
    mjSENS_SUBTREEANGMOM               # 3D angular momentum of subtree

                                     # user-defined sensor
    mjSENS_USER                        # sensor data provided by mjcb_sensor callback
end

@cenum mjtStage::Cint begin           # computation stage
    mjSTAGE_NONE = 0            # no computations
    mjSTAGE_POS                        # position-dependent computations
    mjSTAGE_VEL                        # velocity-dependent computations
    mjSTAGE_ACC                        # acceleration/force-dependent computations
end

@cenum mjtDataType::Cint begin        # data type for sensors
    mjDATATYPE_REAL = 0            # real values, no constraints
    mjDATATYPE_POSITIVE                # positive values; 0 or negative: inactive
    mjDATATYPE_AXIS                    # 3D unit vector
    mjDATATYPE_QUATERNION              # unit quaternion
end

@cenum mjtLRMode::Cint begin          # mode for actuator length range computation
    mjLRMODE_NONE = 0                # do not process any actuators
    mjLRMODE_MUSCLE                    # process muscle actuators
    mjLRMODE_MUSCLEUSER                # process muscle and user actuators
    mjLRMODE_ALL                       # process all actuators
end

@uninitable mutable struct mjLROpt # options for mj_setLengthRange()
    # flags
    mode::Cint         # which actuators to process (mjtLRMode)
    useexisting::Cint  # use existing length range if available
    uselimit::Cint     # use joint and tendon limits if available

    # algorithm parameters
    accel::mjtNum      # target acceleration used to compute force
    maxforce::mjtNum   # maximum force 0: no limit
    timeconst::mjtNum  # time constant for velocity reduction min 0.01
    timestep::mjtNum   # simulation timestep 0: use mjOption.timestep
    inttotal::mjtNum   # total simulation time interval
    inteval::mjtNum    # evaluation time interval (at the end)
    tolrange::mjtNum   # convergence tolerance (relative to range)
end

@uninitable mutable struct mjVFS
    nfile::Cint
    filename::SVector{mjMAXVFS,SVector{mjMAXVFSNAME,UInt8}}
    filesize::SVector{mjMAXVFS,Cint}
    filedata::SVector{mjMAXVFS,Ptr{Cvoid}}
end

@uninitable struct mjOption
    timestep::mjtNum
    apirate::mjtNum

    impratio::mjtNum
    tolerance::mjtNum
    noslip_tolerance::mjtNum
    mpr_tolerance::mjtNum

    gravity::SVector{3,mjtNum}
    wind::SVector{3,mjtNum}
    magnetic::SVector{3,mjtNum}
    density::mjtNum
    viscosity::mjtNum

    o_margin::mjtNum
    o_solref::SVector{mjNREF,mjtNum}
    o_solimp::SVector{mjNIMP,mjtNum}

    integrator::Cint
    collision::Cint
    cone::Cint
    jacobian::Cint
    solver::Cint
    iterations::Cint
    noslip_iterations::Cint
    mpr_iterations::Cint
    disableflags::Cint
    enableflags::Cint
end


# Julia doesn't allow struct definition's inside of structs like in C,
# so we define the inner structs of mjVisual outside, prefixed with `mjVisual_`
@uninitable struct mjVisual_global
    fovy::Cfloat
    ipd::Cfloat
    linewidth::Cfloat
    glow::Cfloat
    offwidth::Cint
    offheight::Cint
end

@uninitable struct mjVisual_quality
    shadowsize::Cint
    offsamples::Cint
    numslices::Cint
    numstacks::Cint
    numquads::Cint
end

@uninitable struct mjVisual_headlight
    ambient::SVector{3,Cfloat}
    diffuse::SVector{3,Cfloat}
    specular::SVector{3,Cfloat}
    active::Cint
end

@uninitable struct mjVisual_map
    stiffness::Cfloat
    stiffnessrot::Cfloat
    force::Cfloat
    torque::Cfloat
    alpha::Cfloat
    fogstart::Cfloat
    fogend::Cfloat
    znear::Cfloat
    zfar::Cfloat
    haze::Cfloat
    shadowclip::Cfloat
    shadowscale::Cfloat
    actuatortendon::Cfloat
end

@uninitable struct mjVisual_scale
    forcewidth::Cfloat
    contactwidth::Cfloat
    contactheight::Cfloat
    connect::Cfloat
    com::Cfloat
    camera::Cfloat
    light::Cfloat
    selectpoint::Cfloat
    jointlength::Cfloat
    jointwidth::Cfloat
    actuatorlength::Cfloat
    actuatorwidth::Cfloat
    framelength::Cfloat
    framewidth::Cfloat
    constraint::Cfloat
    slidercrank::Cfloat
end

@uninitable struct mjVisual_rgba
    fog::SVector{4,Cfloat}
    haze::SVector{4,Cfloat}
    force::SVector{4,Cfloat}
    inertia::SVector{4,Cfloat}
    joint::SVector{4,Cfloat}
    actuator::SVector{4,Cfloat}
    actuatornegative::SVector{4,Cfloat}
    actuatorpositive::SVector{4,Cfloat}
    com::SVector{4,Cfloat}
    camera::SVector{4,Cfloat}
    light::SVector{4,Cfloat}
    selectpoint::SVector{4,Cfloat}
    connect::SVector{4,Cfloat}
    contactpoint::SVector{4,Cfloat}
    contactforce::SVector{4,Cfloat}
    contactfriction::SVector{4,Cfloat}
    contacttorque::SVector{4,Cfloat}
    contactgap::SVector{4,Cfloat}
    rangefinder::SVector{4,Cfloat}
    constraint::SVector{4,Cfloat}
    slidercrank::SVector{4,Cfloat}
    crankbroken::SVector{4,Cfloat}
end

@uninitable struct mjVisual
    _global::mjVisual_global
    quality::mjVisual_quality
    headlight::mjVisual_headlight
    map::mjVisual_map
    scale::mjVisual_scale
    rgba::mjVisual_rgba
end


@uninitable struct mjStatistic
    meaninertia::mjtNum
    meanmass::mjtNum
    meansize::mjtNum
    extent::mjtNum
    center::SVector{3,mjtNum}
end

# TODO immutable
mutable struct mjModel
    nq::Cint
    nv::Cint
    nu::Cint
    na::Cint
    nbody::Cint
    njnt::Cint
    ngeom::Cint
    nsite::Cint
    ncam::Cint
    nlight::Cint
    nmesh::Cint
    nmeshvert::Cint
    nmeshtexvert::Cint
    nmeshface::Cint
    nmeshgraph::Cint
    nskin::Cint
    nskinvert::Cint
    nskintexvert::Cint
    nskinface::Cint
    nskinbone::Cint
    nskinbonevert::Cint
    nhfield::Cint
    nhfielddata::Cint
    ntex::Cint
    ntexdata::Cint
    nmat::Cint
    npair::Cint
    nexclude::Cint
    neq::Cint
    ntendon::Cint
    nwrap::Cint
    nsensor::Cint
    nnumeric::Cint
    nnumericdata::Cint
    ntext::Cint
    ntextdata::Cint
    ntuple::Cint
    ntupledata::Cint
    nkey::Cint
    nuser_body::Cint
    nuser_jnt::Cint
    nuser_geom::Cint
    nuser_site::Cint
    nuser_cam::Cint
    nuser_tendon::Cint
    nuser_actuator::Cint
    nuser_sensor::Cint
    nnames::Cint

    nM::Cint
    nemax::Cint
    njmax::Cint
    nconmax::Cint
    nstack::Cint
    nuserdata::Cint
    nmocap::Cint
    nsensordata::Cint

    nbuffer::Cint

    opt::mjOption
    vis::mjVisual
    stat::mjStatistic

    buffer::Ptr{Cvoid}

    qpos0::Ptr{mjtNum}
    qpos_spring::Ptr{mjtNum}

    body_parentid::Ptr{Cint}
    body_rootid::Ptr{Cint}
    body_weldid::Ptr{Cint}
    body_mocapid::Ptr{Cint}
    body_jntnum::Ptr{Cint}
    body_jntadr::Ptr{Cint}
    body_dofnum::Ptr{Cint}
    body_dofadr::Ptr{Cint}
    body_geomnum::Ptr{Cint}
    body_geomadr::Ptr{Cint}
    body_simple::Ptr{mjtByte}
    body_sameframe::Ptr{mjtByte}
    body_pos::Ptr{mjtNum}
    body_quat::Ptr{mjtNum}
    body_ipos::Ptr{mjtNum}
    body_iquat::Ptr{mjtNum}
    body_mass::Ptr{mjtNum}
    body_subtreemass::Ptr{mjtNum}
    body_inertia::Ptr{mjtNum}
    body_invweight0::Ptr{mjtNum}
    body_user::Ptr{mjtNum}

    jnt_type::Ptr{Cint}
    jnt_qposadr::Ptr{Cint}
    jnt_dofadr::Ptr{Cint}
    jnt_bodyid::Ptr{Cint}
    jnt_group::Ptr{Cint}
    jnt_limited::Ptr{mjtByte}
    jnt_solref::Ptr{mjtNum}
    jnt_solimp::Ptr{mjtNum}
    jnt_pos::Ptr{mjtNum}
    jnt_axis::Ptr{mjtNum}
    jnt_stiffness::Ptr{mjtNum}
    jnt_range::Ptr{mjtNum}
    jnt_margin::Ptr{mjtNum}
    jnt_user::Ptr{mjtNum}

    dof_bodyid::Ptr{Cint}
    dof_jntid::Ptr{Cint}
    dof_parentid::Ptr{Cint}
    dof_Madr::Ptr{Cint}
    dof_simplenum::Ptr{Cint}
    dof_solref::Ptr{mjtNum}
    dof_solimp::Ptr{mjtNum}
    dof_frictionloss::Ptr{mjtNum}
    dof_armature::Ptr{mjtNum}
    dof_damping::Ptr{mjtNum}
    dof_invweight0::Ptr{mjtNum}
    dof_M0::Ptr{mjtNum}

    geom_type::Ptr{Cint}
    geom_contype::Ptr{Cint}
    geom_conaffinity::Ptr{Cint}
    geom_condim::Ptr{Cint}
    geom_bodyid::Ptr{Cint}
    geom_dataid::Ptr{Cint}
    geom_matid::Ptr{Cint}
    geom_group::Ptr{Cint}
    geom_priority::Ptr{Cint}
    geom_sameframe::Ptr{mjtByte}
    geom_solmix::Ptr{mjtNum}
    geom_solref::Ptr{mjtNum}
    geom_solimp::Ptr{mjtNum}
    geom_size::Ptr{mjtNum}
    geom_rbound::Ptr{mjtNum}
    geom_pos::Ptr{mjtNum}
    geom_quat::Ptr{mjtNum}
    geom_friction::Ptr{mjtNum}
    geom_margin::Ptr{mjtNum}
    geom_gap::Ptr{mjtNum}
    geom_user::Ptr{mjtNum}
    geom_rgba::Ptr{Cfloat}

    site_type::Ptr{Cint}
    site_bodyid::Ptr{Cint}
    site_matid::Ptr{Cint}
    site_group::Ptr{Cint}
    site_sameframe::Ptr{mjtByte}
    site_size::Ptr{mjtNum}
    site_pos::Ptr{mjtNum}
    site_quat::Ptr{mjtNum}
    site_user::Ptr{mjtNum}
    site_rgba::Ptr{Cfloat}

    cam_mode::Ptr{Cint}
    cam_bodyid::Ptr{Cint}
    cam_targetbodyid::Ptr{Cint}
    cam_pos::Ptr{mjtNum}
    cam_quat::Ptr{mjtNum}
    cam_poscom0::Ptr{mjtNum}
    cam_pos0::Ptr{mjtNum}
    cam_mat0::Ptr{mjtNum}
    cam_fovy::Ptr{mjtNum}
    cam_ipd::Ptr{mjtNum}
    cam_user::Ptr{mjtNum}

    light_mode::Ptr{Cint}
    light_bodyid::Ptr{Cint}
    light_targetbodyid::Ptr{Cint}
    light_directional::Ptr{mjtByte}
    light_castshadow::Ptr{mjtByte}
    light_active::Ptr{mjtByte}
    light_pos::Ptr{mjtNum}
    light_dir::Ptr{mjtNum}
    light_poscom0::Ptr{mjtNum}
    light_pos0::Ptr{mjtNum}
    light_dir0::Ptr{mjtNum}
    light_attenuation::Ptr{Cfloat}
    light_cutoff::Ptr{Cfloat}
    light_exponent::Ptr{Cfloat}
    light_ambient::Ptr{Cfloat}
    light_diffuse::Ptr{Cfloat}
    light_specular::Ptr{Cfloat}

    mesh_vertadr::Ptr{Cint}
    mesh_vertnum::Ptr{Cint}
    mesh_texcoordadr::Ptr{Cint}
    mesh_faceadr::Ptr{Cint}
    mesh_facenum::Ptr{Cint}
    mesh_graphadr::Ptr{Cint}
    mesh_vert::Ptr{Cfloat}
    mesh_normal::Ptr{Cfloat}
    mesh_texcoord::Ptr{Cfloat}
    mesh_face::Ptr{Cint}
    mesh_graph::Ptr{Cint}

    skin_matid::Ptr{Cint}
    skin_rgba::Ptr{Cfloat}
    skin_inflate::Ptr{Cfloat}
    skin_vertadr::Ptr{Cint}
    skin_vertnum::Ptr{Cint}
    skin_texcoordadr::Ptr{Cint}
    skin_faceadr::Ptr{Cint}
    skin_facenum::Ptr{Cint}
    skin_boneadr::Ptr{Cint}
    skin_bonenum::Ptr{Cint}
    skin_vert::Ptr{Cfloat}
    skin_texcoord::Ptr{Cfloat}
    skin_face::Ptr{Cint}
    skin_bonevertadr::Ptr{Cint}
    skin_bonevertnum::Ptr{Cint}
    skin_bonebindpos::Ptr{Cfloat}
    skin_bonebindquat::Ptr{Cfloat}
    skin_bonebodyid::Ptr{Cint}
    skin_bonevertid::Ptr{Cint}
    skin_bonevertweight::Ptr{Cfloat}

    hfield_size::Ptr{mjtNum}
    hfield_nrow::Ptr{Cint}
    hfield_ncol::Ptr{Cint}
    hfield_adr::Ptr{Cint}
    hfield_data::Ptr{Cfloat}

    tex_type::Ptr{Cint}
    tex_height::Ptr{Cint}
    tex_width::Ptr{Cint}
    tex_adr::Ptr{Cint}
    tex_rgb::Ptr{mjtByte}

    mat_texid::Ptr{Cint}
    mat_texuniform::Ptr{mjtByte}
    mat_texrepeat::Ptr{Cfloat}
    mat_emission::Ptr{Cfloat}
    mat_specular::Ptr{Cfloat}
    mat_shininess::Ptr{Cfloat}
    mat_reflectance::Ptr{Cfloat}
    mat_rgba::Ptr{Cfloat}

    pair_dim::Ptr{Cint}
    pair_geom1::Ptr{Cint}
    pair_geom2::Ptr{Cint}
    pair_signature::Ptr{Cint}
    pair_solref::Ptr{mjtNum}
    pair_solimp::Ptr{mjtNum}
    pair_margin::Ptr{mjtNum}
    pair_gap::Ptr{mjtNum}
    pair_friction::Ptr{mjtNum}

    exclude_signature::Ptr{Cint}

    eq_type::Ptr{Cint}
    eq_obj1id::Ptr{Cint}
    eq_obj2id::Ptr{Cint}
    eq_active::Ptr{mjtByte}
    eq_solref::Ptr{mjtNum}
    eq_solimp::Ptr{mjtNum}
    eq_data::Ptr{mjtNum}

    tendon_adr::Ptr{Cint}
    tendon_num::Ptr{Cint}
    tendon_matid::Ptr{Cint}
    tendon_group::Ptr{Cint}
    tendon_limited::Ptr{mjtByte}
    tendon_width::Ptr{mjtNum}
    tendon_solref_lim::Ptr{mjtNum}
    tendon_solimp_lim::Ptr{mjtNum}
    tendon_solref_fri::Ptr{mjtNum}
    tendon_solimp_fri::Ptr{mjtNum}
    tendon_range::Ptr{mjtNum}
    tendon_margin::Ptr{mjtNum}
    tendon_stiffness::Ptr{mjtNum}
    tendon_damping::Ptr{mjtNum}
    tendon_frictionloss::Ptr{mjtNum}
    tendon_lengthspring::Ptr{mjtNum}
    tendon_length0::Ptr{mjtNum}
    tendon_invweight0::Ptr{mjtNum}
    tendon_user::Ptr{mjtNum}
    tendon_rgba::Ptr{Cfloat}

    wrap_type::Ptr{Cint}
    wrap_objid::Ptr{Cint}
    wrap_prm::Ptr{mjtNum}

    actuator_trntype::Ptr{Cint}
    actuator_dyntype::Ptr{Cint}
    actuator_gaintype::Ptr{Cint}
    actuator_biastype::Ptr{Cint}
    actuator_trnid::Ptr{Cint}
    actuator_group::Ptr{Cint}
    actuator_ctrllimited::Ptr{mjtByte}
    actuator_forcelimited::Ptr{mjtByte}
    actuator_dynprm::Ptr{mjtNum}
    actuator_gainprm::Ptr{mjtNum}
    actuator_biasprm::Ptr{mjtNum}
    actuator_ctrlrange::Ptr{mjtNum}
    actuator_forcerange::Ptr{mjtNum}
    actuator_gear::Ptr{mjtNum}
    actuator_cranklength::Ptr{mjtNum}
    actuator_acc0::Ptr{mjtNum}
    actuator_length0::Ptr{mjtNum}
    actuator_lengthrange::Ptr{mjtNum}
    actuator_user::Ptr{mjtNum}

    sensor_type::Ptr{Cint}
    sensor_datatype::Ptr{Cint}
    sensor_needstage::Ptr{Cint}
    sensor_objtype::Ptr{Cint}
    sensor_objid::Ptr{Cint}
    sensor_dim::Ptr{Cint}
    sensor_adr::Ptr{Cint}
    sensor_cutoff::Ptr{mjtNum}
    sensor_noise::Ptr{mjtNum}
    sensor_user::Ptr{mjtNum}

    numeric_adr::Ptr{Cint}
    numeric_size::Ptr{Cint}
    numeric_data::Ptr{mjtNum}

    text_adr::Ptr{Cint}
    text_size::Ptr{Cint}
    text_data::Ptr{UInt8}

    tuple_adr::Ptr{Cint}
    tuple_size::Ptr{Cint}
    tuple_objtype::Ptr{Cint}
    tuple_objid::Ptr{Cint}
    tuple_objprm::Ptr{mjtNum}

    key_time::Ptr{mjtNum}
    key_qpos::Ptr{mjtNum}
    key_qvel::Ptr{mjtNum}
    key_act::Ptr{mjtNum}

    name_bodyadr::Ptr{Cint}
    name_jntadr::Ptr{Cint}
    name_geomadr::Ptr{Cint}
    name_siteadr::Ptr{Cint}
    name_camadr::Ptr{Cint}
    name_lightadr::Ptr{Cint}
    name_meshadr::Ptr{Cint}
    name_skinadr::Ptr{Cint}
    name_hfieldadr::Ptr{Cint}
    name_texadr::Ptr{Cint}
    name_matadr::Ptr{Cint}
    name_pairadr::Ptr{Cint}
    name_excludeadr::Ptr{Cint}
    name_eqadr::Ptr{Cint}
    name_tendonadr::Ptr{Cint}
    name_actuatoradr::Ptr{Cint}
    name_sensoradr::Ptr{Cint}
    name_numericadr::Ptr{Cint}
    name_textadr::Ptr{Cint}
    name_tupleadr::Ptr{Cint}
    name_keyadr::Ptr{Cint}
    names::Ptr{UInt8}
end



