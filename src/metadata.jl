struct JLSize{name, T<:Tuple{Symbol, Vararg{Symbol}}}
    labels::T
end
JLSize{name}(labels...) where {name} = JLSize{name, typeof(labels)}(labels)
Base.nameof(::JLSize{name}) where name = name
Base.length(x::JLSize) = length(x.labels)

"""
    FieldInfo

Metadata for fields of the various MuJoCo structs.
Sizes are declared in the same order as C/row-major order
and should be reversed in Julia. Trailing/leading singleton
dimensions are dropped.

# Fields
- mjsize: the MuJoCo C size declared
- dsize: the dynamic size for pre-allocated, variable length fields (e.g. `mjData.contact`
    is declared as `(nconmax, )` but should be viewed as `(ncon,)`.
- jlsize: The size for "jl" prefixed structs e.g. `jlData`. By default this is
    `mjsize`, but can be provide new sizes
    (e.g. `mjData.xmat` is `(9, nbody)`` while `jlData.xmat` is `(:xmat, nbody)`
    which allows for named indexing like `jlData.xmat[:xz, :foobody]`
"""
struct FieldInfo
    mjsize::Tuple{Vararg{Union{Int,Symbol}}}
    jlsize::Union{Tuple{Vararg{Union{Int,Symbol, JLSize}}}}
    dsize::Union{Tuple{Vararg{Union{Int,Symbol, JLSize}}},Nothing}
end
FieldInfo(mjsize; jlsize=mjsize, dsize=nothing) = FieldInfo(mjsize, jlsize, dsize)


const Quat = JLSize{:quat}(:w, :x, :y, :z)
const XYZ = JLSize{:xyz}(:x, :y, :z)
const RGBA = JLSize{:rgba}(:r, :b, :g, :a)
const Range = JLSize{:range}(:min, :max)
const FrameAxis = JLSize{:frameaxis}(:x, :y, :z)
const JLSizes = (Quat, XYZ, RGBA, Range, FrameAxis)

frame() = (FrameAxis, XYZ)

# source is the thing which holds the runtime size e.g. mjModel
function lookupsize(structname::Symbol, source, fieldname::Symbol)
    jl_size = reverse(STRUCT_INFO[structname][fieldname].mjsize) # reverse b/c Julia is column order
    Base.map(jl_size) do sz
        sz = sz isa Symbol ? getfield(source, sz) : sz
        Int(sz)
    end
end

function lookupsize(structname::Symbol, fieldname::Symbol)
    jl_size = reverse(STRUCT_INFO[structname][fieldname].mjsize) # reverse b/c Julia is column order
    Base.map(jl_size) do sz
        sz isa Symbol && error("Found Symbol, try lookupsize(structname, source, fieldname)")
        Int(sz)
    end
end

@generated function lookup_dsize(
    ::Val{structname},
    source,
    ::Val{fieldname},
) where {structname,fieldname}
    jl_size = reverse(STRUCT_INFO[structname][fieldname].dsize) # reverse b/c Julia is column order
    args = Base.map(jl_size) do sz
        sz = sz isa Symbol ? :(Int(getfield(source, $(QuoteNode(sz))))) : sz
    end
    N = length(args)
    :($(Expr(:tuple, args...))::NTuple{$N,Int})
end

const STRUCT_INFO = Dict{Symbol,Dict{Symbol,<:FieldInfo}}(
    :mjModel => Dict(
        :tex_rgb => FieldInfo((:ntexdata,)),
        :jnt_bodyid => FieldInfo((:njnt,)),
        :tendon_group => FieldInfo((:ntendon,)),
        :mesh_faceadr => FieldInfo((:nmesh,)),
        :name_excludeadr => FieldInfo((:nexclude,)),
        :tuple_objid => FieldInfo((:ntupledata,)),
        :actuator_forcerange => FieldInfo((:nu, 2), jlsize=(:nu, Range)),
        :sensor_noise => FieldInfo((:nsensor,)),
        :tendon_range => FieldInfo((:ntendon, 2), jlsize=(:ntendon, Range)),
        :body_ipos => FieldInfo((:nbody, 3), jlsize=(:nbody, XYZ)),
        :body_dofadr => FieldInfo((:nbody,)),
        :geom_solmix => FieldInfo((:ngeom,)),
        :skin_vert => FieldInfo((:nskinvert, 3), jlsize=(:nskinvert, XYZ)),
        :wrap_type => FieldInfo((:nwrap,)),
        :name_lightadr => FieldInfo((:nlight,)),
        :skin_bonenum => FieldInfo((:nskin,)),
        :sensor_type => FieldInfo((:nsensor,)),
        :tuple_objtype => FieldInfo((:ntupledata,)),
        :hfield_nrow => FieldInfo((:nhfield,)),
        :body_dofnum => FieldInfo((:nbody,)),
        :key_qpos => FieldInfo((:nkey, :nq)),
        :names => FieldInfo((:nnames,)),
        :tendon_margin => FieldInfo((:ntendon,)),
        :skin_bonebodyid => FieldInfo((:nskinbone,)),
        :light_ambient => FieldInfo((:nlight, 3)),
        :tuple_adr => FieldInfo((:ntuple,)),
        :geom_priority => FieldInfo((:ngeom,)),
        :dof_solimp => FieldInfo((:nv, 5)),
        :geom_size => FieldInfo((:ngeom, 3)),
        :mesh_texcoordadr => FieldInfo((:nmesh,)),
        :eq_active => FieldInfo((:neq,)),
        :actuator_forcelimited => FieldInfo((:nu,)),
        :body_mass => FieldInfo((:nbody,)),
        :skin_bonebindpos => FieldInfo((:nskinbone, 3), jlsize=(:nskinbone, XYZ)),
        :hfield_ncol => FieldInfo((:nhfield,)),
        :cam_fovy => FieldInfo((:ncam,)),
        :name_geomadr => FieldInfo((:ngeom,)),
        :light_castshadow => FieldInfo((:nlight,)),
        :tendon_invweight0 => FieldInfo((:ntendon,)),
        :jnt_limited => FieldInfo((:njnt,)),
        :eq_type => FieldInfo((:neq,)),
        :body_mocapid => FieldInfo((:nbody,)),
        :body_inertia => FieldInfo((:nbody, 3)),
        :skin_faceadr => FieldInfo((:nskin,)),
        :name_textadr => FieldInfo((:ntext,)),
        :skin_texcoord => FieldInfo((:nskintexvert, 2)),
        :skin_face => FieldInfo((:nskinface, 3)),
        :mesh_facenum => FieldInfo((:nmesh,)),
        :sensor_objid => FieldInfo((:nsensor,)),
        :body_invweight0 => FieldInfo((:nbody, 2)),
        :cam_bodyid => FieldInfo((:ncam,)),
        :light_pos => FieldInfo((:nlight, 3), jlsize=(:nlight, XYZ)),
        :light_attenuation => FieldInfo((:nlight, 3)),
        :site_pos => FieldInfo((:nsite, 3), jlsize=(:nsite, XYZ)),
        :dof_jntid => FieldInfo((:nv,)),
        :mesh_texcoord => FieldInfo((:nmeshtexvert, 2)),
        :jnt_solimp => FieldInfo((:njnt, 5)),
        :name_skinadr => FieldInfo((:nskin,)),
        :mesh_vertadr => FieldInfo((:nmesh,)),
        :geom_matid => FieldInfo((:ngeom,)),
        :dof_bodyid => FieldInfo((:nv,)),
        :tex_width => FieldInfo((:ntex,)),
        :tendon_stiffness => FieldInfo((:ntendon,)),
        :body_jntadr => FieldInfo((:nbody,)),
        :tex_type => FieldInfo((:ntex,)),
        :jnt_dofadr => FieldInfo((:njnt,)),
        :jnt_range => FieldInfo((:njnt, 2)),
        :tendon_solref_lim => FieldInfo((:ntendon, 2)),
        :pair_solimp => FieldInfo((:npair, 5)),
        :cam_poscom0 => FieldInfo((:ncam, 3), jlsize=(:ncam, XYZ)),
        :cam_mat0 => FieldInfo((:ncam, 9)),
        :light_cutoff => FieldInfo((:nlight,)),
        :key_qvel => FieldInfo((:nkey, :nv)),
        :hfield_adr => FieldInfo((:nhfield,)),
        :site_bodyid => FieldInfo((:nsite,)),
        :skin_bonevertadr => FieldInfo((:nskinbone,)),
        :body_weldid => FieldInfo((:nbody,)),
        :geom_dataid => FieldInfo((:ngeom,)),
        :jnt_pos => FieldInfo((:njnt, 3), jlsize=(:njnt, XYZ)),
        :name_pairadr => FieldInfo((:npair,)),
        :site_type => FieldInfo((:nsite,)),
        :name_eqadr => FieldInfo((:neq,)),
        :tendon_length0 => FieldInfo((:ntendon,)),
        :dof_parentid => FieldInfo((:nv,)),
        :name_texadr => FieldInfo((:ntex,)),
        :light_dir => FieldInfo((:nlight, 3)),
        :dof_solref => FieldInfo((:nv, 2)),
        :skin_bonevertnum => FieldInfo((:nskinbone,)),
        :text_size => FieldInfo((:ntext,)),
        :mat_rgba => FieldInfo((:nmat, 4), jlsize=(:nmat, RGBA)),
        :body_sameframe => FieldInfo((:nbody,)),
        :exclude_signature => FieldInfo((:nexclude,)),
        :mesh_vert => FieldInfo((:nmeshvert, 3), jlsize=(:nmeshvert, XYZ)),
        :body_geomadr => FieldInfo((:nbody,)),
        :dof_M0 => FieldInfo((:nv,)),
        :mat_reflectance => FieldInfo((:nmat,)),
        :geom_quat => FieldInfo((:ngeom, 4)),
        :jnt_qposadr => FieldInfo((:njnt,)),
        :actuator_dynprm => FieldInfo((:nu, 10)),
        :pair_gap => FieldInfo((:npair,)),
        :actuator_gainprm => FieldInfo((:nu, 10)),
        :skin_bonevertid => FieldInfo((:nskinbonevert,)),
        :name_bodyadr => FieldInfo((:nbody,)),
        :cam_pos0 => FieldInfo((:ncam, 3), jlsize=(:ncam, XYZ)),
        :light_bodyid => FieldInfo((:nlight,)),
        :mesh_normal => FieldInfo((:nmeshvert, 3)),
        :name_hfieldadr => FieldInfo((:nhfield,)),
        :mesh_vertnum => FieldInfo((:nmesh,)),
        :mesh_graphadr => FieldInfo((:nmesh,)),
        :mesh_face => FieldInfo((:nmeshface, 3)),
        :skin_facenum => FieldInfo((:nskin,)),
        :tendon_num => FieldInfo((:ntendon,)),
        :hfield_data => FieldInfo((:nhfielddata,)),
        :text_adr => FieldInfo((:ntext,)),
        :cam_user => FieldInfo((:ncam, :nuser_cam)),
        :tendon_rgba => FieldInfo((:ntendon, 4)),
        :geom_pos => FieldInfo((:ngeom, 3)),
        :text_data => FieldInfo((:ntextdata,)),
        :geom_bodyid => FieldInfo((:ngeom,)),
        :skin_bonevertweight => FieldInfo((:nskinbonevert,)),
        :body_quat => FieldInfo((:nbody, 4)),
        :tendon_lengthspring => FieldInfo((:ntendon,)),
        :light_mode => FieldInfo((:nlight,)),
        :skin_bonebindquat => FieldInfo((:nskinbone, 4)),
        :name_tendonadr => FieldInfo((:ntendon,)),
        :light_diffuse => FieldInfo((:nlight, 3)),
        :tuple_size => FieldInfo((:ntuple,)),
        :skin_rgba => FieldInfo((:nskin, 4)),
        :geom_group => FieldInfo((:ngeom,)),
        :key_act => FieldInfo((:nkey, :na)),
        :geom_margin => FieldInfo((:ngeom,)),
        :body_iquat => FieldInfo((:nbody, 4)),
        :dof_invweight0 => FieldInfo((:nv,)),
        :cam_quat => FieldInfo((:ncam, 4)),
        :skin_vertadr => FieldInfo((:nskin,)),
        :actuator_gaintype => FieldInfo((:nu,)),
        :geom_solimp => FieldInfo((:ngeom, 5)),
        :dof_frictionloss => FieldInfo((:nv,)),
        :body_geomnum => FieldInfo((:nbody,)),
        :sensor_needstage => FieldInfo((:nsensor,)),
        :actuator_group => FieldInfo((:nu,)),
        :sensor_adr => FieldInfo((:nsensor,)),
        :eq_data => FieldInfo((:neq, 7)),
        :geom_user => FieldInfo((:ngeom, :nuser_geom)),
        :body_user => FieldInfo((:nbody, :nuser_body)),
        :skin_boneadr => FieldInfo((:nskin,)),
        :jnt_solref => FieldInfo((:njnt, 2)),
        :body_pos => FieldInfo((:nbody, 3)),
        :sensor_objtype => FieldInfo((:nsensor,)),
        :dof_Madr => FieldInfo((:nv,)),
        :site_quat => FieldInfo((:nsite, 4)),
        :sensor_datatype => FieldInfo((:nsensor,)),
        :dof_damping => FieldInfo((:nv,)),
        :site_user => FieldInfo((:nsite, :nuser_site)),
        :actuator_dyntype => FieldInfo((:nu,)),
        :pair_geom1 => FieldInfo((:npair,)),
        :light_specular => FieldInfo((:nlight, 3)),
        :jnt_axis => FieldInfo((:njnt, 3)),
        :mat_texid => FieldInfo((:nmat,)),
        :name_numericadr => FieldInfo((:nnumeric,)),
        :cam_ipd => FieldInfo((:ncam,)),
        :site_size => FieldInfo((:nsite, 3)),
        :geom_sameframe => FieldInfo((:ngeom,)),
        :tendon_limited => FieldInfo((:ntendon,)),
        :tendon_solref_fri => FieldInfo((:ntendon, 2)),
        :eq_obj2id => FieldInfo((:neq,)),
        :site_sameframe => FieldInfo((:nsite,)),
        :tex_adr => FieldInfo((:ntex,)),
        :tendon_damping => FieldInfo((:ntendon,)),
        :name_matadr => FieldInfo((:nmat,)),
        :geom_gap => FieldInfo((:ngeom,)),
        :sensor_dim => FieldInfo((:nsensor,)),
        :dof_armature => FieldInfo((:nv,)),
        :geom_conaffinity => FieldInfo((:ngeom,)),
        :name_actuatoradr => FieldInfo((:nu,)),
        :cam_targetbodyid => FieldInfo((:ncam,)),
        :geom_type => FieldInfo((:ngeom,)),
        :actuator_biastype => FieldInfo((:nu,)),
        :key_time => FieldInfo((:nkey,)),
        :body_subtreemass => FieldInfo((:nbody,)),
        :skin_inflate => FieldInfo((:nskin,)),
        :mat_shininess => FieldInfo((:nmat,)),
        :site_rgba => FieldInfo((:nsite, 4)),
        :jnt_user => FieldInfo((:njnt, :nuser_jnt)),
        :tendon_solimp_lim => FieldInfo((:ntendon, 5)),
        :cam_mode => FieldInfo((:ncam,)),
        :jnt_margin => FieldInfo((:njnt,)),
        :pair_signature => FieldInfo((:npair,)),
        :name_meshadr => FieldInfo((:nmesh,)),
        :wrap_prm => FieldInfo((:nwrap,)),
        :numeric_data => FieldInfo((:nnumericdata,)),
        :sensor_cutoff => FieldInfo((:nsensor,)),
        :site_matid => FieldInfo((:nsite,)),
        :hfield_size => FieldInfo((:nhfield, 4)),
        :sensor_user => FieldInfo((:nsensor, :nuser_sensor)),
        :actuator_trnid => FieldInfo((:nu, 2)),
        :name_camadr => FieldInfo((:ncam,)),
        :geom_solref => FieldInfo((:ngeom, 2)),
        :qpos_spring => FieldInfo((:nq,)),
        :cam_pos => FieldInfo((:ncam, 3)),
        :name_jntadr => FieldInfo((:njnt,)),
        :actuator_acc0 => FieldInfo((:nu,)),
        :jnt_stiffness => FieldInfo((:njnt,)),
        :body_jntnum => FieldInfo((:nbody,)),
        :actuator_length0 => FieldInfo((:nu,)),
        :pair_geom2 => FieldInfo((:npair,)),
        :name_siteadr => FieldInfo((:nsite,)),
        :actuator_trntype => FieldInfo((:nu,)),
        :dof_simplenum => FieldInfo((:nv,)),
        :pair_friction => FieldInfo((:npair, 5)),
        :name_keyadr => FieldInfo((:nkey,)),
        :tendon_user => FieldInfo((:ntendon, :nuser_tendon)),
        :actuator_ctrlrange => FieldInfo((:nu, 2)),
        :body_simple => FieldInfo((:nbody,)),
        :actuator_biasprm => FieldInfo((:nu, 10)),
        :tendon_frictionloss => FieldInfo((:ntendon,)),
        :actuator_cranklength => FieldInfo((:nu,)),
        :light_active => FieldInfo((:nlight,)),
        :mat_specular => FieldInfo((:nmat,)),
        :numeric_size => FieldInfo((:nnumeric,)),
        :tendon_adr => FieldInfo((:ntendon,)),
        :actuator_ctrllimited => FieldInfo((:nu,)),
        :geom_rgba => FieldInfo((:ngeom, 4)),
        :light_directional => FieldInfo((:nlight,)),
        :body_parentid => FieldInfo((:nbody,)),
        :light_dir0 => FieldInfo((:nlight, 3)),
        :tex_height => FieldInfo((:ntex,)),
        :pair_dim => FieldInfo((:npair,)),
        :mat_emission => FieldInfo((:nmat,)),
        :eq_solref => FieldInfo((:neq, 2)),
        :mesh_graph => FieldInfo((:nmeshgraph,)),
        :jnt_group => FieldInfo((:njnt,)),
        :light_poscom0 => FieldInfo((:nlight, 3)),
        :skin_texcoordadr => FieldInfo((:nskin,)),
        :mat_texrepeat => FieldInfo((:nmat, 2)),
        :jnt_type => FieldInfo((:njnt,)),
        :mat_texuniform => FieldInfo((:nmat,)),
        :geom_rbound => FieldInfo((:ngeom,)),
        :qpos0 => FieldInfo((:nq,)),
        :light_targetbodyid => FieldInfo((:nlight,)),
        :actuator_lengthrange => FieldInfo((:nu, 2)),
        :geom_contype => FieldInfo((:ngeom,)),
        :site_group => FieldInfo((:nsite,)),
        :pair_solref => FieldInfo((:npair, 2)),
        :light_pos0 => FieldInfo((:nlight, 3)),
        :tuple_objprm => FieldInfo((:ntupledata,)),
        :tendon_matid => FieldInfo((:ntendon,)),
        :actuator_user => FieldInfo((:nu, :nuser_actuator)),
        :actuator_gear => FieldInfo((:nu, 6)),
        :geom_condim => FieldInfo((:ngeom,)),
        :eq_obj1id => FieldInfo((:neq,)),
        :numeric_adr => FieldInfo((:nnumeric,)),
        :skin_matid => FieldInfo((:nskin,)),
        :tendon_width => FieldInfo((:ntendon,)),
        :tendon_solimp_fri => FieldInfo((:ntendon, 5)),
        :eq_solimp => FieldInfo((:neq, 5)),
        :name_sensoradr => FieldInfo((:nsensor,)),
        :body_rootid => FieldInfo((:nbody,)),
        :geom_friction => FieldInfo((:ngeom, 3)),
        :skin_vertnum => FieldInfo((:nskin,)),
        :name_tupleadr => FieldInfo((:ntuple,)),
        :pair_margin => FieldInfo((:npair,)),
        :wrap_objid => FieldInfo((:nwrap,)),
        :light_exponent => FieldInfo((:nlight,)),
    ),
    :mjData => Dict(
        :qLD => FieldInfo((:nM,)),
        :efc_aref => FieldInfo((:njmax,)),
        :efc_type => FieldInfo((:njmax,)),
        :userdata => FieldInfo((:nuserdata,)),
        :efc_force => FieldInfo((:njmax,)),
        :qfrc_unc => FieldInfo((:nv,)),
        :ten_wrapnum => FieldInfo((:ntendon,)),
        :cinert => FieldInfo((:nbody, 10)),
        :efc_JT_rowadr => FieldInfo((:nv,)),
        :ximat => FieldInfo((:nbody, 9)),
        :actuator_moment => FieldInfo((:nu, :nv)),
        :efc_AR_rownnz => FieldInfo((:njmax,)),
        :efc_J_rowsuper => FieldInfo((:njmax,)),
        :efc_J_rowadr => FieldInfo((:njmax,)),
        :efc_id => FieldInfo((:njmax,)),
        :qacc_unc => FieldInfo((:nv,)),
        :efc_margin => FieldInfo((:njmax,)),
        :mocap_quat => FieldInfo((:nmocap, 4), jlsize=(:nmocap, Quat)),
        :ten_length => FieldInfo((:ntendon,)),
        :xipos => FieldInfo((:nbody, 3)),
        :actuator_velocity => FieldInfo((:nu,)),
        :light_xpos => FieldInfo((:nlight, 3)),
        :xmat => FieldInfo((:nbody, 9), jlsize=(:nbody, frame()...)),
        :act => FieldInfo((:na,)),
        :efc_D => FieldInfo((:njmax,)),
        :qfrc_applied => FieldInfo((:nv,)),
        :ten_wrapadr => FieldInfo((:ntendon,)),
        :efc_frictionloss => FieldInfo((:njmax,)),
        :efc_JT_rownnz => FieldInfo((:nv,)),
        :cdof_dot => FieldInfo((:nv, 6)),
        :ten_J_rowadr => FieldInfo((:ntendon,)),
        :cdof => FieldInfo((:nv, 6)),
        :efc_JT => FieldInfo((:nv, :njmax)),
        :efc_b => FieldInfo((:njmax,)),
        :qfrc_bias => FieldInfo((:nv,)),
        :qacc => FieldInfo((:nv,)),
        :qfrc_passive => FieldInfo((:nv,)),
        :efc_AR => FieldInfo((:njmax, :njmax)),
        :geom_xpos => FieldInfo((:ngeom, 3)),
        :mocap_pos => FieldInfo((:nmocap, 3)),
        :qacc_warmstart => FieldInfo((:nv,)),
        :efc_KBIP => FieldInfo((:njmax, 4)),
        :cam_xpos => FieldInfo((:ncam, 3)),
        :qvel => FieldInfo((:nv,)),
        :ctrl => FieldInfo((:nu,)),
        :subtree_com => FieldInfo((:nbody, 3)),
        :efc_pos => FieldInfo((:njmax,)),
        :site_xmat => FieldInfo((:nsite, 9)),
        :wrap_xpos => FieldInfo((:nwrap, 6)),
        :qM => FieldInfo((:nM,)),
        :cam_xmat => FieldInfo((:ncam, 9)),
        :geom_xmat => FieldInfo((:ngeom, 9)),
        :xaxis => FieldInfo((:njnt, 3)),
        :efc_diagApprox => FieldInfo((:njmax,)),
        :crb => FieldInfo((:nbody, 10)),
        :efc_JT_colind => FieldInfo((:nv, :njmax)),
        :efc_J => FieldInfo((:njmax, :nv)),
        :qfrc_constraint => FieldInfo((:nv,)),
        :contact => FieldInfo((:nconmax,), dsize=(:ncon,)),
        :efc_state => FieldInfo((:njmax,)),
        :wrap_obj => FieldInfo((:nwrap,)),
        :qfrc_inverse => FieldInfo((:nv,)),
        :cfrc_int => FieldInfo((:nbody, 6)),
        :sensordata => FieldInfo((:nsensordata,)),
        :qpos => FieldInfo((:nq,)),
        :qfrc_actuator => FieldInfo((:nv,)),
        :cacc => FieldInfo((:nbody, 6)),
        :light_xdir => FieldInfo((:nlight, 3), jlsize=(:nlight, XYZ)),
        :act_dot => FieldInfo((:na,)),
        :efc_AR_rowadr => FieldInfo((:njmax,)),
        :cfrc_ext => FieldInfo((:nbody, 6)),
        :efc_J_colind => FieldInfo((:njmax, :nv)),
        :efc_AR_colind => FieldInfo((:njmax, :njmax)),
        :actuator_force => FieldInfo((:nu,)),
        :site_xpos => FieldInfo((:nsite, 3), jlsize=(:nsite, XYZ)),
        :subtree_angmom => FieldInfo((:nbody, 3)),
        :qLDiagSqrtInv => FieldInfo((:nv,)),
        :xquat => FieldInfo((:nbody, 4), jlsize=(:nbody, Quat)),
        :ten_J_colind => FieldInfo((:ntendon, :nv)),
        :efc_JT_rowsuper => FieldInfo((:nv,)),
        :xpos => FieldInfo((:nbody, 3), jlsize=(:nbody, XYZ)),
        :cvel => FieldInfo((:nbody, 6)),
        :qLDiagInv => FieldInfo((:nv,)),
        :efc_J_rownnz => FieldInfo((:njmax,)),
        :xanchor => FieldInfo((:njnt, 3), jlsize=(:njnt, XYZ)),
        :ten_velocity => FieldInfo((:ntendon,)),
        :ten_J_rownnz => FieldInfo((:ntendon,)),
        :xfrc_applied => FieldInfo((:nbody, 6)),
        :ten_J => FieldInfo((:ntendon, :nv)),
        :efc_vel => FieldInfo((:njmax,)),
        :subtree_linvel => FieldInfo((:nbody, 3)),
        :actuator_length => FieldInfo((:nu,)),
        :efc_R => FieldInfo((:njmax,)),
    ),
)
