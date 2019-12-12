const OBJTYPE_TO_SIZENAME = Dict(
    MJCore.mjOBJ_BODY => :nbody,
    MJCore.mjOBJ_JOINT => :njnt,
    MJCore.mjOBJ_GEOM => :ngeom,
    MJCore.mjOBJ_SITE => :nsite,
    MJCore.mjOBJ_CAMERA => :ncam,
    MJCore.mjOBJ_LIGHT => :nlight,
    MJCore.mjOBJ_MESH => :nmesh,
    MJCore.mjOBJ_SKIN => :nskin,
    MJCore.mjOBJ_HFIELD => :nhfield,
    MJCore.mjOBJ_TEXTURE => :ntex,
    MJCore.mjOBJ_MATERIAL => :nmat,
    MJCore.mjOBJ_PAIR => :npair,
    MJCore.mjOBJ_EXCLUDE => :nexclude,
    MJCore.mjOBJ_EQUALITY => :neq,
    MJCore.mjOBJ_TENDON => :ntendon,
    MJCore.mjOBJ_ACTUATOR => :nu,
    MJCore.mjOBJ_SENSOR => :nsensor,
    MJCore.mjOBJ_NUMERIC => :nnumeric,
    MJCore.mjOBJ_TEXT => :ntext,
    MJCore.mjOBJ_TUPLE => :ntuple,
    MJCore.mjOBJ_KEY => :nkey,
)


function expandnq(m::jlModel, root, id)
    jtype = m.jnt_type[id]
    if jtype == MJCore.mjJNT_FREE
        map(suffix -> Symbol(root, :_, suffix), (:x, :y, :z, :qw, :qx, :qy, :qz))
    elseif jtype == MJCore.mjJNT_BALL
        map(suffix -> Symbol(root, :_, suffix), (:qw, :qx, :qy, :qz))
    else
        (root,)
    end
end

function expandnv(m::jlModel, root, id)
    jtype = m.jnt_type[id]
    if jtype == MJCore.mjJNT_FREE
        map(suffix -> Symbol(root, :_, suffix), (:x, :y, :z, :rx, :ry, :rz))
    elseif jtype == MJCore.mjJNT_BALL
        map(suffix -> Symbol(root, :_, suffix), (:rx, :ry, :rz))
    else
        (root,)
    end
end

function ragged2linearaxis(expandfn, m::jlModel, donor_labels, ragged_idxs)
    linearlabels = Symbol[]
    for (id, root) in enumerate(donor_labels)
        append!(linearlabels, expandfn(m, root, id)::Tuple)
    end
    linearlabels
end

const RAGGED_SIZENAME_TO_ADREXPANDER = Dict(
    :nq => (:jnt_qposadr, expandnq),
    :nv => (:jnt_dofadr, expandnv),
    #:nsensordata=>:sensor_adr, # can label this instead of _
    #:nnumericdata=>:numeric_adr
)


jl_id2name(m::jlModel, objtype::MJCore.mjtObj, id::Integer) =
    MJCore.mj_id2name(m, objtype, Int32(id - 1))
jl_type2szname(objtype::MJCore.mjtObj) =
    haskey(OBJTYPE_TO_SIZENAME, objtype) ? OBJTYPE_TO_SIZENAME[objtype] : nothing

makeunique(x::Vector{Symbol}; kwargs...) = (makeunique!(copy(x); kwargs...))
makeunique(x::Tuple{Vararg{Symbol}}; kwargs...) = Tuple(makeunique!(collect(x); kwargs...))

function makeunique!(x::AbstractVector{Symbol}; sep = :_)
    uniqs = unique(x)
    for uniq in uniqs
        idxs = findall(isequal(uniq), x)
        if length(idxs) > 1
            for (count, idx) in enumerate(idxs)
                x[idx] = Symbol(uniq, sep, count)
            end
        end
    end
    x
end


abstract type AbstractAxis end

struct LinearAxis <: AbstractAxis
    name::Symbol
    labels::Vector{Symbol}
end

struct UnnamedAxis <: AbstractAxis
    name::Symbol
    len::Int
end

Base.length(ax::LinearAxis) = length(ax.labels)
Base.length(ax::UnnamedAxis) = ax.len


# e.g. `:nsensor` -> MJAxis([:mysensor1, :mysensor2], [1:3, 4:4])
function sizenames2axes(m::jlModel)
    szname2axes = Dict{Symbol,LinearAxis}()
    sinfo = STRUCT_INFO[:mjModel]

    # Get the labels for each kind of object.
    for objtype in instances(MJCore.mjtObj)

        # get the szname corresponding to `objtype`
        szname = jl_type2szname(objtype)
        isnothing(szname) && continue # not all types have sizes (e.g. mjOBJ_UNKNOWN)
        sz = getproperty(m, szname) # get the integer value of `szname`

        # lookup the names for `objtype`
        elnames = map(1:sz) do id
            elname = jl_id2name(m, objtype, id)
            if isnothing(elname)
                # For unnamed objects, default to `typename_id` (e.g. :geom_3)
                Symbol(MJCore.mju_type2Str(objtype), :_, id)
            else
                Symbol(elname)
            end
        end

        # Now every dimension of size `szname` can be labeled with `elnames`.
        szname2axes[szname] = LinearAxis(szname, elnames)
    end

    # "Ragged" axes may inherit their element names from other "non-ragged" axes.
    # For example, the element names for "nv" axis come from "njnt".
    # This is because for system containing 2 joints, 1 slide and 1 free, njnt=2,
    # but nv = 7.
    for (ragged_szname, (addr_fieldname, expandfn)) in pairs(RAGGED_SIZENAME_TO_ADREXPANDER)
        # e.g. `jnt_qposadr` has size `njnt`
        donor_szname = first(sinfo[addr_fieldname].mjsize::Tuple{Symbol}) # for ragged_szname=nv, donor_szname=njnt
        ragged_idxs = getproperty(m, addr_fieldname) .+ 1 # convert to one-indxed
        donor_labels = szname2axes[donor_szname].labels

        linearlabels = ragged2linearaxis(expandfn, m, donor_labels, ragged_idxs)

        szname2axes[ragged_szname] = LinearAxis(ragged_szname, linearlabels)
    end

    # Arrays with dimension `na` correspond to stateful actuators. MuJoCo's
    # compiler requires that these are always defined after stateless actuators,
    # so we only need the final `na` elements in the list of all actuator names.
    if m.na != 0
        act_names = szname2axes[:nu].labels
        szname2axes[:na] = LinearAxis(:na, act_names[end-m.na+1:end])
    end

    # And a sanity check
    for (szname, axis) in szname2axes
        sz = getproperty(m, szname)
        @assert axis.name === szname
        @assert length(axis) == sz
    end

    # Lastly, add the labels for our custom axes
    for jlsize in JLSizes
        szname2axes[nameof(jlsize)] = LinearAxis(nameof(jlsize), collect(jlsize.labels))
    end

    szname2axes
end


# Associate axes in `sznames2axes` with each field of x
function associate_axes(m::jlModel, x::Union{jlModel,jlData}, sznames2axes)
    # Used to create the namedtuple.
    named_axes = Dict{Symbol,Tuple{Vararg{<:AbstractAxis}}}()
    sinfo = x isa jlModel ? STRUCT_INFO[:mjModel] : STRUCT_INFO[:mjData]

    for name in propertynames(x)
        # we only care about arrays that we wrap
        field = getproperty(x, name)
        if field isa Array && haskey(sinfo, name)
            finfo = sinfo[name]

            sznames = reverse(finfo.jlsize)
            sizes = map(sznames) do szname
                if szname isa Symbol
                    Int(hasproperty(m, szname) ? getproperty(m, szname) :
                        getproperty(x, szname))
                elseif szname isa JLSize
                    length(szname)
                else
                    szname
                end
            end

            sznames = map(enumerate(sznames)) do (i, dim)
                # replace Integer sizes with :dim_i
                if dim isa Symbol
                    dim
                elseif dim isa JLSize
                    nameof(dim)
                else
                    Symbol(:dim, i)
                end
            end

            uniquenames = makeunique(sznames)

            # `field`'s axes
            axes = map(sizes, sznames, uniquenames) do sz, szname, uniqname
                if haskey(sznames2axes, szname)
                    LinearAxis(uniqname, sznames2axes[szname].labels)
                else
                    UnnamedAxis(uniqname, sz)
                end
            end
            named_axes[name] = Tuple(axes)
        end
    end
    named_axes
end

function namify_array(A, newaxes)
    axisarray_axes = map(newaxes) do newax
        axname = newax.name
        if newax isa UnnamedAxis
            Axis{axname}(1:length(newax))
        else
            Axis{axname}(SVector{length(newax),Symbol}(newax.labels))
        end
    end
    newsize = length.(axisarray_axes)
    AxisArray(reshape(A, newsize), axisarray_axes)
end

function namify_one(x::Union{jlModel,jlData}, axismapping)
    newfields = []
    fieldnames = Symbol[]
    for fieldname in propertynames(x)
        field = getproperty(x, fieldname)
        if field isa Array && haskey(axismapping, fieldname)
            newaxes = axismapping[fieldname]
            push!(newfields, namify_array(field, newaxes))
            push!(fieldnames, fieldname)
        end
    end
    NamedTuple{Tuple(fieldnames)}(Tuple(newfields))
end

function namify(m::jlModel, d::jlData)
    sz2ax = sizenames2axes(m)
    d_axmappings = associate_axes(m, d, sz2ax)
    m_axmappings = associate_axes(m, m, sz2ax)
    namify_one(m, m_axmappings), namify_one(d, d_axmappings)
end
