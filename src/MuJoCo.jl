module MuJoCo

using MacroTools: combinestructdef
using Base: RefValue
using
    UnsafeArrays,
    ComponentArrays,
    StaticArrays,
    Reexport,
    BangBang

const TESTMODELXML = joinpath(@__DIR__, "../test/testmodel.xml")

export
    # Sugar
    jlData,
    jlModel,
    namify,
    jl_enabled,
    jl_disabled,
    jl_name2id,
    jl_id2name,

    # Core
    MJCore,
    mj_activate,
    mj_resetData,
    mj_resetCallbacks,
    mj_step,
    mj_step1,
    mj_step2,
    mj_forward,
    mj_inverse,
    mj_forwardSkip,
    mj_inverseSkip



include("MJCore/MJCore.jl")
using .MJCore
using .MJCore: AbstractMJModel,
                AbstractMJData,
                UnsafeVector,
                UnsafeMatrix,
                MuJoCoException,
                @mjerror,
                MuJoCoException,
                union_types



####
#### Sugar
####

function build_structdef(
    name::Symbol,
    fields::Vector;
    supertype = Any,
    constructors = Any[],
    mutable = false,
    params = Any[],
)
    Dict{Symbol,Any}(
        :name => name,
        :fields => fields,
        :supertype => supertype,
        :constructors => constructors,
        :mutable => mutable,
        :params => params,
    )
end

function build_funcdef(
    name::Symbol,
    body::Expr;
    args = Any[],
    kwargs = Any[],
    whereparams = (),
)
    Dict{Symbol,Any}(
        :name => name,
        :args => args,
        :kwargs => kwargs,
        :body => body,
        :whereparams => whereparams,
    )
end


include("metadata.jl")
include("jlmodel.jl")
include("jldata.jl")
include("jlmujoco.jl")
include("namify.jl")

const JLTypes = Union{jlModel,jlData}

const STATE_FIELDS = (
    :time,
    :qpos,
    :qvel,
    :act,
    :qacc_warmstart,
    :mocap_pos,
    :mocap_quat,
    :userdata,
)

const CONTROL_FIELDS = (
    :ctrl,
    :xfrc_applied,
    :qfrc_applied
)

# For @set!! macro to work
@inline function BangBang.setproperties!!(value::JLTypes, patch::NamedTuple)
    ntuple(length(patch)) do i
        setproperty!(value, keys(patch)[i], patch[i])
    end
    return value
end
BangBang.possible(::typeof(BangBang.setproperties!), obj::JLTypes, patch) = true

function printstate(io, jd::jlData)
    header = "jlData"
    n = 10
    println(io, repeat('-', n), header, repeat('-', n))
    println(io, "MuJoCo State:")
    for name in STATE_FIELDS
        x = getproperty(jd, name)
        println(io, "$(name):")
        show(io, x)
        println(io)
    end
    println(io, "MuJoCo Controls:")
    for name in CONTROL_FIELDS
        x = getproperty(jd, name)
        println(io, "$(name):")
        show(io, x)
        println(io)
    end
    println(io, repeat('-', n * 2 + length(header)))
    io
end

function Base.show(io::IO, ::MIME"text/plain", d::jlData)
    printstate(io, d)
    mjshow(io, d, 0, filter= (n, T) -> T <: Number)
    io
end

function Base.show(io::IO, ::MIME"text/plain", m::jlModel)
    mjshow(io, m, 0, filter=(n, T) -> T <: Number)
    io
end

Base.show(io::IO, ::MIME"text/plain", x::Union{MJCore.MJTypes,JLTypes}) = mjshow(io, x, 0)

function mjshow(io::IO, x::Union{MJCore.MJTypes,JLTypes}, level::Int; filter=(x, T)->true)
    alltypes = Union{MJCore.MJTypes,JLTypes}
    for name in propertynames(x)
        field = getproperty(x, name)
        T = typeof(field)
        !filter(name, T) && continue

        if (T <: AbstractArray{<:alltypes} ||
            T <: Ptr || name === :ref || (x isa jlData && name in STATE_FIELDS))
            continue
        elseif T <: alltypes
            mjshow(io, field, level + 1)
        elseif !(T <: Ptr)
            for _ = 1:level
                print(io, " ")
            end
            print(io, "$name: ")
            show(io, field)
            println(io)
        end
    end
end

end # module
