module MJCore

using MuJoCo_jll
using StaticArrays, UnsafeArrays, CEnum, MacroTools, Base64
using Base: last


include("exports.jl")
export AbstractMJModel, AbstractMJData, isactivated


abstract type AbstractMJModel end
abstract type AbstractMJData end

const MJArray{T,N} = Union{MArray{<:Tuple,T,N},Array{T,N},UnsafeArray{T,N}} where {T,N}
const MJVec{T} = MJArray{T,1}
const MJMat{T} = MJArray{T,2}

const ERRORSIZE = 1000
const KEY_PATH_ENV_VAR = "MUJOCO_KEY_PATH"
const KEY_ENV_VAR = "MUJOCO_KEY"

const ACTIVATED = Ref(false)
isactivated() = ACTIVATED[]


include("util.jl")

# MuJoCo types, enums, and globals
include("wrapper/mjmodel.jl")
include("wrapper/mjdata.jl")
include("wrapper/mjvisualize.jl")
include("wrapper/mjrender.jl")
include("wrapper/mjui.jl")
include("wrapper/cglobals.jl")

const Model = Union{Ptr{mjModel},mjModel,AbstractMJModel}
const Data = Union{Ptr{mjData},mjData,AbstractMJData}

Base.cconvert(::Type{Ptr{mjModel}}, m::mjModel) = pointer_from_objref(m)
Base.cconvert(::Type{Ptr{mjData}}, d::mjData) = pointer_from_objref(d)

include("wrapper/mjmujoco.jl")


function __init__()
    _init_globals()

    if !isactivated() && haskey(ENV, KEY_PATH_ENV_VAR)
        path = normpath(ENV[KEY_PATH_ENV_VAR])
        isfile(path) || @mjerror "activating with key $path: No such file or directory"
        ACTIVATED[] = Bool(mj_activate(path))
        isactivated() || @mjerror "activating with key $path: invalid key"
    elseif !isactivated() && haskey(ENV, KEY_ENV_VAR)
        mjkey = ENV[KEY_ENV_VAR]
        mktemp() do path, io
            write(io, String(base64decode(mjkey)))
            flush(io)
            ACTIVATED[] = Bool(mj_activate(path))
        end
        isactivated() || @mjerror "Key invalid"
    else
        @warn """

            No key found. Please set the env variable $KEY_PATH_ENV_VAR to point to a valid `mjkey` file.
            Alternatively, use `mj_activate(path_to_mjkey.txt)` to manually activate.
        """
    end
    nothing
end

const MJTypes = Union{
    # mjmodel.jl
    mjModel,
    mjLROpt,
    mjVFS,
    mjOption,

    # mjVisual
    mjVisual,
    mjVisual_global,
    mjVisual_quality,
    mjVisual_headlight,
    mjVisual_map,
    mjVisual_scale,
    mjVisual_rgba,

    mjStatistic,

    # mjdata.jl
    mjData,
    mjContact,
    mjWarningStat,
    mjTimerStat,
    mjSolverStat,

    # mjrender.jl
    mjrRect,
    mjrContext,

    # mjui.jl
    mjuiState,
    mjuiThemeSpacing,
    mjuiThemeColor,
    mjuiItem,
    mjuiSection,
    mjUI,
    mjuiDef,

    # mjvisualize.jl
    mjvPerturb,
    mjvCamera,
    mjvGLCamera,
    mjvGeom,
    mjvLight,
    mjvOption,
    mjvScene,
    mjvFigure,
}

function setup_comparators(T::Type)
    quote
        function Base.:(==)(x::$T, y::$T)
            for name in fieldnames($T)
                !(getfield(x, name) == getfield(y, name)) && return false
            end
            return true
        end
        function Base.isequal(x::$T, y::$T)
            for name in fieldnames($T)
                !isequal(getfield(x, name), getfield(y, name)) && return false
            end
            return true
        end
        function Base.hash(x::$T, h::UInt)
            h = hash($T, h)
            for name in fieldnames($T)
                h = hash(getfield(x, name), h)
            end
            return h
        end
    end
end

for T in Iterators.filter(T->T.mutable, union_types(MJTypes))
    @eval $(setup_comparators(T))
end

function Base.isapprox(x::T, y::T, args...; kwargs...) where {T<:MJTypes}
    for name in fieldnames(T)
        f1 = getfield(x, name)
        f2 = getfield(y, name)
        isapprox(f1, f2, args...; kwargs...) || return false
    end
    true
end

end
