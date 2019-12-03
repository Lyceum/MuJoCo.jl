struct GlobalAccesser end
const CGlobals = GlobalAccesser()

mju_user_warning_cb(msg::Cstring) = (@warn unsafe_string(msg); nothing)
mju_user_error_cb(msg::Cstring) = (@mjerrormsg msg; exit(1))

Base.propertynames(::GlobalAccesser) = (
    # Public callbacks
    :mjcb_passive,
    :mjcb_control,
    :mjcb_contactfilter,
    :mjcb_sensor,
    :mju_user_error,
    :mju_user_warning,
    :mjcb_time,
    :mjcb_act_dyn,
    :mjcb_act_gain,
    :mjcb_act_bias,

    # Strings
    :mjDISABLESTRING,
    :mjENABLESTRING,
    :mjTIMERSTRING,
    :mjLABELSTRING,
    :mjFRAMESTRING,
    :mjRNDSTRING,
    :mjVISSTRING,
)

getglobal(::Val{name}) where {name} = @mjerror "Global $name does not exist"
setglobal!(::Val{name}, x) where {name} = @mjerror "Global $name does not exist"

Base.getproperty(::GlobalAccesser, name::Symbol) = getglobal(Val{name}())
Base.setproperty!(::GlobalAccesser, name::Symbol, x) = setglobal!(Val{name}(), x)

@inline function setmjcb!(p::Ptr{Ptr{Cvoid}}, x::Ptr{Cvoid})
    p == C_NULL && @mjerror "found NULL callback pointer."
    Base.unsafe_store!(p, x)
    p
end

getglobal(::Val{:mjDISABLESTRING}) = load_mj_string_array(:mjDISABLESTRING, mjNDISABLE)
getglobal(::Val{:mjENABLESTRING}) = load_mj_string_array(:mjENABLESTRING, mjNENABLE)
getglobal(::Val{:mjTIMERSTRING}) = load_mj_string_array(:mjTIMERSTRING, mjNTIMER)
getglobal(::Val{:mjLABELSTRING}) = load_mj_string_array(:mjLABELSTRING, mjNLABEL)
getglobal(::Val{:mjFRAMESTRING}) = load_mj_string_array(:mjFRAMESTRING, mjNFRAME)
getglobal(::Val{:mjRNDSTRING}) = load_mj_string_array(:mjRNDSTRING, Int(mjNRNDFLAG) * 3)
getglobal(::Val{:mjVISSTRING}) = load_mj_string_array(:mjVISSTRING, Int(mjNVISFLAG) * 3)

# Callbacks
for cbname in (
    :mjcb_passive,
    :mjcb_control,
    :mjcb_contactfilter,
    :mjcb_sensor,
    :mju_user_error,
    :mju_user_warning,
    :mjcb_time,
    :mjcb_act_dyn,
    :mjcb_act_gain,
    :mjcb_act_bias,
)
    @eval getglobal(::Val{$(QuoteNode(cbname))}) =
        cglobal(($(QuoteNode(cbname)), libmujoco), Ptr{Cvoid})
    @eval setglobal!(::Val{$(QuoteNode(cbname))}, x) =
        setmjcb!(cglobal(($(QuoteNode(cbname)), libmujoco), Ptr{Cvoid}), x)
end