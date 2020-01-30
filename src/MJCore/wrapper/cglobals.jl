# Global strings
const MJStrings = (
    (:mjDISABLESTRING, (mjNDISABLE, )),
    (:mjENABLESTRING, (mjNENABLE, )),
    (:mjTIMERSTRING, (mjNTIMER, )),
    (:mjLABELSTRING, (mjNLABEL, )),
    (:mjFRAMESTRING, (mjNFRAME, )),
    (:mjRNDSTRING, (3, mjNRNDFLAG)),
    (:mjVISSTRING, (3, mjNVISFLAG)),
)

for (name, dims) in MJStrings
    @eval const $name = Ref{Array{String, length($dims)}}()
end

# Callbacks
for name in (
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
    @eval const $name = CRef{$(QuoteNode(name)), Ptr{Cvoid}}()
end

mju_user_warning_cb(msg::Cstring) = (@warn unsafe_string(msg); nothing)
mju_user_error_cb(msg::Cstring) = (@mjerrormsg msg; exit(1))

function _init_globals()
    warncb = @cfunction(mju_user_warning_cb, Cvoid, (Cstring,))
    errcb = @cfunction(mju_user_error_cb, Cvoid, (Cstring,))
    MJCore.mju_user_warning[] = warncb
    MJCore.mju_user_error[] = errcb

    # load all strings
    for (name, dims) in MJStrings
        ref = getfield(@__MODULE__, name)
        s = load_mj_string_array(name, dims...)
        ref[] = s
    end
end


