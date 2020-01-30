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
    dims = map(Int, dims)
    T = SArray{Tuple{dims...}, String, length(dims), prod(dims)}
    V = SArray{Tuple{dims...}, Ptr{Cchar}, length(dims), prod(dims)}
    @eval const $name = Ref{$T}()
    @eval const $(Symbol(:_, name)) = CRef{$(QuoteNode(name)), $V}()
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
        cref = getfield(@__MODULE__, Symbol(:_, name))
        A = map(cref[]) do x
            replace(Base.unsafe_string(x), "&" => "")
        end
        ref[] = A
    end
end