# Global strings
function _unsafe_mj_string(A::AbstractArray{Ptr{Cchar}})
    map(A) do x
        replace(Base.unsafe_string(x), "&" => "")
    end
end

for (name, dims) in (
    (:mjDISABLESTRING, (mjNDISABLE, )),
    (:mjENABLESTRING, (mjNENABLE, )),
    (:mjTIMERSTRING, (mjNTIMER, )),
    (:mjLABELSTRING, (mjNLABEL, )),
    (:mjFRAMESTRING, (mjNFRAME, )),
    (:mjRNDSTRING, (3, mjNRNDFLAG)),
    (:mjVISSTRING, (3, mjNVISFLAG)),
)
    # TODO is this safe to do at module initialization time?
    dims = map(Int, dims)
    T = SArray{Tuple{dims...}, Ptr{Cchar}, length(dims), prod(dims)}
    cref = CRef{name, T}()
    A = _unsafe_mj_string(cref[])
    @eval const $name = $A
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


const mjCOLLISIONFUNC = let N = Int(mjNGEOMTYPES), L = N*N
    MMatrix{N,N,Ptr{Cvoid},L}(ntuple(_->C_NULL, Val(L)))
end

function _init_globals()
    warncb = @cfunction(mju_user_warning_cb, Cvoid, (Cstring,))
    errcb = @cfunction(mju_user_error_cb, Cvoid, (Cstring,))
    MJCore.mju_user_warning[] = warncb
    MJCore.mju_user_error[] = errcb

    ptr = cglobal((:mjCOLLISIONFUNC, libmujoco), typeof(mjCOLLISIONFUNC))
    A = Base.unsafe_load(ptr)
    for i in eachindex(A, mjCOLLISIONFUNC)
        mjCOLLISIONFUNC[i] = A[i]
    end
    nothing
end
