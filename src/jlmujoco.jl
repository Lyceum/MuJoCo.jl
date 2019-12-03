jl_name2id(m::jlModel, objtype, name::String) = Int(MJCore.mj_name2id(m, objtype, name) + 1)
jl_id2name(m::jlModel, objtype, id::Integer) = MJCore.mj_id2name(m, objtype, Int32(id - 1))

jl_enabled(m::jlModel, flag::MJCore.mjtEnableBit) = _flagtest(m.opt.enableflags, flag)
jl_disabled(m::jlModel, flag::Union{Integer, MJCore.mjtDisableBit}) = _flagtest(m.opt.disableflags, flag)
_flagtest(flags, flag) = Integer(flags) & Integer(flag) == Integer(flag)
