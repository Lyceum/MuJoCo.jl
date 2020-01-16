jl_name2id(m::jlModel, objtype, name::String) = Int(MJCore.mj_name2id(m, objtype, name) + 1)
jl_id2name(m::jlModel, objtype, id::Integer) = MJCore.mj_id2name(m, objtype, Int32(id - 1))

jl_enabled(m::jlModel, flag::MJCore.mjtEnableBit) = _flagtest(m.opt.enableflags, flag)
jl_disabled(m::jlModel, flag::Union{Integer, MJCore.mjtDisableBit}) = _flagtest(m.opt.disableflags, flag)
_flagtest(flags, flag) = Integer(flags) & Integer(flag) == Integer(flag)

@propagate_inbounds _site(site)::SVector{3,Float64} = SA_F64[site[1], site[2], site[3]]
@propagate_inbounds _site(sites, id)::SVector{3,Float64} = SA_F64[sites[1,id], sites[2,id], sites[3,id]]
