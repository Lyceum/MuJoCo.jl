function build_jlData()
    fields = Tuple{Symbol,Type}[(:d, mjData), (:cptr, Ptr{mjData})]
    ctorargs = []

    getpropbody = :(getfield(value, name))
    setpropbody = :(setfield!(value, name, x))
    propnames = Symbol[]
    skippedfields = [:buffer, :stack]

    sinfo = STRUCT_INFO[:mjData]
    for (name, T) in zip(fieldnames(mjData), fieldtypes(mjData))
        if name in skippedfields
            continue
        end
        push!(propnames, name)
        if T <: Ptr
            if haskey(sinfo, name)
                finfo = sinfo[name]
                N = length(finfo.mjsize)
                T = Array{eltype(T),N}
                push!(fields, (name, T))
                push!(ctorargs, quote
                    ptr = getfield(d, $(QuoteNode(name)))
                    size = lookupsize(:mjData, m, $(QuoteNode(name)))
                    Base.unsafe_wrap(Array, ptr, size)
                end)
                if !isnothing(finfo.dsize)
                    getpropbody = Expr(
                        :elseif,
                        :(name === $(QuoteNode(name))),
                        quote
                            A = getfield(value, $(QuoteNode(name)))
                            d = getfield(value, :d)
                            sz = lookup_dsize(
                                Val{:mjData}(),
                                d,
                                Val{$(QuoteNode(name))}(),
                            )
                            UnsafeArray(pointer(A), sz)
                        end,
                        getpropbody,
                    )
                end
            else
                @warn "Found Ptr $name but no size, skipping"
            end
        elseif T <: StaticArray
            N = ndims(T)
            elT = eltype(T)
            sz = size(T)
            Tnew = Array{eltype(T),N}
            foffset = fieldoffset(mjData, Base.fieldindex(mjData, name))

            push!(fields, (name, Tnew))
            push!(ctorargs, quote
                ptr = Ptr{$elT}(pd + $foffset)
                Base.unsafe_wrap(Array, ptr, $sz)
            end)
        else
            # forward
            getpropbody = Expr(
                :elseif,
                :(name === $(QuoteNode(name))),
                :(getfield(getfield(value, :d), $(QuoteNode(name)))),
                getpropbody,
            )
            setpropbody = Expr(
                :elseif,
                :(name === $(QuoteNode(name))),
                :(setfield!(getfield(value, :d), $(QuoteNode(name)), convert($T, x))),
                setpropbody,
            )
        end
    end
    structdef = build_structdef(:jlData, fields, mutable = true, supertype = AbstractMJData)
    structexpr = combinestructdef(structdef)

    getpropbody = Expr(:if, getpropbody.args...)
    setpropbody = Expr(:if, setpropbody.args...)

    quote
        $structexpr
        function Base.getproperty(value::jlData, name::Symbol)
            $getpropbody
        end
        function Base.setproperty!(value::jlData, name::Symbol, x)
            $setpropbody
        end
        function Base.propertynames(::Union{jlData,Type{jlData}})
            $propnames
        end
        function jlData(
            pm::Ptr{mjModel},
            pd::Ptr{mjData};
            ownmodel::Bool = false,
            owndata::Bool = false,
        )
            m = unsafe_load(pm)
            d = unsafe_load(pd)
            jd = jlData(d, pd, $(ctorargs...))
            mj_resetData(pm, jd)
            mj_forward(pm, jd)
            # must free C-allocated Ptr/pd!
            owndata && Base.finalizer(jd -> MJCore.mj_deleteData(jd.cptr), jd)
            ownmodel && mj_deleteModel(pm)
            jd
        end
        function jlData(pm::Ptr{mjModel}; own::Bool = false)
            jlData(pm, mj_makeData(pm), ownmodel = own, owndata = true)
        end
        jlData(jm::jlModel) = jlData(jm.cptr, own = false)
        Base.cconvert(::Type{Ptr{mjData}}, jd::jlData) = Base.cconvert(Ptr{mjData}, jd.d)
        Base.deepcopy_internal(jd::jlData, ::IdDict) = Base.copy(jd)
        function Base.copy(jd::jlData)
            error("jlData is not deep copiable. See `mj_copyData` and `mj_makeData` instead.")
        end

    end
end

@eval $(build_jlData())