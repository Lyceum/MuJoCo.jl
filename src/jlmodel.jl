function build_jlModel()
    fields = Tuple{Symbol,Type}[(:m, mjModel), (:cptr, Ptr{mjModel})]
    ctorargs = []

    getpropbody = :(getfield(value, name))
    setpropbody = :(setfield!(value, name, x))
    propnames = Symbol[]
    skippedfields = [:buffer, :stack]

    #forwardedfields = []
    sinfo = STRUCT_INFO[:mjModel]
    for (name, T) in zip(fieldnames(mjModel), fieldtypes(mjModel))
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
                    ptr = getfield(m, $(QuoteNode(name)))
                    size = lookupsize(:mjModel, m, $(QuoteNode(name)))
                    Base.unsafe_wrap(Array, ptr, size)
                end)
                if !isnothing(finfo.dsize)
                    getpropbody = Expr(
                        :elseif,
                        :(name === $(QuoteNode(name))),
                        quote
                            A = getfield(value, $(QuoteNode(name)))
                            m = getfield(value, :m)
                            sz = lookup_dsize(
                                Val{mjModel}(),
                                m,
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
            foffset = fieldoffset(mjModel, Base.fieldindex(mjModel, name))

            push!(fields, (name, Tnew))
            push!(ctorargs, quote
                ptr = Ptr{$elT}(pm + $foffset)
                Base.unsafe_wrap(Array, ptr, $sz)
            end)
        else
            # forward
            getpropbody = Expr(
                :elseif,
                :(name === $(QuoteNode(name))),
                :(getfield(getfield(value, :m), $(QuoteNode(name)))),
                getpropbody,
            )
            setpropbody = Expr(
                :elseif,
                :(name === $(QuoteNode(name))),
                :(setfield!(getfield(value, :m), $(QuoteNode(name)), convert($T, x))),
                setpropbody,
            )
        end
    end
    structdef = build_structdef(
        :jlModel,
        fields,
        mutable = true,
        supertype = AbstractMJModel,
    )
    structexpr = combinestructdef(structdef)

    getpropbody = Expr(:if, getpropbody.args...)
    setpropbody = Expr(:if, setpropbody.args...)

    quote
        $structexpr
        function Base.getproperty(value::jlModel, name::Symbol)
            $getpropbody
        end
        function Base.setproperty!(value::jlModel, name::Symbol, x)
            $setpropbody
        end
        function Base.propertynames(::Union{jlModel,Type{jlModel}})
            $propnames
        end
        function jlModel(pm::Ptr{mjModel}; own::Bool = false)
            m = unsafe_load(pm)
            jm = jlModel(m, pm, $(ctorargs...))
            # must free C-allocated Ptr!
            own && Base.finalizer(jm -> MJCore.mj_deleteModel(jm.cptr), jm)
            jm
        end
        function jlModel(path::String)
            pm = if endswith(path, "xml") || endswith(path, "XML")
                mj_loadXML(path)
            elseif endswith(path, "mjb") || endswith(path, "MJB")
                mj_loadModel(path)
            else
                @mjerror "path must end with one of [xml, XML, mjb, MJB]. Got: $path"
            end
            pm == C_NULL && @mjerror "$path not a valid MJCF or MJB file"
            jlModel(pm, own = true)
        end
        Base.cconvert(::Type{Ptr{mjModel}}, jm::jlModel) = Base.cconvert(Ptr{mjModel}, jm.m)
        jlModel(jm::jlModel) = jlModel(mj_copyModel(jm.cptr), own = true)
        Base.deepcopy_internal(jm::jlModel, ::IdDict) = jlModel(jm)
    end
end

@eval $(build_jlModel())