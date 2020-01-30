# TODO PR to UnsafeArrays.jl
const UnsafeVector{T} = UnsafeArray{T,1}
const UnsafeMatrix{T} = UnsafeArray{T,2}

# Loads char** (NULL-terminated)
load_mj_string_array(name::Symbol, dims...) = load_mj_string_array(name, map(Int, dims))
function load_mj_string_array(name::Symbol, dims::Dims)
    p = convert(Ptr{Ptr{Cchar}}, @eval cglobal(($(QuoteNode(name)), libmujoco)))
    @assert !(p == C_NULL)
    A = Base.unsafe_wrap(Array, p, dims)
    @assert !any(ptr -> (ptr == C_NULL), A)
    S = Base.map(Base.unsafe_string, A)

    map(S) do x
        replace(x, "&" => "")
    end
end

function modelkind(path::AbstractString)
    ext = last(split(last(splitext(path)), '.'))
    if ext in ("xml", "XML", "mjcf", "MJCF")
        return :MJCF
    elseif ext in ("mjb", "MJB")
        return :MJB
    else
        throw(ArgumentError("model path must end with one of [xml, XML, mjcf, MJCF, mjb, MJB]. Got: $ext"))
    end
end

function check_modelpath(path::AbstractString)
    if isfile(path)
        return modelkind(path)
    else
        throw(ArgumentError("model path $path does not exist"))
    end
end


tostring(error::String) = error
tostring(error::Cstring) = unsafe_string(error)
tostring(error::AbstractVector{UInt8}) = String(readuntil(IOBuffer(error), 0x00))


struct MuJoCoException <: Exception
    error::String
    extra::Union{Nothing, String}
    function MuJoCoException(error, extra = nothing)
        new(tostring(error), extra === nothing ? nothing : tostring(extra))
    end
end

Base.show(io::IO, ::MIME"text/plain", ex::MuJoCoException) = show(io, ex)
function Base.show(io::IO, ex::MuJoCoException)
    lines = readlines(IOBuffer(ex.error))
    if ex.extra !== nothing
        println(io, "MuJoCoException(", ex.extra, "):")
        foreach(l->println(io, l), lines)
    elseif length(lines) > 1
        println(io, "MuJoCoException:")
        foreach(l->println(io, l), lines)
    else
        print(io, "MuJoCoException(", first(lines), ')')
    end
end

function MuJoCoException(
    errstr::Union{SVector{N,UInt8},MVector{N,UInt8},Vector{UInt8}};
    checknullterminated::Bool = true,
) where {N}
    errstr = String(errstr)
    if checknullterminated
        nullidx = findfirst('\0', errstr)
        endidx = nullidx isa Integer && 0 <= nullidx <= length(errstr) ? nullidx :
                 length(errstr)
        errstr = errstr[1:endidx]
    end
    MuJoCoException(errstr)
end

macro mjerror(errstr)
    :(throw(MuJoCoException($(esc(errstr)))))
end

macro mjerror(msg, errstr)
    :(throw(MuJoCoException($(esc(msg)), $(esc(errstr)))))
end

macro mjerrormsg(s)
    quote
        local error = MuJoCoException($(esc(s)))
        Base.with_output_color(get(stderr, :color, false) ? Base.error_color() : :nothing, stderr) do io
            showerror(io, error)
        end
    end
end



union_types(x::Union) = (x.a, union_types(x.b)...)
union_types(x::Type) = (x,)

macro uninitable(ex)
    sdef = MacroTools.splitstructdef(ex)
    esc(create_constructors(sdef))
end

function create_constructors(sdef)
    length(sdef[:params]) > 0 && error("Parametric types not supported")

    uninitialized_ctor = :($(sdef[:name])() = new())

    fieldexprs, fieldnames = [], []
    for (name, T) in sdef[:fields]
        push!(fieldexprs, :($name::$T))
        push!(fieldnames, name)
    end

    posonly_innerctor_body_typed = Expr(:call, :new, fieldexprs...)
    posonly_innerctor_typed = MacroTools.combinedef(Dict(
        :params => [],
        :name => sdef[:name],
        :args => fieldexprs,
        :kwargs => [],
        :body => posonly_innerctor_body_typed,
        :whereparams => ()
    ))

    posonly_innerctor_body = Expr(:call, :new, fieldnames...)
    posonly_innerctor = MacroTools.combinedef(Dict(
        :params => [],
        :name => sdef[:name],
        :args => fieldnames,
        :kwargs => [],
        :body => posonly_innerctor_body,
        :whereparams => ()
    ))


    push!(sdef[:constructors], uninitialized_ctor)
    push!(sdef[:constructors], posonly_innerctor_typed)
    push!(sdef[:constructors], posonly_innerctor)
    MacroTools.combinestructdef(sdef)
end

struct CRef{name, T}
    CRef{name, T}() where {name, T<:Ptr} = new{name::Symbol, T}()
end

Base.getindex(::CRef{name,T}) where {name,T} = getglobal(Val(name), T)
Base.setindex!(::CRef{name,T}, x) where {name,T} = setglobal!(Val(name), T, x)

getglobal(::Val{name}, ::Type{T}) where {name,T} = cglobal((name, libmujoco), T)

function setglobal!(::Val{name}, ::Type{T}, x) where {name,T}
    ptr = cglobal((name, libmujoco), T)
    Base.unsafe_store!(ptr, Base.unsafe_convert(T, Base.cconvert(T, x)))
end