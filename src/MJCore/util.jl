# TODO PR to UnsafeArrays.jl
const UnsafeVector{T} = UnsafeArray{T,1}
const UnsafeMatrix{T} = UnsafeArray{T,2}

# Loads char** (NULL-terminated)
function load_mj_string_array(name::Symbol, n)
    p = convert(Ptr{Ptr{Cchar}}, @eval cglobal(($(QuoteNode(name)), libmujoco)))
    @assert !(p == C_NULL)
    A = Base.unsafe_wrap(Array, p, Int(n))
    @assert !any(ptr -> (ptr == C_NULL), A)
    Base.map(Base.unsafe_string, A)
end


struct MuJoCoException <: Exception
    errstr::String
end

MuJoCoException(errstr::Cstring) = MuJoCoException(unsafe_string(errstr))

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

macro mjerror(s)
    :(throw(MuJoCoException($(esc(s)))))
end

macro mjerrormsg(s)
    quote
        local error = MuJoCoException($(esc(s)))
        Base.with_output_color(get(stderr, :color, false) ? Base.error_color() : :nothing, stderr) do io
            showerror(io, error)
        end
    end
end


macro check_isfile(path)
    quote
        local path = $(esc(path))
        isfile(path) || throw(MuJoCoException("$path: No such file"))
    end
end

macro check_isvalidfilepath(path)
    quote
        local path = $(esc(path))
        local dir = dirname(path)
        if isdirpath(path) || !(isdir(path) || isdirpath(dir) || isdir(dir))
            throw(MuJoCoException("$path: not a valid path"))
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