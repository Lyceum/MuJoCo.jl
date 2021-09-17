using Test
using UnsafeArrays, BangBang

using MuJoCo
using MuJoCo: TESTMODELXML

using MuJoCo.MJCore
using MuJoCo.MJCore: mjtNum, mjNSOLVER, mjNREF

const TESTMODELMJB = joinpath(@__DIR__, "testmodel.mjb")

rm(TESTMODELMJB, force=true)
mj_saveModel(mj_loadXML(TESTMODELXML), TESTMODELMJB)

function makemd()
    pm = mj_loadXML(TESTMODELXML)
    pd = mj_makeData(pm)
    m = unsafe_load(pm)
    d = unsafe_load(pd)
    mj_resetData(m, d)
    mj_forward(m, d)
    m, d, pm, pd
end

function jlmakemd()
    pm = mj_loadXML(TESTMODELXML)
    pd = mj_makeData(pm)
    m = unsafe_load(pm)
    d = unsafe_load(pd)
    mj_resetData(m, d)
    mj_forward(m, d)
    m, d, jlModel(pm), jlData(pm, pd)
end

function test_uninit(T)
    zeroarg = T()
    posonly = T((getfield(zeroarg, name) for name in fieldnames(T))...)
    isequal(zeroarg, posonly) && hash(zeroarg) == hash(posonly)
end

@testset "MuJoCo.jl" begin
    include("MJCore/MJCore.jl")
    include("MJCore/mjmujoco.jl")
    #include("MJCore/cglobals.jl")
    include("sugar.jl")
end
