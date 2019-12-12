# For custom types, test get/setproperty for each "kind"
# of wrapped field:
# 1) isbits types like Cint/SArray (e.g. jlModel.nq or jlData.solver) -> pass through to inner mj type (e.g. jlData.d::mjData)
# 2) Ptr to "statically" sized Array (e.g. d.qpos) -> Base.Array
# 3) Ptr to dynamically sized UnsafeArray (e.g. d.contact) -> Dynamically sized UnsafeArrays.UnsafeArray
using LinearAlgebra

@testset "Sugar" begin

    @testset "@set!!" begin
        jm = jlModel(TESTMODELXML)
        jd = jlData(jm)
        mj_resetData(jm, jd)
        @test jm.opt.timestep == 0.001
        qpos0 = copy(jd.qpos)
        mj_step(jm, jd)
        @set!! jm.opt.timestep = -0.001
        @test jm.opt.timestep == -0.001
        mj_step(jm, jd)
        @test isapprox(qpos0, jd.qpos; atol = 1e-3)
        @test jd.time == 0.0
    end


    @testset "Basic" begin

        @testset "jlModel" begin
            _m, _d, _pm, _pd = makemd()
            _jm = @inferred jlModel(_pm)
            f = x -> x.nq
            @test @inferred(f(_jm)) isa Cint

            f = x -> x.body_mass
            @test @inferred(f(_jm)) isa Array{mjtNum,1}

            jm1 = @inferred(jlModel(TESTMODELXML))
            jm2 = @inferred(jlModel(TESTMODELMJB))
            @testset "$name" for name in propertynames(jm1)
                f1 = getproperty(jm1, name)
                if typeof(f1) <: Ptr
                    true
                else
                    f2 = getproperty(jm2, name)
                    ret = isapprox(f1, f2) # TODO
                    #ret=if Sys.iswindows() && typeof(f1) <: AbstractArray
                    #    # For some reason on Windows, subsequent allocations yield oh-so-slightly different values
                    #    return isapprox(f1, f2)
                    #else
                    #    return f1 == f2
                    #end
                    #if !ret
                    #    @warn name
                    #    @info f1 f2
                    #end
                end
            end

            let m1 = jlModel(TESTMODELXML)
                m2 = copy(m1)
                m3 = deepcopy(m1)
                m4 = jlModel(m1)
                @test all(propertynames(jlModel)) do n
                    x1 = getproperty(m1, n)
                    x2 = getproperty(m2, n)
                    x3 = getproperty(m3, n)
                    x4 = getproperty(m4, n)
                    if x1 isa Number || x2 isa AbstractArray
                        return x1 == x2 == x3 == x4
                    else
                        return true
                    end
                end
            end

        end

        @testset "jlData" begin
            _m, _d, _pm, _pd = makemd()
            _jd = @inferred jlData(_pm, _pd)
            f = x -> x.ncon
            @test @inferred(f(_jd)) isa Cint

            f = x -> x.qpos
            @test @inferred(f(_jd)) isa Array{mjtNum,1}

            f = x -> x.contact
            @test @inferred(f(_jd)) isa UnsafeArray{mjContact,1}
            @test length(_jd.contact) == _d.ncon
            @test length(getfield(_jd, :contact)) == _m.nconmax

            f = x -> x.solver
            @test @inferred(f(_jd)) isa Array{mjSolverStat,1}
            @test length(_jd.solver) == length(_d.solver) == mjNSOLVER
        end

        @testset "Simulation" begin
            m, d, _, _ = jlmakemd()
            jm = jlModel(TESTMODELXML)
            jd = jlData(jm)
            mj_resetData(m, d)
            mj_resetData(jm, jd)

            @test m.opt.timestep == jm.opt.timestep == 0.001

            qpos0 = copy(jd.qpos)

            mj_step(m, d)
            mj_step(jm, jd)
            qpos1 = copy(jd.qpos)

            @test jd.qpos != qpos0
            @test unsafe_wrap(Array, d.qpos, size(qpos0)) == jd.qpos
            @test d.time == jd.time == 0.001

            # backwards step
            @set!! m.opt.timestep = -0.001
            @set!! jm.m.opt.timestep = -0.001

            mj_step(m, d)
            mj_step(jm, jd)

            @test isapprox(qpos0, unsafe_wrap(Array, d.qpos, size(qpos0)); atol = 1e-3)
            @test isapprox(qpos0, jd.qpos; atol = 1e-3)

            @test d.time == 0.0
            @test jd.time == 0.0

            mj_resetData(m, d)
            mj_resetData(jm, jd)

            @test qpos0 == unsafe_wrap(Array, d.qpos, size(qpos0)) == jd.qpos
            @test d.time == jd.time == 0.0

            @set!! m.opt.timestep = 0.001
            @set!! jm.m.opt.timestep = 0.001

            mj_step1(m, d)
            mj_step2(m, d)

            mj_step1(jm, jd)
            mj_step2(jm, jd)

            @test jd.qpos == unsafe_wrap(Array, d.qpos, size(qpos0)) == qpos1
        end
    end

    # Test one of each of our custom axes
    @testset "namify" begin
        m = jlModel(TESTMODELXML)
        d = jlData(m)
        mn, dn = namify(m, d)

        ans = [
            0 1 0
            1 0 0
            0 0 -1
        ]
        @test isapprox(dn.xmat[:, :, :box1_body], ans)
        @test isapprox(dn.xquat[:w, :box1_body], 0.0)
        @test isapprox(dn.xquat[:x, :box1_body], sqrt(2)/2)
        @test isapprox(dn.xquat[:y, :box1_body], sqrt(2)/2)
        @test isapprox(dn.xquat[:z, :box1_body], 0.0)

        @test mn.actuator_forcerange[:min, :box1_motor] == -1.0
        @test mn.actuator_forcerange[:max, :box1_motor] == 1.0

        @test mn.mat_rgba[:] == Float32[0.1, 0.2, 0.3, 0.4]

        @test all(propertynames(mn)) do name
            fn = getproperty(mn, name)
            f = getproperty(m, name)
            length(fn) == length(f)
        end

        @test all(propertynames(dn)) do name
            fn = getproperty(dn, name)
            f = getproperty(d, name)
            length(fn) == length(f)
        end
    end

    @testset "jlmujoco" begin
        jm = jlModel(TESTMODELXML)
        @test jl_enabled(jm, MJCore.mjENBL_ENERGY)
        @test jl_disabled(jm, MJCore.mjDSBL_CLAMPCTRL)
    end

end