#####
##### Activation
#####

#@test_throws MuJoCoException mj_activate("")
#@test_skip mj_activate("")


@testset "Parse and compile" begin
    pm = mj_loadXML(TESTMODELXML)
    pd = mj_makeData(pm)
    m = unsafe_load(pm)
    d = unsafe_load(pd)
    mj_resetData(m, d)
    mj_forward(m, d)

    pm2 = mj_loadXML(TESTMODELXML)
    pd2 = mj_makeData(pm2)
    m2 = unsafe_load(pm2)
    d2 = unsafe_load(pd2)
    mj_resetData(m2, d2)
    mj_forward(m2, d2)

    @test m !== m2
    @test d !== d2
end

@testset "Main simulation" begin
    m, d, jm, jd = jlmakemd()

    @test @inferred(mj_step(m, d)) === nothing
    @test @inferred(mj_step1(m, d)) === nothing
    @test @inferred(mj_step2(m, d)) === nothing

    @test @inferred(mj_forward(m, d)) === nothing
    @test @inferred(mj_inverse(m, d)) === nothing
    @test @inferred(mj_forwardSkip(m, d, MJCore.mjSTAGE_NONE, false)) === nothing
    @test @inferred(mj_inverseSkip(m, d, MJCore.mjSTAGE_NONE, false)) === nothing

    @test @inferred(mj_step(jm, jd)) === nothing
    @test @inferred(mj_step1(jm, jd)) === nothing
    @test @inferred(mj_step2(jm, jd)) === nothing
    @test @inferred(mj_forward(jm, jd)) === nothing
    @test @inferred(mj_inverse(jm, jd)) === nothing
    @test @inferred(mj_forwardSkip(jm, jd, MJCore.mjSTAGE_NONE, false)) === nothing
    @test @inferred(mj_inverseSkip(jm, jd, MJCore.mjSTAGE_NONE, false)) === nothing
end

@testset "Initialization" begin
    let pm = mj_loadXML(TESTMODELXML), pd = mj_makeData(pm)
        @test @inferred(MJCore.mj_deleteData(pd)) === nothing
        @test @inferred(MJCore.mj_deleteModel(pm)) === nothing
    end

    # load 4 models from: 1) two different mj_loadXML calls, 1 deepcopy, and 1 constructor call
    # all should all be equal.
    let
        pm1 = mj_loadXML(TESTMODELXML)
        pm2 = mj_copyModel(pm1)
        m1 = unsafe_load(pm1)
        m2 = unsafe_load(pm2)
        jm1 = jlModel(pm1)
        jm2 = jlModel(pm2)
        jm3 = deepcopy(jm1)
        jm4 = jlModel(jm1)

        @testset "jlModel: $name" for name in propertynames(jlModel)
            jf1 = getproperty(jm1, name)
            jf2 = getproperty(jm2, name)
            jf3 = getproperty(jm3, name)
            jf4 = getproperty(jm4, name)
            ret = jf1 == jf2 == jf3 == jf4
            if fieldtype(mjModel, name) <: Ptr
                f1 = getfield(m1, name)
                f2 = getfield(m2, name)
                ret &= (f1 != f2)
            end
            @test ret
        end
    end

    # load datas from 4 different ways: pointers to mjData and mjModel,
    # just mjModel, and from instance of jlModel
    let
        pm = mj_loadXML(TESTMODELXML)
        jm = jlModel(pm)

        pd1 = mj_makeData(pm)
        jd1 = jlData(pm, pd1)

        jd2 = jlData(pm)

        jd3 = jlData(jm)

        mj_step(pm, jd1)
        mj_step(pm, jd2)
        mj_step(pm, jd3)

        @testset "jlData: $name" for name in propertynames(jlData)
            jf1 = getproperty(jd1, name)
            jf2 = getproperty(jd2, name)
            jf3 = getproperty(jd3, name)
            test() = jf1 == jf2 == jf3

            #if name === :solver_nnz
            #    # TODO
            #    @test_skip test()
            #else
            #    @test test()
            #end
            @test test()

            #if name === :solver_nnz
            #    # NOTE: bug in MuJoCo? d1.solver_nnz == 729,
            #    # but subsequent data's have solver_nnz == 0
            #    @test_broken test()
            #elseif name === :timer
            #    # jd1.timer has non zero entries for # number?
            #    @test_broken test()
            #else
            #    @test test()
            #end
        end
    end

    let jm = jlModel(TESTMODELXML), jd1 = jlData(jm), jd2 = jlData(jm)
        jd1.qpos .= 12345
        @test jd2.qpos != jd1.qpos
        mj_copyData(jd2, jm, jd1)
        @test jd2.qpos == jd1.qpos
    end
end
