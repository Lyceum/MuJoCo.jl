
@testset "MJCore" begin

    @testset "mjmujoco" begin
        include("mjmujoco.jl")
    end

    @testset "cglobals" begin
        include("cglobals.jl")
    end

    @testset "Uninitialized constructor" begin
        skipped = (mjModel, mjData)
        toobig = (mjUI, mjuiDef, mjvFigure)
        uninit_types = Iterators.filter(T->!(T in skipped), MuJoCo.union_types(MJCore.MJTypes))
        @testset "$T" for T in uninit_types
            if T in toobig
                @test_skip test_uninit(T)
            else
                @test test_uninit(T)
            end
        end
    end
end