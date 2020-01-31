const STRING_GLOBALS = (
    mjDISABLESTRING = [
        "Constraint",
        "Equality",
        "Frictionloss",
        "Limit",
        "Contact",
        "Passive",
        "Gravity",
        "Clampctrl",
        "Warmstart",
        "Filterparent",
        "Actuation",
        "Refsafe",
    ],
    mjENABLESTRING = ["Override", "Energy", "Fwdinv", "Sensornoise"],
    mjTIMERSTRING = [
        "step",
        "forward",
        "inverse",
        "position",
        "velocity",
        "actuation",
        "acceleration",
        "constraint",
        "pos_kinematics",
        "pos_inertia",
        "pos_collision",
        "pos_make",
        "pos_project",
    ],
    mjLABELSTRING = [
        "None",
        "Body",
        "Joint",
        "Geom",
        "Site",
        "Camera",
        "Light",
        "Tendon",
        "Actuator",
        "Constraint",
        "Skin",
        "Selection",
        "SelPoint",
        "ContactForce",
    ],
    mjFRAMESTRING = ["None", "Body", "Geom", "Site", "Camera", "Light", "World"],
    mjVISSTRING = permutedims([
        "Convex Hull" "0" "H"
        "Texture" "1" "X"
        "Joint" "0" "J"
        "Actuator" "0" "U"
        "Camera" "0" "Q"
        "Light" "0" "Z"
        "Tendon" "1" "V"
        "Range Finder" "1" "Y"
        "Constraint" "0" "N"
        "Inertia" "0" "I"
        "Scale Inertia" "0" "'"
        "Perturb Force" "0" "B"
        "Perturb Object" "1" "O"
        "Contact Point" "0" "C"
        "Contact Force" "0" "F"
        "Contact Split" "0" "P"
        "Transparent" "0" "T"
        "Auto Connect" "0" "A"
        "Center of Mass" "0" "M"
        "Select Point" "0" "E"
        "Static Body" "1" "D"
        "Skin" "1" ";"
    ]),
    mjRNDSTRING = permutedims([
        "Shadow" "1" "S"
        "Wireframe" "0" "W"
        "Reflection" "1" "R"
        "Additive" "0" "L"
        "Skybox" "1" "K"
        "Fog" "0" "G"
        "Haze" "1" "/"
        "Segment" "0" ","
        "Id Color" "0" "."
    ]),
)

const CB_GLOBALS = (
    :mjcb_passive,
    :mjcb_control,
    :mjcb_contactfilter,
    :mjcb_sensor,
    :mjcb_time,
    :mjcb_act_dyn,
    :mjcb_act_gain,
    :mjcb_act_bias,
)

const mjCOLLISIONFUNC = [
    1  1  1  1  1  1  1  1
    1  1  1  1  1  1  1  1
    0  0  0  1  1  1  1  1
    0  0  0  0  1  1  1  1
    0  0  0  0  0  1  1  1
    0  0  0  0  0  0  1  1
    0  0  0  0  0  0  0  1
    0  0  0  0  0  0  0  0
]


@testset "$name" for (name, testval) in pairs(STRING_GLOBALS)
    @test getfield(MJCore, name) == testval
end

@testset "$name" for name in CB_GLOBALS
    ptr = Base.unsafe_convert(Ptr, getfield(MJCore, name))
    val = getfield(MJCore, name)[]
    @test ptr isa Ptr{Ptr{Cvoid}} && ptr != C_NULL
    @test val isa Ptr{Cvoid} && val == C_NULL
end

@testset "warn/error CB" begin
    for name in (:mju_user_error, :mju_user_warning)
        ptr = Base.unsafe_convert(Ptr, getfield(MJCore, name))
        val = getfield(MJCore, name)[]
        @test ptr isa Ptr{Ptr{Cvoid}} && ptr != C_NULL
        @test val isa Ptr{Cvoid} && val != C_NULL
    end
end

@test mjCOLLISIONFUNC == map(isequal(C_NULL), MJCore.mjCOLLISIONFUNC)
