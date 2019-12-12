globals = (
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
    mjVISSTRING = [
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
        "SCL Inertia" "0" "S"
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
    ],
    mjRNDSTRING = [
        "Shadow",
        "1",
        "S",
        "Wireframe",
        "0",
        "W",
        "Reflection",
        "1",
        "R",
        "Additive",
        "0",
        "L",
        "Skybox",
        "1",
        "K",
        "Fog",
        "0",
        "G",
        "Haze",
        "1",
        "/",
        "Segment",
        "0",
        ",",
        "Id Color",
        "0",
        ".",
    ],
)

@testset "$k" for (k, v) in pairs(globals)
    if k === :mjVISSTRING
        @info k
        @test_broken MJCore.getglobal(Val{k}()) == v
    else
        @test MJCore.getglobal(Val{k}()) == v
    end
end
