####
#### Activation
####

"""
    mj_activate(filename::String) -> Cint

Activate the license located at `filename`, return 1 if successful,
otherwise throws a `MuJoCoException`.
"""
function mj_activate(filename::String)
    ccall((:mj_activate, libmujoco), Cint, (Cstring,), filename)
end

"""
    mj_deactivate() -> Cvoid

Deactivate license and free all MuJoCo memory
"""
function mj_deactivate()
    ccall((:mj_deactivate, libmujoco), Cvoid, ())
end

"""
    mj_certQuestion(question::MVector{16,mjtNum}) -> Cvoid

Generate server-side certificate question and store it in `question`.
"""
function mj_certQuestion(question::MVector{16,mjtNum})
    ccall((:mj_certQuestion, libmujoco), Cvoid, (Ptr{mjtNum},), question)
end

"""
    mj_certAnswer(question::MVector{16, mjtNum}, answer::MVector{16,mjtNum}) -> Cvoid

Generate client-side certificate answer given `question` and store it in `answer`.
"""
function mj_certAnswer(question::MVector{16,mjtNum}, answer::MVector{16,mjtNum})
    ccall((:mj_certAnswer, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), question, answer)
end

"""
    mj_certCheck(question::MVector{16,mjtNum}, answer::MVector{16,mjtNum}) -> Cint

Check certificate question-answer pair on server. Return 1 if match, 0 if mismatch.
"""
function mj_certCheck(question::MVector{16,mjtNum}, answer::MVector{16,mjtNum})
    ccall((:mj_certCheck, libmujoco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}), question, answer)
end


####
#### Virtual file system
####

"""
    mj_defaultVFS(vfs::Ref{mjVFS}) -> Cvoid

Initialize `vfs` to empty (no deallocation).
"""
function mj_defaultVFS(vfs::Ref{mjVFS})
    ccall((:mj_defaultVFS, libmujoco), Cvoid, (Ptr{mjVFS},), vfs)
end

"""
    mj_addFileVFS(vfs::Ref{mjVFS}, directory::String, filename::String) -> Cint

Add `joinpath(directory, filename)` to `vfs`,
return 0: success, 1: full, 2: repeated name, -1: not found on disk.
"""
function mj_addFileVFS(vfs::Ref{mjVFS}, directory::String, filename::String)
    ccall(
        (:mj_addFileVFS, libmujoco),
        Cint,
        (Ptr{mjVFS}, Cstring, Cstring),
        vfs,
        directory,
        filename,
    )
end

"""
    mj_makeEmptyFileVFS(vfs::Ref{mjVFS}, filename::String, filesize::Integer) -> Cint

Make empty file `filename` of `filesize` bytes in `vfs`, return 0: success, 1: full, 2: repeated name.
"""
function mj_makeEmptyFileVFS(vfs::Ref{mjVFS}, filename::String, filesize::Integer)
    ccall(
        (:mj_makeEmptyFileVFS, libmujoco),
        Cint,
        (Ptr{mjVFS}, Cstring, Cint),
        vfs,
        filename,
        filesize,
    )
end

"""
    mj_findFileVFS(vfs::Ref{mjVFS}, filename::String) -> Cint

Return file index of `filename` in `vfs`, or -1 if not found in `vfs`.
"""
function mj_findFileVFS(vfs::Ref{mjVFS}, filename::String)
    ccall((:mj_findFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Cstring), vfs, filename)
end

"""
    mj_deleteFileVFS(vfs::Ref{mjVFS}, filename::String) -> Cint

Delete `filename` from `vfs`, return 0: success, -1: not found in VFS.
"""
function mj_deleteFileVFS(vfs::Ref{mjVFS}, filename::String)
    ccall((:mj_deleteFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Cstring), vfs, filename)
end

"""
    mj_deleteVFS(vfs::Ref{mjVFS}) -> Cvoid

Delete all files from `vfs`.
"""
function mj_deleteVFS(vfs::Ref{mjVFS})
    ccall((:mj_deleteVFS, libmujoco), Cvoid, (Ptr{mjVFS},), vfs)
end


####
#### Parse and compile
####

"""
    mj_loadXML(filename::String[, vfs::Ref{mjVFS}) -> Ptr{mjModel}

Parse `filename` XML file in MJCF or URDF format, compile it, return low-level model.
If `vfs` is specified, look up files in `vfs` before checking disk.
Throws `MuJoCoException` if failure.
"""
@inline mj_loadXML(filename::String, vfs::Ref{mjVFS}) = _mj_loadXML(filename, vfs)
@inline mj_loadXML(filename::String) = _mj_loadXML(filename, Ptr{mjVFS}(0))
function _mj_loadXML(filename::String, vfs::Union{Ref{mjVFS},Ptr{mjVFS}})
    modelkind(filename) === :MJCF || throw(ArgumentError("Not a XML/MJCF file: $filename"))
    err = MVector{ERRORSIZE,UInt8}(undef)
    pm = ccall(
        (:mj_loadXML, libmujoco),
        Ptr{mjModel},
        (Cstring, Ptr{mjVFS}, Ptr{UInt8}, Cint),
        filename,
        vfs,
        err,
        ERRORSIZE,
    )
    pm == C_NULL && @mjerror SVector(err) "Not a valid XML file"
    return pm
end

"""
    mj_saveLastXML(filename::String, m::Model) -> Cint

Update XML data structures with info from low-level model `m`, save as MJCF
to `filename`. Returns 1 if success, else throws a `MuJoCoException`.
"""
function mj_saveLastXML(filename::String, m::Ptr{mjModel})
    errsz = 1000
    err = MVector{ERRORSIZE,UInt8}(undef)
    ret = ccall(
        (:mj_saveLastXML, libmujoco),
        Cint,
        (Cstring, Ptr{mjModel}, Ptr{UInt8}, Cint),
        filename,
        m,
        err,
        ERRORSIZE,
    )
    ret == 0 && @mjerror SVector(err)
    return ret
end

"""
    mj_freeLastXML() -> Cvoid

    Free last XML model if loaded. Called internally at each load.
"""
function mj_freeLastXML()
    ccall((:mj_freeLastXML, libmujoco), Cvoid, (Cvoid,), Cvoid)
end


####
#### Main simulation
####

"""
    mj_step(m::Model, d::Data) -> Cvoid

Advance simulation. use control callback if specified, otherwise use
controls in `d.ctrl`, `d.xfrc_applied`, `d.qfrc_applied`. RK4 available.
See also: [`mj_step1`](@ref), [`mj_step2`](@ref)
"""
function mj_step(m::Model, d::Data)
    ccall((:mj_step, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_step1(m::Model, d::Data) -> Cvoid

Advance simulation in two steps: before external force/control is set by user.
See also: [`mj_step2`](@ref), [`mj_step`](@ref)
"""
function mj_step1(m::Model, d::Data)
    ccall((:mj_step1, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_step2(m::Model, d::Data) -> Cvoid

Advance simulation in two steps: after external force/control is set by user.
See also: [`mj_step1`](@ref), [`mj_step`](@ref)
"""
function mj_step2(m::Model, d::Data)
    ccall((:mj_step2, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_forward(m::Model, d::Data) -> Cvoid

Compute forward dynamics.
"""
function mj_forward(m::Model, d::Data)
    ccall((:mj_forward, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_inverse(m::Model, d::Data) -> Cvoid

Compute inverse dynamics.
"""
function mj_inverse(m::Model, d::Data)
    ccall((:mj_inverse, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_forwardSkip(m::Model, d::Data, skipstage::mjtStage, skipsensor::Bool) -> Cvoid

Compute forward dynamics, skipping parts of computation pipeline as specified by
`skipstage`. Sensor computations can be skipped using `skipsensor`.
See also: [`mjtStage`](@ref)
"""
function mj_forwardSkip(m::Model, d::Data, skipstage::mjtStage, skipsensor::Bool)
    ccall(
        (:mj_forwardSkip, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Cint),
        m,
        d,
        skipstage,
        skipsensor,
    )
end

"""
    mj_inverseSkip(m::Model, d::Data, skipstage::mjtStage, skipsensor::Bool) -> Cvoid

Compute inverse dynamics, skipping parts of computation pipeline as specified by
`skipstage`. Sensor computations can be skipped using `skipsensor`.
See also: [`mjtStage`](@ref)
"""
function mj_inverseSkip(m::Model, d::Data, skipstage::mjtStage, skipsensor::Bool)
    ccall(
        (:mj_inverseSkip, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Cint),
        m,
        d,
        skipstage,
        skipsensor,
    )
end


####
#### Initialization
####

"""
    mj_defaultLROpt(opt::Ref{mjLROpt}) -> Cvoid

Set default options for length range computation.
"""
function mj_defaultLROpt(opt::Ref{mjLROpt})
    ccall((:mj_defaultLROpt, libmujoco), Cvoid, (Ptr{mjLROpt},), opt)
end

"""
    mj_defaultSolRefImp(solref::MJVec{mjtNum}, solimp::MJVec{mjtNum}) -> Cvoid

Set default solver parameters.
"""
function mj_defaultSolRefImp(solref::MJVec{mjtNum}, solimp::MJVec{mjtNum})
    ccall(
        (:mj_defaultSolRefImp, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}),
        solref,
        solimp,
    )
end

"""
    mj_defaultOption(opt::Ref{mjOption}) -> Cvoid

Set physics options to default values.
"""
function mj_defaultOption(opt::Ref{mjOption})
    ccall((:mj_defaultOption, libmujoco), Cvoid, (Ptr{mjOption},), opt)
end

"""
    mj_defaultVisual(vis::Ref{mjVisual}) -> Cvoid

Set visual options to default values.
"""
function mj_defaultVisual(vis::Ref{mjVisual})
    ccall((:mj_defaultVisual, libmujoco), Cvoid, (Ptr{mjVisual},), vis)
end

"""
    mj_copyModel(dest::Model, src::Model) -> Ref{mjModel}

Copy model `src` into `dest`
"""
function mj_copyModel(dest::Model, src::Model)
    ccall((:mj_copyModel, libmujoco), Ptr{mjModel}, (Ptr{mjModel}, Ptr{mjModel}), dest, src)
    dest
end

"""
    mj_copyModel(m::Model) -> Ptr{mjModel}

Create a copy of `m`, allocating new memory.
"""
function mj_copyModel(m::Model)
    ccall((:mj_copyModel, libmujoco), Ptr{mjModel}, (Ptr{mjModel}, Ptr{mjModel}), C_NULL, m)
end

"""
    mj_saveModel(m::Model, filename::String) -> Cvoid

Save `m` to `filename`
"""
function mj_saveModel(m::Model, filename::String)
    ccall(
        (:mj_saveModel, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Cstring, Ptr{Cvoid}, Cint),
        m,
        filename,
        C_NULL,
        0,
    )
end

"""
    mj_saveModel(m::Model, buffer::MJVec{UInt8}) -> Cvoid

Save `m` to `buffer`.
"""
function mj_saveModel(m::Model, buffer::MJVec{UInt8})
    ccall(
        (:mj_saveModel, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Cstring, Ptr{Cvoid}, Cint),
        m,
        C_NULL,
        buffer,
        length(buffer),
    )
end

"""
    mj_loadModel(filename::String) -> Ptr{mjModel}

Load binary `MJB` file from `filename` located on local filesystem.
"""
function mj_loadModel(filename::String)
    ccall((:mj_loadModel, libmujoco), Ptr{mjModel}, (Cstring, Ptr{mjVFS}), filename, C_NULL)
end

"""
    mj_loadModel(filename::String) -> Ptr{mjModel}

Load binary `MJB` file from `filename` in `vfs`.
"""
function mj_loadModel(filename::String, vfs::Ref{mjVFS})
    ccall((:mj_loadModel, libmujoco), Ptr{mjModel}, (Cstring, Ptr{mjVFS}), filename, vfs)
end

"""
    mj_deleteModel(m::Ptr{mjModel}) -> Cvoid

De-allocate model `m`.
Should only be called with `m` allocated by MuJoCo.
"""
function mj_deleteModel(m::Ptr{mjModel})
    ccall((:mj_deleteModel, libmujoco), Cvoid, (Ptr{mjModel},), m)
end

"""
    mj_sizeModel(m::Model) -> Cint

Size of buffer in bytes needed to hold `m`.
"""
function mj_sizeModel(m::Model)
    ccall((:mj_sizeModel, libmujoco), Cint, (Ptr{mjModel},), m)
end

"""
    mj_makeData(m::Model) -> Ptr{mjData}
Allocate `mjData` correponding to `m`.
"""
function mj_makeData(m::Model)
    ccall((:mj_makeData, libmujoco), Ptr{mjData}, (Ptr{mjModel},), m)
end

"""
    mj_copyData(dest::Data, m::Model, src::Data) -> Ptr{mjData}

Copy mjData `src` into `dest`.
"""
function mj_copyData(dest::Data, m::Model, src::Data)
    ccall(
        (:mj_copyData, libmujoco),
        Ptr{mjData},
        (Ptr{mjData}, Ptr{mjModel}, Ptr{mjData}),
        dest,
        m,
        src,
    )
end

"""
    mj_resetData(m::Model, d::Data) -> Cvoid

Set data to defaults.
"""
function mj_resetData(m::Model, d::Data)
    ccall((:mj_resetData, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_resetDataDebug(m::Model, d::Data, debug_value::Cuchar) -> Cvoid

Set `d` to defaults, fill everything else with `debug_value`.
"""
function mj_resetDataDebug(m::Model, d::Data, debug_value::Cuchar)
    ccall(
        (:mj_resetDataDebug, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cuchar),
        m,
        d,
        debug_value,
    )
end

"""
    mj_resetDataKeyframe(m::Model, d::Data, key::Integer) -> Cvoid

Reset `d` and set fields from keyframe `key`.
"""
function mj_resetDataKeyframe(m::Model, d::Data, key::Integer)
    ccall(
        (:mj_resetDataKeyframe, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint),
        m,
        d,
        key,
    )
end

"""
    mj_deleteData(d::Ptr{mjData}) -> Nothiing

De-allocate `d`.
Should only be called with `d` allocated by MuJoCo.
"""
function mj_deleteData(d::Ptr{mjData})
    ccall((:mj_deleteData, libmujoco), Cvoid, (Ptr{mjData},), d)
end

"""
    mj_resetCallbacks() -> Cvoid
Reset all callbacks to C_NULL (C_NULL is the default).
"""
function mj_resetCallbacks()
    ccall((:mj_resetCallbacks, libmujoco), Cvoid, ())
end

"""
    mj_setConst(m::Model, d::Data) -> Cvoid

Set constant fields of mjModel, corresponding to qpos0 configuration.
"""
function mj_setConst(m::Model, d::Data)
    ccall((:mj_setConst, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_setLengthRange(m::Model, d::Data, index::Integer, opt::Ref{mjLROpt})

Set actuator_lengthrange for specified actuator; return 1 if ok, otherwise
throws `MuJoCoException`.
"""
function mj_setLengthRange(m::Model, d::Data, index::Integer, opt::Ref{mjLROpt})
    err = MVector{ERRORSIZE,UInt8}(undef)
    ret = ccall(
        (:mj_setLengthRange, libmujoco),
        Cint,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjLROpt}, Cstring, Cint),
        m,
        d,
        index,
        opt,
        error,
        ERRORSIZE,
    )
    ret == 0 && @mjerror SVector(err)
    return ret
end

####
#### Printing
####

"""
    mj_printModel(m::Model, filename::String) -> Cvoid

Print `m` to `filename` on local filesystem.
"""
function mj_printModel(m::Model, filename::String)
    ccall((:mj_printModel, libmujoco), Cvoid, (Ptr{mjModel}, Cstring), m, filename)
end

"""
    mj_printData(m::Model, d::Data, filename::String) -> Cvoid
Print `d` to `filename` on local filesystem.
"""
function mj_printData(m::Model, d::Data, filename::String)
    ccall(
        (:mj_printData, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cstring),
        m,
        d,
        filename,
    )
end


####
#### Forward simulation components
####

"""
    mj_fwdPosition(m::Model, d::Data) -> Cvoid

Run position-dependent computations.
"""
function mj_fwdPosition(m::Model, d::Data)
    ccall((:mj_fwdPosition, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_fwdVelocity(m::Model, d::Data) -> Cvoid

Run velocity-dependent computations.
"""
function mj_fwdVelocity(m::Model, d::Data)
    ccall((:mj_fwdVelocity, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_fwdActuation(m::Model, d::Data) -> Cvoid

Compute actuator force `d.qfrc_actuation`.
"""
function mj_fwdActuation(m::Model, d::Data)
    ccall((:mj_fwdActuation, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_fwdAcceleration(m::Model, d::Data) -> Cvoid

Add up all non-constraint forces, compute d.qacc_unc
"""
function mj_fwdAcceleration(m::Model, d::Data)
    ccall((:mj_fwdAcceleration, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_fwdConstraint(m::Model, d::Data) -> Cvoid

Run selected constraint solver
"""
function mj_fwdConstraint(m::Model, d::Data)
    ccall((:mj_fwdConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_Euler(m::Model, d::Data)

Run Euler integrator, semi-implicit in velocity.
"""
function mj_Euler(m::Model, d::Data)
    ccall((:mj_Euler, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_RungeKutta(m::Model, d::Data, N::Integer) -> Cvoid

Run Runge-Kutta explicit order-`N` integrator
"""
function mj_RungeKutta(m::Model, d::Data, N::Integer)
    ccall((:mj_RungeKutta, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, N)
end

####
#### Inverse simulation components
####

"""
    mj_invPosition(m::Model, d::Data) -> Cvoid

Run position-dependent computations in inverse dynamics.
"""
function mj_invPosition(m::Model, d::Data)
    ccall((:mj_invPosition, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_invVelocity(m::Model, d::Data) -> Cvoid

Run velocity-dependent computations in inverse dynamics.
"""
function mj_invVelocity(m::Model, d::Data)
    ccall((:mj_invVelocity, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_invConstraint(m::Model, d::Data) -> Cvoid

Apply the analytical formula for inverse constraint dynamics.
"""
function mj_invConstraint(m::Model, d::Data)
    ccall((:mj_invConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_compareFwdInv(m::Model, d::Data) -> Cvoid

Compare forward and inverse dynamics, without changing results of forward dynamics.
Saves results in `d.fwdinv`.
"""
function mj_compareFwdInv(m::Model, d::Data)
    ccall((:mj_compareFwdInv, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end


####
#### Sub-components
####

"""

    mj_sensorPos(m::Model, d::Data) -> Cvoid

Evaluate position-dependent sensors.
"""
function mj_sensorPos(m::Model, d::Data)
    ccall((:mj_sensorPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_sensorVel(m::Model, d::Data) -> Cvoid

Evaluate velcoity-dependent sensors.
"""
function mj_sensorVel(m::Model, d::Data)
    ccall((:mj_sensorVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_sensorAcc(m::Model, d::Data) -> Cvoid

Evaluate acceleration and force-dependent sensors.
"""
function mj_sensorAcc(m::Model, d::Data)
    ccall((:mj_sensorAcc, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_energyPos(m::Model, d::Data) -> Cvoid

Evaluate position-dependent energy (potential).
"""
function mj_energyPos(m::Model, d::Data)
    ccall((:mj_energyPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_energyVel(m::Model, d::Data) -> Cvoid

Evaluate velocity-dependent energy (kinetic).
"""
function mj_energyVel(m::Model, d::Data)
    ccall((:mj_energyVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_checkPos(m::Model, d::Data) -> Cvoid

Check qpos, reset if any element is too big or nan.
"""
function mj_checkPos(m::Model, d::Data)
    ccall((:mj_checkPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_checkVel(m::Model, d::Data) -> Cvoid

Check qvel, reset if any element is too big or nan.
"""
function mj_checkVel(m::Model, d::Data)
    ccall((:mj_checkVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_checkAcc(m::Model, d::Data) -> Cvoid

Check qacc, reset if any element is too big or nan.
"""
function mj_checkAcc(m::Model, d::Data)
    ccall((:mj_checkAcc, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_kinematics(m::Model, d::Data) -> Cvoid

Run forward kinematics.
"""
function mj_kinematics(m::Model, d::Data)
    ccall((:mj_kinematics, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_comPos(m::Model, d::Data) -> Cvoid

Map inertias and motion dofs to global frame centered at CoM.
"""
function mj_comPos(m::Model, d::Data)
    ccall((:mj_comPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_camlight(m::Model, d::Data) -> Cvoid

Compute camera and light positions and orientations.
"""
function mj_camlight(m::Model, d::Data)
    ccall((:mj_camlight, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_tendon(m::Model, d::Data) -> Cvoid

Compute tendon lengths, velocities and moment arms.
"""
function mj_tendon(m::Model, d::Data)
    ccall((:mj_tendon, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_transmission(m::Model, d::Data) -> Cvoid

Compute actuator transmission lengths and moments.
"""
function mj_transmission(m::Model, d::Data)
    ccall((:mj_transmission, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_crb(m::Model, d::Data) -> Cvoid

Run composite rigid body inertia algorithm (CRB).
"""
function mj_crb(m::Model, d::Data)
    ccall((:mj_crb, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_factorM(m::Model, d::Data) -> Cvoid

Compute sparse L'*D*L factorizaton of the inertia matrix
"""
function mj_factorM(m::Model, d::Data)
    ccall((:mj_factorM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end


# TODO for mj_solveM and mj_solveM2, what is n? Test. checkbounds
"""
    mj_solveM(m, d, x, y, n) -> Cvoid

Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y.
See also: [`mj_solveM2`](@ref)

# Arguments
- `m::Model`
- `d::Data`
- `x::MJVec{mjtNum}`
- `y::MJVec{mjtNum}`
- `n::Integer`

"""
function mj_solveM(m::Model, d::Data, x::MJVec{mjtNum}, y::MJVec{mjtNum}, n::Integer)
    ccall(
        (:mj_solveM, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        x,
        y,
        length,
    )
end

"""
    mj_solveM2(m, d, x, y, n) -> Cvoid

Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
Same arguments as [`mj_solveM`](@ref).
"""
function mj_solveM2(m::Model, d::Data, x::MJVec, y::MJVec, n::Integer)
    ccall(
        (:mj_solveM2, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        x,
        y,
        n,
    )
end

"""
    mj_comVel(m::Model, d::Data) -> Cvoid

Compute cvel, cdof_dot.
"""
function mj_comVel(m::Model, d::Data)
    ccall((:mj_comVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_passive(m::Model, d::Data) -> Cvoid

Compute qfrc_passive from spring-dampers, viscosity and density.
"""
function mj_passive(m::Model, d::Data)
    ccall((:mj_passive, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_subtreeVel(m::Model, d::Data) -> Cvoid

Compute subtree linear velocity and angular momentum.
"""
function mj_subtreeVel(m::Model, d::Data)
    ccall((:mj_subtreeVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_rne(m::Model, d::Data, flg_acc::Integer, result::MJVec{mjtNum}) -> Cvoid

RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.
"""
function mj_rne(m::Model, d::Data, flg_acc::Integer, result::MJVec)
    ccall(
        (:mj_rne, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}),
        m,
        d,
        flg_acc,
        result,
    )
end

"""
    mj_rnePostConstraint(m::Model, d::Data) -> Cvoid

RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
"""
function mj_rnePostConstraint(m::Model, d::Data)
    ccall((:mj_rnePostConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_collision(m::Model, d::Data) -> Cvoid

Run collision detection.
"""
function mj_collision(m::Model, d::Data)
    ccall((:mj_collision, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_makeConstraint(m::Model, d::Data) -> Cvoid

Construct constraints.
"""
function mj_makeConstraint(m::Model, d::Data)
    ccall((:mj_makeConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_projectConstraint(m::Model, d::Data) -> Cvoid

Compute inverse constaint inertia `efc_AR`.
"""
function mj_projectConstraint(m::Model, d::Data)
    ccall((:mj_projectConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_referenceConstraint(m::Model, d::Data) -> Cvoid

Compute `efc_vel`, `efc_aref`.
"""
function mj_referenceConstraint(m::Model, d::Data)
    ccall((:mj_referenceConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

"""
    mj_constraintUpdate(m, d, jar, flg_coneHessian)
    mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)

Compute `efc_state`, `efc_force`, `qfrc_constraint`, and (optionally) cone Hessians.
If specified, uses cost = s(`jar`) where jar = `Jac`*`qacc`-`aref`.

# Arguments
- `m::Model`
- `d::Data`
- `jar::MJVec{mjtNum}`
- `cost::MJVec{mjtNum}`
- `flg_coneHessian::Integer`
"""
@inline function mj_constraintUpdate(
    m::Model,
    d::Data,
    jar::MJVec{mjtNum},
    flg_coneHessian::Integer,
)
    _mj_constraintUpdate(m, d, jar, C_NULL, flg_coneHessian)
end
@inline function mj_constraintUpdate(
    m::Model,
    d::Data,
    jar::MJVec{mjtNum},
    cost::MJVec{mjtNum},
    flg_coneHessian::Integer,
)
    _mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)
end
function _mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)
    ccall(
        (:mj_constraintUpdate, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, MJVec, MJVec, Integer),
        m,
        d,
        jar,
        cost,
        flg_coneHessian,
    )
end



####
#### Support functions
####

"""
    mj_addContact(m::Model, d::Data, con::Ref{mjContact}) -> Cint

Add contact `con` to `d.contact` list. Return 0 if success; 1 if buffer full.
"""
function mj_addContact(m::Model, d::Data, con::Ref{mjContact})
    ccall(
        (:mj_addContact, libmujoco),
        Cint,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjContact}),
        m,
        d,
        con,
    )
end

"""
    mj_isPyramidal(m::Model) -> Cint

Determine type of friction cone.
"""
function mj_isPyramidal(m::Model)
    ccall((:mj_isPyramidal, libmujoco), Cint, (Ptr{mjModel},), m)
end

"""
    mj_isSparse(m::Model) -> Cint

Determine type of constraint Jacobian.
"""
function mj_isSparse(m::Model)
    ccall((:mj_isSparse, libmujoco), Cint, (Ptr{mjModel},), m)
end

"""
    mj_isDual(m::Model) -> Cint

Determine type of solver (PGS is dual, CG and Newton are primal).
"""
function mj_isDual(m::Model)
    ccall((:mj_isDual, libmujoco), Cint, (Ptr{mjModel},), m)
end

"""
    mj_mulJacVec(m::Model, d::Data, res::MJVec{mjtNum}, vec::MJVec{mjtNum}) -> Cvoid

Multiply dense or sparse constraint Jacobian by vector.
See also: [`mj_mulJacTVec`](@ref)
"""
function mj_mulJacVec(m::Model, d::Data, res::MJVec{mjtNum}, vec::MJVec)
    ccall(
        (:mj_mulJacVec, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        res,
        vec,
    )
end

"""
    mj_mulJacTVec(m::Model, d::Data, res::MJVec{mjtNum}, vec::MJVec{mjtNum}) -> Cvoid

Multiply dense or sparse constraint Jacobian transpose by vector.
See also: [`mj_mulJacVec`](@ref)
"""
function mj_mulJacTVec(m::Model, d::Data, res::MJVec, vec::MJVec)
    ccall(
        (:mj_mulJacTVec, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        res,
        vec,
    )
end

"""
    mj_jac(m, d, jacp, jacr, point, body) -> Cvoid

Compute 3/6-by-nv Jacobian of global `point` attached to `body`.

# Arguments
- `m::Model`
- `d::Data`
- `jacp::MJVec{mjtNum}`
- `jacr::MJVec{mjtNum}`
- `point::SizedMJVec{3,mjtNum}`:
- `body::Integer`
"""
function mj_jac(
    m::Model,
    d::Data,
    jacp::MJVec{mjtNum},
    jacr::MJVec{mjtNum},
    point::StaticVector{3,mjtNum},
    body::Integer,
)
    ccall(
        (:mj_jac, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, SVector{3,mjtNum}, Cint),
        m,
        d,
        jacp,
        jacr,
        point,
        body,
    )
end

"""
    mj_jacBody(m, d, jacp, jacr, point, body) -> Cvoid

Compute body frame end-effector Jacobian.

# Arguments
- `m::Model`
- `d::Data`
- `jacp::MJVec`
- `jacr::MJVec`
- `point::SVector{3,mjtNum}`
- `body::Integer`
"""
function mj_jacBody(
    m::Model,
    d::Data,
    jacp::MJVec{mjtNum},
    jacr::MJVec{mjtNum},
    body::Integer,
)
    ccall(
        (:mj_jacBody, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        jacp,
        jacr,
        body,
    )
end

"""
    mj_jacBody(m, d, jacp, jacr, point, body) -> Cvoid

Compute body frame end-effector Jacobian.

# Arguments
- `m::Model`
- `d::Data`
- `jacp::MJVec`
- `jacr::MJVec`
- `body::Integer`
"""
function mj_jacBodyCom(m::Model, d::Data, jacp::MJVec, jacr::MJVec, body::Integer)
    ccall(
        (:mj_jacBodyCom, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        jacp,
        jacr,
        body,
    )
end

"""
    mj_jacGeom(m, d, jacp, jacr, point, geom) -> Cvoid

Compute body frame end-effector Jacobian.

# Arguments
- `m::Model`
- `d::Data`
- `jacp::MJVec`
- `jacr::MJVec`
- `geom::Integer`
"""
function mj_jacGeom(m::Model, d::Data, jacp::MJVec, jacr::MJVec, geom::Integer)
    ccall(
        (:mj_jacGeom, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        jacp,
        jacr,
        geom,
    )
end

"""
    mj_jacSite(m, d, jacp, jacr, point, geom) -> Cvoid

Compute body frame end-effector Jacobian.

# Arguments
- `m::Model`
- `d::Data`
- `jacp::MJVec`
- `jacr::MJVec`
- `site::Integer`
"""
function mj_jacSite(m::Model, d::Data, jacp::MJVec, jacr::MJVec, site::Integer)
    ccall(
        (:mj_jacSite, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        m,
        d,
        jacp,
        jacr,
        site,
    )
end

"""
Compute translation Jacobian of point, and rotation Jacobian of axis
"""
function mj_jacPointAxis(
    m::Model,
    d::Data,
    jacPoint::MJVec{mjtNum},
    jacAxis::MJVec{mjtNum},
    point::SVector{3,mjtNum},
    axis::SVector{3,mjtNum},
    body::Integer,
)
    ccall(
        (:mj_jacPointAxis, libmujoco),
        Cvoid,
        (
         Ptr{mjModel},
         Ptr{mjData},
         Ptr{mjtNum},
         Ptr{mjtNum},
         SVector{3,mjtNum},
         SVector{3,mjtNum},
         Cint,
        ),
        m,
        d,
        jacPoint,
        jacAxis,
        point,
        axis,
        body,
    )
end

"""get id of object with specified name; -1: not found; type is mjtObj"""
function mj_name2id(m::Model, type::Union{Integer,mjtObj}, name::String)
    ccall(
        (:mj_name2id, libmujoco),
        Cint,
        (Ptr{mjModel}, Cint, Cstring),
        m,
        type,
        pointer(name),
    )
end

"""get name of object with specified id; 0: invalid type or id; type is mjtObj"""
function mj_id2name(m::Model, type::Union{Integer,mjtObj}, id::Integer)
    name = ccall(
        (:mj_id2name, libmujoco),
        Cstring,
        (Ptr{mjModel}, Cint, Cint),
        m,
        type,
        id,
    ) # julia to c indexing
    name == C_NULL ? nothing : unsafe_string(name)
end

"""convert sparse inertia matrix M into full matrix"""
function mj_fullM(m::Model, dst::MJVec, M::MJVec)
    ccall(
        (:mj_fullM, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        dst,
        M,
    )
end

"""multiply vector by inertia matrix"""
function mj_mulM(m::Model, d::Data, res::MJVec, vec::MJVec)
    ccall(
        (:mj_mulM, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        res,
        vec,
    )
end

"""Multiply vector by (inertia matrix)^(1/2)."""
function mj_mulM2(m::Model, d::Data, res::MJVec, vec::MJVec)
    ccall(
        (:mj_mulM2, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        res,
        vec,
    )
end

"""Add inertia matrix to destination matrix.
Destination can be sparse uncompressed, or dense when all int* are NULL
"""
function mj_addM(
    m::Model,
    d::Data,
    dst::MJVec,
    rownnz::Vector{Integer},
    rowadr::Vector{Integer},
    colind::Vector{Integer},
)
    ccall(
        (:mj_addM, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}),
        m,
        d,
        dst,
        rownnz,
        rowadr,
        colind,
    )
end

"""apply cartesian force and torque (outside xfrc_applied mechanism)"""
function mj_applyFT(
    m::Model,
    d::Data,
    force::MJVec,
    torque::MJVec,
    point::MJVec,
    body::Integer,
    qfrc_target::MJVec,
)
    ccall(
        (:mj_applyFT, libmujoco),
        Cvoid,
        (
         Ptr{mjModel},
         Ptr{mjData},
         Ptr{mjtNum},
         Ptr{mjtNum},
         Ptr{mjtNum},
         Cint,
         Ptr{mjtNum},
        ),
        m,
        d,
        force,
        torque,
        point,
        body,
        qfrc_target,
    )
end

"""compute object 6D velocity in object-centered frame, world/local orientation"""
function mj_objectVelocity(
    m::Model,
    d::Data,
    objtype::Integer,
    objid::Integer,
    res::MJVec,
    flg_local::Integer,
)
    ccall(
        (:mj_objectVelocity, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint),
        m,
        d,
        objtype,
        objid,
        res,
        flg_local,
    )
end

"""compute object 6D acceleration in object-centered frame, world/local orientation"""
function mj_objectAcceleration(
    m::Model,
    d::Data,
    objtype::Integer,
    objid::Integer,
    res::MJVec,
    flg_local::Integer,
)
    ccall(
        (:mj_objectAcceleration, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint),
        m,
        d,
        objtype,
        objid,
        res,
        flg_local,
    )
end

"""extract 6D force:torque for one contact, in contact frame"""
function mj_contactForce(m::Model, d::Data, id::Integer, result::MJVec)
    ccall(
        (:mj_contactForce, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}),
        m,
        d,
        id,
        result,
    )
end

"""compute velocity by finite-differencing two positions"""
function mj_differentiatePos(m::Model, qvel::MJVec, dt::mjtNum, qpos1::MJVec, qpos2::MJVec)
    ccall(
        (:mj_differentiatePos, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        qvel,
        dt,
        qpos1,
        qpos2,
    )
end

"""integrate position with given velocity"""
function mj_integratePos(m::Model, qpos::MJVec, qvel::MJVec, dt::mjtNum)
    ccall(
        (:mj_integratePos, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum),
        m,
        qpos,
        qvel,
        dt,
    )
end

"""normalize all quaterions in qpos-type vector"""
function mj_normalizeQuat(m::Model, qpos::MJVec)
    ccall((:mj_normalizeQuat, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}), m, qpos)
end

"""map from body local to global Cartesian coordinates"""
function mj_local2Global(
    d::Data,
    xpos::MJVec,
    xmat::MJVec,
    pos::MJVec,
    quat::MJVec,
    body::Integer,
    sameframe::mjtByte,
)
    ccall(
        (:mj_local2Global, libmujoco),
        Cvoid,
        (Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, mjtByte),
        d,
        xpos,
        xmat,
        pos,
        quat,
        body,
        sameframe,
    )
end

"""sum all body masses"""
function mj_getTotalmass(m::Model)
    ccall((:mj_getTotalmass, libmujoco), mjtNum, (Ptr{mjModel},), m)
end

"""scale body masses and inertias to achieve specified total mass"""
function mj_setTotalmass(m::Model, newmass::mjtNum)
    ccall((:mj_setTotalmass, libmujoco), Cvoid, (Ptr{mjModel}, mjtNum), m, newmass)
end

"""version number: 1.0.2 is encoded as 102 #TODO comment??"""
function mj_version()
    ccall((:mj_version, libmujoco), Cint, ())
end

#---------------------- Ray collisions -------------------------------------------------

"""
Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
Return geomid and distance (x) to nearest surface, or -1 if no intersection.
geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.
"""
function mj_ray(
    m::Model,
    d::Data,
    pnt::MJVec,
    vec::MJVec,
    geomgroup::Vector{mjtByte},
    flg_static::mjtByte,
    bodyexclude::Integer,
    geomid::Vector{Integer},
)
    ccall(
        (:mj_ray, libmujoco),
        mjtNum,
        (
         Ptr{mjModel},
         Ptr{mjData},
         Ptr{mjtNum},
         Ptr{mjtNum},
         Ptr{mjtByte},
         mjtByte,
         Cint,
         Ptr{Cint},
        ),
        m,
        d,
        pnt,
        vec,
        geomgroup,
        flg_static,
        bodyexclude,
        geomid,
    )
end

"""Interect ray with hfield, return nearest distance or -1 if no intersection."""
function mj_rayHfield(m::Model, d::Data, geomid::Integer, pnt::MJVec, vec::MJVec)
    ccall(
        (:mj_rayHfield, libmujoco),
        mjtNum,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        geomid,
        pnt,
        vec,
    )
end

"""Interect ray with mesh, return nearest distance or -1 if no intersection."""
function mj_rayMesh(m::Model, d::Data, geomid::Integer, pnt::MJVec, vec::MJVec)
    ccall(
        (:mj_rayMesh, libmujoco),
        mjtNum,
        (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}),
        m,
        d,
        geomid,
        pnt,
        vec,
    )
end


#---------------------- Abstract interaction -------------------------------------------

"""set default camera"""
function mjv_defaultCamera(cam::Ref{mjvCamera})
    ccall((:mjv_defaultCamera, libmujoco), Cvoid, (Ptr{mjvCamera},), cam)
end

"""set default perturbation"""
function mjv_defaultPerturb(pert::Ref{mjvPerturb})
    ccall((:mjv_defaultPerturb, libmujoco), Cvoid, (Ptr{mjvPerturb},), pert)
end

"""transform pose from room to model space"""
function mjv_room2model(
    modelpos::MJVec,
    modelquat::MJVec,
    roompos::MJVec,
    roomquat::MJVec,
    scn::Ref{mjvScene},
)
    ccall(
        (:mjv_room2model, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}),
        modelpos,
        modelquat,
        roompos,
        roomquat,
        scn,
    )
end

"""transform pose from model to room space"""
function mjv_model2room(
    roompos::MJVec,
    roomquat::MJVec,
    modelpos::MJVec,
    modelquat::MJVec,
    scn::Ref{mjvScene},
)
    ccall(
        (:mjv_model2room, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}),
        roompos,
        roomquat,
        modelpos,
        modelquat,
        scn,
    )
end

"""get camera info in model space: average left and right OpenGL cameras"""
function mjv_cameraInModel(headpos::MJVec, forward::MJVec, scn::Ref{mjvScene})
    ccall(
        (:mjv_cameraInModel, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}),
        headpos,
        forward,
        scn,
    )
end

"""get camera info in room space: average left and right OpenGL cameras"""
function mjv_cameraInRoom(headpos::MJVec, forward::MJVec, scn::Ref{mjvScene})
    ccall(
        (:mjv_cameraInRoom, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}),
        headpos,
        forward,
        scn,
    )
end

"""get frustum height at unit distance from camera; average left and right OpenGL cameras"""
function mjv_frustumHeight(scn::Ref{mjvScene})
    ccall((:mjv_frustumHeight, libmujoco), mjtNum, (Ptr{mjvScene},), scn)
end

"""rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)"""
function mjv_alignToCamera(res::MJVec, vec::MJVec, forward::MJVec)
    ccall(
        (:mjv_alignToCamera, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}),
        res,
        vec,
        forward,
    )
end

"""move camera with mouse; action is mjtMouse"""
function mjv_moveCamera(
    m::Model,
    action::Union{Integer, mjtMouse},
    reldx::mjtNum,
    reldy::mjtNum,
    scn::Ref{mjvScene},
    cam::Ref{mjvCamera},
)
    ccall(
        (:mjv_moveCamera, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}),
        m,
        action,
        reldx,
        reldy,
        scn,
        cam,
    )
end

"""move perturb object with mouse; action is mjtMouse"""
function mjv_movePerturb(
    m::Model,
    d::Data,
    action::Union{Integer, mjtMouse},
    reldx::mjtNum,
    reldy::mjtNum,
    scn::Ref{mjvScene},
    pert::Ref{mjvPerturb},
)
    ccall(
        (:mjv_movePerturb, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}),
        m,
        d,
        action,
        reldx,
        reldy,
        scn,
        pert,
    )
end

"""move model with mouse; action is mjtMouse"""
function mjv_moveModel(
    m::Model,
    action::Integer,
    reldx::mjtNum,
    reldy::mjtNum,
    roomup::MJVec,
    scn::Ref{mjvScene},
)
    ccall(
        (:mjv_moveModel, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjtNum}, Ptr{mjvScene}),
        m,
        action,
        reldx,
        reldy,
        roomup,
        scn,
    )
end

"""copy perturb pos,quat from selected body; set scale for perturbation"""
function mjv_initPerturb(m::Model, d::Data, scn::Ref{mjvScene}, pert::Ref{mjvPerturb})
    ccall(
        (:mjv_initPerturb, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}, Ptr{mjvPerturb}),
        m,
        d,
        scn,
        pert,
    )
end

"""set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
d->qpos written only if flg_paused and subtree root for selected body has free joint
"""
function mjv_applyPerturbPose(m::Model, d::Data, pert::Ref{mjvPerturb}, flg_paused::Integer)
    ccall(
        (:mjv_applyPerturbPose, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}, Cint),
        m,
        d,
        pert,
        flg_paused,
    )
end

"""set perturb force,torque in d->xfrc_applied, if selected body is dynamic"""
function mjv_applyPerturbForce(m::Model, d::Data, pert::Ref{mjvPerturb})
    ccall(
        (:mjv_applyPerturbForce, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}),
        m,
        d,
        pert,
    )
end

"""Return the average of two OpenGL cameras."""
function mjv_averageCamera(cam1::Ptr{mjvGLCamera}, cam2::Ref{mjvGLCamera})
    ccall(
        (:mjv_averageCamera, libmujoco),
        mjvGLCamera,
        (Ptr{mjvGLCamera}, Ptr{mjvGLCamera}),
        cam1,
        cam2,
    )
end

"""Select geom or skin with mouse, return bodyid; -1: none selected."""
function mjv_select(
    m::Model,
    d::Data,
    vopt::Ref{mjvOption},
    aspectratio::mjtNum,
    relx::mjtNum,
    rely::mjtNum,
    scn::Ref{mjvScene},
    selpnt::MJVec,
    geomid::Ref{Cint},
    skinid::Ref{Cint}
)
    ccall(
        (:mjv_select, libmujoco),
        Cint,
        (
         Ptr{mjModel},
         Ptr{mjData},
         Ptr{mjvOption},
         mjtNum,
         mjtNum,
         mjtNum,
         Ptr{mjvScene},
         Ptr{mjtNum},
         Ptr{Cint},
         Ptr{Cint},
        ),
        m,
        d,
        vopt,
        aspectratio,
        relx,
        rely,
        scn,
        selpnt,
        geomid,
        skinid,
    )
end


#---------------------- Asbtract visualization -----------------------------------------

"""set default visualization options"""
function mjv_defaultOption(opt::Ref{mjvOption})
    ccall((:mjv_defaultOption, libmujoco), Cvoid, (Ptr{mjvOption},), opt)
end

"""Set default figure."""
function mjv_defaultFigure(fig::Ref{mjvFigure})
    ccall((:mjv_defaultFigure, libmujoco), Cvoid, (Ptr{mjvFigure},), fig)
end

"""Initialize given geom fields when not NULL, set the rest to their default values."""
function mjv_initGeom(
    geom::Ref{mjvGeom},
    type::Integer,
    size::MJVec,
    pos::MJVec,
    mat::MJVec,
    rgba::Ptr{Float32},
)
    ccall(
        (:mjv_initGeom, libmujoco),
        Cvoid,
        (Ptr{mjvGeom}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cfloat}),
        geom,
        type,
        size,
        pos,
        mat,
        rbga,
    )
end

"""Set (type, size, pos, mat) for connector-type geom between given points.
Assume that mjv_initGeom was already called to set all other properties.
"""
function mjv_makeConnector(
    geom::Ref{mjvGeom},
    type::Integer,
    width::mjtNum,
    a0::mjtNum,
    a1::mjtNum,
    a2::mjtNum,
    b0::mjtNum,
    b1::mjtNum,
    b2::mjtNum,
)
    ccall(
        (:mjv_makeConnector, libmujoco),
        Cvoid,
        (Ptr{mjvGeom}, Cint, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum),
        geom,
        type,
        width,
        a0,
        a1,
        a2,
        b0,
        b1,
        b2,
    )
end

"""Set default abstract scene."""
function mjv_defaultScene(scn::Ref{mjvScene})
    ccall((:mjv_defaultScene, libmujoco), Cvoid, (Ptr{mjvScene},), scn)
end

"""allocate and init abstract scene"""
function mjv_makeScene(m::Model, scn::Ref{mjvScene}, maxgeom::Integer)
    ccall(
        (:mjv_makeScene, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjvScene}, Cint),
        m,
        scn,
        maxgeom,
    )
end

"""free abstract scene"""
function mjv_freeScene(scn::Ref{mjvScene})
    ccall((:mjv_freeScene, libmujoco), Cvoid, (Ptr{mjvScene},), scn)
end

"""update entire scene"""
function mjv_updateScene(
    m::Model,
    d::Data,
    opt::Ref{mjvOption},
    pert::Ref{mjvPerturb},
    cam::Ref{mjvCamera},
    catmask::Union{Integer, mjtCatBit},
    scn::Ref{mjvScene},
)
    ccall(
        (:mjv_updateScene, libmujoco),
        Cvoid,
        (
         Ptr{mjModel},
         Ptr{mjData},
         Ptr{mjvOption},
         Ptr{mjvPerturb},
         Ptr{mjvCamera},
         Cint,
         Ptr{mjvScene},
        ),
        m,
        d,
        opt,
        pert,
        cam,
        catmask,
        scn,
    )
end


"""add geoms from selected categories"""
function mjv_addGeoms(
    m::Model,
    d::Data,
    opt::Ref{mjvOption},
    pert::Ref{mjvPerturb},
    catmask::Union{Integer, mjtCatBit},
    scn::Ref{mjvScene},
)
    ccall(
        (:mjv_addGeoms, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Cint, Ptr{mjvScene}),
        m,
        d,
        opt,
        pert,
        catmask,
        scn,
    )
end

"""Make list of lights."""
function mjv_makeLights(m::Model, d::Data, scn::Ref{mjvScene})
    ccall(
        (:mjv_makeLights, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}),
        m,
        d,
        scn,
    )
end

"""update camera only"""
function mjv_updateCamera(m::Model, d::Data, cam::Ref{mjvCamera}, scn::Ref{mjvScene})
    ccall(
        (:mjv_updateCamera, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvCamera}, Ptr{mjvScene}),
        m,
        d,
        cam,
        scn,
    )
end

"""Update skins."""
function mjv_updateSkin(m::Model, d::Data, scn::Ref{mjvScene})
    ccall(
        (:mjv_updateSkin, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}),
        m,
        d,
        scn,
    )
end


#---------------------- OpenGL rendering -----------------------------------------------

"""set default mjrContext"""
function mjr_defaultContext(con::Ref{mjrContext})
    ccall((:mjr_defaultContext, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end

"""allocate resources in custom OpenGL context; fontscale is mjtFontScale"""
function mjr_makeContext(m::Model, con::Ref{mjrContext}, fontscale::Union{mjtFontScale, Integer})
    ccall(
        (:mjr_makeContext, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjrContext}, Cint),
        m,
        con,
        fontscale,
    )
end

"""Change font of existing context."""
function mjr_changeFont(fontscale::Integer, con::Ref{mjrContext})
    ccall((:mjr_changeFont, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), fontscale, con)
end

"""Add Aux buffer with given index to context; free previous Aux buffer."""
function mjr_addAux(
    index::Integer,
    width::Integer,
    height::Integer,
    samples::Integer,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_addAux, libmujoco),
        Cvoid,
        (Cint, Cint, Cint, Cint, Ptr{mjrContext}),
        index,
        width,
        height,
        samples,
        con,
    )
end

"""free resources in custom OpenGL context, set to default"""
function mjr_freeContext(con::Ref{mjrContext})
    ccall((:mjr_freeContext, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end

"""(re) upload texture to GPU"""
function mjr_uploadTexture(m::Model, con::Ref{mjrContext}, texid::Integer)
    ccall(
        (:mjr_uploadTexture, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjrContext}, Cint),
        m,
        con,
        texid,
    )
end

"""(re) upload mesh to GPU"""
function mjr_uploadMesh(m::Model, con::Ref{mjrContext}, meshid::Integer)
    ccall(
        (:mjr_uploadMesh, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjrContext}, Cint),
        m,
        con,
        meshid,
    )
end

"""(re) upload height field to GPU"""
function mjr_uploadHField(m::Model, con::Ref{mjrContext}, hfieldid::Integer)
    ccall(
        (:mjr_uploadHField, libmujoco),
        Cvoid,
        (Ptr{mjModel}, Ptr{mjrContext}, Cint),
        m,
        con,
        hfieldid,
    )
end

"""Make con->currentBuffer current again."""
function mjr_restoreBuffer(con::Ref{mjrContext})
    ccall((:mjr_restoreBuffer, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end

"""set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN
if only one buffer is available, set that buffer and ignore framebuffer argument
"""
function mjr_setBuffer(framebuffer::Integer, con::Ref{mjrContext})
    ccall((:mjr_setBuffer, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), framebuffer, con)
end

"""read pixels from current OpenGL framebuffer to client buffer
viewport is in OpenGL framebuffer; client buffer starts at (0,0)
"""
function mjr_readPixels(
    rgb::MJVec{Cuchar}, # TODO can rgb be C_NULL as well?
    depth::Union{Ptr{Nothing}, MJVec},
    viewport::mjrRect,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_readPixels, libmujoco),
        Cvoid,
        (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}),
        rgb,
        depth,
        viewport,
        con,
    )
end

"""draw pixels from client buffer to current OpenGL framebuffer
viewport is in OpenGL framebuffer; client buffer starts at (0,0)
"""
function mjr_drawPixels(
    rgb::MJVec{Cuchar},
    depth::MJVec,
    viewport::mjrRect,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_drawPixels, libmujoco),
        Cvoid,
        (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}),
        rgb,
        depth,
        viewport,
        con,
    )
end

"""blit from src viewpoint in current framebuffer to dst viewport in other framebuffer
if src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR
"""
function mjr_blitBuffer(
    src::mjrRect,
    dst::mjrRect,
    flg_color::Integer,
    flg_depth::Integer,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_blitBuffer, libmujoco),
        Cvoid,
        (mjrRect, mjrRect, Cint, Cint, Ptr{mjrContext}),
        src,
        dst,
        flg_color,
        flg_depth,
        con,
    )
end

"""Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done)."""
function mjr_setAux(index::Integer, con::Ref{mjrContext})
    ccall((:mjr_setAux, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), index, con)
end

"""Blit from Aux buffer to con->currentBuffer."""
function mjr_blitAux(
    index::Integer,
    src::mjrRect,
    left::Integer,
    bottom::Integer,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_blitAux, libmujoco),
        Cvoid,
        (Cint, mjrRect, Cint, Cint, Ptr{mjrContext}),
        index,
        src,
        left,
        bottom,
        con,
    )
end

"""draw text at (x,y) in relative coordinates; font is mjtFont"""
function mjr_text(
    font::Union{Integer, mjtFont},
    txt::AbstractString,
    con::Ref{mjrContext},
    x::Real,
    y::Real,
    r::Real,
    g::Real,
    b::Real,
)
    ccall(
        (:mjr_text, libmujoco),
        Cvoid,
        (Cint, Cstring, Ptr{mjrContext}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat),
        font,
        txt,
        con,
        x,
        y,
        r,
        g,
        b,
    )
end

"""draw text overlay; font is mjtFont; gridpos is mjtGridPos"""
function mjr_overlay(
    font::Union{Integer, mjtFont},
    gridpos::Union{Integer, mjtGridPos},
    viewport::mjrRect,
    overlay::String,
    overlay2::String,
    con::Ref{mjrContext},
)
    ccall(
        (:mjr_overlay, libmujoco),
        Cvoid,
        (Cint, Cint, mjrRect, Cstring, Cstring, Ptr{mjrContext}),
        font,
        gridpos,
        viewport,
        overlay,
        overlay2,
        con,
    )
end

"""get maximum viewport for active buffer"""
function mjr_maxViewport(con::Ref{mjrContext})
    ccall((:mjr_maxViewport, libmujoco), mjrRect, (Ptr{mjrContext},), con)
end

"""draw rectangle"""
function mjr_rectangle(viewport::mjrRect, r::Cfloat, g::Cfloat, b::Cfloat, a::Cfloat)
    ccall(
        (:mjr_rectangle, libmujoco),
        Cvoid,
        (mjrRect, Cfloat, Cfloat, Cfloat, Cfloat),
        viewport,
        r,
        g,
        b,
        a,
    )
end

"""draw lines"""
function mjr_figure(viewport::mjrRect, fig::Ref{mjvFigure}, con::Ref{mjrContext})
    ccall(
        (:mjr_figure, libmujoco),
        Cvoid,
        (mjrRect, Ptr{mjvFigure}, Ptr{mjrContext}),
        viewport,
        fig,
        con,
    )
end

"""3D rendering"""
function mjr_render(viewport::mjrRect, scn::Ref{mjvScene}, con::Ref{mjrContext})
    ccall(
        (:mjr_render, libmujoco),
        Cvoid,
        (mjrRect, Ptr{mjvScene}, Ptr{mjrContext}),
        viewport,
        scn,
        con,
    )
end

"""call glFinish"""
function mjr_finish()
    ccall((:mjr_finish, libmujoco), Cvoid, ())
end

"""call glGetError and return result"""
function mjr_getError()
    ccall((:mjr_getError, libmujoco), Cint, ())
end

"""Find first rectangle containing mouse, -1: not found."""
function mjr_findRect(x::Integer, y::Integer, nrect::Integer, rect::MJVec{mjrRect})
    ccall(
        (:mjr_findRect, libmujoco),
        Cint,
        (Cint, Cint, Cint, Ptr{mjrRect}),
        x,
        y,
        nrect,
        rect,
    )
end


#---------------------- Utility functions: error and memory ----------------------------

"""print matrix to screen"""
function mju_printMat(mat::MJVec, nr::Integer, nc::Integer)
    ccall((:mju_printMat, libmujoco), Cvoid, (Ptr{mjtNum}, Cint, Cint), mat, nr, nc)
end

"""Print sparse matrix to screen."""
function mju_printMatSparse(
    mat::MJVec,
    nr::Integer,
    rownnz::Vector{Integer},
    rowadr::Vector{Integer},
    colind::Vector{Integer},
)
    ccall(
        (:mju_printMatSparse, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}),
        mat,
        nr,
        rownnz,
        rowadr,
        colind,
    )
end

"""Interect ray with pure geom, return nearest distance or -1 if no intersection."""
function mju_rayGeom(
    pos::MJVec,
    mat::MJVec,
    size::MJVec,
    pnt::MJVec,
    vec::MJVec,
    geomtype::Integer,
)
    ccall(
        (:mju_rayGeom, libmujoco),
        mjtNum,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        pos,
        mat,
        size,
        pnt,
        vec,
        geomtype,
    )
end

"""Interect ray with skin, return nearest vertex id."""
function mju_raySkin(
    nface::Integer,
    nvert::Integer,
    face::MJVec{Integer},
    vert::MJVec{Float32},
    pnt::MJVec,
    vec::MJVec,
    vertid::MJVec{Integer},
)
    ccall(
        (:mju_raySkin, libmujoco),
        mjtNum,
        (Integer, Integer, Ptr{Cint}, Ptr{Cfloat}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}),
        nface,
        nvert,
        face,
        veert,
        pnt,
        vec,
        vertid,
    )
end

"""main error function; does not return to caller"""
function mju_error(msg::String)
    ccall((:mju_error, libmujoco), Cvoid, (Cstring,), msg)
end

"""error function mju_with int argument; msg is a printf format string"""
function mju_error_i(msg::String, i::Integer)
    ccall((:mju_error_i, libmujoco), Cvoid, (Cstring, Cint), msg, i)
end

"""error function mju_with string argument"""
function mju_error_s(msg::String, text::String)
    ccall((:mju_error_s, libmujoco), Cvoid, (Cstring, Cstring), msg, text)
end

"""main warning function; returns to caller"""
function mju_warning(msg::String)
    ccall((:mju_warning, libmujoco), Cvoid, (Cstring,), msg)
end

"""warning function mju_with int argument"""
function mju_warning_i(msg::String, i::Integer)
    ccall((:mju_warning_i, libmujoco), Cvoid, (Cstring, Cint), msg, i)
end

"""warning function mju_with string argument"""
function mju_warning_s(msg::String, text::String)
    ccall((:mju_warning_s, libmujoco), Cvoid, (Cstring, Cstring), msg, text)
end

"""clear user error and memory handlers"""
function mju_clearHandlers()
    ccall((:mju_clearHandlers, libmujoco), Cvoid, ())
end

"""allocate memory; byte-align on 8; pad size to multiple of 8"""
function mju_malloc(size::Integer)
    ccall((:mju_malloc, libmujoco), Ptr{Cvoid}, (Cint,), size)
end

"""free memory (with free() by default)"""
function mju_free(ptr::Ptr{Cvoid})
    ccall((:mju_free, libmujoco), Cvoid, (Ptr{Cvoid},), ptr)
end

"""high-level warning function: count warnings in Data, print only the first"""
function mj_warning(d::Data, warning::Integer, info::Integer)
    ccall((:mj_warning, libmujoco), Cvoid, (Ptr{mjData}, Cint, Cint), d, warning, info)
end

"""Write [datetime, type: message] to MUJOCO_LOG.TXT."""
function mju_writeLog(type::String, msg::String)
    ccall((:mju_writeLog, libmujoco), Cvoid, (Cstring, Cstring), type, msg)
end



#---------------------- Utility functions: miscellaneous -------------------------------

"""Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."""
function mju_muscleGain(
    len::mjtNum,
    vel::mjtNum,
    lengthrange::MJVec,
    acc0::mjtNum,
    prm::MJVec,
)
    @assert length(lengthrange) >= 2
    @assert length(prm) >= 9
    ccall(
        (:mju_muscleGain, libmujoco),
        mjtNum,
        (mjtNum, mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}),
        len,
        vel,
        lengthrange,
        acc0,
        prm,
    )
end

"""Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."""
function mju_muscleBias(len::mjtNum, lengthrange::MJVec, acc0::mjtNum, prm::MJVec)
    @assert length(lengthrange) >= 2
    @assert length(prm) >= 9
    ccall(
        (:mju_muscleBias, libmujoco),
        mjtNum,
        (mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}),
        len,
        lengthrange,
        acc0,
        prm,
    )
end

"""Muscle activation dynamics, prm = (tau_act, tau_deact)."""
function mju_muscleDynamics(ctrl::mjtNum, act::mjtNum, prm::MJVec)
    @assert length(prm) >= 2
    ccall(
        (:mju_muscleDynamics, libmujoco),
        mjtNum,
        (mjtNum, mjtNum, Ptr{mjtNum}),
        ctrl,
        act,
        prm,
    )
end

"""convert contact force to pyramid representation"""
function mju_encodePyramid(pyramid::MJVec, force::MJVec, mu::MJVec, dim::Integer)
    ccall(
        (:mju_encodePyramid, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        pyramid,
        force,
        mu,
        dim,
    )
end

"""convert pyramid representation to contact force"""
function mju_decodePyramid(force::MJVec, pyramid::MJVec, mu::MJVec, dim::Integer)
    ccall(
        (:mju_decodePyramid, libmujoco),
        Cvoid,
        (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint),
        force,
        pyramid,
        mu,
        dim,
    )
end

"""integrate spring-damper analytically, return pos(dt)"""
function mju_springDamper(pos0::mjtNum, vel0::mjtNum, Kp::mjtNum, Kv::mjtNum, dt::mjtNum)
    ccall(
        (:mju_springDamper, libmujoco),
        mjtNum,
        (mjtNum, mjtNum, mjtNum, mjtNum, mjtNum),
        pos0,
        vel0,
        Kp,
        Kv,
        dt,
    )
end

"""convert type id (mjtObj) to type name"""
function mju_type2Str(type::Union{mjtObj, Integer})
    name = ccall((:mju_type2Str, libmujoco), Cstring, (Cint,), type)
    name == C_NULL ? nothing : unsafe_string(name)
end

"""convert type name to type id (mjtObj)"""
function mju_str2Type(str::String)
    ccall((:mju_str2Type, libmujoco), Cint, (Cstring,), str)
end

"""warning text"""
function mju_warningText(warning::Integer, info::Integer)
    ccall((:mju_warningText, libmujoco), Cstring, (Cint, Cint), warning, info)
end

"""return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise"""
function mju_isBad(x::mjtNum)
    ccall((:mju_isBad, libmujoco), Cint, (mjtNum,), x)
end

"""return 1 if all elements are 0"""
function mju_isZero(vec::MJVec, n::Integer)
    ccall((:mju_isZero, libmujoco), Cint, (Ptr{mjtNum}, Cint), vec, n)
end

"""standard normal random number generator (optional second number)"""
function mju_standardNormal(num2::MJVec)
    ccall((:mju_standardNormal, libmujoco), mjtNum, (Ptr{mjtNum},), num2)
end

"""convert from float to mjtNum"""
function mju_f2n(res::MJVec, vec::MJVec, n::Integer)
    ccall((:mju_f2n, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{Cfloat}, Cint), res, vec, n)
end

"""convert from mjtNum to float"""
function mju_n2f(res::MJVec, vec::MJVec, n::Integer)
    ccall((:mju_n2f, libmujoco), Cvoid, (Ptr{Cfloat}, Ptr{mjtNum}, Cint), res, vec, n)
end

"""convert from double to mjtNum"""
function mju_d2n(res::MJVec, vec::MJVec{Cdouble}, n::Integer)
    ccall((:mju_d2n, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{Cdouble}, Cint), res, vec, n)
end

"""convert from mjtNum to double"""
function mju_n2d(res::MJVec{Cdouble}, vec::MJVec, n::Integer)
    ccall((:mju_n2d, libmujoco), Cvoid, (Ptr{Cdouble}, Ptr{mjtNum}, Cint), res, vec, n)
end

"""insertion sort, increasing order"""
function mju_insertionSort(list::MJVec, n::Integer)
    ccall((:mju_insertionSort, libmujoco), Cvoid, (Ptr{mjtNum}, Cint), list, n)
end

"""Generate Halton sequence."""
function mju_Halton(index::Integer, base::Integer)
    ccall((:mju_Halton, libmujoco), mjtNum, (Cint, Cint), index, base)
end

"""Call strncpy, then set dst[n-1] = 0."""
function mju_strncpy(dst::String, src::String, n::Integer)
    ccall((:mju_strncpy, libmujoco), String, (Cstring, Cstring, Cint), dst, src, n)
end
