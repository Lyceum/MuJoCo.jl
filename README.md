# MuJoCo

*A Julia wrapper for the MuJoCo physics simulator, a physics simulator designed for
multi-joint dynamics with contacts that has become a standard in robotics,
reinforcement learning, and trajectory optimization, both in both academia and industry.*

![](https://github.com/Lyceum/MuJoCo.jl/workflows/CI/badge.svg)

`MuJoCo.jl` is part of the [Lyceum](https://www.lyceum.ml), a software suite for
machine learning, control, and trajectory optimization. For an example of how to use
MuJoCo.jl, check out [LyceumMuJoCo.jl](https://github.com/Lyceum/LyceumMuJoCo.jl).


## Obtaining a License

To use MuJoCo, you'll need a valid license which you can obtain from
[here](https://www.roboti.us/license.html). Up to three, thirty-day trials can be obtained
for free from MuJoCo's webiste and students are eligible for a free personal license.
Once you have obtained the license file, set the environment variable `MUJOCO_KEY_PATH`
to point to its location. On Linux machines this would look like:
```
$ export MUJOCO_KEY_PATH=/path/to/mjkey.txt
```

## Interface

**MuJoCo.jl** provides the following:

1. `MuJoCo.MJCore`: A minimal wrapper for the [MuJoCo](http://mujoco.org/) physics simulator,
    providing a nearly one-to-one mapping to MuJoCo's C interface. MuJoCo features that Julia already
    provides are not wrapped (e.g. `mju_add3` is just a normal array operation). Additionally, MuJoCo's
    C interface may be altered in cases where it improves ease of use or safety (e.g. MuJoCo.jl never
    requires pre-allocated error string buffers to be passed in).
2. `MuJoCo`: Zero-cost abstractions over the types contained in `MuJoCo.MJCore` that
    provide a more convenient interface for interacting with MuJoCo (e.g. by wrapping a raw
   `Ptr{Float64}` with a `Array{Float64}`). These types are compatible with all the functions
   defined in `MuJoCo.MJCore`. We also provide extra utility functions that you may find useful. These functions and types are prefixed with `jl_` and `jl`, respectively.

Functions from each module are exported by MuJoCo.jl, while only the types in `MuJoCo` are exported (e.g. `jlModel`). We recommend using these types over the raw `MJCore` types wherever possible.

All functions are documented for convenience:
```
help?> mj_step
```

For more complete about the MuJoCo physics simulator, see [MuJoCo's documentation](http://www.mujoco.org/book).


### MuJoCo


#### `jlData` and `jlModel`

`MuJoCo` provides `jlData` and `jlModel` which differ from `MJCore.mjData` and `MJCore.mjModel` as follows:

   1) Fields of type `SArray{S,T,N}` become `Array{T, N}` (see [#7](https://github.com/Lyceum/MuJoCo.jl/issues/7))
   2) Field of type `Ptr{T<:Number}` become `Array{T, N}` (e.g. `size(jlData.qpos) == (nq, )`).
   3) For fields that change size dynamically (e.g. `mjData.contact`), we provide
      dynamically-sized `view`'s into the underlying array (e.g. `size(jlData.contact) == (ncon, )` not `(nconmax, )`).
   4) Fields that are not useful in Julia are not exposed (e.g. `mjData.buffer`)
   5) `mj_deleteModel`/`mj_deleteData` are called by Julia's garbage collector automatically.

`jlData` and `jlModel` (as well as `mjData` and `mjModel`) are mutable structs, while the fields of these types are not. To assist in mutating nested structs, we recommend using the `@set!!` macro provided by [BangBang.jl](https://github.com/tkf/BangBang.jl):
```julia
using MuJoCo, BangBang
m = jlModel("humanoid.xml")
@assert m.opt.timestep == 0.002
@set!! m.opt.timestep = 0.001
@assert m.opt.timestep == 0.001
```

#### Globals

All MuJoCo globals are available under in the `MJCore` module (e.g. `MJCore.mjVISSTRING`).
`const` global primitives like `Int` (e.g. `mjMAXIMP`) are defined directly in Julia, while
MuJoCo string arrays (e.g. `mjVISSTRING`) are stored as `RefValue`'s and loaded upon
module initialization. Global callbacks are stored in a `RefValue`-like object
called a `CRef` and can be used in the same manner as `RefValue`. Under the hood,
calls to `getindex`/`setindex!` perform the appropriate load/store from the corresponding
`Ptr{Cvoid}`.

##### Example

```julia
@assert MJCore.mjMAXIMP == 0.9999
@assert MJCore.mjDISABLESTRING[][1] == "Constraint"
# Set the `mjcb_user_warning_cb` callback to generate Julia warnings:
my_warning_cb(msg::Cstring) = (@warn unsafe_string(msg); nothing)
warncb = @cfunction(my_warning_cb, Cvoid, (Cstring,))
MJCore.mju_user_warning[] = warncb
```

See [this blog post](https://julialang.org/blog/2013/05/callback) for more information on how to set C callbacks from Julia.

## Getting Started

MuJoCo.jl is currently registered in Lyceum's package registry. Until it is moved to General,
you will need to add `Lyceum/LyceumRegistry`.

From the Julia REPL, type `]` to enter Pkg mode:
```julia-repl
julia> ]
(v1.3) pkg> registry add https://github.com/Lyceum/LyceumRegistry.git
(v1.3) pkg> add MuJoCo
```

Below we simulate passive dynamics and print out joint positions
at each timestep:
```julia
using MuJoCo
mj_activate("mjkey.txt")
# alternatively: set the environment variable `MUJOCO_KEY_PATH`
#                before `using MuJoCo`.

m = jlModel("humanoid.xml")
d = jlData(m)
for i=1:100
    mj_step(m, d);
    println(d.qpos)
end
```

and compared to C:
```c
#include "mujoco.h"
#include <stdio.h>
char error[1000] = "Could not load xml model";
mj_activate("mjkey.txt");

m = mj_loadXML("humanoid.xml", 0, error, 1000);
d = mj_makeData(m);
for( int step=0; step<100; step++ )
{
    mj_step(m, d);
    for(i=0; i < m->nq; i++)
        printf("%d ", d->qpos[i]);
    printf("\n");
}
mj_deleteData(d);
mj_deleteModel(m);
```

**Big thanks to [@klowrey](https://github.com/klowrey) for providing the [original](https://github.com/klowrey/MuJoCo.jl) MuJoCo wrapper**.
