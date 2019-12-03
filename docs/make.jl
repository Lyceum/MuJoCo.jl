using Documenter, MuJoCo

const PAGES = [
    "Home" => "index.md",
    "API" => [
        "api/mujoco.md",
        "api/mjcore.md",
    ],
]

makedocs(;
    modules = [MuJoCo],
    format = Documenter.HTML(prettyurls = get(ENV, "GITHUB_ACTIONS", nothing) == "true"),
    pages = PAGES,
    sitename = "MuJoCo.jl",
    authors = "Colin Summers",
    clean = true,
    doctest = true,
    checkdocs = :exports,
    linkcheck = :true,
    linkcheck_ignore = [r"^https://github.com/Lyceum/.*/actions"],
)

deploydocs(repo = "github.com/Lyceum/MuJoCo.jl.git")
