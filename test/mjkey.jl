ENV["MJKEY"] = ENV["MJKEY"] * '\n'

path, io = mktemp()
write(io, ENV["MJKEY"])
flush(io)

using MuJoCo.MJCore
mj_activate(path)




