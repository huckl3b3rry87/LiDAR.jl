module LiDAR

using PyCall
@pyimport imp
Echo=imp.load_source("Echo",Pkg.dir("LiDAR/src/EchoNodeModule/Echo.py"));
#https://github.com/JuliaPy/PyCall.jl/issues/48

#=
r = @spawn Echo[:main]()
proc_py=addprocs(1);
echo=remotecall(2, Echo[:main]())
=#

export Echo

println("loaded")
end # module
