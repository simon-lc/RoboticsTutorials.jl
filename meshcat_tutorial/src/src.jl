using Blink
using Colors: RGBA, RGB
using CoordinateTransformations
using FileIO
using GeometryTypes:
    GeometryTypes, HyperRectangle, Vec, Point, Rectangle, Cylinder,
    HomogenousMesh, SignedDistanceField, HyperSphere, GLUVMesh, Pyramid
using JLD2
using LinearAlgebra
using MeshCat
using MeshIO

include("structure.jl")
include("parameters.jl")
# camera mvt
#multiple cameras
# coordiante transform
# tree structure works
# add stuff to the tree: objects and relationship between them
# loading backgrounds

function load_data()
    # Load data
	load("animation/trajectory/scene.jld2", "X", "U",
		"problem_parameters", "scene_parameters")
end

function car_transformations(X, scene_param, prob_param)
    car_offset = scene_param.car_offset
    N = prob_param.N
    p = prob_param.p
    car_translations = []
    car_rotations = []
    for k=1:N
        car_translation = [Translation(X[1:2,k,id]..., 0) for id=1:p]
        if k==N
            # Deals with the last step, we assume constant heading
            car_rotation = car_rotations[end]
        else
            angles = [atan(X[2,k+1,id]-X[2,k,id], X[1,k+1,id]-X[1,k,id]) for id=1:p]
            car_rotation = [LinearMap(AngleAxis(angles[id], 0, 1, 0)) for id=1:p]
        end
        push!(car_translations, car_translation)
        push!(car_rotations, car_rotation)
    end
    return car_translations, car_rotations
end

function animation(scene_param, prob_param)
    # Visualizes the trajectories obtained using the L1 cost optimizer using Meshcat.
    road_dims = scene_param.road_dims
	line_width = scene_param.line_width
	boundary_width = scene_param.boundary_width
	road_trans = scene_param.road_trans
	road_translation = Translation(road_trans...)
	boundary_height = scene_param.boundary_height
	cylinder_height = scene_param.cylinder_height
	car_offset = scene_param.car_offset
	color_names = scene_param.color_names
	color_values = scene_param.color_values

    # Load Data
    X, U, prob_param, scene_param = load_data()
    # Extract the rescaled state and control trajectories
    p = prob_param.p
	radius = prob_param.radius
	state_bound_points = prob_param.boundary_points
	merge_point = prob_param.merge_point

    # Open visualizer
    vis = Visualizer()
	vis.core.scope
    open(vis)

    # Plot Cars in MeshCat
    car = load("animation/object/car/car_geometry.obj", GLUVMesh)
    for id=1:p
        # Add car
        car_image = PngImage("animation/image/" * color_names[id] * ".png")
        car_texture = Texture(image=car_image)
        car_material = MeshLambertMaterial(map=car_texture)
        setobject!(vis["car$id"], car, car_material)
        settransform!(vis["car$id"], car_offset)
        # Add collision avoidance cylinders
        collision_cylinder = Cylinder(Point(0.0,0.0,0.0),
            Point(0.0,0.0,cylinder_height), radius)
        cylinder_material = MeshPhongMaterial(color=RGBA(color_values[id,:]..., 9e-1))
        setobject!(vis["collision_cylinder$id"], collision_cylinder, cylinder_material)
        settransform!(vis["collision_cylinder$id"], Translation(0.0, 0.0, 0.0))
    end

    # Plot Road in Meshcat
    road_image = PngImage("animation/image/road.png")
    road_texture = Texture(image=road_image)
    road_material = MeshLambertMaterial(map=road_texture)
	road_width = (state_bound_points[1,2]+radius) - (state_bound_points[3,2]-radius) + 2*line_width
	road = HyperRectangle(Vec(0.0, -(road_width/2-road_dims[2]), -road_dims[3]), Vec(road_dims[1], road_width, road_dims[3]))
    setobject!(vis["road"], road, road_material)
    settransform!(vis["road"], road_translation)

    # Plot lines in Meshcat
    line_material = MeshPhongMaterial(color=RGBA(1, 1, 0, 1.0))
    line = Rectangle(0.0, 0.0, road_dims[1], line_width)
    for i=1:3
        setobject!(vis["line$i"], line, line_material)
        line_translation = compose(Translation(0, (i-1)*road_dims[2]-line_width/2, 0.04), road_translation)
        settransform!(vis["line$i"], line_translation)
    end

    # Plot Road Boundaries in Meshcat
    boundary_image = PngImage("animation/image/black_boundary.png")
    boundary_texture = Texture(image=boundary_image)
	boundary_material = MeshLambertMaterial(map=boundary_texture)
	ramp_extension = 10.0
    # Upper Boundary
    boundary1 =	HyperRectangle(Vec(0.0, 0.0, 0.0), Vec(road_dims[1], boundary_width, boundary_height))
	setobject!(vis["boundary1"], boundary1, boundary_material)
	boundary1_translation = compose(Translation(0, state_bound_points[1,2]+radius, 0.0), road_translation)
	settransform!(vis["boundary1"], boundary1_translation)
	# Lower Boundary 1
	Δ = [merge_point, state_bound_points[3,2] - state_bound_points[2,2]]
	length = sqrt(Δ'*Δ) + ramp_extension
	angle = atan(Δ[2], Δ[1])
	boundary2 =	HyperRectangle(Vec(-ramp_extension, -boundary_width, 0.0), Vec(length - radius*sin(angle/2), boundary_width, boundary_height))
	setobject!(vis["boundary2"], boundary2, boundary_material)
	boundary2_translation = Translation(radius*cos(-pi/2+angle), state_bound_points[2,2]+radius*sin(-pi/2+angle), 0.0)
	boundary2_translation = compose(Translation(-4*radius*sin(angle/2), 0, 0), boundary2_translation)
	boundary2_translation = compose(LinearMap(AngleAxis(angle, 0, 0, 1)), boundary2_translation)
	settransform!(vis["boundary2"], boundary2_translation)
	# Lower Boundary 2
	boundary3 =	HyperRectangle(Vec(0.0, -boundary_width, 0.0), Vec(road_dims[1]-merge_point+road_trans[1]-radius*sin(angle/2), boundary_width, boundary_height))
	setobject!(vis["boundary3"], boundary3, boundary_material)
	boundary3_translation = Translation(merge_point+radius*sin(angle/2), state_bound_points[3,2]-radius, 0.0)
	settransform!(vis["boundary3"], boundary3_translation)

	# Plot Ramp in Meshcat
	ramp_width = road_width - road_dims[2]
	ramp = HyperRectangle(Vec(-ramp_extension, 0.0, -road_dims[3]), Vec(length, ramp_width, road_dims[3]))
	ramp_translation = Translation(0.0, state_bound_points[2,2]-radius, -0.04)
	ramp_translation = compose(LinearMap(AngleAxis(angle, 0, 0, 1)), ramp_translation)
    setobject!(vis["ramp"], ramp, road_material)
    settransform!(vis["ramp"], ramp_translation)

	# Animate the scene
	vis, anim = scene_animation(vis, X, U, scene_param, prob_param)
    MeshCat.setanimation!(vis, anim)
end

function scene_animation(vis, X, U, scene_param, prob_param)
	# Animate the scene
	cylinder_height = scene_param.cylinder_height
	car_offset = scene_param.car_offset
	tracking_id = scene_param.tracking_id
    zoom = scene_param.zoom
	traveling_trans = scene_param.traveling_trans
	traveling_rot = scene_param.traveling_rot
	birdseye_trans = scene_param.birdseye_trans
    birdseye_rot = scene_param.birdseye_rot
	color_names = scene_param.color_names
	color_values = scene_param.color_values

    n = prob_param.n
    m = prob_param.m
    p = prob_param.p
	N = prob_param.N
	radius = prob_param.radius

	# Plot Trajectory
    anim = MeshCat.Animation()
	default_framerate = 6
	anim = MeshCat.Animation(anim.clips, default_framerate)
    # Compute car transformations
    car_translations, car_rotations = car_transformations(X, scene_param, prob_param)
	# We get rid of the last frame if the final state constraints are relaxed

	if prob_param.animation_type == :video
		for k=1:N-1
	        # Set the poses of the two cars.
	        MeshCat.atframe(anim, vis, k) do frame
	            for id=1:p
	                car_translations[k][id]
	                car_rotations[k][id]
	                settransform!(frame["car$id"],
	                    compose(compose(car_translations[k][id],
	                    car_offset),car_rotations[k][id]))
	                settransform!(frame["collision_cylinder$id"], car_translations[k][id])
	            end
				@show frame["/Grid/<object>"].path
				@show typeof(frame)
				setprop!(frame["/Cameras/default/rotated/<object>"], "zoom", zoom)
				setprop!(frame["/Cameras/default/rotated/<object>"], "zoom", zoom)
	            camera_transformation = compose(compose(
	                Translation(X[1:2,k,tracking_id]..., 0), # follow car with id tracking_id
	                traveling_trans),
	                traveling_rot
	                )
	            settransform!(frame["/Cameras/default"], camera_transformation)
	        end
	    end
	elseif prob_param.animation_type == :image
		car = load("animation/object/car/car_geometry.obj", GLUVMesh)
		collision_cylinder = Cylinder(Point(0.0,0.0,0.0),
			Point(0.0,0.0,cylinder_height), radius)
		alpha = 3e-1
		for k=1:N-1

	        # Set the poses of the two cars.
	        MeshCat.atframe(anim, vis, k) do frame
	            for id=1:p
	                car_translations[k][id]
	                car_rotations[k][id]
	                settransform!(frame["car$id"],
	                    compose(compose(car_translations[k][id],
	                    car_offset),car_rotations[k][id]))
	                settransform!(frame["collision_cylinder$id"], car_translations[k][id])
	            end
				setprop!(frame["/Cameras/default/rotated/<object>"], "zoom", zoom)
				setprop!(frame["/Lights/DirectionalLight/<object>"], "intensity", 1.2)
	            camera_transformation = compose(
	                birdseye_trans,
	                birdseye_rot)
	            settransform!(frame["/Cameras/default"], camera_transformation)
	        end
	    end
	end
	# setprop!(framevis["/Cameras/default/rotated/<object>"], "zoom", 0.5)
	# setprop!(vis, "/Lights/DirectionalLight/<object>", "intensity", 1.2)
	return vis, anim
end

function convert_meshcat_to_video(;filename=nothing,
	input_path="animation/meshcat_sequence/",
	output_path="animation/video/")
	# Saving MeshCat sequence as a video.
	meshcat_sequence_dir = joinpath(@__DIR__, "..", input_path)
	if filename==nothing
		filenames = readdir(meshcat_sequence_dir)
	else
		filenames = [filename * ".tar"]
	end
	for filename in filenames
		println(filename)
		video_dir = joinpath(@__DIR__, "..", output_path, filename[1:end-4] * ".mp4",)
		MeshCat.convert_frames_to_video(
			meshcat_sequence_dir * filename,
			video_dir,
			overwrite=true)
	end
	return
end

# convert_meshcat_to_video()

X, U, prob_param, scene_param= load_data()
prob_param = initialize_problem_parameters()
scene_param = initialize_scene_parameters()
animation(scene_param, prob_param)




# Open visualizer
vis = Visualizer()
# open(vis)
# open(vis, Blink.Window(Dict(:width => 1280, :height => 70, :useContentSize => true)))
# Animate the scene
cylinder_height = scene_param.cylinder_height
car_offset = scene_param.car_offset
tracking_id = scene_param.tracking_id
zoom = scene_param.zoom
traveling_trans = scene_param.traveling_trans
traveling_rot = scene_param.traveling_rot
birdseye_trans = scene_param.birdseye_trans
birdseye_rot = scene_param.birdseye_rot
color_names = scene_param.color_names
color_values = scene_param.color_values

n = prob_param.n
m = prob_param.m
p = prob_param.p
N = prob_param.N
radius = prob_param.radius

# Plot Trajectory
anim = MeshCat.Animation()
default_framerate = 6
anim = MeshCat.Animation(anim.clips, default_framerate)
delete!(vis["/Grid"])
delete!(vis["/Axes"])
# delete!(vis["Background"])

# Compute car transformations
car_translations, car_rotations = car_transformations(X, scene_param, prob_param)
# We get rid of the last frame if the final state constraints are relaxed

if prob_param.animation_type == :video
	for k=1:N-1
		# Set the poses of the two cars.
		MeshCat.atframe(anim, vis, k) do frame
			for id=1:p
				car_translations[k][id]
				car_rotations[k][id]
				settransform!(frame["car$id"],
					compose(compose(car_translations[k][id],
					car_offset),car_rotations[k][id]))
				settransform!(frame["collision_cylinder$id"], car_translations[k][id])
			end
			# @show fieldnames(typeof(frame["/Grid/<object>"].core.tree))
			# @show typeof(frame["/Grid/<object>"].path.entries)
			# @show frame["/Grid/<object>"].path.entries
			# @show frame["/Grid/<object>"].core.tree.object
			# @show frame["/Grid/<object>"].core.tree.transform
			# @show frame["/Grid/<object>"].core.tree.children
			# key = keys(frame["/Grid/<object>"].core.tree.children)[1]
			# @show keys(frame["/Grid/<object>"].core.tree.children)
			# @show frame.:core
			# @show typeof(frame)
			# delete!(frame["/Grid"])
			@show frame.path.entries
			@show frame.core.tree
			# @show frame["/Cameras/default/rotated/<object>"].core.tree
			# @show frame["/Cameras/default/rotated/<object>"].core.tree.object
			# @show frame["/Cameras/default/rotated/<object>"].core.tree.transform
			# @show frame["/Cameras/default/rotated"].core.tree.children
			# @show keys(frame["/Cameras/default/rotated"].core.tree.children)
			# delete!(frame["/Grid"])
			# delete!(frame["/Axes"])
			setprop!(frame["/Cameras/default/rotated/<object>"], "zoom", zoom)
			setprop!(frame["../Background"], "hide_background", true)
			setprop!(frame["../Background"], "show_background", false)
			camera_transformation = compose(compose(
				Translation(X[1:2,k,tracking_id]..., 0), # follow car with id tracking_id
				traveling_trans),
				traveling_rot
				)
			settransform!(frame["/Cameras/default"], camera_transformation)
		end
	end
end
send(vis.core, hide_background)

# anim.clips[vis["/meshcat/car2"].path]

# parameters = define_problem_parameters()
# animation_parameters = define_animation_parameters()
# problem_parameters = initialize_problem_parameters()
# scene_parameters = initialize_scene_parameters()
# FileIO.save("animation/trajectory/scene.jld2",
# 	"X", X,
# 	"U", U,
# 	"problem_parameters", problem_parameters,
# 	"scene_parameters", scene_parameters)
