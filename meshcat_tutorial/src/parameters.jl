function initialize_problem_parameters()
	N = 25 # node points
	p = 5 # number of players
	n = 4 # state size
	m = 2 # control size
	boundary_points = [0 8.20; 0 -3.20; 0 1.80] # points defining the state bounds
	merge_point = 25.0 # point at which the two lower state bound merges
	radius = 3.0 # collision avoidance radius (need to be a float) ###
	road_width = 10.0 # width of the 2-lane road
	animation_type = :video # the type of MeshCat animation, "image" or "video"
	prob_param = ProblemParameters(N, p, n, m, animation_type,
		radius, road_width, merge_point, boundary_points)
end

function initialize_scene_parameters()
	road_dims = [105.0, 5.0, 0.1] # dimensions of the road [length, width, thickness] in m
	road_trans = [-25.0, 0.0, 0.0] # translations of the road wrt the origin in m
	line_width = 0.2 # width of the yellow lines on the road in m
	boundary_width = 0.4 # width of the road boundaries in m
	cylinder_height = 0.1 # width of the collision cylinders in m
	boundary_height = 0.8 # height of the boundaries
    tracking_id = 1 # id of the car tracked by the camera
 	zoom = 1.0 # zoom of the camera in MeshCat
    traveling_trans = Translation(8.0, 0.0, 10.0) # translation of the camera in m
    traveling_rot = compose(compose(
            LinearMap(AngleAxis(0*pi,  0, 0, 1)),
            LinearMap(AngleAxis(-pi/4, 0, 1, 0))),
            LinearMap(AngleAxis(0*pi,  0, 0, 1))) # rotation of the camera
	birdseye_trans = Translation(20.0, 2.0, 42.0) # translation of the camera in m # for 2 players only
		# "fixed_camera_translation" => Translation(25.0, 2.0, 55.0), # translation of the camera in m
		# "fixed_camera_translation" => Translation(28.0, 2.0, 62.0), # translation of the camera in m # for 8 players only
    birdseye_rot = LinearMap(AngleAxis(-0.397*pi, 0, 1, 0)) #compose(compose(
            # LinearMap(AngleAxis(0.0,  0, 0, 1)),
            # LinearMap(AngleAxis(0.0, 0, 1, 0))),
            # LinearMap(AngleAxis(0.0,  0, 0, 1))), # rotation of the camera
	car_offset = compose(compose(compose(
			Translation(0.0, 0.0, -0.04),
			LinearMap(AngleAxis(pi/200, 0, 1, 0))),
			LinearMap(AngleAxis(pi/2, 0, 0, 1))),
			LinearMap(AngleAxis(pi/2, 1, 0, 0)))
	color_names = ["cornflowerblue", "orange", "forestgreen", "gray",
		"yellow", "red", "orange", "lime"]
	color_values = [100/255 149/255 237/255; # cornflowerblue
					1       165/255 0;       # orange
					34/255  139/255 34/255;  # forestgreen
					0.5     0.5     0.5;     # gray
					1       1       0;       # yellow
					1       0       0;       # red
					1       165/255 0;       # orange
					0       1       0]       # Lime

	scene_parameters = SceneParameters(
		road_dims,
		road_trans,
		line_width,
		boundary_width,
		boundary_height,
		cylinder_height,
		tracking_id,
		zoom,
		traveling_trans,
		traveling_rot,
		birdseye_trans,
		birdseye_rot,
		car_offset,
		color_names,
		color_values)
end
