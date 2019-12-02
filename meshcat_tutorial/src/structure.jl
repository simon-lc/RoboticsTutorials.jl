mutable struct SceneParameters{T}
	road_dims::Array{T,1}
	road_trans::Array{T,1}
	line_width::T
	boundary_width::T
	boundary_height::T
	cylinder_height::T
	tracking_id::Integer
	zoom::T
	traveling_trans::Translation
	traveling_rot::LinearMap
	birdseye_trans::Translation
	birdseye_rot::LinearMap
	car_offset::AffineMap
	color_names::Array{String,1}
	color_values::Array{T,2}
end

mutable struct ProblemParameters{T}
	N::Integer
	p::Integer
	n::Integer
	m::Integer
	animation_type::Symbol
	radius::T
	road_width::T
	merge_point::T
	boundary_points::Array{T,2}
end
