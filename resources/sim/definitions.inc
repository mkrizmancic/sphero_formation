define sphero position
(
#	block
#	(
#			points 3
#			point[0] [1 0.5]
#			point[1] [0 1]
#			point[2] [0 0]                
#			z [0 1]
#	)

	size [0.075 0.075 0.075]
	origin [0 0 0 0]

	gui_nose 1
	drive "omni"
	localization "gps"
	localization_origin [0 0 0 0]
	update_interval  10 # update timestep in milliseconds
)

define floorplan model
(
	# sombre, sensible, artistic
	color "gray30"

	# most maps will need a bounding box
	boundary 1

	gui_nose 1
	gui_grid 0
	gui_outline 0

	update_interval  10 # update timestep in milliseconds
)

# set the resolution of the underlying raytrace model in meters
resolution 0.01

interval_sim 100  # simulation timestep in milliseconds

window
( 
	size [ 430.000 480.000 ] 
	center [ 0 -0.3 ]
	rotate [ 0.000 0 ]
	scale 42
)