world {}

table (world){
    shape:ssBox, Q:<t(0 0. .2)>, size:[4. 4. .1 .02], color:[.2 .55 0.2]
    contact, logical:{ }
    friction:.1
}

f_wall (table){
    shape:ssBox, Q:<t(0 2 .1)>, size:[4 .02 .3 .01], color:[.7 .5 .5]
    contact, logical:{ }
    friction:.1
}

b_wall (table){
    shape:ssBox, Q:<t(0 -2 .1)>, size:[4. .02 .3 .01], color:[.7 .5 .5]
    contact, logical:{ }
    friction:.1
}

l_wall (table){
    shape:ssBox, Q:<t(2 0 .1)>, size:[.02 4. .3 .01], color:[.7 .5 .5]
    contact, logical:{ }
    friction:.1
}

r_wall (table){
    shape:ssBox, Q:<t(-2 0 .1)>, size:[.02 4. .3 .01], color:[.7 .5 .5]
    contact, logical:{ }
    friction:.1
}


#L_lift (table){ joint:transZ, limits:[0 .5] }

Prefix: "A_"
Include: 'panda_moveGripper.g'

Prefix: "B_"
Include: 'panda_moveGripper.g'

Prefix!

Edit A_panda_link0 (table) { Q:<t(-1.5 0 .1) d(0 0 0 1)> }
Edit A_gripper { contact:1 }
Edit B_panda_link0 (table)  { Q:<t( 1.5 0 .1) d(180 0 0 1)> }
Edit R_gripper { contact:1 }

camera(world){
    Q:<t(1 0 2) d(30 1 90 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}


ball1 	{  shape:sphere, size:[.05],, mass:0.2 X:<[-1.5, -.5, 1, 1, 0, 0, 0]> , color:[1 1 .5], contact}

ball2 	{  shape:sphere, size:[.05],, mass:0.2 X:<[-1.5, .5, 1,  1, 0, 0, 0]> , color:[1 .5 .5], contact}

ball3 	{  shape:sphere, size:[.05],, mass:0.2 X:<[-0.75, 0, 1,  1, 0, 0, 0]> , color:[0.5 .5 1], contact}



#shape ramp_1 (table){ type=mesh rel=<T -0.75 0 .15 0.7071 0 0 -0.7071 >  mesh='ramp_1.stl'  meshscale=0.01  rel_includes_mesh_center,  color:[.8 0 1]  }
ramp_1 (table) {
    shape:ssCvx, # this states "sphere-swept convex"
    mesh:'ramp1.arr', # this specifies the core of the object
    size:[.03] # the sphere-sweeping radius
    color:[.8, 0, 1], # the color
    Q:<t(-1.2 0 0) d(-90 0 0 1)> # relative pose: translated by (-0.75, 0, .15)
}

#shape ramp_2 (table){ type=mesh rel=<T -0.7 0 .35 0.7071 0 0 -0.7071 >  mesh='ramp_2.stl'  meshscale=0.01  rel_includes_mesh_center,  color:[.5 0 1]}

#shape deflector(table) { type=mesh rel=<T 1.2 0 .3 0.7071 0 0 -0.7071 >  mesh='deflector.stl'  meshscale=0.0014  rel_includes_mesh_center,   color:[1 0 0] }

shape deflector(B_gripper) { type=mesh rel=<T 0 0 -0.55 1 0 0 0 >  mesh='deflector.stl'  meshscale=0.0014  rel_includes_mesh_center,   color:[1 0 0] }
#1/sqrt2
#deflector_side (deflector) {shape:ssBox, Q:<t(-2 0 .1)>, size:[.1 0.1 .01 .01], color:[.7 .5 .5]}


Prefix: "A_"
Include: 'bin.g'

Prefix: "B_"
Include: 'bin.g'

Edit A_bin_base (table) { Q:<t(.1 -1.5 0) d(-135 0 0 1)> ,color:[1 .5 1]}
Edit B_bin_base (table) { Q:<t(.1 1.5 0) d(135 0 0 1)> ,color:[.5 1 .5]}


#shape bin_1 (table){ type=mesh rel=<T .1 -1.5 .15 0.38268 0 0 0.92387 >  mesh='bin.stl'  meshscale=0.02  rel_includes_mesh_center,   color:[1 1 .5] }
#shape bin_2 (table){ type=mesh rel=<T .1 1.5 .15 0.92387 0 0 0.38268 >  mesh='bin.stl'  meshscale=0.02  rel_includes_mesh_center,   color:[.5 1 .5] }
#shape bin_3 (table){ type=mesh rel=<T 0 0 .15 0.7071 0 0 -0.7071 >  mesh='bin.stl'  meshscale=0.02  rel_includes_mesh_center,   color:[.5 1 .5] }
