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

Prefix: "L_"
Include: 'panda_moveGripper.g'

Prefix: "R_"
Include: 'panda_moveGripper.g'

Prefix!
        
Edit L_panda_link0 (table) { Q:<t(-.4 -.4 .1) d(90 0 0 1)> }
Edit R_panda_link0 (table)  { Q:<t( .4 -.4 .1) d(90 0 0 1)> }

camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}

shape ramp_1 (table){ type=mesh rel=<T -0.0282159 0.0517696 0.272424 1 0 0 0 >  mesh='ramp_1.stl'  meshscale=0.01  rel_includes_mesh_center,  }

shape ramp_2 (table){ type=mesh rel=<T -0.0582159 0.0517696 0.372424 1 0 0 0 >  mesh='ramp_2.stl'  meshscale=0.01  rel_includes_mesh_center,  color:[.9 0 0]}

shape deflector (table){ type=mesh rel=<T -0.0782159 0.0517696 0.472424 1 0 0 0 >  mesh='deflector.stl'  meshscale=0.001  rel_includes_mesh_center,  }

shape bin (table){ type=mesh rel=<T -0.0982159 0.0517696 0.472424 1 0 0 0 >  mesh='bin.stl'  meshscale=0.01  rel_includes_mesh_center,  }
