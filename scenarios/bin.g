world {}

top (world){
    shape:ssBox, Q:<t(.1 -1.5 .15) d(0 0 0 1)>, size:[1. 1. .1 .02], color:[1 1 .5]
    contact, logical:{ }
    friction:1
}

bk_wall (top){
    shape:ssBox, Q:<t(.1 -1.5 .15) d(90 90 0 1)>, size:[.5 .5 .1 .02], color:[1 1 .5]
    contact, logical:{ }
    friction:1
}

rg_wall (top){
    shape:ssBox, Q:<t(.1 -1.5 .15) d(90 0 1 1)>, size:[1. .5 .1 .02], color:[1 1 .5]
    contact, logical:{ }
    friction:1
}

lf_wall (top){
    shape:ssBox, Q:<t(.1 -1.5 .15) d(90 0 0 1)>, size:[1. .5 .1 .02], color:[1 1 .5]
    contact, logical:{ }
    friction:1
}