// Written by Kolja Poreski
//<9poreski@informatik.uni-hamburg.de>
//

translate([0,0,25]){
    difference(){
    cube([8,2,8],center=true);
        translate([0,0,-4]){
            cube([5.1,1,5],center=true);
        }
    }
}


translate([0,0,11.5]){
    cube([12.5,0.97,12.5],center=true);
}

translate([0,0,0]){
    cube([1.5,0.97,11],center=true);
}



translate([0,0,-6]){
    difference(){
    cube([5,0.97,2],center=true);
    for(i=[-1,1]) {
            translate([i*1.5, 0, 0]) {
                union(){
                    cylinder(r=0.23, h=100, center=true, $fn=60);
                     for(i=[0:3]) {
                         translate([0,i*0.3,0]) 
                         rotate([0,0,30])
                        cylinder(r=0.43, h=0.4, center=false, $fn=6);
                     }
                }
                }
             }
    }
}


translate([0,0,-10]){
    difference(){
        union(){
            cube([10,4,1],center=true);
            translate([0,0,1]){
                cube([6,2,2],center=true);
            }
        }
        translate([0,0,1.5]){
            cube([5.1,1,2],center=true);
        }
        for(i=[-1,1]) {
            translate([i*1.5, 0, -0.52]) {
                cylinder(r=0.23, h=100, center=true, $fn=50);
                cylinder(r=0.5, h=0.4, center=false, $fn=50);
            }
        }
    }
}
