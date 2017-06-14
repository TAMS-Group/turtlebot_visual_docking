// Written by Kolja Poreski
//<9poreski@informatik.uni-hamburg.de>
//

translate([0,0,150]){
    union(){
    cube([125,5,125],center=true);
    translate([0,8,-60]){
  difference(){
    cube([125,20,5],center=true);
   for(i=[-10,10]) {
            translate([i*1.5, 0, -5.2]) {
                cylinder(r=2.3, h=1000, center=true, $fn=50);
               translate([0,0,5.0]){ 
                cylinder(r=5.0, h=4.0, center=false, $fn=50);
               }
        }
       }
    }
 }
    }
}

translate([0,0,0]){
    cube([15,9.7,110],center=true);
}

translate([0,0,60]){
    difference(){
    cube([50,9.7,20],center=true);
    for(i=[-10,10]) {
            translate([i*1.5, 0, 0]) {
                union(){
                    cylinder(r=2.3, h=1000, center=true, $fn=60);
                     for(i=[0:3]) {
                         translate([0,i*3.0,0]) 
                         rotate([0,0,30])
                        cylinder(r=4.3, h=4.0, center=false, $fn=6);
                     }
                }
                }
             }
    }
}

translate([0,0,-60]){
    difference(){
    cube([50,9.7,20],center=true);
    for(i=[-1,1]) {
            translate([i*15, 0, 0]) {
                union(){
                    cylinder(r=2.3, h=1000, center=true, $fn=60);
                     for(i=[0:3]) {
                         translate([0,i*3.0,0]) 
                         rotate([0,0,30])
                        cylinder(r=4.3, h=4.0, center=false, $fn=6);
                     }
                }
                }
             }
    }
}


translate([0,0,-100]){
    difference(){
        union(){
            cube([100,40,10],center=true);
            translate([0,0,10]){
                cube([60,20,20],center=true);
            }
        }
        translate([0,0,15]){
            cube([51,10,20],center=true);
        }
        for(i=[-1,1]) {
            translate([i*15, 0, -5.2]) {
                cylinder(r=2.3, h=1000, center=true, $fn=50);
                cylinder(r=5.0, h=4.0, center=false, $fn=50);
            }
        }
    }
}
